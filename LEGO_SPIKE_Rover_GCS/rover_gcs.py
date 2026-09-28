#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
 SPIKE Rover GCS  --  PySide6 독립 실행형 지상관제소
===============================================================================
 브라우저를 전혀 쓰지 않습니다. bleak 로 LEGO SPIKE Prime 허브에 직접 BLE 연결해
 센서/IMU 텔레메트리를 받고 모터를 제어합니다.

 필요 패키지
     pip install PySide6 bleak pyserial

 허브 준비
     hub_rover.py 를 슬롯에 다운로드하고 실행해 둡니다.
     GCS의 [프로그램 시작] 버튼으로도 실행할 수 있습니다.

 통신 구조 (HTML 버전과 동일한 규약)
     GCS  --BLE TunnelMessage(0x32)-->  hub.config["module_tunnel"].callback
     GCS  <--BLE TunnelMessage(0x32)--  module_tunnel.send() (콘솔도 수신)
     USB Serial은 별도 진단 경로이며 펌웨어의 stdin 지원 확인이 필요합니다.

 명령
     T<좌>,<우>  주행      S 정지        E 비상정지
     H0/H1 자세유지        Y<각도> 목표각  Z 기준각 리셋
     K<kp>,<ki>,<kd> 게인   V<최대속도>    A<포트>,<속도> 보조모터
     P<좌>,<우> 포트지정    I<0|1>,<0|1> 반전   Q 상태조회
 텔레메트리
     #T,yaw,pitch,roll,lvel,rvel,lpos,rpos,dist,refl,batt,hold,ms
===============================================================================
"""

import sys
import time
import math
import asyncio
import threading
import traceback
import html
from camera_panel import CameraPanel
from collections import deque

from PySide6.QtCore import (Qt, QObject, Signal, QTimer, QPointF, QRectF, QEvent)
from PySide6.QtGui import (QPainter, QColor, QPen, QBrush, QFont, QPolygonF,
                           QPainterPath, QKeyEvent)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QSlider, QCheckBox, QLineEdit, QTextEdit, QGroupBox, QGridLayout,
    QHBoxLayout, QVBoxLayout, QListWidget, QListWidgetItem, QSpinBox,
    QDoubleSpinBox, QFileDialog, QSizePolicy, QFrame, QScrollArea, QAbstractSpinBox)

# ---- 선택적 의존성 ---------------------------------------------------------
try:
    from bleak import BleakScanner, BleakClient
    HAVE_BLEAK = True
except ImportError:                                    # pragma: no cover
    HAVE_BLEAK = False

try:
    import serial as pyserial
    import serial.tools.list_ports as list_ports
    HAVE_SERIAL = True
except ImportError:                                    # pragma: no cover
    HAVE_SERIAL = False


# =============================================================================
#  1. LEGO SPIKE Prime BLE 프로토콜
#     lego.github.io/spike-prime-docs 의 레퍼런스 구현을 그대로 옮긴 것
# =============================================================================
SPIKE_SERVICE = "0000fd02-0000-1000-8000-00805f9b34fb"
SPIKE_RX_CHAR = "0000fd02-0001-1000-8000-00805f9b34fb"   # 허브로 쓰기
SPIKE_TX_CHAR = "0000fd02-0002-1000-8000-00805f9b34fb"   # 허브에서 알림

MSG_INFO_REQ = 0x00
MSG_INFO_RES = 0x01
MSG_PROGRAM_FLOW_REQ = 0x1E
MSG_PROGRAM_FLOW_NOTI = 0x20
MSG_CONSOLE_NOTI = 0x21
MSG_TUNNEL = 0x32

DELIMITER = 0x02
NO_DELIMITER = 0xFF
COBS_CODE_OFFSET = DELIMITER
MAX_BLOCK_SIZE = 84
COBS_XOR = 3


def cobs_encode(data: bytes) -> bytearray:
    buffer = bytearray()
    code_index = block = 0

    def begin_block():
        nonlocal code_index, block
        code_index = len(buffer)
        buffer.append(NO_DELIMITER)
        block = 1

    begin_block()
    for byte in data:
        if byte > DELIMITER:
            buffer.append(byte)
            block += 1
        if byte <= DELIMITER or block > MAX_BLOCK_SIZE:
            if byte <= DELIMITER:
                buffer[code_index] = byte * MAX_BLOCK_SIZE + block + COBS_CODE_OFFSET
            begin_block()
    buffer[code_index] = block + COBS_CODE_OFFSET
    return buffer


def cobs_decode(data: bytes) -> bytearray:
    buffer = bytearray()

    def unescape(code):
        if code == 0xFF:
            return None, MAX_BLOCK_SIZE + 1
        value, blk = divmod(code - COBS_CODE_OFFSET, MAX_BLOCK_SIZE)
        if blk == 0:
            blk = MAX_BLOCK_SIZE
            value -= 1
        return value, blk

    if not data:
        return buffer
    value, block = unescape(data[0])
    for byte in data[1:]:
        block -= 1
        if block > 0:
            buffer.append(byte)
            continue
        if value is not None:
            buffer.append(value)
        value, block = unescape(byte)
    return buffer


def cobs_pack(data: bytes) -> bytes:
    buf = cobs_encode(data)
    for i in range(len(buf)):
        buf[i] ^= COBS_XOR
    buf.append(DELIMITER)
    return bytes(buf)


def cobs_unpack(frame: bytes) -> bytes:
    if not frame:
        return b""
    start = 1 if frame[0] == 0x01 else 0          # 사용하지 않는 priority 바이트
    end = len(frame) - 1 if frame[-1] == DELIMITER else len(frame)
    if end <= start:
        return b""
    unxored = bytes(b ^ COBS_XOR for b in frame[start:end])
    return bytes(cobs_decode(unxored))


def make_tunnel(text: str) -> bytes:
    payload = (text + "\r\n").encode("utf-8")
    return bytes([MSG_TUNNEL]) + len(payload).to_bytes(2, "little") + payload


# =============================================================================
#  2. 링크 (BLE / Serial) — 둘 다 같은 시그널을 낸다
# =============================================================================
class LinkBase(QObject):
    sig_line = Signal(str)          # 허브가 보낸 텍스트 한 줄
    sig_raw = Signal(bytes)         # 원시 바이트 (진단용)
    sig_log = Signal(str, str)      # 메시지, 등급(info/tx/rx/warn/err)
    sig_conn = Signal(bool, str)    # 연결여부, 표시이름
    sig_tx = Signal(int)            # 전송 요청 누적 건수 (허브 ACK 아님)

    def __init__(self):
        super().__init__()
        self.connected = False
        self.safe_mode = False
        self.kind = ""
        self.last_rx = 0.0
        self.last_telemetry = 0.0
        self.command_ready = False
        self.connected_at = 0.0
        self.fw_info = ""
        self._text = ""
        self._tunnel_text = ""
        self._tunnel_rx_count = 0
        self.label = ""
        self.tx_count = 0
        self._last_drop_warn = 0.0

    # ---- 수신 텍스트를 줄 단위로 쪼갠다 --------------------------------
    def _feed_text(self, chunk: str, channel="console"):
        attr = "_tunnel_text" if channel == "tunnel" else "_text"
        self.last_rx = time.monotonic()
        buf = getattr(self, attr, "") + chunk
        while True:
            idx = -1
            for i, c in enumerate(buf):
                if c in "\r\n":
                    idx = i
                    break
            if idx < 0:
                break
            line = buf[:idx].strip()
            buf = buf[idx + 1:]
            if line:
                if line.startswith("#T,"):
                    try:
                        fields = line[3:].split(",")
                        if len(fields) < 12:
                            raise ValueError("short telemetry")
                        [float(v) for v in fields[:12]]
                        self.last_telemetry = time.monotonic()
                    except ValueError:
                        pass
                if line == "#I,ack=Q":
                    self.command_ready = True
                self.sig_line.emit(line)
        setattr(self, attr, "" if len(buf) > 4096 else buf)

    def program_alive(self) -> bool:
        return self.connected and self.last_telemetry > 0 and (time.monotonic() - self.last_telemetry) < 3.0

    # ---- 전송 관문 ------------------------------------------------------
    # 모든 명령은 여기를 통과한다. 조용히 버려지는 경로가 없도록
    # 차단될 때마다 이유를 로그로 남기고 False 를 돌려준다.
    def set_safe_mode(self, on: bool):
        self.safe_mode = bool(on)
        self.sig_log.emit(
            "안전 모드 ON — 허브로 아무것도 보내지 않습니다"
            if on else "안전 모드 OFF — 이제 명령이 전송됩니다",
            "warn" if on else "info")
        self.sig_conn.emit(self.connected, self.label)

    def _warn_drop(self, why: str):
        t = time.monotonic()
        if t - self._last_drop_warn < 2.0:
            return                       # 도배 방지
        self._last_drop_warn = t
        self.sig_log.emit("전송 차단 — " + why, "warn")

    def can_send(self) -> bool:
        return self.connected and not self.safe_mode

    def send(self, line: str) -> bool:
        if not self.connected:
            self._warn_drop("허브가 연결되어 있지 않습니다")
            return False
        if self.safe_mode:
            self._warn_drop("안전 모드입니다. [안전 모드 해제]를 누르세요.")
            return False
        if line and line[0].upper() not in ("Q", "E", "S"):
            if not self.program_alive() or not self.command_ready:
                self._warn_drop("허브 프로그램의 텔레메트리와 Q 응답을 먼저 확인하세요")
                return False
        if self._send_impl(line):
            self.tx_count += 1
            self.sig_tx.emit(self.tx_count)
            return True
        return False

    def _send_impl(self, line: str) -> bool:
        raise NotImplementedError

    def program_flow(self, stop: bool, slot: int):
        raise NotImplementedError

    def interrupt(self):
        """실행 중인 것을 끊고 파이썬 REPL 프롬프트로 빠져나온다.
        REPL 에 명령을 보내려면 이게 먼저다 (Ctrl-C = 0x03)."""
        raise NotImplementedError


class BleLink(LinkBase):
    """bleak 를 별도 스레드의 asyncio 루프에서 돌리고 Qt 시그널로 전달한다."""

    sig_devices = Signal(list)      # [(name, address, rssi)]

    def __init__(self):
        super().__init__()
        self.kind = "ble"
        self.max_packet = 20
        self._scan_objs = {}            # 주소 → BLEDevice (스캔 결과 원본)
        self.force_rediscover = False   # OS 서비스 캐시를 무시할지
        self.link_only = False          # 알림 등록도 하지 않는 순수 연결 시험
        self._connecting = False
        self._client = None
        self._frame = bytearray()
        self._priority_frame = bytearray()
        self._high_priority = False
        self._info_event = None
        self._session = 0
        self._lock = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="ble-loop")
        self._thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _submit(self, coro):
        if not self._loop.is_running():
            coro.close()
            return None
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    # ------------------------------ 스캔 --------------------------------
    def scan(self, all_devices=False):
        self._submit(self._scan(all_devices))

    async def _scan(self, all_devices):
        if not HAVE_BLEAK:
            self.sig_log.emit("bleak 가 설치되지 않았습니다: pip install bleak", "err")
            return
        self.sig_log.emit("BLE 스캔 중… (약 6초)", "info")
        try:
            try:
                kwargs = {} if all_devices else {"service_uuids": [SPIKE_SERVICE]}
                found = await BleakScanner.discover(timeout=6.0, **kwargs)
            except TypeError:
                # 구버전 bleak 은 service_uuids 를 받지 않는다
                found = await BleakScanner.discover(timeout=6.0)
            out = []
            self._scan_objs = {}
            for d in found:
                self._scan_objs[d.address.upper()] = d
                rssi = getattr(d, "rssi", None)
                out.append((d.name or "(이름없음)", d.address,
                            rssi if rssi is not None else 0))
            self.sig_devices.emit(out)
            if out:
                self.sig_log.emit(f"스캔 완료: {len(out)}개 발견", "info")
            else:
                self.sig_log.emit(
                    "허브를 찾지 못했습니다. 허브 오른쪽 위 블루투스 버튼을 눌러 "
                    "파란불이 깜빡이게 한 뒤 다시 스캔하세요. "
                    "([모든 기기]로 필터 없이 볼 수도 있습니다)", "warn")
        except Exception as e:
            self.sig_log.emit(f"스캔 실패: {e}", "err")

    # ------------------------------ 연결 --------------------------------
    def connect_to(self, address, safe=False, link_only=False):
        if self._connecting:
            self.sig_log.emit("이미 연결을 시도하는 중입니다. 잠시 기다리세요.", "warn")
            return
        self._connecting = True
        self.link_only = link_only
        self._submit(self._connect(address, safe))

    async def _connect(self, address, safe):
        try:
            await self._connect_inner(address, safe)
        finally:
            self._connecting = False

    async def _connect_inner(self, address, safe):
        if self._client is not None:
            await self._disconnect()
            await asyncio.sleep(0.5)
        try:
            # ble_probe.py 는 BLEDevice 객체로 연결해 30초를 버텼다.
            # GCS 도 같은 조건이 되도록 스캔 때 받은 객체를 그대로 쓴다.
            target = self._scan_objs.get(str(address).upper(), address)
            self.sig_log.emit(
                f"① 연결 시도: {address}"
                + (" (스캔 객체 사용)" if target is not address else ""), "info")
            # ※ use_cached_services=False 는 연결 직후 GATT 전체 재검색을
            #   유발한다. 무선 활동이 급증하므로 기본은 캐시 사용.
            if self.force_rediscover:
                try:
                    client = BleakClient(
                        target, disconnected_callback=self._on_disconnected,
                        winrt={"use_cached_services": False})
                except TypeError:
                    client = BleakClient(
                        target, disconnected_callback=self._on_disconnected)
            else:
                client = BleakClient(
                    target, disconnected_callback=self._on_disconnected)
            for attempt in range(1, 4):
                try:
                    await client.connect()
                    break
                except Exception as e:
                    self.sig_log.emit(f"   시도 {attempt}/3 실패: {e}", "warn")
                    if attempt == 3:
                        raise
                    await asyncio.sleep(0.8)

            self._client = client
            self._session += 1
            self.max_packet = 20
            self._info_event = asyncio.Event()
            self._priority_frame.clear()
            self._high_priority = False
            self._frame.clear()
            self._text = ""
            self._tunnel_text = ""
            self._tunnel_rx_count = 0
            try:
                mtu = getattr(client, "mtu_size", 0) or 0
                if mtu > 23:
                    self.max_packet = 20  # 보수적인 ATT 기본 크기; 긴 메시지는 분할
            except Exception:
                pass
            self.sig_log.emit(f"② 연결됨 (패킷 {self.max_packet}B)", "info")

            if self.link_only:
                self.connected = True
                self.safe_mode = True
                self.label = address
                self.connected_at = time.monotonic()
                self.last_rx = 0.0
                self.last_telemetry = 0.0
                self.command_ready = False
                self.sig_conn.emit(True, address)
                self.sig_log.emit(
                    "◆ 링크 시험 모드 — 알림 등록도, 전송도 하지 않습니다. "
                    "15초 동안 연결이 유지되는지만 봅니다.", "warn")
                for _ in range(15):
                    await asyncio.sleep(1.0)
                    if not self.connected:
                        return
                self.sig_log.emit(
                    "✅ 15초 동안 링크 유지됨 — 순수 BLE 연결은 안정적입니다.", "tx")
                return

            # 연결 직후 곧바로 알림을 켜면 허브의 무선 송신이 급격히 늘어난다.
            # 링크가 안정될 시간을 준 뒤 단계적으로 진행한다.
            await asyncio.sleep(1.2)
            if not client.is_connected:
                raise RuntimeError("연결 직후 링크가 끊겼습니다 "
                                   "(허브 전원이 내려갔을 가능성)")

            try:
                await client.start_notify(SPIKE_TX_CHAR, self._on_notify)
            except Exception as e:
                if "not found" in str(e).lower():
                    raise RuntimeError(
                        "SPIKE 특성을 찾지 못했습니다. 허브를 껐다 켜고, "
                        "Windows 설정 > Bluetooth 에서 허브를 '디바이스 제거' 한 뒤 "
                        "다시 연결하세요. (OS의 서비스 캐시 문제)") from None
                raise
            self.sig_log.emit("③ 알림 등록 완료", "info")
            await asyncio.sleep(0.8)

            self._lock = asyncio.Lock()
            self.connected = True
            self.safe_mode = safe
            self.label = address
            self.connected_at = time.monotonic()
            self.last_rx = 0.0
            self.last_telemetry = 0.0
            self.command_ready = False
            self.sig_conn.emit(True, address)

            if safe:
                self.sig_log.emit(
                    "안전 모드: 허브로 아무것도 보내지 않습니다. "
                    "알림 등록·펌웨어·전원·PC Bluetooth 원인은 아직 구분되지 않았습니다.", "warn")
            else:
                await self._handshake()
        except Exception as e:
            self.sig_log.emit(f"연결 실패: {e}", "err")
            try:
                if self._client is not None:
                    await self._client.disconnect()
            except Exception:
                pass
            self._client = None
            self.connected = False
            self.sig_conn.emit(False, "")

    def _on_disconnected(self, _client):
        if _client is not self._client:
            return
        self._session += 1
        self.command_ready = False
        self.last_telemetry = 0.0
        held = time.monotonic() - self.connected_at if self.connected_at else 0
        got = self.last_rx > 0
        self.connected = False
        self._client = None
        self.sig_conn.emit(False, "")
        self.sig_log.emit(
            f"BLE 연결이 끊어졌습니다 (유지 {held:.1f}초, 허브 수신 "
            f"{'있었음' if got else '없음'})", "warn")
        if held < 20:
            self.sig_log.emit(
                "→ 연결 직후 끊김. BLE 단절만으로 전원 종료를 판정할 수 없습니다.\n"
                "   판별 1: 허브를 USB 전원에 꽂고 같은 동작 → 안 꺼지면 배터리\n"
                "   판별 2: LEGO SPIKE App 으로 블루투스 연결 → 거기서도 꺼지면\n"
                "           GCS와 무관한 허브/배터리 문제입니다", "err")

    def disconnect(self):
        self._submit(self._disconnect())

    async def _disconnect(self):
        self._session += 1
        self.command_ready = False
        self.last_telemetry = 0.0
        client = self._client
        try:
            if client is not None and self.connected and not self.safe_mode:
                await asyncio.wait_for(self._write_msg(make_tunnel("E")), timeout=0.5)
        except Exception as exc:
            self.sig_log.emit(f"연결 해제 전 정지 전송 실패: {exc}", "warn")
        finally:
            try:
                if client is not None:
                    await client.disconnect()
            finally:
                if self._client is client:
                    self._client = None
                    self.connected = False
                    self.sig_conn.emit(False, "")

    async def _handshake(self):
        self._info_event.clear()
        await self._write_msg(bytes([MSG_INFO_REQ]))
        self.sig_log.emit("④ InfoRequest 전송 (주행 명령 없음)", "tx")
        try:
            await asyncio.wait_for(self._info_event.wait(), 4.0)
            self.sig_log.emit("⑤ InfoResponse 확인 — 허브 프로그램 텔레메트리 대기", "info")
        except asyncio.TimeoutError:
            self.sig_log.emit("InfoResponse 시간 초과: 원시 HEX와 허브 표시를 확인하세요", "warn")

    def set_safe_mode(self, on):
        if self.link_only and not on:
            self.sig_log.emit("링크 시험은 알림이 없습니다. 연결 해제 후 일반 연결하세요.", "warn")
            return
        super().set_safe_mode(on)
        if not on and self.connected:
            self._submit(self._handshake())

    # ------------------------------ 수신 --------------------------------
    def _on_notify(self, _sender, data: bytearray):
        self.last_rx = time.monotonic()
        self.sig_raw.emit(bytes(data))
        for b in data:
            if b == 1:
                if self._high_priority:
                    self._frame.clear()
                self._priority_frame.clear()
                self._high_priority = True
                continue
            queue = self._priority_frame if self._high_priority else self._frame
            if b == DELIMITER:
                if queue:
                    try:
                        self._on_message(cobs_unpack(bytes(queue)))
                    except Exception as exc:
                        self.sig_log.emit(f"프레임 해석 실패: {exc}", "warn")
                queue.clear()
                self._high_priority = False
            else:
                queue.append(b)
                if len(queue) > 8192:
                    self._frame.clear()
                    self._priority_frame.clear()
                    self._high_priority = False
                    self.sig_log.emit("수신 프레임 길이 초과: 재동기화", "warn")

    def _on_message(self, msg: bytes):
        if not msg:
            return
        mid = msg[0]
        if mid == MSG_INFO_RES and len(msg) >= 17:
            rpc = f"{msg[1]}.{msg[2]}.{int.from_bytes(msg[3:5],'little')}"
            fw = f"{msg[5]}.{msg[6]}.{int.from_bytes(msg[7:9],'little')}"
            reported = int.from_bytes(msg[9:11], "little")
            self.max_packet = max(1, min(reported or 20, 20))
            max_msg = int.from_bytes(msg[11:13], "little")
            self.fw_info = (f"RPC {rpc} · FW {fw} · 허브 최대패킷 {reported} · 실제 전송단위 {self.max_packet}"
                            f" · maxMsg {max_msg}")
            self.sig_log.emit("허브 정보: " + self.fw_info, "rx")
            self.sig_log.emit("  InfoResponse 원문: " + msg[:17].hex(" "), "info")
            # BLE 쓰기는 협상된 MTU 를 넘을 수 없다. 허브가 보고한 값이
            # 그보다 크면 작은 쪽을 쓴다.
            try:
                neg = getattr(self._client, "mtu_size", 0) or 0
            except Exception:
                neg = 0
            if neg > 23:
                safe = min(self.max_packet, neg - 3)
                if safe < self.max_packet:
                    self.sig_log.emit(
                        f"  전송 단위를 {self.max_packet} → {safe} B 로 낮춥니다"
                        f" (협상 MTU {neg})", "info")
                    self.max_packet = safe
            if self._info_event is not None:
                self._info_event.set()
        elif mid == 0x1F and len(msg) >= 2:
            self.sig_log.emit("프로그램 요청 " + ("ACK" if msg[1] == 0 else "거부됨"), "rx")
        elif mid == MSG_TUNNEL:
            if len(msg) < 3:
                self.sig_log.emit("RX TunnelMessage: 헤더가 너무 짧습니다", "warn")
                return
            size = int.from_bytes(msg[1:3], "little")
            if len(msg) != size + 3:
                self.sig_log.emit(f"RX TunnelMessage 길이 오류: 선언 {size}, 실제 {len(msg)-3}", "warn")
                return
            self._tunnel_rx_count += 1
            if self._tunnel_rx_count == 1:
                self.sig_log.emit("허브 → GCS 터널 수신 확인 (0x32)", "rx")
            self._feed_text(msg[3:].decode("utf-8", "replace"), channel="tunnel")
        elif mid == MSG_CONSOLE_NOTI:
            body = msg[1:]
            zero = body.find(0)
            if zero >= 0:
                body = body[:zero]
            self._feed_text(body.decode("utf-8", "replace"))
        elif mid == MSG_PROGRAM_FLOW_NOTI and len(msg) > 1:
            self.command_ready = False
            self.last_telemetry = 0.0
            self.sig_log.emit(
                "프로그램 " + ("중지됨" if msg[1] else "시작됨"), "rx")

    # ------------------------------ 송신 --------------------------------
    def _send_impl(self, line: str) -> bool:
        if line == "E":
            self._session += 1  # 아직 실행되지 않은 이전 조종 명령 폐기
        return self._submit(self._send_line(line, self._session, time.monotonic())) is not None

    async def _send_line(self, line, session=None, queued_at=None):
        try:
            if session is not None and session != self._session:
                return
            if queued_at is not None and time.monotonic() - queued_at > 0.3:
                return
            if self.safe_mode:
                return
            written = await self._write_msg(make_tunnel(line), session, queued_at)
            if line == "Q" and written:
                self.sig_log.emit("Q BLE 쓰기 완료 — 허브 ACK 대기", "tx")
        except Exception as e:
            self.sig_log.emit(f"전송 실패: {e}", "err")

    async def _write_msg(self, msg: bytes, session=None, queued_at=None):
        if self._client is None or not self._client.is_connected:
            raise ConnectionError("BLE 연결 없음")
        if self._lock is None:
            self._lock = asyncio.Lock()
        framed = cobs_pack(msg)
        client = self._client
        async with self._lock:
            if session is not None and session != self._session:
                return
            if queued_at is not None and time.monotonic() - queued_at > 0.3:
                return
            if queued_at is not None and self.safe_mode:
                return
            if client is not self._client or not client.is_connected:
                raise ConnectionError("BLE 세션 변경")
            for i in range(0, len(framed), self.max_packet):
                await client.write_gatt_char(
                    SPIKE_RX_CHAR, framed[i:i + self.max_packet], response=False)
            return True

    def tunnel_raw(self, payload: bytes):
        """터널 payload 를 가공 없이 그대로 보낸다 (업링크 진단용)."""
        if not self.connected or self.safe_mode:
            return
        self._submit(self._tunnel_raw(payload))

    async def _tunnel_raw(self, payload: bytes):
        try:
            msg = (bytes([MSG_TUNNEL])
                   + len(payload).to_bytes(2, "little") + payload)
            await self._write_msg(msg)
        except Exception as e:
            self.sig_log.emit(f"터널 전송 실패: {e}", "err")

    def interrupt(self):
        if not self.connected or self.safe_mode:
            return
        # 슬롯 프로그램 중지 + 터널로 Ctrl-C / 빈 줄
        self._submit(self._interrupt())

    async def _interrupt(self):
        try:
            await self._write_msg(
                bytes([MSG_PROGRAM_FLOW_REQ, 1, 0]))
            await asyncio.sleep(0.3)
            for payload in (b"\x03", b"\x03", b"\r\n"):
                msg = (bytes([MSG_TUNNEL])
                       + len(payload).to_bytes(2, "little") + payload)
                await self._write_msg(msg)
                await asyncio.sleep(0.15)
        except Exception as e:
            self.sig_log.emit(f"인터럽트 실패: {e}", "warn")

    def program_flow(self, stop: bool, slot: int):
        if not self.connected:
            self.sig_log.emit("허브가 연결되지 않았습니다", "warn")
            return
        if self.safe_mode:
            self._warn_drop("안전 모드입니다. [안전 모드 해제]를 누르세요.")
            return
        self._submit(self._program_flow(stop, slot))

    async def _program_flow(self, stop, slot):
        try:
            await self._write_msg(
                bytes([MSG_PROGRAM_FLOW_REQ, 1 if stop else 0, slot]))
            self.sig_log.emit(
                f"슬롯 {slot} {'중지' if stop else '시작'} 요청", "tx")
        except Exception as e:
            self.sig_log.emit(f"프로그램 제어 실패: {e}", "err")


class SerialLink(LinkBase):
    """USB 케이블 경로. 허브의 파이썬 REPL/프로그램 stdin 에 직접 붙는다."""

    def __init__(self):
        super().__init__()
        self.kind = "serial"
        self._ser = None
        self._stop = threading.Event()
        self._thread = None

    @staticmethod
    def ports():
        if not HAVE_SERIAL:
            return []
        return [(p.device, p.description) for p in list_ports.comports()]

    def connect_to(self, device, safe=False):
        if self.connected or self._ser is not None:
            self.sig_log.emit("이미 연결되어 있습니다. 먼저 [연결 해제]를 누르세요.",
                              "warn")
            return
        try:
            self._ser = pyserial.Serial(device, 115200, timeout=0.1)
        except Exception as e:
            self._ser = None
            msg = str(e)
            self.sig_log.emit(f"시리얼 열기 실패: {msg}", "err")
            if "PermissionError" in msg or "액세스" in msg or "denied" in msg.lower():
                self.sig_log.emit(
                    "→ 다른 프로그램이 이 포트를 쓰고 있습니다. "
                    "SPIKE App(트레이 포함), 아두이노 IDE 시리얼 모니터, "
                    "이 GCS의 다른 창을 모두 닫고 다시 시도하세요.", "warn")
            return
        self.connected = True
        self.safe_mode = safe
        self.label = device
        self.connected_at = time.monotonic()
        self.last_rx = 0.0
        self.last_telemetry = 0.0
        self.command_ready = False
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.sig_conn.emit(True, device)
        self.sig_log.emit(f"USB Serial 연결됨: {device}", "info")

    def _read_loop(self):
        while not self._stop.is_set() and self._ser is not None:
            try:
                n = self._ser.in_waiting
                data = self._ser.read(n if n else 1)
            except Exception:
                break
            if data:
                self.sig_raw.emit(bytes(data))
                self._feed_text(data.decode("utf-8", "replace"))
        self.connected = False
        self.sig_conn.emit(False, "")
        self.sig_log.emit("Serial 연결이 끊어졌습니다", "warn")

    def disconnect(self):
        if self.connected and not self.safe_mode:
            self._send_impl("E")
        self.command_ready = False
        self.last_telemetry = 0.0
        self._stop.set()
        th = self._thread
        if th is not None and th.is_alive():
            th.join(timeout=1.0)
        self._thread = None
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.connected = False

    def _send_impl(self, line: str) -> bool:
        if self._ser is None:
            return False
        try:
            self._ser.write((line + "\r\n").encode())
            return True
        except Exception as e:
            self.sig_log.emit(f"전송 실패: {e}", "err")
            return False

    def write_bytes(self, data: bytes):
        if self._ser is None:
            self.sig_log.emit("USB Serial 연결이 아닙니다", "warn")
            return
        try:
            self._ser.write(data)
        except Exception as e:
            self.sig_log.emit(f"전송 실패: {e}", "err")

    def tunnel_raw(self, payload: bytes):
        self.write_bytes(payload)

    def interrupt(self):
        """USB 는 허브의 파이썬 REPL 에 직결된다. Ctrl-C 로 프롬프트를 연다."""
        if self._ser is None:
            self.sig_log.emit("USB Serial 연결이 아닙니다", "warn")
            return
        self.write_bytes(b"\x03")
        time.sleep(0.15)
        self.write_bytes(b"\x03")
        time.sleep(0.25)
        self.write_bytes(b"\r\n")
        self.sig_log.emit("TX Ctrl-C ×2 + Enter (REPL 프롬프트 열기)", "tx")

    def program_flow(self, stop, slot):
        self.sig_log.emit(
            "USB에서는 [REPL 조종]을 쓰거나 허브 버튼으로 실행하세요.", "warn")


# =============================================================================
#  3. 계기 위젯
# =============================================================================
C_BG = QColor("#0b0f14")
C_PANEL = QColor("#121924")
C_LINE = QColor("#1e2a3a")
C_FG = QColor("#dce6f2")
C_DIM = QColor("#7f93ab")
C_ACC = QColor("#3fb9ff")
C_OK = QColor("#3ddc84")
C_MAG = QColor("#c77dff")
C_WARN = QColor("#ffb020")
C_BAD = QColor("#ff5a5a")


class AttitudeIndicator(QWidget):
    """인공수평의 — pitch/roll 표시"""

    def __init__(self):
        super().__init__()
        self.pitch = 0.0
        self.roll = 0.0
        self.setMinimumSize(110, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_attitude(self, pitch, roll):
        self.pitch, self.roll = pitch, roll
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 8

        path = QPainterPath()
        path.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(path)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)
        off = max(-r, min(r, self.pitch * (r / 40.0)))
        p.translate(0, off)

        big = r * 3
        p.fillRect(QRectF(-big, -big, big * 2, big), QColor("#2b6ea8"))
        p.fillRect(QRectF(-big, 0, big * 2, big), QColor("#7a5230"))
        p.setPen(QPen(QColor("#eaf3ff"), 2))
        p.drawLine(QPointF(-big, 0), QPointF(big, 0))

        p.setPen(QPen(QColor(255, 255, 255, 190), 1))
        f = QFont("monospace", 7)
        p.setFont(f)
        for deg in (-30, -20, -10, 10, 20, 30):
            y = -deg * (r / 40.0)
            half = r * (0.34 if abs(deg) % 20 == 0 else 0.18)
            p.drawLine(QPointF(-half, y), QPointF(half, y))
            if abs(deg) % 20 == 0:
                p.drawText(QRectF(half + 3, y - 7, 24, 14),
                           Qt.AlignVCenter | Qt.AlignLeft, str(abs(deg)))
        p.restore()
        p.setClipping(False)

        p.setPen(QPen(C_LINE, 5))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # 고정 기체 심볼
        p.setPen(QPen(QColor("#ffd24a"), 2.5))
        p.drawLine(QPointF(cx - r * 0.45, cy), QPointF(cx - r * 0.12, cy))
        p.drawLine(QPointF(cx + r * 0.12, cy), QPointF(cx + r * 0.45, cy))
        p.drawLine(QPointF(cx - r * 0.12, cy), QPointF(cx, cy + r * 0.12))
        p.drawLine(QPointF(cx, cy + r * 0.12), QPointF(cx + r * 0.12, cy))
        p.setBrush(QBrush(QColor("#ffd24a")))
        p.drawEllipse(QPointF(cx, cy), 2.5, 2.5)

        # 상단 롤 지시 삼각형
        tri = QPolygonF([QPointF(cx, cy - r + 2), QPointF(cx - 5, cy - r + 11),
                         QPointF(cx + 5, cy - r + 11)])
        p.drawPolygon(tri)
        p.end()


class CompassWidget(QWidget):
    """방위 지시계 — yaw 와 목표 방위(bug)"""

    def __init__(self):
        super().__init__()
        self.yaw = 0.0
        self.target = 0.0
        self.setMinimumSize(110, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_heading(self, yaw, target):
        self.yaw, self.target = yaw, target
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 8

        p.setBrush(QBrush(QColor("#0a1018")))
        p.setPen(QPen(C_LINE, 2))
        p.drawEllipse(QPointF(cx, cy), r, r)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self.yaw)
        p.setFont(QFont("monospace", 8, QFont.Bold))
        for a in range(0, 360, 15):
            major = (a % 45 == 0)
            rad = math.radians(a - 90)
            r1 = r * (0.74 if major else 0.85)
            p.setPen(QPen(C_ACC if major else QColor("#3d5166"),
                          1.6 if major else 1.0))
            p.drawLine(QPointF(r1 * math.cos(rad), r1 * math.sin(rad)),
                       QPointF(r * math.cos(rad), r * math.sin(rad)))
        for a, txt, col in ((0, "N", C_FG), (90, "E", C_DIM),
                            (180, "S", C_DIM), (270, "W", C_DIM)):
            rad = math.radians(a - 90)
            p.setPen(QPen(col))
            rr = r * 0.6
            p.drawText(QRectF(rr * math.cos(rad) - 10, rr * math.sin(rad) - 8,
                              20, 16), Qt.AlignCenter, txt)
        p.restore()

        # 목표 방위 bug
        p.save()
        p.translate(cx, cy)
        p.rotate(wrap180(self.target - self.yaw))
        p.setBrush(QBrush(C_OK))
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0, -r + 1), QPointF(-5, -r + 10),
                                 QPointF(5, -r + 10)]))
        p.restore()

        # 고정 기체
        p.setBrush(QBrush(C_ACC))
        p.setPen(QPen(QColor("#0a1018"), 1))
        p.drawPolygon(QPolygonF([QPointF(cx, cy - r * 0.55),
                                 QPointF(cx - r * 0.22, cy + r * 0.35),
                                 QPointF(cx, cy + r * 0.16),
                                 QPointF(cx + r * 0.22, cy + r * 0.35)]))
        p.end()


class StripChart(QWidget):
    """좌/우 속도와 방위 오차를 흐르게 그린다"""

    def __init__(self, capacity=240):
        super().__init__()
        self.cap = capacity
        self.series = [deque(maxlen=capacity) for _ in range(3)]
        self.scales = [1000.0, 1000.0, 180.0]
        self.colors = [C_ACC, C_MAG, C_OK]
        self.setMinimumHeight(90)

    def push(self, a, b, c):
        self.series[0].append(a)
        self.series[1].append(b)
        self.series[2].append(c)
        self.update()

    def clear(self):
        for s in self.series:
            s.clear()
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#070b11"))
        p.setPen(QPen(QColor("#16202c"), 1))
        for i in range(1, 4):
            y = h * i / 4
            p.drawLine(QPointF(0, y), QPointF(w, y))
        for s, scale, col in zip(self.series, self.scales, self.colors):
            if len(s) < 2:
                continue
            p.setPen(QPen(col, 1.5))
            poly = QPolygonF()
            n = len(s)
            for i, v in enumerate(s):
                x = i / (n - 1) * w
                vv = max(-1.0, min(1.0, v / scale))
                poly.append(QPointF(x, h / 2 - vv * (h / 2 - 3)))
            p.drawPolyline(poly)
        p.end()


class BarGauge(QWidget):
    def __init__(self):
        super().__init__()
        self.value = 0.0
        self.maximum = 1000.0
        self.setFixedHeight(10)

    def set_value(self, v):
        self.value = v
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#0b111a"))
        p.setPen(QPen(C_LINE, 1))
        p.drawRect(0, 0, w - 1, h - 1)
        frac = max(-1.0, min(1.0, self.value / self.maximum))
        col = C_ACC if frac >= 0 else C_MAG
        mid = w / 2
        p.fillRect(QRectF(mid, 1, frac * (mid - 1), h - 2) if frac >= 0
                   else QRectF(mid + frac * (mid - 1), 1, -frac * (mid - 1), h - 2),
                   col)
        p.setPen(QPen(C_DIM, 1))
        p.drawLine(QPointF(mid, 0), QPointF(mid, h))
        p.end()


def wrap180(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


# =============================================================================
#  4. 메인 윈도우
# =============================================================================
STYLE = """
QWidget { background:#0b0f14; color:#dce6f2;
          font-family:'Pretendard','Malgun Gothic','Noto Sans KR',sans-serif;
          font-size:12px; }
QGroupBox { border:1px solid #1e2a3a; border-radius:6px; margin-top:16px;
            padding:10px 8px 8px 8px; background:#121924; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px;
                   color:#7f93ab; font-weight:700; font-size:11px; }
QPushButton { background:#1b2736; border:1px solid #1e2a3a; border-radius:5px;
              padding:6px 10px; font-weight:600; }
QPushButton:hover { background:#243347; }
QPushButton:disabled { color:#4a5c6b; }
QPushButton#primary { background:#3fb9ff; color:#04121c; border-color:#3fb9ff; }
QPushButton#danger  { background:#ff5a5a; color:#ffffff; border-color:#ff5a5a; }
QPushButton#ok      { background:#3ddc84; color:#04140b; border-color:#3ddc84; }
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QListWidget, QTextEdit {
    background:#0b111a; border:1px solid #1e2a3a; border-radius:5px;
    padding:4px 6px; selection-background-color:#2a4a66; }
QTextEdit { font-family:'D2Coding','Consolas',monospace; font-size:11px; }
QListWidget { font-family:'D2Coding','Consolas',monospace; font-size:11px; }
QLabel#val { font-family:'D2Coding','Consolas',monospace; font-size:16px;
             font-weight:700; }
QLabel#cap { color:#7f93ab; font-size:10px; font-weight:600; }
QLabel#chip { border-radius:9px; padding:2px 9px; background:#0c121a;
              color:#7f93ab; font-family:'D2Coding',monospace;
              font-size:11px; font-weight:700; }
QSlider::groove:horizontal { height:4px; background:#1e2a3a; border-radius:2px; }
QSlider::handle:horizontal { width:14px; margin:-6px 0; border-radius:7px;
                             background:#3fb9ff; }
QCheckBox { spacing:6px; }
"""


class RoverGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SPIKE Rover GCS — tunnel-v2 + ESP32-CAM")
        self.resize(1280, 860)
        self.setMinimumSize(900, 560)      # 노트북 화면에서도 열리도록
        self.setStyleSheet(STYLE)

        # ---- 상태 ----
        self.ble = BleLink()
        self.ser = SerialLink()
        self.link = self.ble
        self.tm = dict(yaw=0.0, pitch=0.0, roll=0.0, lvel=0, rvel=0,
                       lpos=0, rpos=0, dist=-1, refl=-1, batt=-1, hold=0, ms=0)
        self.drive = [0, 0]
        self._last_sent = (0, 0)
        self._last_send_t = 0.0
        self.target_yaw = 0.0
        self.hold_on = False
        self.aux_command = None
        self.keys = set()
        self.pad = [0, 0]
        self.test_until = 0.0
        self.tm_count = 0
        self.tm_rate = 0.0
        self._rate_t = time.monotonic()
        self.recording = False
        self.rec_rows = []
        self.raw_view = False
        self.rx_report = ""
        self.repl_mode = False
        self.repl_ping_seen = False
        self._repl_last = (None, None)
        self.uplink_hits = []
        self.repl_ready_seen = False
        self.tm_rx = -1          # 허브가 stdin 으로 받은 누적 바이트 수
        self.uplink_prog_mode = False
        self.uplink_rx_before = 0

        self._build_ui()
        self._wire(self.ble)
        self._wire(self.ser)

        self.t_drive = QTimer(self); self.t_drive.timeout.connect(self.tick_drive)
        self.t_drive.start(50)
        self.t_ui = QTimer(self); self.t_ui.timeout.connect(self.tick_ui)
        self.t_ui.start(100)
        self.t_rate = QTimer(self); self.t_rate.timeout.connect(self.tick_rate)
        self.t_rate.start(1000)
        self.t_repl = QTimer(self); self.t_repl.timeout.connect(self.tick_repl_tm)
        self.t_repl.start(200)
        QApplication.instance().applicationStateChanged.connect(self.on_app_state)
        QApplication.instance().installEventFilter(self)

        self.setFocusPolicy(Qt.StrongFocus)
        self.log("GCS 준비 완료. ① [스캔] → ② 허브 선택 → [연결] → "
                 "③ [프로그램 시작] → ④ WASD 로 조종", "info")
        if not HAVE_BLEAK:
            self.log("bleak 미설치: pip install bleak", "err")

    # ------------------------------------------------------------------ UI
    def _chip(self, text):
        lb = QLabel(); lb.setObjectName("chip")
        lb.setAlignment(Qt.AlignCenter)
        self.set_chip(lb, text)
        return lb

    @staticmethod
    def set_chip(lb: QLabel, text: str, css: str = ""):
        """스타일시트 padding 때문에 글자가 잘리므로 폭을 직접 잡아준다."""
        lb.setText(text)
        lb.setMinimumWidth(lb.fontMetrics().horizontalAdvance(text) + 26)
        lb.setStyleSheet(css)

    def _kv(self, caption):
        box = QVBoxLayout(); box.setSpacing(0)
        cap = QLabel(caption); cap.setObjectName("cap")
        val = QLabel("--"); val.setObjectName("val")
        box.addWidget(cap); box.addWidget(val)
        frame = QFrame(); frame.setLayout(box)
        frame.setStyleSheet("QFrame{background:#0e141d;border:1px solid #1e2a3a;"
                            "border-radius:5px;padding:4px 7px;}")
        return frame, val

    def _build_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 8, 10, 10); outer.setSpacing(8)

        # 창이 화면보다 커지는 것을 막기 위해 본문을 스크롤 영역에 넣는다.
        # (이게 없으면 Qt 가 setGeometry 경고를 쏟아내고 창이 잘린다)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        page = QWidget()
        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(8)
        scroll.setWidget(page)

        # ---------------- 상단 상태바 ----------------
        top = QHBoxLayout()
        title = QLabel("🛰  SPIKE ROVER · GCS")
        title.setStyleSheet("font-size:15px;font-weight:700;")
        self.chip_link = self._chip("미연결")
        self.chip_rate = self._chip("0 Hz")
        self.chip_mode = self._chip("MANUAL")
        self.chip_prog = self._chip("프로그램 ?")
        self.chip_tx = self._chip("TX 0")
        self.chip_hubrx = self._chip("허브RX ?")
        btn_stop = QPushButton("■ 비상정지 (Space)"); btn_stop.setObjectName("danger")
        btn_stop.setFocusPolicy(Qt.NoFocus)
        btn_stop.clicked.connect(self.estop)
        top.addWidget(title)
        top.addSpacing(12)
        for c in (self.chip_link, self.chip_rate, self.chip_mode,
                  self.chip_prog, self.chip_tx, self.chip_hubrx):
            top.addWidget(c)
        top.addStretch(1)
        top.addWidget(btn_stop)
        outer.addLayout(top)          # 상태바는 항상 보이게 스크롤 밖에
        outer.addWidget(scroll, 1)

        body = QHBoxLayout(); body.setSpacing(10)
        root.addLayout(body, 1)

        # ---------------- 좌: 연결 ----------------
        left = QVBoxLayout(); left.setSpacing(8)
        body.addLayout(left, 0)

        g_conn = QGroupBox("1. 허브 연결")
        gl = QGridLayout(g_conn); gl.setSpacing(5)
        self.cb_transport = QComboBox()
        self.cb_transport.addItems(["Bluetooth (BLE)", "USB Serial"])
        self.cb_transport.currentIndexChanged.connect(self.on_transport)
        gl.addWidget(self.cb_transport, 0, 0, 1, 2)

        self.btn_scan = QPushButton("스캔"); self.btn_scan.setObjectName("primary")
        self.btn_scan_all = QPushButton("모든 기기")
        for _b in (self.btn_scan, self.btn_scan_all):
            _b.setFocusPolicy(Qt.NoFocus)
        self.btn_scan.clicked.connect(lambda: self.do_scan(False))
        self.btn_scan_all.clicked.connect(lambda: self.do_scan(True))
        gl.addWidget(self.btn_scan, 1, 0); gl.addWidget(self.btn_scan_all, 1, 1)

        self.lst_dev = QListWidget()
        self.lst_dev.setMinimumHeight(70); self.lst_dev.setMaximumHeight(120)
        self.lst_dev.itemDoubleClicked.connect(lambda _i: self.do_connect(False))
        gl.addWidget(self.lst_dev, 2, 0, 1, 2)

        self.btn_conn = QPushButton("연결"); self.btn_conn.setObjectName("ok")
        self.btn_disc = QPushButton("연결 해제")
        self.btn_conn.clicked.connect(lambda: self.do_connect(False))
        self.btn_disc.clicked.connect(self.do_disconnect)
        gl.addWidget(self.btn_conn, 3, 0); gl.addWidget(self.btn_disc, 3, 1)

        self.btn_safe = QPushButton("안전 모드 연결 (전송 없음)")
        self.btn_safe.setFocusPolicy(Qt.NoFocus)
        self.btn_safe.clicked.connect(lambda: self.do_connect(True))
        gl.addWidget(self.btn_safe, 4, 0, 1, 2)

        self.btn_linkonly = QPushButton("◆ 링크만 시험 (알림·전송 전혀 없음)")
        self.btn_linkonly.setFocusPolicy(Qt.NoFocus)
        self.btn_linkonly.clicked.connect(self.do_link_test)
        gl.addWidget(self.btn_linkonly, 9, 0, 1, 2)

        self.ck_rediscover = QCheckBox("서비스 캐시 무시(재검색)")
        self.ck_rediscover.setFocusPolicy(Qt.NoFocus)
        self.ck_rediscover.toggled.connect(
            lambda on: setattr(self.ble, "force_rediscover", on))
        gl.addWidget(self.ck_rediscover, 10, 0, 1, 2)

        self.btn_safe_off = QPushButton("안전 모드 해제 — 전송 켜기")
        self.btn_safe_off.setFocusPolicy(Qt.NoFocus)
        self.btn_safe_off.clicked.connect(self.toggle_safe)
        self.btn_safe_off.setEnabled(False)   # 자리는 유지 (레이아웃 요동 방지)
        gl.addWidget(self.btn_safe_off, 5, 0, 1, 2)

        gl.addWidget(QLabel("슬롯"), 6, 0)
        self.sp_slot = QSpinBox(); self.sp_slot.setRange(0, 19)
        gl.addWidget(self.sp_slot, 6, 1)
        self.btn_run = QPushButton("▶ 프로그램 시작")
        self.btn_halt = QPushButton("■ 프로그램 중지")
        self.btn_run.clicked.connect(self.start_program)
        self.btn_halt.clicked.connect(lambda: self.link.program_flow(True, self.sp_slot.value()))
        gl.addWidget(self.btn_run, 7, 0); gl.addWidget(self.btn_halt, 7, 1)

        self.btn_query = QPushButton("상태 조회 (Q)")
        self.btn_query.clicked.connect(
            lambda: self.link.send("Q") and self.log("TX Q", "tx"))
        gl.addWidget(self.btn_query, 8, 0, 1, 2)
        left.addWidget(g_conn)

        # ---- REPL 직접 제어 : 허브에 프로그램을 올리지 않고 조종한다 ----
        g_repl = QGroupBox("1-B. REPL 직접 제어 (프로그램 없이)")
        rl2 = QGridLayout(g_repl); rl2.setSpacing(5)
        self.btn_uplink = QPushButton("🔬 업링크 자동 진단 (약 8초)")
        self.btn_uplink.setObjectName("primary")
        self.btn_uplink.setFocusPolicy(Qt.NoFocus)
        self.btn_uplink.clicked.connect(self.uplink_diag)
        rl2.addWidget(self.btn_uplink, 4, 0, 1, 2)

        self.btn_intr = QPushButton("⎋ 인터럽트 (Ctrl-C)")
        self.btn_intr.setFocusPolicy(Qt.NoFocus)
        self.btn_intr.clicked.connect(lambda: self.link.interrupt())
        rl2.addWidget(self.btn_intr, 3, 0, 1, 2)

        self.btn_repl_ping = QPushButton("① REPL 핑 테스트")
        self.btn_repl_ping.setObjectName("primary")
        self.btn_repl_ping.setFocusPolicy(Qt.NoFocus)
        self.btn_repl_ping.clicked.connect(self.repl_ping)
        rl2.addWidget(self.btn_repl_ping, 0, 0, 1, 2)

        self.btn_repl_on = QPushButton("② REPL 조종 시작")
        self.btn_repl_on.setFocusPolicy(Qt.NoFocus)
        self.btn_repl_on.clicked.connect(self.repl_start)
        self.btn_repl_off = QPushButton("REPL 조종 끄기")
        self.btn_repl_off.setFocusPolicy(Qt.NoFocus)
        self.btn_repl_off.clicked.connect(self.repl_stop)
        rl2.addWidget(self.btn_repl_on, 1, 0); rl2.addWidget(self.btn_repl_off, 1, 1)

        _h = QLabel("허브에서 프로그램을 <b>중지</b>한 뒤 ①을 누르세요.\n"
                    "PONG 이 돌아오면 ②로 바로 조종됩니다.")
        _h.setStyleSheet("color:#7f93ab;font-size:10px;"); _h.setWordWrap(True)
        rl2.addWidget(_h, 2, 0, 1, 2)
        g_repl.setTitle("1-B. 명령 수신 진단")
        for btn in (self.btn_intr, self.btn_repl_ping, self.btn_repl_on, self.btn_repl_off):
            btn.hide()
        _h.setText("허브 프로그램 실행 후 Q 응답을 확인합니다. REPL 직접 조종은 사용하지 않습니다.")
        self.btn_uplink.setText("업링크 진단 (Q 응답 확인)")
        left.addWidget(g_repl)

        g_drv = QGroupBox("2. 구동부 설정")
        dl = QGridLayout(g_drv); dl.setSpacing(5)
        self.cb_left = QComboBox(); self.cb_right = QComboBox()
        for cb in (self.cb_left, self.cb_right):
            cb.addItems(list("ABCDEF"))
        self.cb_left.setCurrentIndex(0); self.cb_right.setCurrentIndex(1)
        dl.addWidget(QLabel("좌 모터"), 0, 0); dl.addWidget(self.cb_left, 0, 1)
        dl.addWidget(QLabel("우 모터"), 1, 0); dl.addWidget(self.cb_right, 1, 1)
        self.ck_invl = QCheckBox("좌 반전"); self.ck_invr = QCheckBox("우 반전")
        self.ck_invr.setChecked(True)
        dl.addWidget(self.ck_invl, 2, 0); dl.addWidget(self.ck_invr, 2, 1)
        btn_apply = QPushButton("적용"); btn_apply.clicked.connect(self.apply_ports)
        dl.addWidget(btn_apply, 3, 0, 1, 2)
        dl.addWidget(QLabel("보조모터"), 4, 0)
        self.cb_aux = QComboBox(); self.cb_aux.addItems(list("ABCDEF"))
        self.cb_aux.setCurrentIndex(2)
        dl.addWidget(self.cb_aux, 4, 1)
        aux_row = QHBoxLayout()
        for txt, v in (("▲", 600), ("▼", -600), ("■", 0)):
            b = QPushButton(txt); b.setFocusPolicy(Qt.NoFocus)
            b.clicked.connect(lambda _c=False, vv=v: self.send_aux(vv))
            aux_row.addWidget(b)
        dl.addLayout(aux_row, 5, 0, 1, 2)
        left.addWidget(g_drv)
        left.addStretch(1)

        # ---------------- 중: 계기 ----------------
        mid = QVBoxLayout(); mid.setSpacing(8)
        body.addLayout(mid, 1)

        self.camera = CameraPanel(self)
        self.camera.log.connect(self.log)
        self.camera.lost.connect(self.on_camera_lost)
        mid.addWidget(self.camera, 2)

        g_att = QGroupBox("자세 · 방위 (허브 IMU)")
        al = QGridLayout(g_att)
        self.adi = AttitudeIndicator(); self.hsi = CompassWidget()
        al.addWidget(self.adi, 0, 0); al.addWidget(self.hsi, 0, 1)
        c1 = QLabel("ATTITUDE"); c1.setObjectName("cap"); c1.setAlignment(Qt.AlignCenter)
        c2 = QLabel("HEADING"); c2.setObjectName("cap"); c2.setAlignment(Qt.AlignCenter)
        al.addWidget(c1, 1, 0); al.addWidget(c2, 1, 1)
        row = QHBoxLayout()
        self.f_yaw, self.v_yaw = self._kv("YAW")
        self.f_pit, self.v_pit = self._kv("PITCH")
        self.f_rol, self.v_rol = self._kv("ROLL")
        for f in (self.f_yaw, self.f_pit, self.f_rol):
            row.addWidget(f)
        al.addLayout(row, 2, 0, 1, 2)
        mid.addWidget(g_att)

        g_sen = QGroupBox("센서 · 구동부")
        sl = QGridLayout(g_sen)
        srow = QHBoxLayout()
        self.f_dist, self.v_dist = self._kv("거리 (mm)")
        self.f_refl, self.v_refl = self._kv("반사광 (%)")
        self.f_batt, self.v_batt = self._kv("배터리")
        self.f_ms, self.v_ms = self._kv("허브 시간 (s)")
        for f in (self.f_dist, self.f_refl, self.f_batt, self.f_ms):
            srow.addWidget(f)
        sl.addLayout(srow, 0, 0, 1, 2)

        self.bar_l = BarGauge(); self.bar_r = BarGauge()
        self.lb_l = QLabel("좌 0"); self.lb_r = QLabel("우 0")
        for lb in (self.lb_l, self.lb_r):
            lb.setStyleSheet("font-family:'D2Coding',monospace;")
            lb.setFixedWidth(84)
        sl.addWidget(self.lb_l, 1, 0); sl.addWidget(self.bar_l, 1, 1)
        sl.addWidget(self.lb_r, 2, 0); sl.addWidget(self.bar_r, 2, 1)

        prow = QHBoxLayout()
        self.f_lpos, self.v_lpos = self._kv("좌 회전각")
        self.f_rpos, self.v_rpos = self._kv("우 회전각")
        prow.addWidget(self.f_lpos); prow.addWidget(self.f_rpos)
        sl.addLayout(prow, 3, 0, 1, 2)

        self.chart = StripChart()
        sl.addWidget(self.chart, 4, 0, 1, 2)
        legend = QLabel("■ 좌속도   ■ 우속도   ■ yaw 오차")
        legend.setStyleSheet("color:#7f93ab;font-size:10px;")
        sl.addWidget(legend, 5, 0, 1, 2)
        mid.addWidget(g_sen, 1)

        # ---------------- 우: 조종 ----------------
        right = QVBoxLayout(); right.setSpacing(8)
        body.addLayout(right, 0)

        g_man = QGroupBox("3. 수동 조종")
        ml = QGridLayout(g_man); ml.setSpacing(4)
        pad_defs = [("◤", -1, 1, 0, 0), ("▲", 0, 1, 0, 1), ("◥", 1, 1, 0, 2),
                    ("◀", -1, 0, 1, 0), ("■", 0, 0, 1, 1), ("▶", 1, 0, 1, 2),
                    ("◣", -1, -1, 2, 0), ("▼", 0, -1, 2, 1), ("◢", 1, -1, 2, 2)]
        for txt, s, f, rr, cc in pad_defs:
            b = QPushButton(txt); b.setMinimumHeight(32)
            b.setFocusPolicy(Qt.NoFocus)
            if txt == "■":
                b.setObjectName("danger")
                b.clicked.connect(self.estop)
            else:
                b.pressed.connect(lambda ss=s, ff=f: self.set_pad(ff, ss))
                b.released.connect(lambda: self.set_pad(0, 0))
            ml.addWidget(b, rr, cc)

        self.sl_speed = QSlider(Qt.Horizontal); self.sl_speed.setRange(100, 800)
        self.sl_speed.setValue(250); self.sl_speed.setFocusPolicy(Qt.NoFocus)
        self.lb_speed = QLabel("속도 250")
        self.sl_speed.valueChanged.connect(
            lambda v: self.lb_speed.setText(f"속도 {v}"))
        ml.addWidget(self.lb_speed, 3, 0); ml.addWidget(self.sl_speed, 3, 1, 1, 2)

        self.sl_turn = QSlider(Qt.Horizontal); self.sl_turn.setRange(20, 150)
        self.sl_turn.setValue(60); self.sl_turn.setFocusPolicy(Qt.NoFocus)
        self.lb_turn = QLabel("선회 60%")
        self.sl_turn.valueChanged.connect(
            lambda v: self.lb_turn.setText(f"선회 {v}%"))
        ml.addWidget(self.lb_turn, 4, 0); ml.addWidget(self.sl_turn, 4, 1, 1, 2)

        btn_mt = QPushButton("🔧 모터 테스트 (0.5초)"); btn_mt.setObjectName("ok")
        btn_mt.setFocusPolicy(Qt.NoFocus)
        btn_mt.clicked.connect(self.motor_test)
        ml.addWidget(btn_mt, 5, 0, 1, 3)
        hint = QLabel("W A S D 주행 · Space 비상정지 · Q/E 보조모터 · G 자세유지")
        hint.setStyleSheet("color:#7f93ab;font-size:10px;"); hint.setWordWrap(True)
        ml.addWidget(hint, 6, 0, 1, 3)
        right.addWidget(g_man)

        g_hold = QGroupBox("4. 자세제어 (Heading Hold)")
        hl = QGridLayout(g_hold); hl.setSpacing(5)
        self.btn_hold = QPushButton("자세유지 OFF")
        self.btn_hold.setFocusPolicy(Qt.NoFocus)
        self.btn_hold.clicked.connect(self.toggle_hold)
        btn_zero = QPushButton("기준각 리셋"); btn_zero.setFocusPolicy(Qt.NoFocus)
        btn_zero.clicked.connect(self.reset_yaw)
        hl.addWidget(self.btn_hold, 0, 0); hl.addWidget(btn_zero, 0, 1)
        self.sl_yaw = QSlider(Qt.Horizontal); self.sl_yaw.setRange(-180, 180)
        self.sl_yaw.setFocusPolicy(Qt.NoFocus)
        self.lb_yaw_t = QLabel("목표 0°")
        self.sl_yaw.valueChanged.connect(self.on_target_yaw)
        hl.addWidget(self.lb_yaw_t, 1, 0); hl.addWidget(self.sl_yaw, 1, 1)
        self.sp_kp = QDoubleSpinBox(); self.sp_kp.setRange(0, 50); self.sp_kp.setValue(2.5)
        self.sp_ki = QDoubleSpinBox(); self.sp_ki.setRange(0, 50); self.sp_ki.setValue(0.0)
        self.sp_kd = QDoubleSpinBox(); self.sp_kd.setRange(0, 50); self.sp_kd.setValue(0.9)
        for sp in (self.sp_kp, self.sp_ki, self.sp_kd):
            sp.setSingleStep(0.1); sp.setDecimals(2)
        hl.addWidget(QLabel("Kp"), 2, 0); hl.addWidget(self.sp_kp, 2, 1)
        hl.addWidget(QLabel("Ki"), 3, 0); hl.addWidget(self.sp_ki, 3, 1)
        hl.addWidget(QLabel("Kd"), 4, 0); hl.addWidget(self.sp_kd, 4, 1)
        btn_gain = QPushButton("게인 적용"); btn_gain.setFocusPolicy(Qt.NoFocus)
        btn_gain.clicked.connect(self.apply_gain)
        hl.addWidget(btn_gain, 5, 0, 1, 2)
        right.addWidget(g_hold)

        g_rec = QGroupBox("5. 기록")
        rl = QHBoxLayout(g_rec)
        self.btn_rec = QPushButton("⭕ 기록 시작"); self.btn_rec.setFocusPolicy(Qt.NoFocus)
        self.btn_rec.clicked.connect(self.toggle_record)
        btn_csv = QPushButton("⬇ CSV 저장"); btn_csv.setFocusPolicy(Qt.NoFocus)
        btn_csv.clicked.connect(self.save_csv)
        rl.addWidget(self.btn_rec); rl.addWidget(btn_csv)
        right.addWidget(g_rec)
        right.addStretch(1)

        # ---------------- 하단 로그 ----------------
        g_log = QGroupBox("통신 로그")
        ll = QVBoxLayout(g_log)
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True)
        self.txt_log.setMinimumHeight(110)
        self.txt_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        ll.addWidget(self.txt_log)
        crow = QHBoxLayout()
        self.ed_cmd = QLineEdit()
        self.ed_cmd.setPlaceholderText("직접 명령 입력 (예: T300,300)")
        self.ed_cmd.returnPressed.connect(self.send_manual)
        btn_send = QPushButton("전송"); btn_send.setFocusPolicy(Qt.NoFocus)
        btn_send.clicked.connect(self.send_manual)
        self.ck_raw = QCheckBox("원시 데이터(HEX)")
        self.ck_raw.setFocusPolicy(Qt.NoFocus)
        self.ck_raw.toggled.connect(self.on_raw_toggle)
        self.ck_verbose = QCheckBox("텔레메트리 전부 표시")
        self.ck_verbose.setFocusPolicy(Qt.NoFocus)
        crow.addWidget(self.ed_cmd, 1); crow.addWidget(btn_send)
        crow.addWidget(self.ck_raw); crow.addWidget(self.ck_verbose)
        ll.addLayout(crow)
        root.addWidget(g_log)

    # -------------------------------------------------------------- 시그널
    def _wire(self, link: LinkBase):
        link.sig_line.connect(self.on_line)
        link.sig_log.connect(self.log)
        link.sig_conn.connect(self.on_conn)
        link.sig_raw.connect(self.on_raw)
        link.sig_tx.connect(self.on_tx)
        if isinstance(link, BleLink):
            link.sig_devices.connect(self.on_devices)

    # ---------------------------------------------------------------- 로그
    COLORS = dict(info="#7f93ab", tx="#5ad1a0", rx="#7fb4ff",
                  warn="#ffc46b", err="#ff7c7c")

    def log(self, msg, level="info"):
        col = self.COLORS.get(level, "#7f93ab")
        ts = time.strftime("%H:%M:%S")
        for line in str(msg).split("\n"):
            self.txt_log.append(
                f'<span style="color:#44566a">[{ts}]</span> '
                f'<span style="color:{col}">{html.escape(line)}</span>')
        sb = self.txt_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def on_raw_toggle(self, on):
        self.raw_view = on
        self.log(f"원시 데이터 표시 {'ON' if on else 'OFF'}", "info")

    def on_raw(self, data: bytes):
        if not self.raw_view:
            return
        txt = "".join(chr(b) if 32 <= b < 127 else
                      ("\\n" if b == 10 else "\\r" if b == 13 else "·")
                      for b in data)
        hexs = " ".join(f"{b:02x}" for b in data)
        self.log(f"RAW({len(data)}) {txt}", "rx")
        self.log(f"      {hexs}", "info")

    # ------------------------------------------------------------ 연결 제어
    def on_transport(self, idx):
        self.do_disconnect()
        self.link = self.ble if idx == 0 else self.ser
        self.lst_dev.clear()
        if idx == 1:
            if not HAVE_SERIAL:
                self.log("pyserial 미설치: pip install pyserial", "err")
                return
            for dev, desc in SerialLink.ports():
                QListWidgetItem(f"{dev}  —  {desc}", self.lst_dev)
            self.log("USB 포트 목록을 불러왔습니다. 하나를 고르고 [연결]", "info")
        else:
            self.log("Bluetooth 모드. [스캔]을 누르세요.", "info")

    def do_scan(self, all_devices):
        if self.link is self.ser:
            self.on_transport(1)
            return
        self.lst_dev.clear()
        self.ble.scan(all_devices)

    def on_devices(self, devs):
        self.lst_dev.clear()
        for name, addr, rssi in devs:
            QListWidgetItem(f"{name}   [{addr}]   {rssi} dBm", self.lst_dev)
        self._dev_addrs = [d[1] for d in devs]
        if devs:
            self.lst_dev.setCurrentRow(0)

    def do_link_test(self):
        if self.link is not self.ble:
            self.log("링크 시험은 Bluetooth 모드에서만 의미가 있습니다. "
                     "상단 콤보를 [Bluetooth (BLE)]로 바꾸세요.", "warn")
            return
        if self.ble.connected:
            self.log("먼저 [연결 해제]를 누르세요.", "warn")
            return
        self.log("링크 시험: 연결만 하고 15초간 유지되는지 봅니다. "
                 "여기서 끊기면 BLE 연결 단계·펌웨어·전원을 함께 확인하세요.", "warn")
        self.do_connect(True, link_only=True)

    def do_connect(self, safe, link_only=False):
        if self.link.connected:
            self.log("이미 연결되어 있습니다. 먼저 [연결 해제]를 누르세요.", "warn")
            return
        row = self.lst_dev.currentRow()
        if row < 0:
            self.log("목록에서 기기를 먼저 선택하세요", "warn")
            return
        if self.link is self.ble:
            addr = getattr(self, "_dev_addrs", [None] * (row + 1))[row]
            if not addr:
                self.log("주소를 찾을 수 없습니다. 다시 스캔하세요.", "warn")
                return
            self.ble.connect_to(addr, safe, link_only)
        else:
            dev = self.lst_dev.item(row).text().split("  —  ")[0]
            self.ser.connect_to(dev, safe)

    def do_disconnect(self):
        try:
            self.ble.disconnect()
        except Exception:
            pass
        try:
            self.ser.disconnect()
        except Exception:
            pass

    def start_program(self):
        self.log("프로그램 시작 요청 — 슬롯 번호와 응답 ACK/거부를 확인하세요.", "info")
        self.link.program_flow(False, self.sp_slot.value())

    def on_tx(self, n):
        self.set_chip(self.chip_tx, f"TX {n}")

    def toggle_safe(self):
        self.keys.clear()
        self.pad = [0, 0]
        self.drive = [0, 0]
        self.aux_command = None
        self.test_until = 0.0
        self.link.set_safe_mode(not self.link.safe_mode)

    def on_conn(self, ok, label):
        self.aux_command = None
        self.keys.clear()
        self.pad = [0, 0]
        self.drive = [0, 0]
        self.test_until = 0.0
        self._last_sent = (0, 0)
        self.tm_rx = -1
        self.repl_mode = False
        if ok:
            kind = "BLE" if self.link is self.ble else "USB"
            if self.link.safe_mode:
                self.set_chip(self.chip_link, f"{kind} · 안전모드(전송차단)",
                              "QLabel#chip{background:#ffb020;color:#1a1000;}")
            else:
                self.set_chip(self.chip_link, f"{kind} · {label[:17]}",
                              "QLabel#chip{background:#3ddc84;color:#04140b;}")
            self.btn_safe_off.setEnabled(True)
            self.btn_safe_off.setText(
                "안전 모드 해제 — 전송 켜기" if self.link.safe_mode
                else "안전 모드 켜기 (전송 차단)")
            self.btn_safe_off.setObjectName(
                "primary" if self.link.safe_mode else "")
            self.btn_safe_off.setStyleSheet("")
        else:
            self.set_chip(self.chip_link, "미연결")
            self.set_chip(self.chip_prog, "프로그램 ?")
            self.btn_safe_off.setEnabled(False)

    # ------------------------------------------------------------ 텔레메트리
    def on_line(self, line: str):
        if line.startswith("#T,"):
            parts = line[3:].split(",")
            if len(parts) >= 12:
                try:
                    (y, p, r, lv, rv, lp, rp, dist, refl, batt, hold, ms) = parts[:12]
                    self.tm.update(yaw=float(y), pitch=float(p), roll=float(r),
                                   lvel=int(float(lv)), rvel=int(float(rv)),
                                   lpos=int(float(lp)), rpos=int(float(rp)),
                                   dist=int(float(dist)), refl=int(float(refl)),
                                   batt=int(float(batt)), hold=int(float(hold)),
                                   ms=int(float(ms)))
                    # 13번째(index 12)가 허브 stdin 누적 수신 바이트(rx)
                    if len(parts) >= 13:
                        try:
                            self.tm_rx = int(float(parts[12]))
                        except ValueError:
                            pass
                    self.tm_count += 1
                    err = wrap180(self.target_yaw - self.tm["yaw"])
                    self.chart.push(self.tm["lvel"], self.tm["rvel"], err)
                    if self.recording:
                        self.rec_rows.append(",".join(parts[:12]))
                    if self.ck_verbose.isChecked():
                        self.log("RX " + line, "rx")
                except ValueError:
                    self.log("RX(형식오류) " + line, "warn")
        elif line.startswith("#S,"):
            # 허브가 stdin 으로 실제 받은 양. 명령이 닿는지 판별하는 핵심 지표.
            self.rx_report = line[3:]
            self.log("◆ 허브 수신 계측: " + line[3:], "tx")
            try:
                n = int(line.split("rx=")[1].split(",")[0])
                self.set_chip(self.chip_hubrx, f"허브RX {n}",
                              "QLabel#chip{background:#3ddc84;color:#04140b;}")
            except Exception:
                pass
        elif line.startswith("#I,repl-ready"):
            self.repl_ready_seen = True
            self.log("◆ REPL 셋업 완료 응답 수신 — 조종 가능합니다", "tx")
        elif line.startswith("#I,battery,"):
            self.log("🔋 허브 배터리: " + line[11:], "warn")
        elif line.startswith("#I,"):
            self.log("허브: " + line[3:], "rx")
        elif line.startswith("#E,"):
            self.log("허브 경고: " + line[3:], "warn")
        else:
            for tag in ("UPLINK-A", "UPLINK-B", "UPLINK-C", "UPLINK-D"):
                if tag in line and tag not in getattr(self, "uplink_hits", []):
                    self.uplink_hits.append(tag)
                    self.log(f"◆ 업링크 도달 확인: {tag}", "tx")
                    return
            if "PONG-REPL" in line:
                self.repl_ping_seen = True
                self.log("◆ REPL 응답: " + line, "tx")
                return
            if self.repl_mode:
                # REPL 에코(>>> , 우리가 보낸 줄)는 로그를 어지럽히므로 건너뛴다
                st = line.strip().lstrip(">. ")
                if (not st) or st.startswith(("d(", "tm(", "import ",
                                              "from ", "motor_pair.pair",
                                              "d=lambda", "tm=lambda")):
                    return
            self.log("RX " + line, "rx")

    # -------------------------------------------------------------- 주기 작업
    def on_camera_lost(self):
        self.estop()
        self.log("영상 수신 대기/끊김: 로버 정지 요청. 영상 회복 후 다시 조종하세요.", "warn")

    def tick_drive(self):
        if not self.camera.motion_allowed():
            self.keys.clear()
            self.pad = [0, 0]
            self.drive = [0, 0]
            self.aux_command = None
            self.test_until = 0.0
            return
        if not self.link.connected or self.link.safe_mode:
            # 조종 입력이 있는데 보낼 수 없는 상태면 이유를 알려준다
            if self.keys or self.pad != [0, 0]:
                self.link._warn_drop(
                    "안전 모드입니다. [안전 모드 해제]를 누르세요."
                    if self.link.safe_mode else "허브가 연결되어 있지 않습니다")
            return
        t = time.monotonic()
        if not self.link.program_alive() or not self.link.command_ready:
            if self.keys or self.pad != [0, 0]:
                self.link._warn_drop("허브 프로그램 응답 대기: 새 허브 코드 실행 후 업링크 진단하세요")
            self.aux_command = None
            self.keys.clear()
            self.pad = [0, 0]
            self.drive = [0, 0]
            self.test_until = 0.0
            if self.link.program_alive() and t - getattr(self, "_probe_t", 0) > 1:
                self._probe_t = t
                self.link.send("Q")
            return
        if self.aux_command and t - getattr(self, "_aux_t", 0) > 0.25:
            self._aux_t = t
            pid, velocity = self.aux_command
            self.link.send(f"A{pid},{velocity}")
        if t < self.test_until:
            pass                              # 모터 테스트 중에는 값 유지
        else:
            self.update_manual()
        changed = (self.drive[0], self.drive[1]) != self._last_sent
        moving = self.drive[0] != 0 or self.drive[1] != 0
        if self.repl_mode:
            # REPL 모드: 값이 바뀔 때만 보낸다(REPL 왕복이 있으므로 아껴 쓴다)
            if changed or (moving and t - self._last_send_t > 0.4):
                self.link.send(f"d({self.drive[0]},{self.drive[1]})")
                self._last_sent = (self.drive[0], self.drive[1])
                self._last_send_t = t
            return
        keep = (t - self._last_send_t > 0.25) and (moving or self.link.program_alive())
        if changed or keep:
            self.link.send(f"T{self.drive[0]},{self.drive[1]}")
            self._last_sent = (self.drive[0], self.drive[1])
            self._last_send_t = t

    def update_manual(self):
        spd = self.sl_speed.value()
        trn = self.sl_turn.value() / 100.0
        f = s = 0
        if "w" in self.keys: f += 1
        if "s" in self.keys: f -= 1
        if "a" in self.keys: s -= 1
        if "d" in self.keys: s += 1
        f += self.pad[0]; s += self.pad[1]
        f = max(-1, min(1, f)); s = max(-1, min(1, s))
        if f == 0 and s == 0:
            self.drive = [0, 0]
        else:
            self.drive = [int(spd * f + spd * trn * s),
                          int(spd * f - spd * trn * s)]

    def tick_ui(self):
        tm = self.tm
        self.adi.set_attitude(tm["pitch"], tm["roll"])
        self.hsi.set_heading(tm["yaw"], self.target_yaw)
        self.v_yaw.setText(f'{tm["yaw"]:.1f}°')
        self.v_pit.setText(f'{tm["pitch"]:.1f}°')
        self.v_rol.setText(f'{tm["roll"]:.1f}°')
        self.v_dist.setText("--" if tm["dist"] < 0 else str(tm["dist"]))
        self.v_refl.setText("--" if tm["refl"] < 0 else str(tm["refl"]))
        self.v_batt.setText("--" if tm["batt"] < 0 else f'{tm["batt"]}%')
        self.v_ms.setText(f'{tm["ms"]/1000:.1f}')
        self.v_lpos.setText(str(tm["lpos"])); self.v_rpos.setText(str(tm["rpos"]))
        self.lb_l.setText(f'좌 {tm["lvel"]:>5}'); self.lb_r.setText(f'우 {tm["rvel"]:>5}')
        self.bar_l.set_value(tm["lvel"]); self.bar_r.set_value(tm["rvel"])
        if self.tm_rx >= 0:
            self.set_chip(
                self.chip_hubrx, f"허브RX {self.tm_rx}",
                "QLabel#chip{background:#3ddc84;color:#04140b;}" if self.tm_rx > 0
                else "QLabel#chip{background:#ff5a5a;color:#fff;}")
        alive = self.link.program_alive()
        self.set_chip(
            self.chip_prog,
            ("조종 준비 완료" if self.link.command_ready else "프로그램 수신 · Q 응답 대기") if alive else "텔레메트리 없음",
            "QLabel#chip{background:#3ddc84;color:#04140b;}" if alive else
            ("QLabel#chip{background:#ffb020;color:#1a1000;}"
             if self.link.connected else ""))

    def tick_repl_tm(self):
        """REPL 모드에서는 GCS가 주기적으로 텔레메트리를 '요청'한다."""
        if self.repl_mode and self.link.connected and not self.link.safe_mode:
            self.link.send("tm()")

    def tick_rate(self):
        t = time.monotonic()
        dt = t - self._rate_t
        self.tm_rate = self.tm_count / dt if dt > 0 else 0
        self.tm_count = 0
        self._rate_t = t
        self.set_chip(self.chip_rate, f"{self.tm_rate:.0f} Hz",
                      "QLabel#chip{background:#3ddc84;color:#04140b;}"
                      if self.tm_rate > 3 else "")

    # ------------------------------------------------------------ 동작 핸들러
    def set_pad(self, f, s):
        self.pad = [f, s]

    def uplink_diag(self):
        if not self.link.can_send():
            self.log("일반 연결 후 진단하세요. 안전 모드는 전송을 차단합니다.", "warn")
            return
        link = self.link
        link.command_ready = False
        self.log(f"진단 시작: 텔레메트리={'있음' if link.program_alive() else '없음'}, "
                 f"터널 수신={getattr(link, '_tunnel_rx_count', 0)}건", "info")
        if link.send("Q"):
            self.log("Q 전송 요청 — hub_rover.py tunnel-v2 응답을 기다립니다.", "info")
            QTimer.singleShot(2500, lambda: self.uplink_result(link))

    def uplink_result(self, link):
        if link is not self.link or not link.connected:
            return
        if link.command_ready:
            self.log("업링크 성공: 허브가 Q를 해석하고 ACK를 응답했습니다.", "tx")
        elif not link.program_alive():
            self.log("진단: Q 응답과 텔레메트리 없음. 새 hub_rover.py를 다운로드한 슬롯을 실행하세요. "
                     "[원시 데이터]를 켜고 [프로그램 시작] 후 오류를 확인하세요.", "err")
        else:
            self.log("진단: 텔레메트리는 있으나 Q ACK 없음. 허브 #S의 mode=tunnel 여부와 "
                     "module_tunnel_unavailable/callback 오류를 확인하세요.", "err")

    def repl_ping(self):
        self.log("REPL 대신 hub_rover.py를 실행하고 [업링크 진단]을 사용하세요.", "warn")

    def repl_start(self):
        self.repl_ping()

    def repl_stop(self):
        self.repl_mode = False

    def send_aux(self, v):
        if v and not self.camera.motion_allowed():
            return
        pid = self.cb_aux.currentIndex()
        if self.link.send(f"A{pid},{v}"):
            self.aux_command = (pid, v) if v else None


    def apply_ports(self):
        if self.cb_left.currentIndex() == self.cb_right.currentIndex():
            self.log("좌우 모터는 서로 다른 포트여야 합니다.", "err")
            return
        self.drive = [0, 0]
        self.keys.clear()
        self.pad = [0, 0]
        a = self.link.send(f"P{self.cb_left.currentIndex()},"
                           f"{self.cb_right.currentIndex()}")
        b = self.link.send(f"I{int(self.ck_invl.isChecked())},"
                           f"{int(self.ck_invr.isChecked())}")
        if a and b:
            self.log("구동부 설정 전송 완료 — [상태 조회]로 확인하세요", "tx")

    def apply_gain(self):
        k = lambda sp: int(round(sp.value() * 1000))
        if self.link.send(f"K{k(self.sp_kp)},{k(self.sp_ki)},{k(self.sp_kd)}"):
            self.log("PID 게인 전송", "tx")

    def toggle_hold(self):
        self.hold_on = not self.hold_on
        self.btn_hold.setText(f"자세유지 {'ON' if self.hold_on else 'OFF'}")
        self.btn_hold.setObjectName("ok" if self.hold_on else "")
        self.btn_hold.setStyleSheet("")
        if not self.link.send("H1" if self.hold_on else "H0"):
            self.hold_on = not self.hold_on
            self.btn_hold.setText(f"자세유지 {'ON' if self.hold_on else 'OFF'}")
            return
        if self.hold_on:
            self.target_yaw = self.tm["yaw"]
            self.sl_yaw.blockSignals(True)
            self.sl_yaw.setValue(int(self.target_yaw))
            self.sl_yaw.blockSignals(False)
            self.lb_yaw_t.setText(f"목표 {int(self.target_yaw)}°")
        self.log(f"자세유지 {'ON' if self.hold_on else 'OFF'}", "tx")

    def on_target_yaw(self, v):
        self.target_yaw = float(v)
        self.lb_yaw_t.setText(f"목표 {v}°")
        self.link.send(f"Y{v}")

    def reset_yaw(self):
        if not self.link.send("Z"):
            return
        self.target_yaw = 0.0
        self.sl_yaw.blockSignals(True); self.sl_yaw.setValue(0)
        self.sl_yaw.blockSignals(False)
        self.lb_yaw_t.setText("목표 0°")
        self.log("기준각 리셋", "tx")

    def motor_test(self):
        if not self.link.connected:
            self.log("허브가 연결되지 않았습니다", "err")
            return
        if self.link.safe_mode:
            self.log("안전 모드라 모터 테스트를 보낼 수 없습니다. "
                     "[안전 모드 해제]를 누르세요.", "err")
            return
        if not self.link.program_alive():
            self.log("경고: 텔레메트리가 없습니다. 프로그램이 실행 중인지 확인하세요.", "warn")
        self.log("모터 테스트: 300 deg/s 전진 0.5초", "tx")
        self.drive = [300, 300]
        self.test_until = time.monotonic() + 0.5
        QTimer.singleShot(520, lambda: setattr(self, "drive", [0, 0]))

    def estop(self):
        self.aux_command = None
        self.hold_on = False
        self.btn_hold.setText("자세유지 OFF")
        self.drive = [0, 0]
        self.keys.clear(); self.pad = [0, 0]
        self.test_until = 0.0
        self.link.send("E")
        self.set_chip(self.chip_mode, "E-STOP",
                      "QLabel#chip{background:#ff5a5a;color:#fff;}")
        self.log("*** 비상정지 ***", "err")
        QTimer.singleShot(1200, lambda: self.set_chip(self.chip_mode, "MANUAL"))

    def send_manual(self):
        if not self.camera.motion_allowed() and self.ed_cmd.text().strip() not in ("E", "S", "Q"):
            self.log("영상 수신 대기 중: 조종 명령 차단", "warn")
            return
        txt = self.ed_cmd.text().strip()
        if not txt:
            return
        if self.link.send(txt):
            self.log("TX " + txt, "tx")
        self.ed_cmd.clear()
        self.setFocus()

    def toggle_record(self):
        self.recording = not self.recording
        self.btn_rec.setText("⏹ 기록 중지" if self.recording else "⭕ 기록 시작")
        if self.recording:
            self.rec_rows.clear()
            self.log("텔레메트리 기록 시작", "info")
        else:
            self.log(f"기록 종료 ({len(self.rec_rows)}행)", "info")

    def save_csv(self):
        if not self.rec_rows:
            self.log("기록된 데이터가 없습니다", "warn")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "CSV 저장", f"rover_telemetry_{int(time.time())}.csv",
            "CSV (*.csv)")
        if not path:
            return
        head = ("yaw,pitch,roll,lvel,rvel,lpos,rpos,dist,refl,batt,hold,ms")
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(head + "\n" + "\n".join(self.rec_rows) + "\n")
        self.log(f"저장 완료: {path}", "info")

    # ------------------------------------------------------------------ 키보드
    def eventFilter(self, obj, event):
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease) and self.isActiveWindow():
            if isinstance(QApplication.focusWidget(), (QLineEdit, QTextEdit, QAbstractSpinBox)):
                return super().eventFilter(obj, event)
            keys = (Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Up, Qt.Key_Left,
                    Qt.Key_Down, Qt.Key_Right, Qt.Key_Space, Qt.Key_Q, Qt.Key_E, Qt.Key_G)
            if event.key() in keys:
                if event.type() == QEvent.KeyPress:
                    self.keyPressEvent(event)
                else:
                    self.keyReleaseEvent(event)
                return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, e: QKeyEvent):
        if e.isAutoRepeat():
            return
        k = e.key()
        if k == Qt.Key_Space:
            self.estop(); return
        if k == Qt.Key_G:
            self.toggle_hold(); return
        if k == Qt.Key_Q:
            self.send_aux(-500); return
        if k == Qt.Key_E:
            self.send_aux(500); return
        mapping = {Qt.Key_W: "w", Qt.Key_A: "a", Qt.Key_S: "s", Qt.Key_D: "d",
                   Qt.Key_Up: "w", Qt.Key_Left: "a",
                   Qt.Key_Down: "s", Qt.Key_Right: "d"}
        if k in mapping:
            self.keys.add(mapping[k])
        else:
            super().keyPressEvent(e)

    def keyReleaseEvent(self, e: QKeyEvent):
        if e.isAutoRepeat():
            return
        k = e.key()
        if k in (Qt.Key_Q, Qt.Key_E):
            self.send_aux(0); return
        mapping = {Qt.Key_W: "w", Qt.Key_A: "a", Qt.Key_S: "s", Qt.Key_D: "d",
                   Qt.Key_Up: "w", Qt.Key_Left: "a",
                   Qt.Key_Down: "s", Qt.Key_Right: "d"}
        if k in mapping:
            self.keys.discard(mapping[k])
        else:
            super().keyReleaseEvent(e)

    def on_app_state(self, state):
        if state != Qt.ApplicationActive:
            self.estop()

    def focusOutEvent(self, e):
        if self.aux_command:
            self.send_aux(0)
        self.keys.clear()
        self.pad = [0, 0]
        super().focusOutEvent(e)

    def closeEvent(self, e):
        self.camera.stop()
        self.do_disconnect()
        super().closeEvent(e)


# =============================================================================
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SPIKE Rover GCS")
    win = RoverGCS()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        input("오류가 발생했습니다. Enter 를 누르면 종료합니다…")
