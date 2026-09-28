#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
 SPIKE Rover AI GCS  --  Ollama(로컬 LLM)에게 말로 명령하는 지상관제소
===============================================================================
 "앞으로 30cm 가", "오른쪽으로 90도 돌아", "벽 15cm 앞까지 전진",
 "빨간색 찾아서 그쪽을 봐" 같은 한국어 명령을 Ollama 모델이 도구 호출로 바꾸고,
 GCS가 IMU·엔코더·거리센서·영상분석 값을 보면서 실제 동작을 끝까지 실행합니다.

 필요한 것
     - 같은 폴더: rover_gcs.py, camera_panel.py, mjpeg_parser.py,
                  analysis_panel.py, vision_analysis.py
     - 허브: 기존 hub_rover.py (tunnel-v2) 그대로
     - PC: Ollama 실행 중 + 도구 호출 지원 모델 (qwen2.5 / qwen3 권장)
           ollama pull qwen2.5:7b

 구조
     사용자 문장 ──► Ollama /api/chat (tools) ──► 도구 호출
                                   ▲                  │
                                   └── 결과(JSON) ◄── Executor(50ms 루프)
                                                      │  T/S/E 명령, 센서 확인
                                                      ▼
                                                 BleLink ──► SPIKE 허브

 안전
     - [AI 주행 허용]을 켜야 움직입니다. 비상정지: 버튼, Esc, Space(입력칸 밖)
     - 한 번 이동 최대 200cm / 회전 720° / 시간 10초, 속도 상한은 설정값
     - 전진 중 거리센서가 설정값보다 가까우면 자동 정지
     - 영상 연결을 요청한 상태에서 영상이 끊기면 정지 (기존 GCS와 동일)
     - LLM은 모터를 직접 돌리지 않습니다. 정해진 도구만 GCS가 검사 후 실행합니다.
===============================================================================
"""
import sys
import json
import math
import re
import time
import html
import threading
import traceback
import urllib.request
import urllib.error

from PySide6.QtCore import Qt, QObject, Signal, QTimer, QSettings, QEvent
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QCheckBox, QLineEdit, QTextEdit, QTextBrowser, QGroupBox, QGridLayout,
    QHBoxLayout, QVBoxLayout, QListWidget, QListWidgetItem, QSpinBox,
    QDoubleSpinBox, QSplitter, QScrollArea, QFrame, QAbstractSpinBox, QSizePolicy)

from rover_gcs import BleLink, STYLE, wrap180, HAVE_BLEAK, BLEAK_ERROR
from camera_panel import CameraPanel
from analysis_panel import AnalysisPanel
import vision_analysis as va

APP_VERSION = "ai-v1"

# =============================================================================
#  1. 도구 정의 (Ollama tools 형식)
# =============================================================================
def _tool(name, desc, props=None, required=None):
    return {"type": "function", "function": {
        "name": name, "description": desc,
        "parameters": {"type": "object", "properties": props or {},
                       "required": required or []}}}


SPEED_PROP = {"type": "number",
              "description": "속도 10~100 (%). 언급이 없으면 50. '천천히'=30, '빨리'=80"}

VISION_MODES = [m[0] for m in va.MODES]           # raw, qr, color, shape, motion, edge
COLOR_NAMES = [c[0] for c in getattr(va, "COLOR_DEFS", [])]

TOOLS = [
    _tool("drive_distance",
          "로버를 지정한 거리만큼 직진한다. 양수=전진, 음수=후진. 바퀴 엔코더로 거리를 잰다.",
          {"distance_cm": {"type": "number", "description": "이동 거리 cm. 전진 +, 후진 -. 최대 ±200"},
           "speed": SPEED_PROP}, ["distance_cm"]),
    _tool("drive_time",
          "로버를 지정한 시간 동안 전진 또는 후진한다.",
          {"seconds": {"type": "number", "description": "0.1~10초"},
           "direction": {"type": "string", "enum": ["forward", "backward"]},
           "speed": SPEED_PROP}, ["seconds", "direction"]),
    _tool("turn",
          "제자리에서 지정한 각도만큼 회전한다. 자이로(IMU) 각도로 잰다.",
          {"angle_deg": {"type": "number", "description": "회전 각도(도). 1~720"},
           "direction": {"type": "string", "enum": ["left", "right"],
                         "description": "left=왼쪽(반시계), right=오른쪽(시계)"},
           "speed": SPEED_PROP}, ["angle_deg", "direction"]),
    _tool("drive_until_obstacle",
          "거리센서로 앞의 물체까지 거리를 보면서, 지정한 거리가 될 때까지 전진한다.",
          {"stop_distance_cm": {"type": "number", "description": "이 거리(cm)가 되면 정지. 기본 15"},
           "max_distance_cm": {"type": "number", "description": "최대 이동 거리 cm. 기본 150"},
           "speed": SPEED_PROP}),
    _tool("stop", "즉시 정지한다."),
    _tool("wait", "지정한 시간 동안 멈춰서 기다린다.",
          {"seconds": {"type": "number", "description": "0.1~10초"}}, ["seconds"]),
    _tool("get_status",
          "로버 상태를 읽는다: 방위각(yaw), 기울기, 거리센서(cm), 반사광, 배터리, 연결 상태."),
    _tool("look",
          "카메라 영상 분석의 최신 결과를 읽는다(검출 목록, QR 내용, 목표 방향). "
          "분석이 꺼져 있으면 먼저 set_vision_mode를 호출한다."),
    _tool("set_vision_mode",
          "영상 분석 방식을 바꾸고 약 1초 뒤 결과를 돌려준다. "
          "raw=끔, qr=QR코드, color=색 찾기, shape=도형, motion=움직임, edge=장애물 방향.",
          {"mode": {"type": "string", "enum": VISION_MODES},
           "colors": {"type": "array", "items": {"type": "string", "enum": COLOR_NAMES} if COLOR_NAMES
                      else {"type": "string"},
                      "description": "mode=color일 때 찾을 색 이름 목록 (예: ['빨강'])"}},
          ["mode"]),
    _tool("turn_to_target",
          "현재 영상 분석(color/shape/qr)에서 가장 큰 대상이 화면 가운데 오도록 조금씩 회전한다. "
          "대상이 안 보이면 search=true일 때 제자리에서 돌며 찾는다.",
          {"search": {"type": "boolean", "description": "안 보이면 돌면서 찾기. 기본 true"},
           "max_seconds": {"type": "number", "description": "최대 시간 초. 기본 12"}}),
]
TOOL_NAMES = {t["function"]["name"] for t in TOOLS}
MOTION_TOOLS = {"drive_distance", "drive_time", "turn", "drive_until_obstacle", "turn_to_target"}

SYSTEM_PROMPT = """너는 LEGO SPIKE Prime 탐사 로버를 조종하는 AI 조종사다.
사용자의 한국어 명령을 도구(tool) 호출로 바꿔 로버를 움직이고, 결과를 짧게 보고한다.

규칙
1. 로버는 오직 도구로만 움직인다. 도구를 호출하지 않고 "이동했습니다"라고 말하지 마라.
2. 여러 동작은 순서대로 하나씩 호출하고, 결과를 확인한 다음 호출한다.
3. 단위: 거리 cm, 각도 도(°), 속도 10~100(%). 속도 언급이 없으면 50.
   "조금"=10cm 또는 15°, "한 칸"=20cm, "뒤돌아"=180° 회전.
4. 도구 결과에 ok=false, obstacle, error가 있으면 다음 동작을 멈추고 이유를 알려라.
5. 명령이 모호하거나 한계(이동 200cm, 회전 720°, 10초)를 넘으면 실행하지 말고 짧게 되물어라.
6. 센서/영상 질문에는 get_status, look을 호출해 실제 값으로 답한다. 값을 지어내지 마라.
7. 색·도형·QR을 찾으라는 명령은 set_vision_mode → turn_to_target 순서로 한다.
8. 답변은 한국어 1~3문장. 숫자는 도구 결과에 있는 값을 쓴다."""

JSON_MODE_ADDENDUM = """
[도구 호출 형식]
이 모델은 기본 도구 호출을 지원하지 않으므로 아래 JSON만 출력한다(설명 문장 금지).
{"say": "사용자에게 할 말", "actions": [{"name": "도구이름", "arguments": {...}}]}
동작이 필요 없으면 actions를 빈 배열로 한다. 도구 결과는 다음 메시지로 전달된다.
사용 가능한 도구:
""" + "\n".join(f"- {t['function']['name']}: {t['function']['description']} "
                f"인자 {list(t['function']['parameters']['properties'])}" for t in TOOLS)


# =============================================================================
#  2. Ollama 클라이언트 (표준 라이브러리 urllib, 별도 스레드)
# =============================================================================
class OllamaClient(QObject):
    sig_reply = Signal(int, dict)        # 세대번호, 응답 JSON
    sig_error = Signal(int, str, str)    # 세대번호, 코드, 메시지
    sig_models = Signal(list, str)       # 모델 이름 목록, 오류

    def __init__(self):
        super().__init__()
        self.base = "http://127.0.0.1:11434"
        # 시스템 프록시가 localhost 요청을 가로채지 않도록 프록시를 끈다.
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _request(self, path, body=None, timeout=10):
        url = self.base.rstrip("/") + path
        if body is None:
            req = urllib.request.Request(url)
        else:
            req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"),
                                         headers={"Content-Type": "application/json"})
        with self._opener.open(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    @staticmethod
    def _http_error_text(e):
        try:
            return json.loads(e.read().decode("utf-8")).get("error", str(e))
        except Exception:
            return str(e)

    def list_models(self):
        def run():
            try:
                data = self._request("/api/tags", timeout=5)
                names = [m.get("name", "") for m in data.get("models", [])]
                self.sig_models.emit([n for n in names if n], "")
            except Exception as e:
                self.sig_models.emit([], str(e))
        threading.Thread(target=run, daemon=True).start()

    def chat(self, gen, model, messages, tools, think_off=True):
        body = {"model": model, "messages": messages, "stream": False,
                "keep_alive": "15m", "options": {"temperature": 0.2}}
        if tools:
            body["tools"] = tools
        if think_off:
            body["think"] = False

        def run():
            payload = dict(body)
            for _attempt in range(2):
                try:
                    data = self._request("/api/chat", payload, timeout=240)
                    self.sig_reply.emit(gen, data)
                    return
                except urllib.error.HTTPError as e:
                    msg = self._http_error_text(e)
                    low = msg.lower()
                    if "think" in low and "think" in payload:
                        payload.pop("think", None)       # 생각 모드 미지원 모델
                        continue
                    if "does not support tools" in low or "tools" in low and "support" in low:
                        self.sig_error.emit(gen, "NO_TOOLS", msg)
                        return
                    if "not found" in low:
                        self.sig_error.emit(gen, "NO_MODEL", msg)
                        return
                    self.sig_error.emit(gen, "HTTP", msg)
                    return
                except Exception as e:
                    self.sig_error.emit(gen, "NET", str(e))
                    return
        threading.Thread(target=run, daemon=True).start()


def parse_text_calls(text):
    """도구 호출이 본문 텍스트로 나온 경우(<tool_call>, JSON)를 해석한다."""
    calls, say = [], None
    for m in re.finditer(r"<tool_call>\s*(\{.*?\})\s*</tool_call>", text or "", re.S):
        try:
            calls.append(json.loads(m.group(1)))
        except ValueError:
            pass
    if not calls:
        blob = text or ""
        m = re.search(r"```(?:json)?\s*(\{.*\})\s*```", blob, re.S)
        cand = m.group(1) if m else blob[blob.find("{"):blob.rfind("}") + 1] if "{" in blob else ""
        if cand:
            try:
                obj = json.loads(cand)
                if isinstance(obj, dict) and "actions" in obj:
                    say = obj.get("say")
                    calls = [a for a in obj.get("actions") or [] if isinstance(a, dict)]
                elif isinstance(obj, dict) and ("name" in obj or "tool" in obj):
                    calls = [obj]
            except ValueError:
                pass
    out = []
    for c in calls:
        name = c.get("name") or c.get("tool")
        args = c.get("arguments", c.get("args", c.get("parameters", {})))
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except ValueError:
                args = {}
        if name in TOOL_NAMES:
            out.append({"function": {"name": name, "arguments": args or {}}})
    return out, say


def strip_think(text):
    return re.sub(r"<think>.*?</think>", "", text or "", flags=re.S).strip()


# =============================================================================
#  3. 실행기 — 도구 하나를 50ms 루프로 끝까지 실행 (발생기 기반)
# =============================================================================
class Executor(QObject):
    sig_done = Signal(str, dict)          # 도구 이름, 결과
    sig_progress = Signal(str)

    def __init__(self, gcs):
        super().__init__()
        self.g = gcs
        self.gen = None
        self.name = ""
        self.cmd = (0, 0)
        self._sent = (0, 0)
        self._sent_t = 0.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def busy(self):
        return self.gen is not None

    # ---------------------------------------------------------------- 실행
    def start(self, name, args):
        if self.gen is not None:
            self.abort("이전 동작 취소")
        fn = getattr(self, "act_" + name, None)
        if name not in TOOL_NAMES or fn is None:
            self.sig_done.emit(name, {"ok": False, "error": f"알 수 없는 도구: {name}"})
            return
        if name in MOTION_TOOLS:
            why = self.g.motion_block_reason()
            if why:
                self.sig_done.emit(name, {"ok": False, "error": why})
                return
        try:
            res = fn(**(args or {}))
        except TypeError as e:
            self.sig_done.emit(name, {"ok": False, "error": f"인자 오류: {e}"})
            return
        except Exception as e:
            self.sig_done.emit(name, {"ok": False, "error": str(e)})
            return
        if isinstance(res, dict):
            self.sig_done.emit(name, res)
            return
        self.gen, self.name = res, name
        self.tick()

    def tick(self):
        if self.gen is None:
            return
        if self.name in MOTION_TOOLS:
            why = self.g.motion_block_reason()
            if why:
                self.abort(why, estop=True)
                return
        try:
            next(self.gen)
        except StopIteration as s:
            self._finish(s.value if isinstance(s.value, dict) else {"ok": True})
            return
        except Exception as e:
            self._finish({"ok": False, "error": f"실행 오류: {e}"})
            return
        self._send_drive()

    def _send_drive(self, force=False):
        now = time.monotonic()
        cmd = (int(self.cmd[0]), int(self.cmd[1]))
        moving = cmd != (0, 0)
        if force or cmd != self._sent or (moving and now - self._sent_t > 0.2):
            if cmd == (0, 0):
                self.g.link.send("S")
            else:
                self.g.link.send(f"T{cmd[0]},{cmd[1]}")
            self._sent, self._sent_t = cmd, now

    def _finish(self, result):
        self.gen = None
        name, self.name = self.name, ""
        if self.cmd != (0, 0) or self._sent != (0, 0):
            self.cmd = (0, 0)
            self._send_drive(force=True)
        self.sig_done.emit(name, result)

    def abort(self, why, estop=False, notify=True):
        gen, name = self.gen, self.name
        self.gen, self.name = None, ""
        if gen is not None:
            try:
                gen.close()
            except Exception:
                pass
        self.cmd = (0, 0)
        self._sent = (0, 0)
        self._sent_t = time.monotonic()
        if self.g.link.connected:
            self.g.link.send("E" if estop else "S")
        if gen is not None and notify:
            self.sig_done.emit(name, {"ok": False, "stopped": True, "error": why})

    # ------------------------------------------------------------ 보조 함수
    def speed(self, pct):
        try:
            pct = float(pct)
        except (TypeError, ValueError):
            pct = 50.0
        pct = max(10.0, min(100.0, pct))
        return int(max(60, self.g.sp_maxspd.value() * pct / 100.0))

    def cm_to_deg(self, cm):
        return abs(cm) / (math.pi * self.g.sp_wheel.value()) * 360.0

    def deg_to_cm(self, deg):
        return deg / 360.0 * math.pi * self.g.sp_wheel.value()

    def enc_travel(self, start):
        tm = self.g.tm
        trav = (abs(tm["lpos"] - start[0]) + abs(tm["rpos"] - start[1])) / 2.0
        vel = (abs(tm["lvel"]) + abs(tm["rvel"])) / 2.0
        age = min(max(time.monotonic() - self.g.tm_t, 0.0), 0.3)
        return trav + vel * age                 # 텔레메트리 사이를 속도로 보간

    def dist_cm(self):
        d = self.g.tm["dist"]
        return None if d is None or d < 0 else d / 10.0     # 허브 값은 mm

    def obstacle(self):
        guard = self.g.sp_guard.value()
        d = self.dist_cm()
        return guard > 0 and d is not None and d <= guard

    def _wait(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            yield

    # ------------------------------------------------------------- 동작 도구
    def act_drive_distance(self, distance_cm, speed=50, **_):
        d = max(-200.0, min(200.0, float(distance_cm)))
        if abs(d) < 0.5:
            return {"ok": True, "moved_cm": 0}
        return self._drive_dist(d, speed)

    def _drive_dist(self, d, speed):
        sign = 1 if d > 0 else -1
        v = self.speed(speed)
        target = self.cm_to_deg(d)
        start = (self.g.tm["lpos"], self.g.tm["rpos"])
        t0 = time.monotonic()
        timeout = target / v * 2.0 + 3.0
        self.sig_progress.emit(f"{'전진' if sign > 0 else '후진'} {abs(d):.0f}cm")
        self.cmd = (sign * v, sign * v)
        while True:
            yield
            trav = self.enc_travel(start)
            if trav >= target:
                break
            now = time.monotonic()
            if sign > 0 and self.obstacle():
                self.cmd = (0, 0)
                return {"ok": False, "obstacle": True,
                        "moved_cm": round(self.deg_to_cm(trav), 1),
                        "distance_sensor_cm": self.dist_cm(),
                        "error": f"장애물 {self.dist_cm():.0f}cm 앞에서 자동 정지"}
            if now - t0 > 1.5 and trav < 10:
                self.cmd = (0, 0)
                return {"ok": False, "error": "바퀴 엔코더 변화 없음(모터 포트/전원 확인)"}
            if now - t0 > timeout:
                self.cmd = (0, 0)
                return {"ok": False, "error": "시간 초과", "moved_cm": round(self.deg_to_cm(trav), 1)}
            vv = v if target - trav > 90 else max(70, int(v * 0.5))   # 끝에서 감속
            self.cmd = (sign * vv, sign * vv)
        self.cmd = (0, 0)
        yield from self._wait(0.3)
        trav = self.enc_travel(start)
        return {"ok": True, "moved_cm": round(sign * self.deg_to_cm(trav), 1),
                "yaw": round(self.g.tm["yaw"], 1)}

    def act_drive_time(self, seconds, direction="forward", speed=50, **_):
        sec = max(0.1, min(10.0, float(seconds)))
        sign = -1 if str(direction).lower().startswith(("b", "후", "뒤")) else 1
        return self._drive_time(sec, sign, speed)

    def _drive_time(self, sec, sign, speed):
        v = self.speed(speed)
        start = (self.g.tm["lpos"], self.g.tm["rpos"])
        end = time.monotonic() + sec
        self.sig_progress.emit(f"{'전진' if sign > 0 else '후진'} {sec:.1f}초")
        self.cmd = (sign * v, sign * v)
        while time.monotonic() < end:
            yield
            if sign > 0 and self.obstacle():
                self.cmd = (0, 0)
                return {"ok": False, "obstacle": True,
                        "moved_cm": round(self.deg_to_cm(self.enc_travel(start)), 1),
                        "error": f"장애물 {self.dist_cm():.0f}cm 앞에서 자동 정지"}
        self.cmd = (0, 0)
        yield from self._wait(0.3)
        return {"ok": True, "moved_cm": round(sign * self.deg_to_cm(self.enc_travel(start)), 1)}

    def act_turn(self, angle_deg, direction="right", speed=40, **_):
        a = max(1.0, min(720.0, abs(float(angle_deg))))
        s = str(direction).lower()
        right = not (s.startswith("l") or "왼" in s or "좌" in s)
        return self._turn(a, right, speed)

    def _turn(self, a, right, speed, report=True):
        v = self.speed(speed)
        sgn = 1 if right else -1
        acc, prev, prev_seq = 0.0, self.g.tm["yaw"], self.g.tm_seq
        rate, last_t = 0.0, time.monotonic()
        t0 = time.monotonic()
        timeout = a / 25.0 + 4.0
        if report:
            self.sig_progress.emit(f"{'오른쪽' if right else '왼쪽'} {a:.0f}° 회전")
        self.cmd = (sgn * v, -sgn * v)
        while True:
            yield
            now = time.monotonic()
            if self.g.tm_seq != prev_seq:                  # 새 IMU 값
                y = self.g.tm["yaw"]
                step = abs(wrap180(y - prev))
                dt = max(now - last_t, 0.05)
                rate = 0.6 * rate + 0.4 * (step / dt)
                acc += step
                prev, prev_seq, last_t = y, self.g.tm_seq, now
            predicted = acc + rate * min(now - last_t, 0.3)
            remain = a - predicted
            if remain <= 1.5:
                break
            if now - t0 > 1.5 and acc < 2:
                self.cmd = (0, 0)
                return {"ok": False, "error": "회전이 감지되지 않음(IMU/모터 확인)"}
            if now - t0 > timeout:
                self.cmd = (0, 0)
                return {"ok": False, "error": "회전 시간 초과", "turned_deg": round(acc, 1)}
            vv = v if remain > 30 else max(60, int(v * 0.45))
            self.cmd = (sgn * vv, -sgn * vv)
        self.cmd = (0, 0)
        yield from self._wait(0.35)
        acc += abs(wrap180(self.g.tm["yaw"] - prev))       # 정지하며 더 돈 각도까지 포함
        return {"ok": True, "turned_deg": round(acc, 1), "yaw": round(self.g.tm["yaw"], 1)}

    def act_drive_until_obstacle(self, stop_distance_cm=15, max_distance_cm=150, speed=40, **_):
        stop_cm = max(3.0, min(100.0, float(stop_distance_cm)))
        max_cm = max(5.0, min(200.0, float(max_distance_cm)))
        v = self.speed(speed)
        target = self.cm_to_deg(max_cm)
        start = (self.g.tm["lpos"], self.g.tm["rpos"])
        t0 = time.monotonic()
        timeout = target / v * 2.0 + 3.0
        d = self.dist_cm()
        if d is not None and d <= stop_cm:
            return {"ok": True, "moved_cm": 0, "distance_sensor_cm": d, "note": "이미 정지 거리 안쪽"}
        self.sig_progress.emit(f"장애물 {stop_cm:.0f}cm 앞까지 전진")
        self.cmd = (v, v)
        while True:
            yield
            d = self.dist_cm()
            trav = self.enc_travel(start)
            if d is not None and d <= stop_cm + 2:
                break
            if trav >= target:
                self.cmd = (0, 0)
                yield from self._wait(0.3)
                return {"ok": True, "reached": False, "moved_cm": round(self.deg_to_cm(trav), 1),
                        "distance_sensor_cm": self.dist_cm(),
                        "note": f"최대 {max_cm:.0f}cm 이동까지 장애물을 못 찾음 "
                                "(-1/None은 센서 범위 밖이거나 센서 없음)"}
            if time.monotonic() - t0 > timeout:
                self.cmd = (0, 0)
                return {"ok": False, "error": "시간 초과"}
            vv = v if (d is None or d > stop_cm + 15) else max(70, int(v * 0.5))
            self.cmd = (vv, vv)
        self.cmd = (0, 0)
        yield from self._wait(0.4)
        return {"ok": True, "reached": True,
                "moved_cm": round(self.deg_to_cm(self.enc_travel(start)), 1),
                "distance_sensor_cm": self.dist_cm()}

    def act_stop(self, **_):
        self.cmd = (0, 0)
        if self.g.link.connected:
            self.g.link.send("S")
        return {"ok": True}

    def act_wait(self, seconds=1, **_):
        sec = max(0.1, min(10.0, float(seconds)))
        yield from self._wait(sec)
        return {"ok": True, "waited_s": sec}

    def act_get_status(self, **_):
        g, tm = self.g, self.g.tm
        fresh = g.link.program_alive()
        return {"ok": True,
                "hub_connected": g.link.connected,
                "ready_to_drive": not bool(g.motion_block_reason()),
                "block_reason": g.motion_block_reason() or None,
                "telemetry_fresh": fresh,
                "yaw_deg": round(tm["yaw"], 1), "pitch_deg": round(tm["pitch"], 1),
                "roll_deg": round(tm["roll"], 1),
                "distance_sensor_cm": self.dist_cm(),
                "reflection_pct": tm["refl"] if tm["refl"] >= 0 else None,
                "battery_pct": tm["batt"] if tm["batt"] >= 0 else None,
                "camera": "수신 중" if g.camera.frame_fresh() else "영상 없음",
                "vision_mode": g.analysis.mode}

    def act_look(self, **_):
        return self.g.vision_snapshot()

    def act_set_vision_mode(self, mode, colors=None, **_):
        mode = str(mode).strip().lower()
        if mode not in VISION_MODES:
            return {"ok": False, "error": f"mode는 {VISION_MODES} 중 하나"}
        if mode != "raw" and not va.HAVE_CV:
            return {"ok": False, "error": "OpenCV가 없어 영상 분석을 쓸 수 없음"}
        if mode == "color" and colors:
            if isinstance(colors, str):
                colors = [c.strip() for c in re.split(r"[,\s]+", colors) if c.strip()]
            unknown = [c for c in colors if c not in self.g.analysis.color_checks]
            if unknown:
                return {"ok": False, "error": f"모르는 색 {unknown}. 가능한 색: {COLOR_NAMES}"}
            for name, ck in self.g.analysis.color_checks.items():
                ck.setChecked(name in colors)
        self.g.analysis.set_mode(mode)
        self.g.vision_result, self.g.vision_t = None, 0.0
        if mode == "raw":
            return {"ok": True, "mode": "raw"}
        if not self.g.camera.frame_fresh():
            return {"ok": True, "mode": mode, "warning": "카메라 영상이 없어 결과가 나오지 않음"}
        return self._look_after(1.2)

    def _look_after(self, sec):
        yield from self._wait(sec)
        return self.g.vision_snapshot()

    def act_turn_to_target(self, search=True, max_seconds=12, **_):
        if self.g.analysis.mode not in ("color", "shape", "qr"):
            return {"ok": False, "error": "먼저 set_vision_mode로 color/shape/qr 중 하나를 켜세요"}
        if not self.g.camera.frame_fresh():
            return {"ok": False, "error": "카메라 영상 없음"}
        return self._turn_to_target(bool(search), max(2.0, min(30.0, float(max_seconds))))

    def _turn_to_target(self, search, max_s):
        t0 = time.monotonic()
        searched = 0.0
        self.sig_progress.emit("목표 찾아 정렬")
        v = self.speed(30)
        while time.monotonic() - t0 < max_s:
            asked = time.monotonic()
            while self.g.vision_t < asked and time.monotonic() - asked < 1.5:
                yield                                  # 멈춘 상태의 새 분석 결과 대기
            r = self.g.vision_result or {}
            tgt = r.get("target") if self.g.vision_t >= asked else None
            if tgt:
                dx = float(tgt[0])
                if self.g.camera.mirror.isChecked():
                    dx = -dx                           # 화면 좌우 반전이면 실제 방향은 반대
                if abs(dx) < 0.12:
                    return {"ok": True, "aligned": True, "offset": round(dx, 2),
                            "summary": r.get("summary", ""), "yaw": round(self.g.tm["yaw"], 1)}
                sgn = 1 if dx > 0 else -1
                pulse = 0.08 + 0.22 * min(abs(dx), 1.0)
            elif search:
                if searched >= 360:
                    return {"ok": False, "found": False, "error": "한 바퀴 돌았지만 대상을 못 찾음"}
                sgn, pulse = 1, 0.25
            else:
                return {"ok": False, "found": False, "error": "화면에 대상이 없음"}
            y0 = self.g.tm["yaw"]
            self.cmd = (sgn * v, -sgn * v)
            yield from self._wait(pulse)
            self.cmd = (0, 0)
            yield from self._wait(0.35)
            if not tgt:
                searched += max(abs(wrap180(self.g.tm["yaw"] - y0)), 8.0)
        return {"ok": False, "error": "시간 초과", "summary": (self.g.vision_result or {}).get("summary", "")}


# =============================================================================
#  4. 대화 관리자 — LLM ↔ 도구 실행 반복
# =============================================================================
class Agent(QObject):
    sig_say = Signal(str)                 # AI 답변
    sig_tool = Signal(str, dict, dict)    # 이름, 인자, 결과
    sig_tool_start = Signal(str, dict)
    sig_state = Signal(str)               # idle / thinking / acting
    sig_log = Signal(str, str)

    MAX_ROUNDS = 10

    def __init__(self, gcs):
        super().__init__()
        self.g = gcs
        self.client = OllamaClient()
        self.client.sig_reply.connect(self.on_reply)
        self.client.sig_error.connect(self.on_error)
        self.exe = gcs.exe
        self.exe.sig_done.connect(self.on_tool_done)
        self.messages = []
        self.gen = 0
        self.rounds = 0
        self.queue = []
        self.json_mode = False
        self.state = "idle"

    def reset(self):
        self.cancel()
        self.messages = []
        self.json_mode = False

    def _system(self):
        s = SYSTEM_PROMPT
        if self.json_mode:
            s += JSON_MODE_ADDENDUM
        return {"role": "system", "content": s}

    def _trim(self):
        if len(self.messages) > 40:
            cut = len(self.messages) - 40
            while cut < len(self.messages) and self.messages[cut].get("role") != "user":
                cut += 1
            self.messages = self.messages[cut:]

    def _set(self, st):
        self.state = st
        self.sig_state.emit(st)

    def ask(self, text):
        if self.state != "idle":
            self.cancel()
        self.gen += 1
        self.rounds = 0
        self.queue = []
        self.messages.append({"role": "user", "content": text})
        self._trim()
        self._call()

    def _call(self):
        self.rounds += 1
        if self.rounds > self.MAX_ROUNDS:
            self.sig_say.emit("동작 단계가 너무 많아 여기서 멈춥니다. 명령을 나눠 주세요.")
            self._set("idle")
            return
        self._set("thinking")
        model = self.g.cb_model.currentText().strip()
        msgs = json.loads(json.dumps([self._system()] + self.messages, ensure_ascii=False))
        self.client.base = self.g.ed_url.text().strip() or "http://127.0.0.1:11434"
        self.client.chat(self.gen, model, msgs, None if self.json_mode else TOOLS,
                         think_off=not self.g.ck_think.isChecked())

    def on_error(self, gen, code, msg):
        if gen != self.gen:
            return
        if code == "NO_TOOLS" and not self.json_mode:
            self.json_mode = True
            self.sig_log.emit("이 모델은 도구 호출을 지원하지 않아 JSON 방식으로 전환합니다.", "warn")
            self.rounds -= 1
            self._call()
            return
        if code == "NO_MODEL":
            text = f"모델을 찾을 수 없습니다: {msg}\n→ 명령 프롬프트에서 ollama pull {self.g.cb_model.currentText()}"
        elif code == "NET":
            text = f"Ollama에 연결하지 못했습니다: {msg}\n→ Ollama가 실행 중인지, 주소가 맞는지 확인하세요."
        else:
            text = f"Ollama 오류: {msg}"
        self.sig_log.emit(text, "err")
        self.sig_say.emit("⚠ " + text)
        self._set("idle")

    def on_reply(self, gen, data):
        if gen != self.gen:
            return
        msg = data.get("message") or {}
        content = strip_think(msg.get("content", ""))
        calls = msg.get("tool_calls") or []
        say = None
        if not calls and content:
            calls, say = parse_text_calls(content)
        hist = {"role": "assistant", "content": content}
        if msg.get("tool_calls"):
            hist["tool_calls"] = msg["tool_calls"]
        self.messages.append(hist)
        if msg.get("tool_calls") and content:
            self.sig_say.emit(content)
        if say:
            self.sig_say.emit(say)
        if calls:
            self.queue = list(calls)
            self._next_tool()
            return
        if content and not say:
            self.sig_say.emit(content)
        elif not content:
            self.sig_say.emit("(응답 없음)")
        self._set("idle")

    def _next_tool(self):
        if not self.queue:
            self._call()                          # 결과를 보고 LLM이 다음 판단
            return
        call = self.queue.pop(0)
        fn = call.get("function", {})
        name = fn.get("name", "")
        args = fn.get("arguments") or {}
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except ValueError:
                args = {}
        self._cur = (name, args)
        self._set("acting")
        self.sig_tool_start.emit(name, args)
        self.exe.start(name, args)

    def on_tool_done(self, name, result):
        if self.state != "acting":
            return
        cname, args = getattr(self, "_cur", (name, {}))
        self.sig_tool.emit(cname, args, result)
        content = json.dumps(result, ensure_ascii=False)
        if self.json_mode:
            self.messages.append({"role": "user", "content": f"[도구 결과 {cname}] {content}"})
        else:
            self.messages.append({"role": "tool", "content": content, "tool_name": cname})
        if not result.get("ok", True):
            self.queue = []                       # 실패하면 남은 동작 취소, LLM에 보고
        self._next_tool()

    def cancel(self, estop=False):
        self.gen += 1
        self.queue = []
        busy = self.exe.busy()
        self.exe.abort("사용자 중지", estop=estop, notify=False)
        if self.state != "idle" or busy:
            self.messages.append({"role": "user", "content": "[시스템] 사용자가 동작을 중지했다."})
        self._set("idle")


# =============================================================================
#  5. 메인 윈도우
# =============================================================================
AI_STYLE = STYLE + """
QTextBrowser { background:#0b111a; border:1px solid #1e2a3a; border-radius:6px;
               padding:6px; font-size:13px; }
QLineEdit#prompt { font-size:14px; padding:8px 10px; }
QPushButton#ex { background:#0e141d; color:#9fc3e6; font-weight:500; padding:4px 8px;
                 text-align:left; }
QPushButton#ex:hover { background:#16202c; }
QScrollArea { border:none; background:transparent; }
"""

EXAMPLES = [
    "앞으로 30cm 가줘",
    "오른쪽으로 90도 돌아",
    "앞에 벽이 15cm 남을 때까지 천천히 전진해",
    "지금 상태 알려줘",
    "빨간색을 찾아서 그쪽을 봐",
    "QR 코드 읽어줘",
    "사각형을 그리듯이 20cm 이동하고 90도 회전을 4번 해",
]


class AiGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"SPIKE Rover AI GCS — Ollama ({APP_VERSION})")
        self.resize(1560, 900)
        self.setMinimumSize(1200, 680)
        self.setStyleSheet(AI_STYLE)
        self.settings = QSettings("SPIKE-Rover", "AI-GCS")

        self.link = BleLink()
        self.tm = dict(yaw=0.0, pitch=0.0, roll=0.0, lvel=0, rvel=0,
                       lpos=0, rpos=0, dist=-1, refl=-1, batt=-1, hold=0, ms=0)
        self.tm_t = 0.0
        self.tm_seq = 0
        self.vision_result = None
        self.vision_t = 0.0
        self._dev_addrs = []

        self._build_ui()
        self.exe = Executor(self)
        self.exe.sig_progress.connect(lambda s: self.lb_action.setText("▶ " + s))
        self.agent = Agent(self)
        self.agent.sig_say.connect(self.chat_ai)
        self.agent.sig_tool.connect(self.chat_tool)
        self.agent.sig_tool_start.connect(self.chat_tool_start)
        self.agent.sig_state.connect(self.on_agent_state)
        self.agent.sig_log.connect(self.log)
        self.agent.client.sig_models.connect(self.on_models)

        self.link.sig_line.connect(self.on_line)
        self.link.sig_log.connect(self.log)
        self.link.sig_conn.connect(self.on_conn)
        self.link.sig_devices.connect(self.on_devices)
        self.camera.log.connect(self.log)
        self.camera.lost.connect(self.on_camera_lost)
        self.analysis.log.connect(self.log)
        if getattr(self.analysis, "thread", None) is not None:
            self.analysis.worker.done.connect(self.on_vision)

        self.t_ui = QTimer(self); self.t_ui.timeout.connect(self.tick_ui); self.t_ui.start(200)
        self.t_probe = QTimer(self); self.t_probe.timeout.connect(self.tick_probe); self.t_probe.start(1000)
        QApplication.instance().installEventFilter(self)

        self.chat_sys("① 허브 [스캔] → [연결] → [프로그램 시작]  ② Ollama 모델 선택  "
                      "③ [AI 주행 허용] 체크  ④ 아래 입력칸에 명령. 비상정지: Esc")
        if not HAVE_BLEAK:
            self.log("bleak 불러오기 실패 — " + (BLEAK_ERROR or "pip install bleak"), "err")
        QTimer.singleShot(300, self.refresh_models)

    # ---------------------------------------------------------------- UI 조립
    def _group(self, title):
        g = QGroupBox(title)
        return g

    def _kv(self, caption):
        box = QVBoxLayout(); box.setSpacing(0); box.setContentsMargins(6, 2, 6, 2)
        cap = QLabel(caption); cap.setObjectName("cap")
        val = QLabel("--"); val.setObjectName("val")
        val.setStyleSheet("font-size:14px;")
        box.addWidget(cap); box.addWidget(val)
        fr = QFrame(); fr.setLayout(box); fr.setObjectName("kv")
        fr.setStyleSheet("QFrame#kv{background:#0e141d;border:1px solid #1e2a3a;border-radius:5px;}")
        return fr, val

    def _chip(self, text):
        lb = QLabel(text); lb.setObjectName("chip"); lb.setAlignment(Qt.AlignCenter)
        return lb

    @staticmethod
    def set_chip(lb, text, css=""):
        lb.setText(text)
        lb.setMinimumWidth(lb.fontMetrics().horizontalAdvance(text) + 26)
        lb.setStyleSheet(css)

    def _build_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        outer = QVBoxLayout(central); outer.setContentsMargins(8, 6, 8, 8); outer.setSpacing(6)

        # ---- 상단 ----
        top = QHBoxLayout()
        title = QLabel("🤖  SPIKE ROVER · AI GCS")
        title.setStyleSheet("font-size:15px;font-weight:700;")
        self.chip_link = self._chip("허브 미연결")
        self.chip_prog = self._chip("프로그램 ?")
        self.chip_ai = self._chip("AI 대기")
        self.chip_llm = self._chip("Ollama ?")
        btn_stop = QPushButton("■ 비상정지 (Esc)"); btn_stop.setObjectName("danger")
        btn_stop.setFocusPolicy(Qt.NoFocus); btn_stop.clicked.connect(self.estop)
        top.addWidget(title); top.addSpacing(12)
        for c in (self.chip_link, self.chip_prog, self.chip_llm, self.chip_ai):
            top.addWidget(c)
        top.addStretch(1); top.addWidget(btn_stop)
        outer.addLayout(top)

        split = QSplitter(Qt.Horizontal); split.setChildrenCollapsible(False); split.setHandleWidth(6)
        outer.addWidget(split, 1)

        # ================= 1열: 연결 · Ollama · 설정 · 센서 =================
        col1 = QWidget(); c1 = QVBoxLayout(col1); c1.setContentsMargins(0, 0, 4, 0); c1.setSpacing(6)

        g = self._group("SPIKE 허브 (BLE)"); gl = QGridLayout(g); gl.setSpacing(5)
        b_scan = QPushButton("스캔"); b_scan.setObjectName("primary")
        b_scan.clicked.connect(self.do_scan)
        b_conn = QPushButton("연결"); b_conn.setObjectName("ok"); b_conn.clicked.connect(self.do_connect)
        b_disc = QPushButton("해제"); b_disc.clicked.connect(self.do_disconnect)
        gl.addWidget(b_scan, 0, 0); gl.addWidget(b_conn, 0, 1); gl.addWidget(b_disc, 0, 2)
        self.lst_dev = QListWidget(); self.lst_dev.setMaximumHeight(70)
        self.lst_dev.itemDoubleClicked.connect(lambda _i: self.do_connect())
        gl.addWidget(self.lst_dev, 1, 0, 1, 3)
        gl.addWidget(QLabel("슬롯"), 2, 0)
        self.sp_slot = QSpinBox(); self.sp_slot.setRange(0, 19)
        self.sp_slot.setValue(int(self.settings.value("slot", 0)))
        gl.addWidget(self.sp_slot, 2, 1)
        b_run = QPushButton("▶ 프로그램 시작"); b_run.clicked.connect(self.start_program)
        gl.addWidget(b_run, 2, 2)
        c1.addWidget(g)

        g = self._group("Ollama"); gl = QGridLayout(g); gl.setSpacing(5)
        self.ed_url = QLineEdit(self.settings.value("url", "http://127.0.0.1:11434"))
        gl.addWidget(QLabel("주소"), 0, 0); gl.addWidget(self.ed_url, 0, 1, 1, 2)
        self.cb_model = QComboBox(); self.cb_model.setEditable(True)
        saved_model = self.settings.value("model", "qwen2.5:7b")
        self.cb_model.addItem(saved_model)
        b_ref = QPushButton("↻"); b_ref.setFixedWidth(34); b_ref.setToolTip("모델 목록 새로고침")
        b_ref.clicked.connect(self.refresh_models)
        gl.addWidget(QLabel("모델"), 1, 0); gl.addWidget(self.cb_model, 1, 1); gl.addWidget(b_ref, 1, 2)
        self.ck_think = QCheckBox("생각 모드 (qwen3 · 느려짐)")
        self.ck_think.setChecked(self.settings.value("think", "false") in (True, "true"))
        gl.addWidget(self.ck_think, 2, 0, 1, 3)
        c1.addWidget(g)

        g = self._group("AI 주행 설정"); gl = QGridLayout(g); gl.setSpacing(5)
        self.sp_maxspd = QSpinBox(); self.sp_maxspd.setRange(100, 1000); self.sp_maxspd.setSingleStep(50)
        self.sp_maxspd.setSuffix(" °/s"); self.sp_maxspd.setValue(int(self.settings.value("maxspd", 400)))
        self.sp_maxspd.setToolTip("속도 100%일 때의 바퀴 속도. 허브 MAX_VELOCITY(기본 500)를 넘지 않게")
        self.sp_wheel = QDoubleSpinBox(); self.sp_wheel.setRange(2.0, 20.0); self.sp_wheel.setDecimals(1)
        self.sp_wheel.setSuffix(" cm"); self.sp_wheel.setValue(float(self.settings.value("wheel", 5.6)))
        self.sp_wheel.setToolTip("바퀴 지름. SPIKE 기본 바퀴 5.6cm. 거리 오차가 크면 실측해서 조정")
        self.sp_guard = QSpinBox(); self.sp_guard.setRange(0, 60); self.sp_guard.setSuffix(" cm")
        self.sp_guard.setValue(int(self.settings.value("guard", 8)))
        self.sp_guard.setToolTip("전진 중 거리센서가 이 값보다 가까우면 자동 정지 (0=끔)")
        gl.addWidget(QLabel("최대 속도"), 0, 0); gl.addWidget(self.sp_maxspd, 0, 1)
        gl.addWidget(QLabel("바퀴 지름"), 1, 0); gl.addWidget(self.sp_wheel, 1, 1)
        gl.addWidget(QLabel("장애물 정지"), 2, 0); gl.addWidget(self.sp_guard, 2, 1)
        c1.addWidget(g)

        g = self._group("센서"); gl = QGridLayout(g); gl.setSpacing(4)
        cells = [("방위 YAW", "v_yaw"), ("PITCH", "v_pit"), ("ROLL", "v_rol"),
                 ("거리 cm", "v_dist"), ("반사광", "v_refl"), ("배터리", "v_batt")]
        for i, (cap, attr) in enumerate(cells):
            fr, val = self._kv(cap); setattr(self, attr, val)
            gl.addWidget(fr, i // 3, i % 3)
        c1.addWidget(g)
        c1.addStretch(1)

        sc = QScrollArea(); sc.setWidgetResizable(True); sc.setWidget(col1)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sc.setMinimumWidth(290)
        split.addWidget(sc)

        # ================= 2열: 영상 + 분석 =================
        col2 = QSplitter(Qt.Vertical); col2.setChildrenCollapsible(False)
        self.camera = CameraPanel()
        self.analysis = AnalysisPanel(self.camera)
        col2.addWidget(self.camera); col2.addWidget(self.analysis)
        col2.setStretchFactor(0, 1); col2.setStretchFactor(1, 1)
        self.col2 = col2
        split.addWidget(col2)

        # ================= 3열: AI 대화 =================
        col3 = QSplitter(Qt.Vertical); col3.setChildrenCollapsible(False)
        chat = QGroupBox("AI 명령 · Ollama"); cl = QVBoxLayout(chat); cl.setSpacing(6)
        self.chat = QTextBrowser(); self.chat.setOpenExternalLinks(False)
        cl.addWidget(self.chat, 1)
        ex = QHBoxLayout(); ex.setSpacing(4)
        exw = QWidget(); exw.setObjectName("clear")
        exg = QGridLayout(exw); exg.setContentsMargins(0, 0, 0, 0); exg.setSpacing(4)
        for i, t in enumerate(EXAMPLES):
            b = QPushButton(t); b.setObjectName("ex"); b.setFocusPolicy(Qt.NoFocus)
            b.clicked.connect(lambda _c=False, s=t: self.ed_prompt.setText(s) or self.ed_prompt.setFocus())
            exg.addWidget(b, i // 2, i % 2)
        cl.addWidget(exw)
        self.lb_action = QLabel("대기 중")
        self.lb_action.setStyleSheet("color:#ffd24a;font-weight:700;font-family:'D2Coding',monospace;")
        cl.addWidget(self.lb_action)
        row = QHBoxLayout()
        self.ck_allow = QCheckBox("AI 주행 허용")
        self.ck_allow.setStyleSheet("QCheckBox{color:#3ddc84;font-weight:700;}")
        self.ck_allow.setFocusPolicy(Qt.NoFocus)
        self.ck_allow.toggled.connect(self.on_allow)
        b_new = QPushButton("새 대화"); b_new.setFocusPolicy(Qt.NoFocus)
        b_new.clicked.connect(self.new_chat)
        row.addWidget(self.ck_allow); row.addStretch(1); row.addWidget(b_new)
        cl.addLayout(row)
        row = QHBoxLayout()
        self.ed_prompt = QLineEdit(); self.ed_prompt.setObjectName("prompt")
        self.ed_prompt.setPlaceholderText("로버에게 할 명령을 입력하고 Enter  (예: 앞으로 30cm 가줘)")
        self.ed_prompt.returnPressed.connect(self.send_prompt)
        self.btn_send = QPushButton("보내기"); self.btn_send.setObjectName("primary")
        self.btn_send.clicked.connect(self.send_prompt)
        self.btn_halt = QPushButton("중지"); self.btn_halt.clicked.connect(lambda: self.agent.cancel())
        row.addWidget(self.ed_prompt, 1); row.addWidget(self.btn_send); row.addWidget(self.btn_halt)
        cl.addLayout(row)
        col3.addWidget(chat)

        lg = QGroupBox("통신 로그"); ll = QVBoxLayout(lg)
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True)
        ll.addWidget(self.txt_log)
        col3.addWidget(lg)
        col3.setStretchFactor(0, 4); col3.setStretchFactor(1, 1)
        split.addWidget(col3)

        split.setStretchFactor(0, 0); split.setStretchFactor(1, 3); split.setStretchFactor(2, 4)
        split.setSizes([300, 520, 700])

    def showEvent(self, e):
        super().showEvent(e)
        QTimer.singleShot(0, self._balance)

    def _balance(self):
        # 분석 옵션(색상 체크·슬라이더)이 눌리지 않도록 영상 칸 높이를 제한
        h = self.col2.height()
        need = self.analysis.sizeHint().height() + 40
        cam = max(300, h - need)
        self.col2.setSizes([cam, max(200, h - cam)])

    # ------------------------------------------------------------------ 로그
    COLORS = dict(info="#7f93ab", tx="#5ad1a0", rx="#7fb4ff", warn="#ffc46b", err="#ff7c7c")

    def log(self, msg, level="info"):
        col = self.COLORS.get(level, "#7f93ab")
        ts = time.strftime("%H:%M:%S")
        for line in str(msg).split("\n"):
            self.txt_log.append(f'<span style="color:#44566a">[{ts}]</span> '
                                f'<span style="color:{col}">{html.escape(line)}</span>')
        sb = self.txt_log.verticalScrollBar(); sb.setValue(sb.maximum())

    # ------------------------------------------------------------------ 대화창
    def _chat(self, htm):
        self.chat.append(htm)
        sb = self.chat.verticalScrollBar(); sb.setValue(sb.maximum())

    def chat_user(self, text):
        self._chat(f'<p style="margin:8px 0 2px 0"><span style="color:#3fb9ff;font-weight:700">나 ▸</span> '
                   f'<span style="color:#dce6f2">{html.escape(text)}</span></p>')

    def chat_ai(self, text):
        body = html.escape(text).replace("\n", "<br>")
        self._chat(f'<p style="margin:2px 0 6px 0"><span style="color:#3ddc84;font-weight:700">AI ▸</span> '
                   f'<span style="color:#e8f0f8">{body}</span></p>')

    def chat_sys(self, text):
        self._chat(f'<p style="color:#7f93ab;font-size:11px">{html.escape(text)}</p>')

    @staticmethod
    def _fmt_args(args):
        return ", ".join(f"{k}={v}" for k, v in (args or {}).items())

    def chat_tool_start(self, name, args):
        self.lb_action.setText(f"▶ {name}({self._fmt_args(args)})")

    def chat_tool(self, name, args, result):
        ok = result.get("ok", True)
        col = "#5ad1a0" if ok else "#ffb020"
        short = {k: v for k, v in result.items() if k not in ("ok", "items") and v not in (None, "")}
        txt = ", ".join(f"{k}={v}" for k, v in short.items())
        if len(txt) > 180:
            txt = txt[:180] + "…"
        self._chat(f'<p style="margin:0;font-family:D2Coding,Consolas,monospace;font-size:11px;color:{col}">'
                   f'  🔧 {html.escape(name)}({html.escape(self._fmt_args(args))}) '
                   f'→ {"OK" if ok else "중단"} {html.escape(txt)}</p>')

    def on_agent_state(self, st):
        label = {"idle": "AI 대기", "thinking": "AI 생각 중…", "acting": "AI 실행 중"}.get(st, st)
        css = {"thinking": "QLabel#chip{background:#c77dff;color:#12001e;}",
               "acting": "QLabel#chip{background:#ffb020;color:#1a1000;}"}.get(st, "")
        self.set_chip(self.chip_ai, label, css)
        if st == "idle":
            self.lb_action.setText("대기 중")
        elif st == "thinking":
            self.lb_action.setText("… Ollama 응답 대기")

    def send_prompt(self):
        text = self.ed_prompt.text().strip()
        if not text:
            return
        self.ed_prompt.clear()
        self.chat_user(text)
        self.save_settings()
        self.agent.ask(text)

    def new_chat(self):
        self.agent.reset()
        self.chat.clear()
        self.chat_sys("새 대화를 시작합니다.")

    def on_allow(self, on):
        if not on:
            self.agent.cancel()
        self.log("AI 주행 허용 " + ("ON — AI 명령으로 로버가 움직입니다" if on else "OFF"),
                 "warn" if on else "info")

    # ------------------------------------------------------------------ Ollama
    def refresh_models(self):
        self.agent.client.base = self.ed_url.text().strip() or "http://127.0.0.1:11434"
        self.set_chip(self.chip_llm, "Ollama 확인 중…")
        self.agent.client.list_models()

    def on_models(self, names, err):
        if err:
            self.set_chip(self.chip_llm, "Ollama 연결 안 됨", "QLabel#chip{background:#ff5a5a;color:#fff;}")
            self.log(f"Ollama 연결 실패: {err} — Ollama 앱이 실행 중인지 확인하세요.", "err")
            return
        cur = self.cb_model.currentText()
        names = sorted(names, key=lambda n: (not n.startswith(("qwen", "llama3")), n))
        self.cb_model.clear(); self.cb_model.addItems(names)
        if cur in names:
            self.cb_model.setCurrentText(cur)
        elif names:
            self.cb_model.setCurrentIndex(0)
        else:
            self.cb_model.setEditText(cur)
            self.log("설치된 모델이 없습니다. 명령 프롬프트에서: ollama pull qwen2.5:7b", "warn")
        self.set_chip(self.chip_llm, f"Ollama · {len(names)}개 모델",
                      "QLabel#chip{background:#3ddc84;color:#04140b;}")
        self.log(f"Ollama 모델 {len(names)}개: {', '.join(names[:8])}", "info")

    # ------------------------------------------------------------------ 허브
    def do_scan(self):
        self.lst_dev.clear()
        self.link.scan(False)

    def on_devices(self, devs):
        self.lst_dev.clear()
        self._dev_addrs = [d[1] for d in devs]
        for name, addr, rssi in devs:
            QListWidgetItem(f"{name}  [{addr}]  {rssi} dBm", self.lst_dev)
        if devs:
            self.lst_dev.setCurrentRow(0)

    def do_connect(self):
        if self.link.connected:
            self.log("이미 연결되어 있습니다.", "warn"); return
        row = self.lst_dev.currentRow()
        if row < 0 or row >= len(self._dev_addrs):
            self.log("먼저 [스캔] 후 허브를 선택하세요.", "warn"); return
        self.link.connect_to(self._dev_addrs[row], False, False)

    def do_disconnect(self):
        self.agent.cancel()
        self.link.disconnect()

    def start_program(self):
        self.settings.setValue("slot", self.sp_slot.value())
        self.link.program_flow(False, self.sp_slot.value())

    def on_conn(self, ok, label):
        if ok:
            self.set_chip(self.chip_link, f"BLE · {label[:17]}",
                          "QLabel#chip{background:#3ddc84;color:#04140b;}")
        else:
            self.agent.cancel()
            self.set_chip(self.chip_link, "허브 미연결")

    def tick_probe(self):
        # 허브 프로그램이 텔레메트리를 보내는데 Q 응답 확인 전이면 Q로 준비 상태를 확인
        if self.link.connected and self.link.program_alive() and not self.link.command_ready:
            self.link.send("Q")

    def on_line(self, line):
        if line.startswith("#T,"):
            p = line[3:].split(",")
            if len(p) >= 12:
                try:
                    y, pi, r, lv, rv, lp, rp, dist, refl, batt, hold, ms = p[:12]
                    self.tm.update(yaw=float(y), pitch=float(pi), roll=float(r),
                                   lvel=int(float(lv)), rvel=int(float(rv)),
                                   lpos=int(float(lp)), rpos=int(float(rp)),
                                   dist=int(float(dist)), refl=int(float(refl)),
                                   batt=int(float(batt)), hold=int(float(hold)), ms=int(float(ms)))
                    self.tm_t = time.monotonic()
                    self.tm_seq += 1
                except ValueError:
                    pass
        elif line.startswith("#E,"):
            self.log("허브 경고: " + line[3:], "warn")
        elif line.startswith("#I,"):
            self.log("허브: " + line[3:], "rx")

    def tick_ui(self):
        tm = self.tm
        self.v_yaw.setText(f'{tm["yaw"]:.1f}°')
        self.v_pit.setText(f'{tm["pitch"]:.1f}°')
        self.v_rol.setText(f'{tm["roll"]:.1f}°')
        self.v_dist.setText("--" if tm["dist"] < 0 else f'{tm["dist"] / 10:.1f}')
        self.v_refl.setText("--" if tm["refl"] < 0 else str(tm["refl"]))
        self.v_batt.setText("--" if tm["batt"] < 0 else f'{tm["batt"]}%')
        alive = self.link.program_alive()
        if not self.link.connected:
            self.set_chip(self.chip_prog, "프로그램 ?")
        elif alive and self.link.command_ready:
            self.set_chip(self.chip_prog, "조종 준비 완료", "QLabel#chip{background:#3ddc84;color:#04140b;}")
        elif alive:
            self.set_chip(self.chip_prog, "Q 응답 대기", "QLabel#chip{background:#ffb020;color:#1a1000;}")
        else:
            self.set_chip(self.chip_prog, "텔레메트리 없음 · 프로그램 시작",
                          "QLabel#chip{background:#ffb020;color:#1a1000;}")

    # ------------------------------------------------------------------ 안전
    def motion_block_reason(self):
        if not self.ck_allow.isChecked():
            return "AI 주행 허용이 꺼져 있음 (GCS의 [AI 주행 허용] 체크 필요)"
        if not self.link.connected:
            return "허브가 연결되어 있지 않음"
        if self.link.safe_mode:
            return "안전 모드(전송 차단)"
        if not self.link.program_alive():
            return "허브 프로그램 텔레메트리 없음 (허브 프로그램 실행 필요)"
        if not self.link.command_ready:
            return "허브 Q 응답 대기 중"
        if not self.camera.motion_allowed():
            return "영상이 끊겨 주행 차단됨 (영상 해제 또는 복구 필요)"
        return ""

    def estop(self):
        self.agent.cancel(estop=True)
        if self.link.connected:
            self.link.send("E")
        self.log("*** 비상정지 ***", "err")
        self.chat_sys("■ 비상정지")

    def on_camera_lost(self):
        if self.exe.busy():
            self.estop()
            self.log("영상 끊김: AI 동작 중지", "warn")

    # ------------------------------------------------------------------ 영상
    def on_vision(self, _image, result):
        if "error" in result or result.get("mode") != self.analysis.mode:
            return
        self.vision_result = result
        self.vision_t = time.monotonic()

    def vision_snapshot(self):
        mode = self.analysis.mode
        if mode == "raw":
            return {"ok": True, "mode": "raw", "note": "영상 분석이 꺼져 있음. set_vision_mode로 켜기"}
        if not self.camera.frame_fresh():
            return {"ok": False, "mode": mode, "error": "카메라 영상 없음"}
        r = self.vision_result
        if not r or time.monotonic() - self.vision_t > 2.0:
            return {"ok": False, "mode": mode, "error": "분석 결과 대기 중(잠시 후 다시 look)"}
        out = {"ok": True, "mode": mode, "summary": r.get("summary", ""),
               "items": list(r.get("items", []))[:8]}
        tgt = r.get("target")
        if tgt:
            dx = float(tgt[0])
            if self.camera.mirror.isChecked():
                dx = -dx
            out["target_offset"] = round(dx, 2)
            out["target_direction"] = va.horizontal_word(dx)
        if mode == "qr":
            out["qr_codes"] = list(r.get("codes", []))
        return out

    # ------------------------------------------------------------------ 키보드
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and self.isActiveWindow():
            if event.key() == Qt.Key_Escape:
                self.estop(); return True
            if event.key() == Qt.Key_Space and not isinstance(
                    QApplication.focusWidget(), (QLineEdit, QTextEdit, QAbstractSpinBox)):
                self.estop(); return True
        return super().eventFilter(obj, event)

    def save_settings(self):
        s = self.settings
        s.setValue("url", self.ed_url.text().strip())
        s.setValue("model", self.cb_model.currentText().strip())
        s.setValue("think", "true" if self.ck_think.isChecked() else "false")
        s.setValue("maxspd", self.sp_maxspd.value())
        s.setValue("wheel", self.sp_wheel.value())
        s.setValue("guard", self.sp_guard.value())
        s.setValue("slot", self.sp_slot.value())

    def closeEvent(self, e):
        self.save_settings()
        try:
            self.agent.cancel(estop=True)
        except Exception:
            pass
        self.analysis.shutdown()
        self.camera.stop()
        self.link.disconnect()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SPIKE Rover AI GCS")
    win = AiGCS()
    avail = app.primaryScreen().availableGeometry() if app.primaryScreen() else None
    if avail is not None and (avail.width() < 1600 or avail.height() < 920):
        win.showMaximized()
    else:
        win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        input("오류가 발생했습니다. Enter 를 누르면 종료합니다…")
