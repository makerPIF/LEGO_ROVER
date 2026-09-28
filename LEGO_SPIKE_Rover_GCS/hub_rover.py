# SPIKE Rover GCS — tunnel-v2 (2026-09-21)
# Upload this whole file into a SPIKE App 3 Python slot, then run it.
# BLE: hub.config['module_tunnel'].callback receives payload bytes;
#      module_tunnel.send sends telemetry and acknowledgements back.
# The callback only queues bytes. The runloop parses commands and drives motors.
# No automatic motor movement at startup. A/B drive ports by default.
# USB stdin is an optional fallback, not the BLE receive path.
# Module tunnel is a firmware-internal API; availability is checked at startup.

import sys
import hub as _hub
import time
import motor
import motor_pair
import runloop
from hub import port, motion_sensor, light_matrix

ROVER_VERSION = "tunnel-v2"
_tunnel = None
_tunnel_error = ""
try:
    _candidate = _hub.config["module_tunnel"]
    if not callable(getattr(_candidate, "callback", None)) or not callable(getattr(_candidate, "send", None)):
        raise RuntimeError("module_tunnel callback/send unavailable")
    _tunnel = _candidate
except Exception as exc:
    _tunnel_error = str(exc)

# ---- 있으면 쓰고 없으면 넘어가는 것들 --------------------------------------
try:
    import distance_sensor
except ImportError:
    distance_sensor = None
try:
    import color_sensor
except ImportError:
    color_sensor = None
try:
    import select
except ImportError:
    try:
        import uselect as select
    except ImportError:
        select = None
try:
    from hub import battery as _battery
except ImportError:
    _battery = None


# ============================ 사용자 설정 ====================================
LEFT_PORT_ID = 0          # 0=A 1=B 2=C 3=D 4=E 5=F   좌측 주행모터
RIGHT_PORT_ID = 1         #                            우측 주행모터
DIST_PORT_ID = 4          # 거리센서 (없으면 -1 이 나갈 뿐 문제 없음)
COLOR_PORT_ID = 5         # 컬러센서

INVERT_LEFT = 0           # 좌우 대칭 조립이면 보통 우측만 1
INVERT_RIGHT = 1

MAX_VELOCITY = 500        # deg/s (라지 1050 / 미디엄 1110 이 상한)
#                           ★ 낮게 시작하세요. 전류 돌입이 허브를 꺼뜨립니다.
ACCEL = 400               # 가속도 제한. 낮을수록 전류 돌입이 작다 (기본 1000)
TELEMETRY_MS = 250        # 4Hz — BLE 로 쓸 때 허브 송신 부하를 줄인다.
#                           USB 로만 쓸 거면 100(4Hz)으로 낮춰도 좋다.
WATCHDOG_MS = 800         # 이 시간 동안 명령이 없으면 정지

# ★ 시동 점검은 기본 꺼짐.
#   두 모터를 동시에 기동하면 전류가 튀어 배터리가 약한 허브는 그대로 꺼집니다.
#   배선을 확인할 때만 잠깐 True 로 바꾸고, 바퀴를 공중에 띄우고 실행하세요.
#   (True 로 해도 이제는 한쪽씩, 천천히, 절반 속도로만 돕니다)
STARTUP_JOG = False
# ============================================================================

PORTS = (port.A, port.B, port.C, port.D, port.E, port.F)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def emit(line):
    # Tunnel payload has its own message length at the BLE RPC layer.
    # Keep newline framing inside the payload for the GCS line parser.
    if _tunnel is not None:
        try:
            _tunnel.send((line + "\n").encode("utf-8"))
            return
        except Exception:
            # A temporarily disconnected peer must not kill the control loop.
            pass
    print(line)



def tilt():
    """(yaw, pitch, roll) 도(deg) 단위. 펌웨어는 데시도(1/10도)로 준다."""
    try:
        y, p, r = motion_sensor.tilt_angles()
        return (y / 10.0, p / 10.0, r / 10.0)
    except Exception:
        return (0.0, 0.0, 0.0)


def read_battery():
    if _battery is None:
        return -1
    try:
        return int(_battery.charge())
    except Exception:
        return -1


def battery_report():
    """허브가 꺼지는 원인이 배터리인지 판별하려면 전압이 필요하다."""
    if _battery is None:
        return "batt=n/a"
    out = []
    for name in ("charge", "voltage", "current", "temperature"):
        try:
            out.append("%s=%s" % (name, getattr(_battery, name)()))
        except Exception:
            pass
    return ",".join(out) if out else "batt=n/a"


def read_distance():
    if distance_sensor is None:
        return -1
    try:
        return int(distance_sensor.distance(PORTS[DIST_PORT_ID]))
    except Exception:
        return -1


def read_reflection():
    if color_sensor is None:
        return -1
    try:
        return int(color_sensor.reflection(PORTS[COLOR_PORT_ID]))
    except Exception:
        return -1


def motor_state(pid):
    try:
        return motor.velocity(PORTS[pid]), motor.relative_position(PORTS[pid])
    except Exception:
        return 0, 0


# ============================== 로버 =========================================
class Rover:
    def __init__(self):
        self.left_id = LEFT_PORT_ID
        self.right_id = RIGHT_PORT_ID
        self.inv_l = INVERT_LEFT
        self.inv_r = INVERT_RIGHT
        self.max_v = MAX_VELOCITY

        self.cmd_left = 0
        self.cmd_right = 0
        self.out_left = 0
        self.out_right = 0

        self.hold = False
        self.target_yaw = 0.0
        self.kp = 2.5
        self.ki = 0.0
        self.kd = 0.9
        self._i = 0.0
        self._prev_err = 0.0
        self._prev_t = time.ticks_ms()

        self.last_cmd_ms = time.ticks_ms()
        self.paired = False
        self.estop = False
        self.aux_active = {}
        self.test_deadline = None
        self._last_drive = None

    def pair(self):
        try:
            motor_pair.pair(motor_pair.PAIR_1,
                            PORTS[self.left_id], PORTS[self.right_id])
            self.paired = True
        except Exception as e:
            self.paired = False
            emit("#E,pair_fail:" + str(e))

    def drive(self, l, r):
        l = clamp(int(l), -self.max_v, self.max_v)
        r = clamp(int(r), -self.max_v, self.max_v)
        if (l, r) == self._last_drive:
            return
        if l == 0 and r == 0:
            # 모터에 0속도 구동 명령을 반복하지 않는다.
            for pid in (self.left_id, self.right_id):
                try:
                    motor.stop(PORTS[pid], stop=motor.BRAKE)
                except Exception:
                    pass
            self.out_left = self.out_right = 0
            self._last_drive = (0, 0)
            return
        self.out_left, self.out_right = l, r
        sl = -l if self.inv_l else l
        sr = -r if self.inv_r else r
        try:
            if self.paired:
                motor_pair.move_tank(motor_pair.PAIR_1, sl, sr, acceleration=ACCEL)
            else:
                motor.run(PORTS[self.left_id], sl, acceleration=ACCEL)
                motor.run(PORTS[self.right_id], sr, acceleration=ACCEL)
            self._last_drive = (l, r)
        except Exception as e:
            self.estop = True
            self.stop(coast=True)
            emit("#E,drive:" + str(e))

    def stop(self, coast=False):
        self.cmd_left = self.cmd_right = 0
        self.out_left = self.out_right = 0
        self.hold = False
        self.test_deadline = None
        self._last_drive = (0, 0)
        mode = motor.COAST if coast else motor.BRAKE
        # 독립적으로 정지: 한 포트 실패가 다른 모터의 정지를 막지 않는다.
        for pid in set([self.left_id, self.right_id] + list(self.aux_active)):
            try:
                motor.stop(PORTS[pid], stop=mode)
            except Exception as exc:
                emit("#E,stop_port=%d:%s" % (pid, str(exc)))
        self.aux_active.clear()

    def heading_correction(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._prev_t) / 1000.0
        self._prev_t = now
        if dt <= 0:
            dt = 0.01
        err = wrap180(self.target_yaw - tilt()[0])
        self._i = clamp(self._i + err * dt, -200.0, 200.0)
        d = (err - self._prev_err) / dt
        self._prev_err = err
        return clamp(self.kp * err + self.ki * self._i + self.kd * d,
                     -self.max_v, self.max_v)

    def reset_pid(self):
        self._i = 0.0
        self._prev_err = 0.0
        self._prev_t = time.ticks_ms()


# ========================= stdin 수신 + 계측 =================================
class CommandReader:
    """BLE tunnel callback first; optional stdin fallback. No motors in callback."""

    def __init__(self):
        self.mode = "none"
        self.poller = None
        self.rx_bytes = 0
        self.rx_lines = 0
        self.last_line = "-"
        self.buf = ""
        self.dirty = True          # 보고할 변화가 있는가
        self.last_report = 0
        self.pending = []
        self.overflow = False
        self.discard_line = False
        if _tunnel is not None:
            try:
                _tunnel.callback(self.receive_tunnel)
                self.mode = "tunnel"
                return
            except Exception as exc:
                emit("#E,tunnel_callback:" + str(exc))

        if select is not None:
            try:
                self.poller = select.poll()
                self.poller.register(sys.stdin, select.POLLIN)
                self.mode = "poll"
            except Exception:
                self.poller = None
            if self.poller is None and hasattr(select, "select"):
                self.mode = "select"

    def receive_tunnel(self, payload):
        # Bounded queue; copy callback-owned bytes before returning.
        try:
            if len(payload) > 512 or len(self.pending) >= 16:
                self.overflow = True
                return
            self.pending.append(bytes(payload))
        except Exception:
            self.overflow = True

    def consume(self, text, lines):
        for ch in text:
            if ch in "\r\n":
                if self.buf and not self.discard_line:
                    self.rx_lines += 1
                    self.last_line = self.buf[:24]
                    lines.append(self.buf)
                self.buf = ""
                self.discard_line = False
            elif not self.discard_line:
                self.buf += ch
                if len(self.buf) > 120:
                    self.buf = ""
                    self.discard_line = True

    def _readable(self):
        if self.mode == "poll":
            try:
                return any(event[1] & select.POLLIN for event in self.poller.poll(0))
            except Exception:
                return False
        if self.mode == "select":
            try:
                r, _w, _x = select.select([sys.stdin], [], [], 0)
                return len(r) > 0
            except Exception:
                return False
        return False

    def pump(self):
        lines = []
        if self.mode == "tunnel":
            if self.overflow:
                self.pending = []
                self.buf = ""
                self.overflow = False
                self.discard_line = False
                emit("#E,tunnel_queue_overflow")
                return ["E"]
            for _ in range(min(len(self.pending), 16)):
                data = self.pending.pop(0)
                self.rx_bytes += len(data)
                self.dirty = True
                try:
                    text = data.decode("utf-8")
                except Exception:
                    emit("#E,tunnel_invalid_text")
                    self.buf = ""
                    self.discard_line = True
                    continue
                self.consume(text, lines)
            return lines
        guard = 0
        while self._readable() and guard < 256:
            guard += 1
            try:
                ch = sys.stdin.read(1)
            except Exception as exc:
                emit("#E,stdin_read:" + str(exc))
                break
            if not ch:
                break
            self.rx_bytes += 1
            self.dirty = True
            self.consume(ch, lines)
        return lines

    def report(self, force=False):
        """수신량이 변했을 때만 계측 줄을 보낸다.
        무선 부하를 줄이려고 1초에 한 번으로 제한한다."""
        if not (self.dirty or force):
            return
        now = time.ticks_ms()
        if not force and time.ticks_diff(now, self.last_report) < 1000:
            return
        self.last_report = now
        self.dirty = False
        emit("#S,rx=%d,ln=%d,last=%s,mode=%s"
             % (self.rx_bytes, self.rx_lines, self.last_line, self.mode))


# ============================ 명령 해석 ======================================
def handle(cmd, rv):
    if not cmd:
        return
    op = cmd[0].upper()
    arg = cmd[1:].strip()

    if op == "T":
        try:
            a, b = arg.split(",")
            left, right = int(float(a)), int(float(b))
            rv.cmd_left, rv.cmd_right = left, right
            if rv.cmd_left or rv.cmd_right:
                rv.estop = False
            rv.last_cmd_ms = time.ticks_ms()
        except Exception:
            emit("#E,bad_T:" + arg[:20])

    elif op == "S":
        rv.stop()

    elif op == "E":
        rv.estop = True
        rv.hold = False
        rv.stop(coast=True)

    elif op == "H":
        rv.hold = (arg[:1] == "1") and not rv.estop
        if rv.hold:
            rv.target_yaw = tilt()[0]
            rv.reset_pid()
        emit("#I,hold=%d" % (1 if rv.hold else 0))

    elif op == "Y":
        try:
            rv.target_yaw = wrap180(float(arg))
            rv.reset_pid()
        except Exception:
            pass

    elif op == "Z":
        try:
            motion_sensor.reset_yaw(0)
        except Exception:
            pass
        rv.target_yaw = 0.0
        rv.reset_pid()
        emit("#I,yaw_reset")

    elif op == "K":
        try:
            a, b, c = arg.split(",")
            rv.kp = int(a) / 1000.0
            rv.ki = int(b) / 1000.0
            rv.kd = int(c) / 1000.0
            rv.reset_pid()
            emit("#I,gain=%.2f/%.2f/%.2f" % (rv.kp, rv.ki, rv.kd))
        except Exception:
            pass

    elif op == "V":
        try:
            rv.max_v = clamp(int(arg), 50, 1100)
            emit("#I,max=%d" % rv.max_v)
        except Exception:
            pass

    elif op == "A":
        try:
            p, v = [int(x) for x in arg.split(",")]
            if p not in range(6) or p in (rv.left_id, rv.right_id):
                raise ValueError("aux port must differ from drive ports")
            v = clamp(v, -rv.max_v, rv.max_v)
            if v and rv.estop:
                raise ValueError("estop active")
            if v == 0:
                motor.stop(PORTS[p], stop=motor.BRAKE)
                rv.aux_active.pop(p, None)
            else:
                motor.run(PORTS[p], v, acceleration=ACCEL)
                rv.aux_active[p] = time.ticks_ms()
        except Exception as exc:
            emit("#E,aux:" + str(exc))

    elif op == "P":
        try:
            a, b = [int(x) for x in arg.split(",")]
            if a not in range(6) or b not in range(6) or a == b:
                raise ValueError("distinct ports 0..5 required")
            rv.stop()
            if rv.paired:
                motor_pair.unpair(motor_pair.PAIR_1)
            rv.paired = False
            rv.left_id, rv.right_id = a, b
            rv._last_drive = None
            rv.pair()
            emit("#I,ports=%d,%d,paired=%d" % (a, b, int(rv.paired)))
        except Exception as exc:
            emit("#E,ports:" + str(exc))

    elif op == "I":
        try:
            a, b = arg.split(",")
            rv.stop()
            rv.inv_l = 1 if a.strip() == "1" else 0
            rv.inv_r = 1 if b.strip() == "1" else 0
            emit("#I,invert=%d,%d" % (rv.inv_l, rv.inv_r))
        except Exception:
            pass

    elif op == "L":
        try:
            if len(arg) >= 1:
                light_matrix.write(arg[0])
        except Exception:
            pass

    elif op == "!":
        rv.estop = False
        rv.cmd_left = rv.cmd_right = 150
        rv.last_cmd_ms = time.ticks_ms()
        rv.test_deadline = time.ticks_add(time.ticks_ms(), 800)
        emit("#I,motor_test")

    elif op == "Q":
        emit("#I,ack=Q")
        emit("#I,version=" + ROVER_VERSION + ",mode=" + reader.mode)
        emit("#I,paired=%d,L=%d,R=%d,invL=%d,invR=%d,max=%d,"
             "kp=%.2f,kd=%.2f,estop=%d"
             % (1 if rv.paired else 0, rv.left_id, rv.right_id,
                rv.inv_l, rv.inv_r, rv.max_v, rv.kp, rv.kd,
                1 if rv.estop else 0))

    else:
        emit("#E,unknown_cmd:" + cmd[:20])


# ============================== 태스크 =======================================
rover = Rover()
reader = CommandReader()


async def task_rx():
    while True:
        for line in reader.pump():
            for cmd in line.split(";"):
                handle(cmd.strip(), rover)
        reader.report(force=time.ticks_diff(time.ticks_ms(), reader.last_report) >= 2000)
        await runloop.sleep_ms(10)


def control_step(rv):
    now = time.ticks_ms()
    for pid in list(rv.aux_active):
        if time.ticks_diff(now, rv.aux_active[pid]) > WATCHDOG_MS:
            try:
                motor.stop(PORTS[pid], stop=motor.BRAKE)
            except Exception as exc:
                emit("#E,aux_stop:" + str(exc))
            del rv.aux_active[pid]
    if rv.estop:
        return
    if rv.test_deadline is not None and time.ticks_diff(now, rv.test_deadline) >= 0:
        rv.stop()
        return
    if time.ticks_diff(now, rv.last_cmd_ms) > WATCHDOG_MS:
        if rv.cmd_left or rv.cmd_right or rv.hold or rv.aux_active:
            rv.stop()
            emit("#E,watchdog_stop")
        return
    l, r = rv.cmd_left, rv.cmd_right
    if rv.hold and (l or r):
        base = (l + r) / 2.0
        turn = (l - r) / 2.0
        if abs(turn) > 40:
            rv.target_yaw = tilt()[0]
            rv.reset_pid()
            rv.drive(l, r)
        else:
            corr = rv.heading_correction()
            rv.drive(base + corr, base - corr)
    else:
        rv.drive(l, r)


async def task_control():
    while True:
        control_step(rover)
        await runloop.sleep_ms(20)


async def task_telemetry():
    while True:
        y, p, r = tilt()
        lv, lp = motor_state(rover.left_id)
        rv_, rp = motor_state(rover.right_id)
        emit("#T,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"
             % (y, p, r, lv, rv_, lp, rp,
                read_distance(), read_reflection(), read_battery(),
                1 if rover.hold else 0, time.ticks_ms(), reader.rx_bytes))
        await runloop.sleep_ms(TELEMETRY_MS)


# ============================ 초기화 · 시작 ==================================
# 초기화는 코루틴이 아니라 모듈 레벨에서 끝낸다.
# runloop.run() 에 '끝나는 코루틴'을 섞으면 그 코루틴이 끝날 때
# 프로그램 전체가 종료될 수 있다.
rover.pair()

try:
    motion_sensor.reset_yaw(0)
except Exception:
    pass

try:
    light_matrix.write("R")
except Exception:
    pass

if STARTUP_JOG:
    if rover.paired:
        # 한쪽씩, 절반 속도로, 사이를 띄워 돌린다 (전류 돌입 최소화)
        try:
            motor.run(PORTS[rover.left_id], 150, acceleration=ACCEL)
            time.sleep_ms(250)
            motor.stop(PORTS[rover.left_id], stop=motor.BRAKE)
            time.sleep_ms(250)
            motor.run(PORTS[rover.right_id], 150, acceleration=ACCEL)
            time.sleep_ms(250)
            motor.stop(PORTS[rover.right_id], stop=motor.BRAKE)
        except Exception as e:
            emit("#E,jog:" + str(e))
        rover.pair()                 # 개별 제어 후 다시 페어링
        emit("#I,startup_jog_done")
    else:
        emit("#E,pair_fail_no_jog:A/B 포트에 모터가 인식되지 않았습니다")

emit("#I,rover-ready,paired=%d,L=%d,R=%d,invL=%d,invR=%d,max=%d,accel=%d"
     % (1 if rover.paired else 0, rover.left_id, rover.right_id,
        rover.inv_l, rover.inv_r, rover.max_v, ACCEL))
emit("#I,battery," + battery_report())
emit("#I,version=" + ROVER_VERSION + ",mode=" + reader.mode)
if _tunnel_error:
    emit("#E,module_tunnel_unavailable:" + _tunnel_error)
reader.report(force=True)   # 시작 시 읽기 방식(mode)을 한 번 알린다

try:
    runloop.run(task_rx(), task_control(), task_telemetry())
finally:
    rover.stop(coast=True)
