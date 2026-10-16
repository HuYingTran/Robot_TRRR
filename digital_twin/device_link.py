"""Kết nối với robot thật (Serial) hoặc robot ảo (SIM) cho digital twin.

Giao thức text, mỗi lệnh 1 dòng kết thúc '\\n' (đơn vị mm / độ):
  PC -> robot:
    J <z> <j1> <j2> <j3> <g>     setpoint khớp (gửi liên tục ~20 Hz khi robot chạy)
    H                            về home
    S                            dừng khẩn
    ?                            hỏi vị trí
  robot -> PC:
    P <z> <j1> <j2> <j3> <g>     vị trí thực tế (phản hồi, nên gửi ~20 Hz)
    OK / ERR <thông báo> / dòng khác -> hiển thị ở nhật ký
"""
import time
import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # app vẫn chạy được ở chế độ SIM
    serial = None

SIM_NAME = "SIM (robot ảo)"


def list_ports():
    ports = [SIM_NAME]
    if serial is not None:
        ports += [p.device + " - " + (p.description or "") for p in serial.tools.list_ports.comports()]
    return ports


class SimDevice:
    """Robot ảo: bám setpoint với giới hạn tốc độ + trễ, để thử twin khi chưa có phần cứng."""

    def __init__(self, vmax, lag=0.12, home=None):
        self.home = np.zeros(5) if home is None else np.array(home, float)
        self.q = self.home.copy()
        self.target = self.q.copy()
        self.vmax = np.asarray(vmax, float)
        self.lag = lag
        self.last = time.perf_counter()
        self.lines = []
        self.t_report = 0.0

    def write_line(self, line):
        p = line.split()
        if not p:
            return
        if p[0] == "J" and len(p) == 6:
            self.target = np.array([float(v) for v in p[1:]])
        elif p[0] == "H":
            self.target = self.home.copy()
            self.lines.append("OK HOME")
        elif p[0] == "S":
            self.target = self.q.copy()
            self.lines.append("OK STOP")
        elif p[0] == "?":
            self.t_report = 0

    def read_lines(self):
        now = time.perf_counter()
        dt, self.last = now - self.last, now
        err = self.target - self.q
        step = np.clip(err * min(1.0, dt / self.lag), -self.vmax * dt, self.vmax * dt)
        self.q += step
        self.t_report -= dt
        if self.t_report <= 0:
            self.t_report = 0.05
            self.lines.append("P " + " ".join("%.3f" % v for v in self.q))
        out, self.lines = self.lines, []
        return out

    def close(self):
        pass


class SerialDevice:
    def __init__(self, port, baud):
        if serial is None:
            raise RuntimeError("Chưa cài pyserial (pip install pyserial)")
        self.ser = serial.Serial(port, baud, timeout=0)
        self.buf = b""

    def write_line(self, line):
        self.ser.write((line + "\n").encode("ascii"))

    def read_lines(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        lines = []
        while b"\n" in self.buf:
            ln, self.buf = self.buf.split(b"\n", 1)
            lines.append(ln.decode("ascii", "replace").strip())
        return [l for l in lines if l]

    def close(self):
        self.ser.close()


def parse_feedback(line):
    """'P z j1 j2 j3 g' -> np.array hoặc None."""
    p = line.split()
    if len(p) == 6 and p[0] == "P":
        try:
            return np.array([float(v) for v in p[1:]])
        except ValueError:
            return None
    return None
