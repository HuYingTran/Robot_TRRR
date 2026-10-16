"""Bộ sinh quỹ đạo cho digital twin: jog bám setpoint, PTP (quintic), LIN (thẳng trong không gian Descartes)."""
import numpy as np
from kinematics import wrap180

LIN_SPEED = 80.0  # mm/s ở 100% tốc độ


def quintic(tau):
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)


class Motion:
    def __init__(self, kin, cfg):
        self.kin = kin
        js = cfg["joints"]
        keys = ["z", "j1", "j2", "j3", "g"]
        self.home = np.array([js[k]["home"] for k in keys], float)
        self.vmax = np.array([js[k]["vmax"] for k in keys], float)
        self.q = self.home.copy()
        self.target = self.home.copy()
        self.speed = 0.5
        self.mode = "idle"          # idle | follow | path
        self.path_t = None
        self.path_q = None
        self.t = 0.0
        self.dwell = 0.0
        self.queue = []             # [(kind, q_end, label, dwell)]
        self.current_label = ""
        self.messages = []
        self.on_move_done = None    # callback(label)
        self.collide = None         # callback(q) -> list tên chi tiết va chạm (None = không kiểm tra)

    # ------------------------------------------------------------------ lệnh
    def jog_to(self, target):
        self.queue.clear()
        self.path_t = None
        self.dwell = 0
        self.target = self.kin.clamp(target)
        self.mode = "follow"
        self.current_label = "Jog"

    def stop(self):
        self.queue.clear()
        self.path_t = None
        self.dwell = 0
        self.target = self.q.copy()
        self.mode = "idle"
        self.current_label = ""

    def enqueue(self, kind, q_end, label="", dwell=0.0):
        self.queue.append((kind, np.array(q_end, float), label, float(dwell)))
        if self.mode == "follow":
            self.mode = "idle"

    @property
    def busy(self):
        return self.mode != "idle" or bool(self.queue) or self.dwell > 0

    # ------------------------------------------------------------------ quỹ đạo
    def _plan_ptp(self, q0, q1):
        dq = q1 - q0
        T = float(np.max(np.abs(dq) / (self.vmax * self.speed))) * 1.875
        T = max(T, 0.05)
        n = int(T * 200) + 2
        s = quintic(np.linspace(0, 1, n))
        return np.linspace(0, T, n), q0 + np.outer(s, dq)

    def _plan_lin(self, q0, q1):
        p0, p1 = self.kin.fk(q0), self.kin.fk(q1)
        d = p1 - p0
        d[3] = wrap180(d[3])
        dist = float(np.linalg.norm(d[:3]))
        T = max(dist / (LIN_SPEED * self.speed), abs(d[3]) / (self.vmax[3] * self.speed),
                abs(q1[4] - q0[4]) / (self.vmax[4] * self.speed)) * 1.875
        T = max(T, 0.05)
        n = int(max(T * 200, dist * 2)) + 2
        s = quintic(np.linspace(0, 1, n))
        qs = np.zeros((n, 5))
        qref = q0.copy()
        for i, si in enumerate(s):
            p = p0 + si * d
            q, msg = self.kin.ik(p[0], p[1], p[2], wrap180(p[3]), q_ref=qref, elbow="auto")
            if q is None:
                raise ValueError("LIN không thực hiện được tại %.0f%% quãng đường: %s" % (si * 100, msg))
            if i and np.max(np.abs(q[1:4] - qref[1:4])) > 20:
                raise ValueError("LIN đi qua điểm kỳ dị / đổi cấu hình khuỷu — hãy dùng PTP")
            q[4] = q0[4] + si * (q1[4] - q0[4])
            qs[i] = q
            qref = q
        if np.max(np.abs(qs[-1, :4] - q1[:4])) > 1.0:
            raise ValueError("Điểm đích có cấu hình khuỷu khác điểm đầu — hãy dùng PTP")
        t = np.linspace(0, T, n)
        # kéo giãn thời gian nếu vận tốc khớp vượt giới hạn
        v = np.max(np.abs(np.diff(qs, axis=0)) / np.diff(t)[:, None] / (self.vmax * self.speed))
        if v > 1:
            t *= v
        return t, qs

    def _start_next(self):
        kind, q_end, label, dwell = self.queue.pop(0)
        q_end = np.array(q_end, float)
        bad = self.kin.violations(q_end)
        if bad:
            self.messages.append("Bỏ qua %s: vượt giới hạn %s" % (label, ", ".join(bad)))
            self.queue.clear()
            return
        try:
            if kind == "LIN":
                self.path_t, self.path_q = self._plan_lin(self.q.copy(), q_end)
            else:
                self.path_t, self.path_q = self._plan_ptp(self.q.copy(), q_end)
        except ValueError as e:
            self.messages.append("Lỗi %s: %s" % (label, e))
            self.queue.clear()
            self.path_t = None
            return
        if self.collide is not None and not self.collide(self.path_q[0]):
            step = max(1, len(self.path_q) // 60)
            for i in list(range(0, len(self.path_q), step)) + [len(self.path_q) - 1]:
                hits = self.collide(self.path_q[i])
                if hits:
                    q = self.path_q[i]
                    self.messages.append("Chặn %s %s: va chạm %s tại Z=%.0f J1=%.0f J2=%.0f" % (
                        kind, label, ", ".join(hits), q[0], q[1], q[2]))
                    self.queue.clear()
                    self.path_t = None
                    return
        self.t = 0.0
        self.mode = "path"
        self.pending_dwell = dwell
        self.current_label = "%s %s" % (kind, label)

    # ------------------------------------------------------------------ bước mô phỏng
    def step(self, dt):
        if self.mode == "follow":
            lim = self.vmax * self.speed * dt
            err = self.target - self.q
            q_next = self.q + np.clip(err, -lim, lim)
            if self.collide is not None:
                hits = self.collide(q_next)
                if hits and not self.collide(self.q):
                    self.messages.append("Dừng jog: sắp va chạm " + ", ".join(hits))
                    self.target = self.q.copy()
                    self.mode = "idle"
                    self.current_label = ""
                    return self.q
            self.q = q_next
            if np.all(np.abs(err) < 1e-3):
                self.q = self.target.copy()
                self.mode = "idle"
                self.current_label = ""
        elif self.mode == "path":
            self.t += dt
            T = self.path_t[-1]
            if self.t >= T:
                self.q = self.path_q[-1].copy()
                self.mode = "idle"
                self.path_t = None
                self.dwell = self.pending_dwell
                if self.on_move_done:
                    self.on_move_done(self.current_label)
            else:
                i = int(np.searchsorted(self.path_t, self.t)) - 1
                i = max(0, min(i, len(self.path_t) - 2))
                a = (self.t - self.path_t[i]) / (self.path_t[i + 1] - self.path_t[i])
                self.q = self.path_q[i] + a * (self.path_q[i + 1] - self.path_q[i])
            self.target = self.q.copy()
        if self.mode == "idle":
            if self.dwell > 0:
                self.dwell -= dt
            elif self.queue:
                self._start_next()
            elif self.current_label and self.mode == "idle":
                self.current_label = ""
        return self.q
