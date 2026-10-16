"""Động học robot TRR: trục Z (vít me) + 2 khớp quay SCARA (J1, J2) + cổ tay J3.

Vector khớp q = [z, j1, j2, j3, g]
  z  : độ dịch trục Z so với home (mm)
  j1 : góc vai so với home (độ, dương = ngược chiều kim đồng hồ nhìn từ trên)
  j2 : góc khuỷu so với link 1 (độ)
  j3 : góc cổ tay MG995 so với link 2 (độ)
  g  : góc servo kẹp SG90 (độ) - không ảnh hưởng động học
"""
import math
import numpy as np

JOINT_KEYS = ["z", "j1", "j2", "j3", "g"]


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def rot_z_about(px, py, deg):
    """Ma trận 4x4 quay quanh trục song song Oz đi qua điểm (px, py)."""
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    m = np.eye(4)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
    m[0, 3] = px - c * px + s * py
    m[1, 3] = py - s * px - c * py
    return m


def translate(x, y, z):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


class Kinematics:
    def __init__(self, cfg):
        k = cfg["kinematics"]
        self.cfg = cfg
        self.p1 = np.array(k["j1_axis_xy"], float)
        self.p2 = np.array(k["j2_axis_xy"], float)
        self.p3 = np.array(k["j3_axis_xy"], float)
        self.tcp_z0 = float(k["tcp_z_home"])
        self.v1 = self.p2 - self.p1          # link 1 ở home
        self.v2 = self.p3 - self.p2          # link 2 ở home
        self.L1 = float(np.linalg.norm(self.v1))
        self.L2 = float(np.linalg.norm(self.v2))
        self.a1 = math.degrees(math.atan2(self.v1[1], self.v1[0]))
        self.a2 = math.degrees(math.atan2(self.v2[1], self.v2[0]))
        self.limits = {key: (cfg["joints"][key]["min"], cfg["joints"][key]["max"]) for key in JOINT_KEYS}
        d = cfg["drive"]
        self.usteps = d["steps_per_rev"] * d["microstep"]
        self.lead = d["lead_screw_mm_per_rev"]
        self.r1 = d["j1_belt_ratio"]
        self.r2 = d["j2_belt_ratio"]
        self.g_home = cfg["joints"]["g"]["home"]

    # ---------------------------------------------------------------- FK
    def fk(self, q):
        """Trả về (x, y, z, yaw) của TCP (tâm mặt phẳng đầu ngón kẹp, trên trục J3). yaw = hướng link 2 + j3 (độ)."""
        z, t1, t2, t3 = q[0], q[1], q[2], q[3]
        A1 = math.radians(self.a1 + t1)
        A2 = math.radians(self.a2 + t1 + t2)
        x = self.p1[0] + self.L1 * math.cos(A1) + self.L2 * math.cos(A2)
        y = self.p1[1] + self.L1 * math.sin(A1) + self.L2 * math.sin(A2)
        yaw = wrap180(self.a2 + t1 + t2 + t3)
        return np.array([x, y, self.tcp_z0 + z, yaw])

    def elbow_point(self, q):
        A1 = math.radians(self.a1 + q[1])
        return self.p1 + self.L1 * np.array([math.cos(A1), math.sin(A1)])

    # ---------------------------------------------------------------- IK
    def ik(self, x, y, z, yaw, q_ref=None, elbow="auto"):
        """Động học ngược. elbow: 'auto' (gần q_ref nhất), 'left' (j2>0), 'right' (j2<0).
        Trả về (q, msg) — q=None nếu không với tới hoặc vượt giới hạn."""
        px, py = x - self.p1[0], y - self.p1[1]
        r2 = px * px + py * py
        c2 = (r2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if c2 > 1 + 1e-9 or c2 < -1 - 1e-9:
            return None, "Điểm nằm ngoài vùng với (R = %.1f..%.1f mm)" % (abs(self.L1 - self.L2), self.L1 + self.L2)
        c2 = max(-1.0, min(1.0, c2))
        dz = z - self.tcp_z0
        sols = []
        for sgn in (+1, -1):
            b = sgn * math.acos(c2)                       # góc tương đối link2/link1 (tuyệt đối)
            q1 = math.atan2(py, px) - math.atan2(self.L2 * math.sin(b), self.L1 + self.L2 * math.cos(b))
            t1 = wrap180(math.degrees(q1) - self.a1)
            t2 = wrap180(math.degrees(b) - (self.a2 - self.a1))
            t3 = wrap180(yaw - (self.a2 + t1 + t2))
            g = q_ref[4] if q_ref is not None else self.cfg["joints"]["g"]["home"]
            sols.append(("left" if sgn > 0 else "right", np.array([dz, t1, t2, t3, g])))
        if elbow in ("left", "right"):
            sols = [s for s in sols if s[0] == elbow]
        ok = [s for s in sols if self.within_limits(s[1])]
        if not ok:
            bad = self.violations(sols[0][1])
            return None, "Vượt giới hạn khớp: " + ", ".join(bad)
        if q_ref is not None and len(ok) > 1:
            ok.sort(key=lambda s: np.sum(np.abs(s[1][1:4] - np.asarray(q_ref)[1:4])))
        return ok[0][1], "OK (khuỷu %s)" % ok[0][0]

    # ---------------------------------------------------------------- tiện ích
    def violations(self, q):
        out = []
        for i, key in enumerate(JOINT_KEYS):
            lo, hi = self.limits[key]
            if q[i] < lo - 1e-6 or q[i] > hi + 1e-6:
                out.append("%s=%.1f [%g..%g]" % (key, q[i], lo, hi))
        return out

    def within_limits(self, q):
        return not self.violations(q)

    def clamp(self, q):
        q = np.array(q, float)
        for i, key in enumerate(JOINT_KEYS):
            lo, hi = self.limits[key]
            q[i] = min(hi, max(lo, q[i]))
        return q

    def motor_steps(self, q):
        """Số bước (đã nhân vi bước) của 3 stepper so với home."""
        sz = q[0] / self.lead * self.usteps
        s1 = q[1] * self.r1 / 360.0 * self.usteps
        s2 = q[2] * self.r2 / 360.0 * self.usteps
        return int(round(sz)), int(round(s1)), int(round(s2))

    # ---------------------------------------------------------------- ma trận cho các nhóm chi tiết
    def group_matrices(self, q, pulleys):
        z, t1, t2, t3 = q[0], q[1], q[2], q[3]
        Tz = translate(0, 0, z)
        M1 = Tz @ rot_z_about(self.p1[0], self.p1[1], t1)
        M2 = M1 @ rot_z_about(self.p2[0], self.p2[1], t2)
        M3 = M2 @ rot_z_about(self.p3[0], self.p3[1], t3)
        pa = pulleys["Body010"]["axis_xy"]
        pb = pulleys["Body015"]["axis_xy"]
        return {
            "base": np.eye(4),
            "screw": rot_z_about(0, 0, z / self.lead * 360.0),
            "carriage": Tz,
            "motor_j1": Tz @ rot_z_about(pa[0], pa[1], t1 * self.r1),
            "link1": M1,
            "motor_j2": M1 @ rot_z_about(pb[0], pb[1], t2 * self.r2),
            "link2": M2,
            "tool": M3,
            "spool": M3 @ rot_z_about(self.p3[0], self.p3[1], q[4] - self.g_home),
        }
