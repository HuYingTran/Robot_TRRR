"""Mô phỏng tay kẹp mềm 3 ngón: độ cong ngón theo góc servo SG90, tiếp xúc và kẹp vật hình trụ.

Mỗi ngón là chuỗi đoạn cứng nối bằng bản lề dẻo ở lưng ngón. Kéo cước -> mọi bản lề cùng gập
một góc theo tỉ lệ curl (0 = thẳng, 1 = khe chặn khép hết). Toạ độ cục bộ của 1 ngón:
x = hướng kính (ra ngoài), y = tiếp tuyến, z = theo trục kẹp (0 = mặt dưới đế, âm = xuống).
"""
import math
import numpy as np


class GripperSim:
    def __init__(self, cfg, kin):
        g = cfg["gripper"]
        self.kin = kin
        self.pulleys = cfg["pulleys"]
        self.origin_z = float(cfg["collision"]["gripper_origin_z"])
        self.axis_xy = np.array(cfg["kinematics"]["j3_axis_xy"], float)
        self.fingers = {k: math.radians(v) for k, v in g["finger_parts"].items()}
        self.x_in = float(g["x_in"])
        self.x_hinge = float(g["x_hinge"])
        self.hinge_z = sorted((float(z) for z in g["hinge_z"]), reverse=True)   # từ trên xuống
        self.hinge_max = math.radians(float(g["hinge_max_deg"]))
        self.f_l = float(g["f_l"])
        self.f_t = float(g["f_t"])
        self.g_open = float(g["g_open"])
        self.g_closed = float(g["g_closed"])
        self.top_clear_z = float(g["top_clear_z"])       # vật không được cao hơn mức này (ống cuộn / đế)
        self.center_tol = float(g.get("center_tol", 6.0))
        self.base = {}                                     # part -> (x, y, z_local, segment) ở tư thế thẳng
        zs = np.linspace(0.0, -self.f_l, 161)
        self.face = np.c_[np.full_like(zs, self.x_in), zs]  # mặt trong ngón (để tính tiếp xúc)
        self.face_seg = self._segment(zs)
        # 3 ngon rong f_w xep cach nhau 120 do: dau ngon cham nhau khi mat trong con r = (f_w/2)/tan(60)
        r_meet = float(g.get("f_w", 12.0)) / 2 / math.tan(math.radians(60))
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if self.tip_radius(mid) > r_meet else (lo, mid)
        self.c_self = lo                                  # do cong toi da (dau ngon cham nhau)
        self.min_d = 2 * r_meet                           # vat nho hon -> khong kep duoc

    # ------------------------------------------------------------------ hình học ngón
    def curl_from_g(self, g):
        c = min(1.0, max(0.0, (g - self.g_open) / (self.g_closed - self.g_open)))
        return min(c, getattr(self, "c_self", 1.0))

    def _segment(self, z):
        return np.sum(z[:, None] < np.array(self.hinge_z)[None, :], axis=1)

    def _transforms(self, curl):
        """Ma trận affine 2D (x, z) cho từng đoạn ngón (đoạn 0 = gốc, không đổi)."""
        th = -self.hinge_max * curl                       # gập vào trong
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        T = [np.eye(3)]
        cur = np.eye(3)
        for zk in self.hinge_z:
            piv = cur @ np.array([self.x_hinge, zk, 1.0])
            step = np.eye(3)
            step[:2, :2] = R
            step[:2, 2] = piv[:2] - R @ piv[:2]
            cur = step @ cur
            T.append(cur.copy())
        return np.array(T)

    def register_finger(self, part, points_world):
        """Lưu toạ độ cục bộ của mesh ngón (tư thế home, ngón thẳng)."""
        a = self.fingers[part]
        d = np.array([math.cos(a), math.sin(a)])
        t = np.array([-math.sin(a), math.cos(a)])
        rel = points_world[:, :2] - self.axis_xy
        x, y = rel @ d, rel @ t
        z = points_world[:, 2] - self.origin_z
        self.base[part] = (x, y, z, self._segment(z), d, t)

    def deform(self, part, curl):
        """Toạ độ world (tư thế home của robot) của mesh ngón khi cong `curl`."""
        x, y, z, seg, d, t = self.base[part]
        T = self._transforms(curl)[seg]
        xn = T[:, 0, 0] * x + T[:, 0, 1] * z + T[:, 0, 2]
        zn = T[:, 1, 0] * x + T[:, 1, 1] * z + T[:, 1, 2]
        xy = self.axis_xy + xn[:, None] * d + y[:, None] * t
        return np.c_[xy, zn + self.origin_z]

    def tip_radius(self, curl):
        T = self._transforms(curl)[self.face_seg]
        p = self.face
        return float(np.min(T[:, 0, 0] * p[:, 0] + T[:, 0, 1] * p[:, 1] + T[:, 0, 2]))

    def _touch(self, curl, R, zlo, zhi):
        T = self._transforms(curl)[self.face_seg]
        p = self.face
        xn = T[:, 0, 0] * p[:, 0] + T[:, 0, 1] * p[:, 1] + T[:, 0, 2]
        zn = T[:, 1, 0] * p[:, 0] + T[:, 1, 1] * p[:, 1] + T[:, 1, 2]
        m = (zn >= zlo) & (zn <= zhi)
        return bool(m.any() and np.min(xn[m]) <= R)

    def contact_curl(self, R, zlo, zhi):
        """Độ cong tại đó ngón chạm vật trụ bán kính R (z cục bộ zlo..zhi). None nếu không chạm."""
        if not self._touch(1.0, R, zlo, zhi):
            return None
        lo, hi = 0.0, 1.0
        if self._touch(0.0, R, zlo, zhi):
            return 0.0
        for _ in range(25):
            mid = (lo + hi) / 2
            if self._touch(mid, R, zlo, zhi):
                hi = mid
            else:
                lo = mid
        return hi

    # ------------------------------------------------------------------ trạng thái + vật
    def gripper_frame(self, q):
        """(tâm trục kẹp world, z gốc kẹp world, ma trận tool)."""
        M = self.kin.group_matrices(q, self.pulleys)["tool"]
        p = M @ np.array([self.axis_xy[0], self.axis_xy[1], self.origin_z, 1.0])
        return p[:2], p[2], M

    def evaluate(self, q, obj):
        """Tính trạng thái kẹp ở tư thế q với vật obj (dict R, H, M, attached, visible).
        Trả về dict: curl (thực tế), curl_cmd, contact (curl chạm vật hoặc None), centered, hits (va chạm)."""
        cmd = self.curl_from_g(q[4])
        res = {"curl_cmd": cmd, "curl": cmd, "contact": None, "centered": False, "hits": []}
        if obj is None or not obj["visible"]:
            return res
        axy, az, M = self.gripper_frame(q)
        R, H = obj["R"], obj["H"]
        if obj["attached"]:
            res["contact"] = obj["contact"]
            res["curl"] = min(cmd, obj["contact"])
            res["centered"] = True
            if obj["M"][2, 3] < -0.5:
                res["hits"].append("vật chạm sàn")
            return res
        oc = obj["M"][:2, 3]
        zlo, zhi = obj["M"][2, 3] - az, obj["M"][2, 3] + H - az       # z cục bộ của vật
        overlap = zhi > -self.f_l and zlo < 0
        d = float(np.linalg.norm(oc - axy))
        if not overlap:
            return res
        if d <= self.center_tol:
            res["centered"] = True
            if R >= self.x_in - 0.3:
                res["hits"].append("vật quá to (Ø > %.0f)" % (2 * self.x_in))
            if zhi > self.top_clear_z:
                res["hits"].append("vật chạm ống cuộn/đế")
            c = self.contact_curl(R, zlo, zhi)
            if c is not None:
                res["contact"] = c
                res["curl"] = min(cmd, c)
            return res
        # lệch tâm: ngón có đâm vào vật không?
        for part, a in self.fingers.items():
            fc = axy + (self.x_in + self.f_t / 2) * np.array([math.cos(a), math.sin(a)])
            if np.linalg.norm(fc - oc) < R + self.f_t / 2:
                res["hits"].append("ngón chạm vật")
                break
        return res
