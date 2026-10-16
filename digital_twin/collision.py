"""Kiểm tra va chạm tay kẹp ↔ thân robot / sàn cho digital twin.

Tay kẹp được bao bằng một vật thể tròn xoay quanh trục J3 (profile r(z) trong robot_config.json).
Bề mặt các chi tiết khác được lấy mẫu thành đám điểm; điểm nào lọt vào trong vỏ bao là va chạm.
"""
import numpy as np


def sample_surface(mesh, spacing):
    """Lấy mẫu điểm đều theo diện tích trên mesh tam giác (pyvista PolyData)."""
    tri = mesh.triangulate()
    V = np.asarray(tri.points, float)
    F = np.asarray(tri.faces).reshape(-1, 4)[:, 1:]
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.ceil(area / (spacing * spacing)).astype(int)
    rng = np.random.default_rng(0)
    idx = np.repeat(np.arange(len(F)), n)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    pts = a[idx] + u[:, None] * (b[idx] - a[idx]) + v[:, None] * (c[idx] - a[idx])
    return np.vstack([V, pts])


class CollisionChecker:
    def __init__(self, cfg, kin, meshes, spacing=3.0):
        """meshes: {part_name: PolyData ở tư thế home (toạ độ global)}."""
        c = cfg["collision"]
        self.kin = kin
        self.pulleys = cfg["pulleys"]
        self.margin = float(c.get("margin", 1.0))
        self.z0 = float(c["gripper_origin_z"])
        self.floor_z = float(c.get("floor_z", 0.0))
        prof = np.array(c["gripper_profile"], float)
        self.zmin, self.zmax = prof[:, 0].min(), prof[:, 0].max()
        self.dz = 0.5
        zs = np.arange(self.zmin, self.zmax + self.dz, self.dz)
        r = np.zeros_like(zs)
        for (za, ra), (zb, rb) in zip(prof[:-1], prof[1:]):
            if zb <= za:
                continue
            m = (zs >= za - 1e-9) & (zs <= zb + 1e-9)
            r[m] = np.maximum(r[m], ra + (zs[m] - za) / (zb - za) * (rb - ra))
        self.r_table = r + self.margin
        self.groups = {}
        for g in c["check_groups"]:
            pts, ids, names = [], [], []
            for part in cfg["groups"][g]["parts"]:
                if part not in meshes:
                    continue
                p = sample_surface(meshes[part], spacing)
                pts.append(p)
                ids.append(np.full(len(p), len(names)))
                names.append(part)
            if pts:
                self.groups[g] = (np.vstack(pts), np.concatenate(ids), names)
        self.labels = cfg.get("part_labels", {})

    def check(self, q):
        """Trả về danh sách tên chi tiết va chạm với tay kẹp ở tư thế q (rỗng nếu an toàn)."""
        mats = self.kin.group_matrices(q, self.pulleys)
        axis = mats["tool"] @ np.array([self.kin.p3[0], self.kin.p3[1], self.z0, 1.0])
        cx, cy, cz = axis[:3]
        hits = []
        if cz + self.zmin < self.floor_z:
            hits.append("Sàn")
        for g, (pts, ids, names) in self.groups.items():
            M = mats[g]
            P = pts @ M[:3, :3].T + M[:3, 3]
            zl = P[:, 2] - cz
            m = (zl >= self.zmin) & (zl <= self.zmax)
            if not m.any():
                continue
            k = ((zl[m] - self.zmin) / self.dz).astype(int)
            r2 = (P[m, 0] - cx) ** 2 + (P[m, 1] - cy) ** 2
            hit = r2 < self.r_table[k] ** 2
            if hit.any():
                for i in np.unique(ids[m][hit]):
                    hits.append(self.labels.get(names[i], names[i]))
        return hits
