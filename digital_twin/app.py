"""Digital Twin robot TRR — PyQt5 + PyVista.

Chạy:  python app.py
Mesh lấy từ thư mục meshes/ (xuất bằng: freecadcmd export_meshes.py).
"""
import os
import sys
import json
import math
import time
from collections import deque

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
import pyvista as pv
from pyvistaqt import QtInteractor

from kinematics import Kinematics, JOINT_KEYS, wrap180, translate, rot_z_about
from motion import Motion
from collision import CollisionChecker
from gripper_sim import GripperSim
import device_link as dl

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "meshes")
PROG_DIR = os.path.join(HERE, "programs")
TICK_MS = 20
SEND_PERIOD = 0.05
GHOST_COLOR = "#00d5ff"


def load_config():
    with open(os.path.join(HERE, "robot_config.json"), encoding="utf-8") as f:
        return json.load(f)


# ====================================================================== điều khiển 1 khớp
class JointRow(QtCore.QObject):
    changed = QtCore.pyqtSignal(int, float)

    SCALE = 10

    def __init__(self, idx, spec, grid, row):
        super().__init__()
        self.idx = idx
        self.lbl = QtWidgets.QLabel(spec["label"])
        self.slider = QtWidgets.QSlider(Qt.Horizontal)
        self.slider.setRange(int(spec["min"] * self.SCALE), int(spec["max"] * self.SCALE))
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(spec["min"], spec["max"])
        self.spin.setDecimals(1)
        self.spin.setSuffix(" " + spec["unit"])
        self.spin.setKeyboardTracking(False)
        self.spin.setMinimumWidth(90)
        self.actual = QtWidgets.QLabel("0.0")
        self.actual.setMinimumWidth(52)
        self.actual.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.actual.setToolTip("Vị trí thực tế của mô hình")
        grid.addWidget(self.lbl, row, 0)
        grid.addWidget(self.slider, row, 1)
        grid.addWidget(self.spin, row, 2)
        grid.addWidget(self.actual, row, 3)
        self.slider.valueChanged.connect(lambda v: self._emit(v / self.SCALE))
        self.spin.valueChanged.connect(self._emit)

    def _emit(self, v):
        self.set_value(v)
        self.changed.emit(self.idx, float(v))

    def set_value(self, v):
        for w, val in ((self.slider, int(round(v * self.SCALE))), (self.spin, v)):
            if w.hasFocus() and w is self.spin:
                continue
            w.blockSignals(True)
            w.setValue(val)
            w.blockSignals(False)

    def set_actual(self, v):
        self.actual.setText("%.1f" % v)


# ====================================================================== cửa sổ chính
class TwinWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.cfg = load_config()
        self.kin = Kinematics(self.cfg)
        self.motion = Motion(self.kin, self.cfg)
        self.motion.on_move_done = self._on_move_done
        self.device = None
        self.q_fb = None
        self.t_fb = 0
        self.last_send = 0
        self.last_sent_q = None
        self.program = []
        self.trail = deque(maxlen=3000)
        self.last_q_drawn = None
        self.run_loop = False

        self.setWindowTitle("Digital Twin — Robot TRR")
        self.resize(1500, 900)
        self._build_view()
        self._build_dock()
        self._build_toolbar()
        self.statusBar().showMessage("Sẵn sàng")

        self.t_prev = time.perf_counter()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(TICK_MS)
        self._apply_q(self.motion.q, force=True)
        self._sync_widgets(self.motion.q)

    # ------------------------------------------------------------------ 3D
    def _build_view(self):
        self.plotter = QtInteractor(self)
        self.setCentralWidget(self.plotter.interactor)
        pl = self.plotter
        pl.set_background("#eef2f6", top="#c3cfdb")
        pl.enable_anti_aliasing("ssaa")
        pl.add_axes(line_width=3, labels_off=False)

        floor = pv.Plane(center=(0, -90, -0.2), direction=(0, 0, 1), i_size=600, j_size=600, i_resolution=24, j_resolution=24)
        pl.add_mesh(floor, color="#d7dde3", show_edges=True, edge_color="#b5bec8", lighting=False, pickable=False)

        self.actors = {}       # group -> list(actor)
        self.ghosts = {}
        overrides = self.cfg.get("part_colors", self.cfg.get("stepper_colors", {}))
        self.home_meshes = {}
        self.base_colors = {}
        finger_parts = self.cfg.get("gripper", {}).get("finger_parts", {})
        self.finger_meshes = {}
        missing = []
        for gname, g in self.cfg["groups"].items():
            self.actors[gname], self.ghosts[gname] = [], []
            for part in g["parts"]:
                path = os.path.join(MESH_DIR, part + ".stl")
                if not os.path.exists(path):
                    missing.append(part)
                    continue
                mesh = pv.read(path).clean().compute_normals(feature_angle=35, split_vertices=True, auto_orient_normals=False)
                self.home_meshes[part] = mesh
                self.base_colors[part] = overrides.get(part, g["color"])
                a = pl.add_mesh(mesh, color=overrides.get(part, g["color"]), smooth_shading=True,
                                specular=0.35, specular_power=20, ambient=0.15, name=part)
                a._part = part
                self.actors[gname].append(a)
                gmesh = mesh.copy() if part in finger_parts else mesh
                if part in finger_parts:
                    self.finger_meshes[part] = (mesh, gmesh)
                gh = pl.add_mesh(gmesh, color=GHOST_COLOR, opacity=0.28, smooth_shading=True, name="ghost_" + part)
                gh.SetVisibility(False)
                self.ghosts[gname].append(gh)
        self.coll = None
        if "collision" in self.cfg:
            self.coll = CollisionChecker(self.cfg, self.kin, self.home_meshes)
        self.hits = []
        # tay kep: bien dang ngon + vat de gap
        self.gsim = None
        self.obj = None
        self.curl_drawn = {False: None, True: None}
        if "gripper" in self.cfg and self.finger_meshes:
            self.gsim = GripperSim(self.cfg, self.kin)
            for part, (m, _) in self.finger_meshes.items():
                self.gsim.register_finger(part, np.asarray(m.points).copy())
            oc = self.cfg["gripper"].get("object", {})
            xy = oc.get("xy", [80, -160])
            self.obj = {"R": oc.get("diameter", 30) / 2, "H": oc.get("height", 40), "visible": True,
                        "attached": False, "contact": None, "rel": None, "M": translate(xy[0], xy[1], 0)}
            self.obj_poly = pv.Cylinder(center=(0, 0, self.obj["H"] / 2), direction=(0, 0, 1),
                                        radius=self.obj["R"], height=self.obj["H"], resolution=48)
            self.obj_actor = pl.add_mesh(self.obj_poly, color=oc.get("color", "#e0457b"), smooth_shading=True,
                                         specular=0.3, name="object")
            self.obj_actor.SetUserMatrix(pv.vtkmatrix_from_array(self.obj["M"]))
            self.grip = {"curl": 0.0, "curl_cmd": 0.0, "contact": None, "hits": []}
        if missing:
            QtWidgets.QMessageBox.warning(self, "Thiếu mesh",
                "Không tìm thấy mesh: %s\nChạy: freecadcmd export_meshes.py" % ", ".join(missing))

        # TCP + hướng dụng cụ + vết + đích
        self.tcp_actor = pl.add_mesh(pv.Sphere(radius=3.5), color="#ffcc00", name="tcp")
        self.tcp_dir = pl.add_mesh(pv.Arrow(start=(0, 0, 0), direction=(1, 0, 0), scale=30, tip_radius=0.12, shaft_radius=0.04),
                                   color="#ffcc00", name="tcp_dir")
        self.target_actor = pl.add_mesh(pv.Sphere(radius=4), color="#22c55e", opacity=0.6, name="target")
        self.target_actor.SetVisibility(False)
        self.trail_poly = pv.lines_from_points(np.array([[0, 0, 0], [0, 0, 0.01]], float))
        self.trail_actor = pl.add_mesh(self.trail_poly, color="#e11d48", line_width=2.5, name="trail")
        self.work_actor = pl.add_mesh(self._workspace_cloud(), color="#7c3aed", point_size=3, opacity=0.35,
                                      render_points_as_spheres=True, name="workspace")
        self.work_actor.SetVisibility(False)

        pl.camera_position = [(520, -560, 430), (0, -90, 150), (0, 0, 1)]

    def _workspace_cloud(self):
        lo1, hi1 = self.kin.limits["j1"]
        lo2, hi2 = self.kin.limits["j2"]
        pts = []
        for t1 in np.arange(lo1, hi1 + 0.1, 3):
            for t2 in np.arange(lo2, hi2 + 0.1, 3):
                p = self.kin.fk([0, t1, t2, 0, 0])
                pts.append((p[0], p[1], 0.5))
        return pv.PolyData(np.array(pts))

    # ------------------------------------------------------------------ dock
    def _build_dock(self):
        dock = QtWidgets.QDockWidget("Bảng điều khiển", self)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable | QtWidgets.QDockWidget.DockWidgetFloatable)
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)

        # khung trạng thái
        box = QtWidgets.QGroupBox("Trạng thái")
        bl = QtWidgets.QGridLayout(box)
        self.lbl_tcp = QtWidgets.QLabel()
        self.lbl_steps = QtWidgets.QLabel()
        self.lbl_state = QtWidgets.QLabel()
        self.lbl_coll = QtWidgets.QLabel()
        f = QtGui.QFont("Consolas", 10)
        for i, l in enumerate((self.lbl_tcp, self.lbl_steps, self.lbl_state, self.lbl_coll)):
            l.setFont(f)
            bl.addWidget(l, i, 0)
        lay.addWidget(box)

        # tốc độ
        sp = QtWidgets.QHBoxLayout()
        sp.addWidget(QtWidgets.QLabel("Tốc độ"))
        self.speed = QtWidgets.QSlider(Qt.Horizontal)
        self.speed.setRange(5, 100)
        self.speed.setValue(50)
        self.lbl_speed = QtWidgets.QLabel("50 %")
        self.speed.valueChanged.connect(self._on_speed)
        sp.addWidget(self.speed)
        sp.addWidget(self.lbl_speed)
        lay.addLayout(sp)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._tab_joints(), "Khớp")
        tabs.addTab(self._tab_cart(), "Descartes")
        tabs.addTab(self._tab_gripper(), "Tay kẹp")
        tabs.addTab(self._tab_program(), "Chương trình")
        tabs.addTab(self._tab_device(), "Kết nối robot")
        lay.addWidget(tabs, 1)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFixedHeight(130)
        lay.addWidget(QtWidgets.QLabel("Nhật ký"))
        lay.addWidget(self.log)

        dock.setWidget(w)
        dock.setMinimumWidth(470)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

    def _tab_joints(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        grid = QtWidgets.QGridLayout()
        self.rows = []
        for i, key in enumerate(JOINT_KEYS):
            r = JointRow(i, self.cfg["joints"][key], grid, i)
            r.changed.connect(self._on_joint_changed)
            self.rows.append(r)
        grid.setColumnStretch(1, 1)
        v.addLayout(grid)
        h = QtWidgets.QHBoxLayout()
        for text, fn in (("Về Home", self._go_home), ("Dừng", self._stop), ("Lấy từ robot thật", self._sync_from_fb)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            h.addWidget(b)
        v.addLayout(h)
        d = self.cfg["drive"]
        note = QtWidgets.QLabel(
            "Truyền động: vít me %g mm/vòng · đai J1 %g:1 · đai J2 %g:1 · %d bước/vòng × %d vi bước\n"
            "L1 = %.1f mm · L2 = %.1f mm (thông số trong robot_config.json)"
            % (d["lead_screw_mm_per_rev"], d["j1_belt_ratio"], d["j2_belt_ratio"], d["steps_per_rev"], d["microstep"],
               self.kin.L1, self.kin.L2))
        note.setStyleSheet("color:#555")
        note.setWordWrap(True)
        v.addWidget(note)
        v.addStretch(1)
        return w

    def _tab_cart(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        form = QtWidgets.QGridLayout()
        self.cart = []
        for i, (name, lo, hi) in enumerate((("X", -300, 300), ("Y", -400, 200), ("Z", 0, 400), ("Yaw", -180, 180))):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(1)
            s.setSuffix(" °" if name == "Yaw" else " mm")
            s.valueChanged.connect(self._preview_target)
            form.addWidget(QtWidgets.QLabel(name), i, 0)
            form.addWidget(s, i, 1)
            self.cart.append(s)
        self.elbow = QtWidgets.QComboBox()
        self.elbow.addItems(["auto", "left", "right"])
        form.addWidget(QtWidgets.QLabel("Khuỷu"), 4, 0)
        form.addWidget(self.elbow, 4, 1)
        v.addLayout(form)
        h = QtWidgets.QHBoxLayout()
        for text, fn in (("Lấy vị trí hiện tại", self._cart_from_current),
                         ("Chạy PTP", lambda: self._cart_go("PTP")),
                         ("Chạy LIN", lambda: self._cart_go("LIN"))):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            h.addWidget(b)
        v.addLayout(h)
        self.lbl_ik = QtWidgets.QLabel("")
        self.lbl_ik.setWordWrap(True)
        v.addWidget(self.lbl_ik)

        jog = QtWidgets.QGroupBox("Jog Descartes (chuyển động thẳng)")
        g = QtWidgets.QGridLayout(jog)
        self.jog_step = QtWidgets.QComboBox()
        self.jog_step.addItems(["1", "5", "10", "20", "50"])
        self.jog_step.setCurrentIndex(2)
        g.addWidget(QtWidgets.QLabel("Bước (mm / °)"), 0, 0)
        g.addWidget(self.jog_step, 0, 1, 1, 3)
        for r, (axis, name) in enumerate(((0, "X"), (1, "Y"), (2, "Z"), (3, "Yaw")), start=1):
            for c, sgn in enumerate((-1, +1)):
                b = QtWidgets.QPushButton("%s %s" % (name, "−" if sgn < 0 else "+"))
                b.setAutoRepeat(True)
                b.setAutoRepeatInterval(250)
                b.clicked.connect(lambda _=False, a=axis, s=sgn: self._jog_cart(a, s))
                g.addWidget(b, r, c * 2, 1, 2)
        v.addWidget(jog)
        v.addStretch(1)
        return w

    def _tab_program(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.table = QtWidgets.QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(["Tên", "Kiểu", "Z", "J1", "J2", "J3", "Kẹp", "Chờ (s)"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.itemChanged.connect(self._on_table_edit)
        self.table.cellDoubleClicked.connect(lambda r, c: self._goto_point(r) if c == 0 else None)
        v.addWidget(self.table, 1)
        grid = QtWidgets.QGridLayout()
        btns = [("Ghi điểm (PTP)", lambda: self._teach("PTP")), ("Ghi điểm (LIN)", lambda: self._teach("LIN")),
                ("Cập nhật điểm", self._update_point), ("Xoá", self._del_point),
                ("Lên", lambda: self._move_point(-1)), ("Xuống", lambda: self._move_point(1)),
                ("Đi tới điểm", lambda: self._goto_point(self.table.currentRow())), ("▶ Chạy", self._run_program),
                ("■ Dừng", self._stop), ("Mở...", self._load_program), ("Lưu...", self._save_program)]
        for i, (t, fn) in enumerate(btns):
            b = QtWidgets.QPushButton(t)
            b.clicked.connect(fn)
            grid.addWidget(b, i // 4, i % 4)
        self.chk_loop = QtWidgets.QCheckBox("Lặp")
        grid.addWidget(self.chk_loop, 2, 3)
        v.addLayout(grid)
        hint = QtWidgets.QLabel("Nhấp đúp tên điểm để đi tới. Sửa trực tiếp ô bảng; Kiểu = PTP hoặc LIN.")
        hint.setStyleSheet("color:#555")
        v.addWidget(hint)
        return w

    def _tab_device(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        h = QtWidgets.QHBoxLayout()
        self.port = QtWidgets.QComboBox()
        self.port.setMinimumWidth(220)
        self.baud = QtWidgets.QComboBox()
        self.baud.addItems(["9600", "57600", "115200", "230400", "460800", "921600"])
        self.baud.setCurrentText(str(self.cfg["serial"]["baud"]))
        rb = QtWidgets.QPushButton("⟳")
        rb.setFixedWidth(32)
        rb.clicked.connect(self._refresh_ports)
        self.btn_conn = QtWidgets.QPushButton("Kết nối")
        self.btn_conn.clicked.connect(self._toggle_connect)
        h.addWidget(self.port, 1)
        h.addWidget(rb)
        h.addWidget(self.baud)
        h.addWidget(self.btn_conn)
        v.addLayout(h)
        self._refresh_ports()

        self.chk_send = QtWidgets.QCheckBox("Gửi setpoint xuống robot (twin → robot)")
        self.chk_send.setChecked(True)
        self.chk_mirror = QtWidgets.QCheckBox("Mô hình chính hiển thị vị trí robot thật (robot → twin)")
        self.chk_mirror.setToolTip("Bật: mô hình đặc theo phản hồi; bóng xanh là setpoint")
        v.addWidget(self.chk_send)
        v.addWidget(self.chk_mirror)

        self.fb_table = QtWidgets.QTableWidget(5, 3)
        self.fb_table.setHorizontalHeaderLabels(["Lệnh", "Thực tế", "Sai lệch"])
        self.fb_table.setVerticalHeaderLabels([self.cfg["joints"][k]["label"] for k in JOINT_KEYS])
        self.fb_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.fb_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.fb_table.setFixedHeight(190)
        for r in range(5):
            for c in range(3):
                self.fb_table.setItem(r, c, QtWidgets.QTableWidgetItem("—"))
        v.addWidget(self.fb_table)
        self.lbl_link = QtWidgets.QLabel("Chưa kết nối")
        v.addWidget(self.lbl_link)

        h2 = QtWidgets.QHBoxLayout()
        self.cmd_line = QtWidgets.QLineEdit()
        self.cmd_line.setPlaceholderText("Lệnh thô, ví dụ: H  |  S  |  ?")
        self.cmd_line.returnPressed.connect(self._send_raw)
        b = QtWidgets.QPushButton("Gửi")
        b.clicked.connect(self._send_raw)
        h2.addWidget(self.cmd_line, 1)
        h2.addWidget(b)
        v.addLayout(h2)
        proto = QtWidgets.QLabel(
            "<b>Giao thức (text, 1 dòng/lệnh):</b><br>"
            "PC→robot: <code>J z j1 j2 j3 g</code> (20 Hz), <code>H</code> home, <code>S</code> dừng, <code>?</code> hỏi vị trí<br>"
            "robot→PC: <code>P z j1 j2 j3 g</code> (phản hồi vị trí)<br>Đơn vị mm/độ, so với home.")
        proto.setWordWrap(True)
        proto.setStyleSheet("color:#444")
        v.addWidget(proto)
        v.addStretch(1)
        return w

    def _build_toolbar(self):
        tb = self.addToolBar("Xem")
        tb.setMovable(False)
        pl = self.plotter
        views = (("Iso", lambda: self._set_cam([(520, -560, 430), (0, -90, 150), (0, 0, 1)])),
                 ("Trên", lambda: self._set_cam([(0, -90, 900), (0, -90, 150), (0, 1, 0)])),
                 ("Trước", lambda: self._set_cam([(0, -800, 180), (0, -90, 180), (0, 0, 1)])),
                 ("Bên", lambda: self._set_cam([(800, -90, 180), (0, -90, 180), (0, 0, 1)])),
                 ("Vừa khung", lambda: (pl.reset_camera(), pl.render())))
        for t, fn in views:
            tb.addAction(t, fn)
        tb.addSeparator()
        self.act_trail = tb.addAction("Vết TCP")
        self.act_trail.setCheckable(True)
        self.act_trail.setChecked(True)
        self.act_trail.toggled.connect(lambda on: (self.trail_actor.SetVisibility(on), pl.render()))
        tb.addAction("Xoá vết", self._clear_trail)
        act_ws = tb.addAction("Vùng làm việc")
        act_ws.setCheckable(True)
        act_ws.toggled.connect(lambda on: (self.work_actor.SetVisibility(on), pl.render()))
        self.act_ghost = tb.addAction("Bóng robot thật")
        self.act_ghost.setCheckable(True)
        self.act_ghost.setChecked(True)
        self.act_block = tb.addAction("Chặn va chạm")
        self.act_block.setCheckable(True)
        self.act_block.setToolTip("Kiểm tra quỹ đạo/jog: dừng nếu tay kẹp sắp va vào thân robot hoặc sàn")
        self.act_block.toggled.connect(self._on_block_toggled)
        self.act_block.setChecked(self.coll is not None)
        self.act_block.setEnabled(self.coll is not None)
        tb.addSeparator()
        tb.addAction("Chụp ảnh", self._screenshot)

    def _on_block_toggled(self, on):
        self.motion.collide = self._collide if (on and self.coll is not None) else None

    def _object_hits(self, q):
        if self.gsim is None or not self.obj["visible"]:
            return []
        if self.obj["attached"]:
            _, _, M = self.gsim.gripper_frame(q)
            return ["vật chạm sàn"] if (M @ self.obj["rel"])[2, 3] < -0.5 else []
        return self.gsim.evaluate(q, self.obj)["hits"]

    def _collide(self, q):
        return self.coll.check(q) + self._object_hits(q)

    def _update_collision(self, q):
        if self.coll is None:
            return
        hits = self.coll.check(q) + (self.grip["hits"] if self.gsim else [])
        if hits != self.hits:
            red = bool(hits)
            for a in self.actors.get("tool", []):
                a.prop.color = "#ff2d2d" if red else self.base_colors[a._part]
            self.hits = hits
        self.lbl_coll.setText("Va chạm: " + (", ".join(hits) if hits else "không"))
        self.lbl_coll.setStyleSheet("color:%s" % ("#b91c1c" if hits else "#15803d"))

    def _set_cam(self, pos):
        self.plotter.camera_position = pos
        self.plotter.render()

    # ------------------------------------------------------------------ cập nhật mô hình
    def _set_group_mats(self, actors, q):
        mats = self.kin.group_matrices(q, self.cfg["pulleys"])
        for gname, lst in actors.items():
            m = pv.vtkmatrix_from_array(mats[gname])
            for a in lst:
                a.SetUserMatrix(m)

    def _apply_q(self, q, force=False):
        if not force and self.last_q_drawn is not None and np.allclose(q, self.last_q_drawn, atol=1e-4):
            return False
        self.last_q_drawn = np.array(q)
        self._set_group_mats(self.actors, q)
        p = self.kin.fk(q)
        self.tcp_actor.SetUserMatrix(pv.vtkmatrix_from_array(translate(*p[:3])))
        m = translate(*p[:3]) @ rot_z_about(0, 0, p[3])
        self.tcp_dir.SetUserMatrix(pv.vtkmatrix_from_array(m))
        self._update_gripper(q)
        self._update_collision(q)
        if self.act_trail.isChecked():
            if not self.trail or np.linalg.norm(np.array(self.trail[-1]) - p[:3]) > 0.5:
                self.trail.append(tuple(p[:3]))
                if len(self.trail) >= 2:
                    self.trail_poly.copy_from(pv.lines_from_points(np.array(self.trail)))
        return True

    # ------------------------------------------------------------------ tay kep
    def _deform_fingers(self, curl, ghost=False):
        if self.gsim is None or (self.curl_drawn[ghost] is not None and abs(curl - self.curl_drawn[ghost]) < 1e-3):
            return
        self.curl_drawn[ghost] = curl
        for part, meshes in self.finger_meshes.items():
            m = meshes[1 if ghost else 0]
            m.points = self.gsim.deform(part, curl)
            m.compute_normals(cell_normals=False, split_vertices=False, inplace=True, auto_orient_normals=False)

    def _update_gripper(self, q):
        if self.gsim is None:
            return
        ob = self.obj
        st = self.gsim.evaluate(q, ob)
        axy, _, Mt = self.gsim.gripper_frame(q)
        blocked = any(h.startswith("vật quá to") or h.startswith("vật chạm ống") for h in st["hits"])
        if (not ob["attached"] and ob["visible"] and st["centered"] and st["contact"] is not None
                and not blocked and st["curl_cmd"] >= st["contact"] - 1e-3):
            ob["M"][:2, 3] = axy                                   # ngon tu dinh tam vat
            ob["attached"], ob["contact"] = True, st["contact"]
            ob["rel"] = np.linalg.inv(Mt) @ ob["M"]
            self._log("Đã kẹp vật Ø%.0f (ngón cong %.0f%%)" % (2 * ob["R"], 100 * st["contact"] / self.gsim.c_self))
            st = self.gsim.evaluate(q, ob)
        elif ob["attached"]:
            ob["M"] = Mt @ ob["rel"]
            if st["curl_cmd"] < ob["contact"] - 0.01:
                ob["attached"] = False
                h = ob["M"][2, 3]
                self._log("Nhả vật — rơi %.0f mm xuống sàn" % h if h > 1 else "Đã đặt vật")
                M = np.eye(4)
                M[:3, :3] = ob["M"][:3, :3]
                M[:2, 3] = ob["M"][:2, 3]
                ob["M"] = M
                st = self.gsim.evaluate(q, ob)
        self.grip = st
        self.obj_actor.SetUserMatrix(pv.vtkmatrix_from_array(ob["M"]))
        self._deform_fingers(st["curl"])

    def _grip_state_text(self):
        if self.gsim is None:
            return ""
        st, ob = self.grip, self.obj
        if ob["attached"]:
            txt = "Đang kẹp vật Ø%.0f" % (2 * ob["R"])
        elif st["curl"] < 0.01:
            txt = "Mở"
        elif st["curl"] >= self.gsim.c_self - 1e-3:
            txt = "Đóng hết (đầu ngón chạm nhau)"
        else:
            txt = "Đang đóng"
        return "%s — ngón cong %.0f%%" % (txt, 100 * st["curl"] / self.gsim.c_self)

    def _set_g(self, g):
        self.run_loop = False
        self._on_joint_changed(4, float(g))
        self.rows[4].set_value(float(g))

    def _obj_changed(self, *_):
        if self.gsim is None:
            return
        ob = self.obj
        ob["R"], ob["H"] = self.obj_d.value() / 2, self.obj_h.value()
        ob["visible"] = self.chk_obj.isChecked()
        if not ob["attached"]:
            ob["M"] = translate(self.obj_x.value(), self.obj_y.value(), 0)
        self.obj_poly.copy_from(pv.Cylinder(center=(0, 0, ob["H"] / 2), direction=(0, 0, 1),
                                            radius=ob["R"], height=ob["H"], resolution=48))
        self.obj_actor.SetVisibility(ob["visible"])
        self._apply_q(self.motion.q, force=True)
        self.plotter.render()

    def _obj_under_gripper(self):
        if self.gsim is None:
            return
        self.obj["attached"] = False
        axy, _, _ = self.gsim.gripper_frame(self.motion.q)
        for w, v in ((self.obj_x, axy[0]), (self.obj_y, axy[1])):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)
        self._obj_changed()

    def _tab_gripper(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        if self.gsim is None:
            v.addWidget(QtWidgets.QLabel("Chưa có cấu hình tay kẹp (mục gripper trong robot_config.json)."))
            return w
        gs = self.gsim
        g_full = gs.g_open + gs.c_self * (gs.g_closed - gs.g_open)
        self.lbl_grip = QtWidgets.QLabel()
        self.lbl_grip.setStyleSheet("font-weight:bold")
        self.bar_curl = QtWidgets.QProgressBar()
        self.bar_curl.setRange(0, 100)
        self.bar_curl.setFormat("Độ cong ngón %p%")
        v.addWidget(self.lbl_grip)
        v.addWidget(self.bar_curl)
        h = QtWidgets.QHBoxLayout()
        for text, g in (("Mở kẹp", gs.g_open), ("Kẹp 50%", (gs.g_open + g_full) / 2), ("Đóng kẹp", g_full)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda _=False, gg=g: self._set_g(gg))
            h.addWidget(b)
        v.addLayout(h)
        info = QtWidgets.QLabel("Servo SG90: %g° = ngón thẳng, %.0f° = đầu ngón chạm nhau. Kẹp được vật trụ "
                                "Ø%.0f–%.0f mm đặt giữa 3 ngón (lệch tâm ≤ %g mm). Khi kẹp, vật đi theo tay kẹp; "
                                "mở kẹp thì vật được đặt/rơi xuống sàn."
                                % (gs.g_open, g_full, gs.min_d + 3, 2 * gs.x_in - 2, gs.center_tol))
        info.setWordWrap(True)
        info.setStyleSheet("color:#555")
        v.addWidget(info)

        box = QtWidgets.QGroupBox("Vật thể (hình trụ) để thử gắp")
        f = QtWidgets.QFormLayout(box)
        self.chk_obj = QtWidgets.QCheckBox("Hiện vật")
        self.chk_obj.setChecked(True)
        self.chk_obj.toggled.connect(self._obj_changed)
        f.addRow(self.chk_obj)
        oc = self.cfg["gripper"].get("object", {})

        def spin(lo, hi, val):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(1)
            s.setValue(val)
            s.setSuffix(" mm")
            s.setKeyboardTracking(False)
            s.valueChanged.connect(self._obj_changed)
            return s
        xy = oc.get("xy", [80, -160])
        self.obj_d = spin(4, 80, oc.get("diameter", 30))
        self.obj_h = spin(5, 120, oc.get("height", 40))
        self.obj_x = spin(-300, 300, xy[0])
        self.obj_y = spin(-400, 200, xy[1])
        f.addRow("Đường kính", self.obj_d)
        f.addRow("Chiều cao", self.obj_h)
        f.addRow("X", self.obj_x)
        f.addRow("Y", self.obj_y)
        b = QtWidgets.QPushButton("Đặt vật ngay dưới tay kẹp")
        b.clicked.connect(self._obj_under_gripper)
        f.addRow(b)
        v.addWidget(box)
        v.addStretch(1)
        return w

    def _clear_trail(self):
        self.trail.clear()
        self.trail_poly.copy_from(pv.lines_from_points(np.array([[0, 0, 0], [0, 0, 0.01]], float)))
        self.plotter.render()

    def _sync_widgets(self, q_cmd):
        for r, v in zip(self.rows, q_cmd):
            r.set_value(v)

    # ------------------------------------------------------------------ vòng lặp chính
    def _tick(self):
        now = time.perf_counter()
        dt = min(0.1, now - self.t_prev)
        self.t_prev = now
        q = self.motion.step(dt).copy()
        for m in self.motion.messages:
            self._log(m, warn=True)
        self.motion.messages.clear()

        self._poll_device(now)

        mirror = self.chk_mirror.isChecked() and self.q_fb is not None
        q_show = self.q_fb if mirror else q
        ghost_q = q if mirror else self.q_fb
        dirty = self._apply_q(q_show)
        show_ghost = ghost_q is not None and self.act_ghost.isChecked() and self.device is not None
        for lst in self.ghosts.values():
            for a in lst:
                a.SetVisibility(show_ghost)
        if show_ghost:
            self._set_group_mats(self.ghosts, ghost_q)
            if self.gsim is not None:
                self._deform_fingers(self.gsim.curl_from_g(ghost_q[4]), ghost=True)
            dirty = True

        if self.motion.mode == "path":
            self._sync_widgets(q)
        for r, v in zip(self.rows, q):
            r.set_actual(v)
        p = self.kin.fk(q_show)
        self.lbl_tcp.setText("TCP  X=%7.1f  Y=%7.1f  Z=%7.1f mm  Yaw=%6.1f°" % tuple(p))
        sz, s1, s2 = self.kin.motor_steps(q)
        self.lbl_steps.setText("Bước  Z=%7d  J1=%7d  J2=%7d   Servo J3=%5.1f° G=%5.1f°" % (sz, s1, s2, q[3] + 90, q[4]))
        st = self.motion.current_label or ("Đang chờ %.1fs" % self.motion.dwell if self.motion.dwell > 0 else "Đứng yên")
        if self.motion.queue:
            st += "  (còn %d lệnh)" % len(self.motion.queue)
        self.lbl_state.setText("Trạng thái: " + st)
        if self.gsim is not None:
            gt = self._grip_state_text()
            self.lbl_grip.setText("Tay kẹp: " + gt)
            self.bar_curl.setValue(int(round(100 * self.grip["curl"] / self.gsim.c_self)))

        if self.run_loop and not self.motion.busy:
            if self.chk_loop.isChecked():
                self._enqueue_program()
            else:
                self.run_loop = False
                self._log("Chương trình kết thúc")

        if self.device is not None and self.chk_send.isChecked() and now - self.last_send >= SEND_PERIOD:
            if self.last_sent_q is None or not np.allclose(q, self.last_sent_q, atol=1e-3):
                self._send("J " + " ".join("%.3f" % v for v in q), quiet=True)
                self.last_sent_q = q.copy()
            self.last_send = now
        if dirty:
            self.plotter.render()

    # ------------------------------------------------------------------ khớp
    def _on_speed(self, v):
        self.lbl_speed.setText("%d %%" % v)
        self.motion.speed = v / 100.0

    def _on_joint_changed(self, idx, val):
        tgt = self.motion.target.copy() if self.motion.mode == "follow" else self.motion.q.copy()
        tgt[idx] = val
        self.run_loop = False
        self.motion.jog_to(tgt)

    def _go_home(self):
        self.run_loop = False
        self.motion.stop()
        self.motion.enqueue("PTP", self.motion.home, "Home")
        self._sync_widgets(self.motion.home)

    def _stop(self):
        self.run_loop = False
        self.motion.stop()
        self._sync_widgets(self.motion.q)
        if self.device is not None:
            self._send("S")
        self._log("Dừng")

    def _sync_from_fb(self):
        if self.q_fb is None:
            self._log("Chưa có phản hồi từ robot", warn=True)
            return
        self.motion.stop()
        self.motion.q = self.kin.clamp(self.q_fb)
        self.motion.target = self.motion.q.copy()
        self._sync_widgets(self.motion.q)
        self._log("Đã đồng bộ twin theo robot thật")

    # ------------------------------------------------------------------ Descartes
    def _cart_from_current(self):
        p = self.kin.fk(self.motion.q)
        for s, v in zip(self.cart, p):
            s.blockSignals(True)
            s.setValue(v)
            s.blockSignals(False)
        self._preview_target()

    def _cart_values(self):
        return [s.value() for s in self.cart]

    def _preview_target(self):
        x, y, z, yaw = self._cart_values()
        q, msg = self.kin.ik(x, y, z, yaw, q_ref=self.motion.q, elbow=self.elbow.currentText())
        self.lbl_ik.setText(("✔ " if q is not None else "✘ ") + msg +
                            ("" if q is None else "  →  Z=%.1f J1=%.1f J2=%.1f J3=%.1f" % tuple(q[:4])))
        self.lbl_ik.setStyleSheet("color:%s" % ("#15803d" if q is not None else "#b91c1c"))
        self.target_actor.SetUserMatrix(pv.vtkmatrix_from_array(translate(x, y, z)))
        self.target_actor.SetVisibility(True)
        self.plotter.render()
        return q

    def _cart_go(self, kind):
        q = self._preview_target()
        if q is None:
            return
        self.run_loop = False
        self.motion.stop()
        self.motion.enqueue(kind, q, "Descartes")

    def _jog_cart(self, axis, sgn):
        step = float(self.jog_step.currentText()) * sgn
        base = self.motion.queue[-1][1] if self.motion.queue else (
            self.motion.path_q[-1] if self.motion.mode == "path" else self.motion.q)
        p = self.kin.fk(base)
        p[axis] += step
        q, msg = self.kin.ik(p[0], p[1], p[2], wrap180(p[3]), q_ref=base, elbow="auto")
        if q is None:
            self.statusBar().showMessage("Jog: " + msg, 3000)
            return
        if len(self.motion.queue) < 2:
            self.run_loop = False
            self.motion.enqueue("LIN", q, "Jog")

    # ------------------------------------------------------------------ chương trình
    def _refresh_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.program))
        for r, wp in enumerate(self.program):
            vals = [wp["name"], wp["mode"]] + ["%.1f" % v for v in wp["q"]] + ["%.1f" % wp.get("dwell", 0)]
            for c, t in enumerate(vals):
                self.table.setItem(r, c, QtWidgets.QTableWidgetItem(t))
        self.table.blockSignals(False)

    def _teach(self, mode):
        name = "P%d" % (len(self.program) + 1)
        self.program.append({"name": name, "mode": mode, "q": [round(float(v), 2) for v in self.motion.q], "dwell": 0.0})
        self._refresh_table()
        self.table.selectRow(len(self.program) - 1)
        self._log("Ghi điểm %s (%s)" % (name, mode))

    def _update_point(self):
        r = self.table.currentRow()
        if 0 <= r < len(self.program):
            self.program[r]["q"] = [round(float(v), 2) for v in self.motion.q]
            self._refresh_table()
            self.table.selectRow(r)

    def _del_point(self):
        r = self.table.currentRow()
        if 0 <= r < len(self.program):
            del self.program[r]
            self._refresh_table()

    def _move_point(self, d):
        r = self.table.currentRow()
        n = r + d
        if 0 <= r < len(self.program) and 0 <= n < len(self.program):
            self.program[r], self.program[n] = self.program[n], self.program[r]
            self._refresh_table()
            self.table.selectRow(n)

    def _on_table_edit(self, item):
        r, c = item.row(), item.column()
        wp = self.program[r]
        t = item.text().strip()
        try:
            if c == 0:
                wp["name"] = t
            elif c == 1:
                if t.upper() not in ("PTP", "LIN"):
                    raise ValueError
                wp["mode"] = t.upper()
            elif c == 7:
                wp["dwell"] = max(0.0, float(t))
            else:
                v = float(t)
                key = JOINT_KEYS[c - 2]
                lo, hi = self.kin.limits[key]
                if not lo <= v <= hi:
                    raise ValueError
                wp["q"][c - 2] = v
        except ValueError:
            self.statusBar().showMessage("Giá trị không hợp lệ", 3000)
        self._refresh_table()

    def _goto_point(self, r):
        if 0 <= r < len(self.program):
            wp = self.program[r]
            self.run_loop = False
            self.motion.stop()
            self.motion.enqueue(wp["mode"], wp["q"], wp["name"])

    def _enqueue_program(self):
        for wp in self.program:
            self.motion.enqueue(wp["mode"], wp["q"], wp["name"], wp.get("dwell", 0))

    def _run_program(self):
        if not self.program:
            self._log("Chương trình trống — hãy ghi điểm trước", warn=True)
            return
        self.motion.stop()
        self._enqueue_program()
        self.run_loop = True
        self._log("Chạy chương trình (%d điểm)" % len(self.program))

    def _on_move_done(self, label):
        pass

    def _save_program(self):
        os.makedirs(PROG_DIR, exist_ok=True)
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Lưu chương trình", PROG_DIR, "JSON (*.json)")
        if fn:
            with open(fn, "w", encoding="utf-8") as f:
                json.dump({"robot": "TRR", "points": self.program}, f, ensure_ascii=False, indent=1)
            self._log("Đã lưu " + fn)

    def _load_program(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Mở chương trình", PROG_DIR, "JSON (*.json)")
        if fn:
            with open(fn, encoding="utf-8") as f:
                self.program = json.load(f)["points"]
            self._refresh_table()
            self._log("Đã mở %s (%d điểm)" % (fn, len(self.program)))

    # ------------------------------------------------------------------ kết nối
    def _refresh_ports(self):
        cur = self.port.currentText()
        self.port.clear()
        self.port.addItems(dl.list_ports())
        if cur:
            self.port.setCurrentText(cur)

    def _toggle_connect(self):
        if self.device is not None:
            self.device.close()
            self.device = None
            self.q_fb = None
            self.btn_conn.setText("Kết nối")
            self.lbl_link.setText("Chưa kết nối")
            self._log("Ngắt kết nối")
            return
        name = self.port.currentText()
        try:
            if name == dl.SIM_NAME:
                self.device = dl.SimDevice(self.motion.vmax, home=self.motion.home)
                self.device.q = self.motion.q.copy()
                self.device.target = self.motion.q.copy()
            else:
                self.device = dl.SerialDevice(name.split(" - ")[0], int(self.baud.currentText()))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Lỗi kết nối", str(e))
            self.device = None
            return
        self.last_sent_q = None
        self.btn_conn.setText("Ngắt")
        self._log("Đã kết nối " + name)
        self._send("?")

    def _send(self, line, quiet=False):
        if self.device is None:
            return
        try:
            self.device.write_line(line)
            if not quiet:
                self._log("→ " + line)
        except Exception as e:
            self._log("Lỗi gửi: %s" % e, warn=True)
            self._toggle_connect()

    def _send_raw(self):
        t = self.cmd_line.text().strip()
        if t:
            self._send(t)
            self.cmd_line.clear()

    def _poll_device(self, now):
        if self.device is None:
            return
        try:
            lines = self.device.read_lines()
        except Exception as e:
            self._log("Lỗi đọc: %s" % e, warn=True)
            self._toggle_connect()
            return
        for ln in lines:
            q = dl.parse_feedback(ln)
            if q is not None:
                self.q_fb = q
                self.t_fb = now
            else:
                self._log("← " + ln)
        age = now - self.t_fb if self.q_fb is not None else None
        if age is None:
            self.lbl_link.setText("Đã kết nối — chưa nhận phản hồi P ...")
        else:
            self.lbl_link.setText("Phản hồi cách đây %.0f ms" % (age * 1000) + ("  ⚠ mất tín hiệu" if age > 1 else ""))
        if self.q_fb is not None:
            q = self.motion.q
            for r in range(5):
                err = q[r] - self.q_fb[r]
                self.fb_table.item(r, 0).setText("%.2f" % q[r])
                self.fb_table.item(r, 1).setText("%.2f" % self.q_fb[r])
                it = self.fb_table.item(r, 2)
                it.setText("%.2f" % err)
                it.setForeground(QtGui.QColor("#b91c1c" if abs(err) > 2 else "#15803d"))

    # ------------------------------------------------------------------ khác
    def _log(self, msg, warn=False):
        t = time.strftime("%H:%M:%S")
        self.log.appendPlainText("[%s] %s%s" % (t, "⚠ " if warn else "", msg))
        if warn:
            self.statusBar().showMessage(msg, 5000)

    def _screenshot(self):
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Lưu ảnh", os.path.join(HERE, "twin.png"), "PNG (*.png)")
        if fn:
            self.plotter.screenshot(fn)
            self._log("Đã lưu ảnh " + fn)

    def closeEvent(self, e):
        self.timer.stop()
        if self.device is not None:
            self.device.close()
        self.plotter.close()
        super().closeEvent(e)


def demo_pick_place(w):
    """Chương trình mẫu: gắp vật trụ ở vị trí cấu hình rồi đặt sang chỗ khác."""
    k, gs = w.kin, w.gsim
    if gs is None:
        return [{"name": "Home", "mode": "PTP", "q": list(w.motion.home), "dwell": 0}]
    ox, oy = w.obj["M"][0, 3], w.obj["M"][1, 3]
    px, py = -60.0, -170.0
    g_open, g_close = gs.g_open, gs.g_open + gs.c_self * (gs.g_closed - gs.g_open)
    pts, q = [], w.motion.home.copy()
    for name, mode, x, y, z, g, dwell in (("Tren vat", "PTP", ox, oy, 60, g_open, 0.2),
                                          ("Ha xuong", "LIN", ox, oy, 2, g_open, 0.2),
                                          ("Kep", "PTP", ox, oy, 2, g_close, 0.4),
                                          ("Nhac len", "LIN", ox, oy, 70, g_close, 0),
                                          ("Sang cho dat", "PTP", px, py, 70, g_close, 0),
                                          ("Ha dat", "LIN", px, py, 2, g_close, 0.2),
                                          ("Nha", "PTP", px, py, 2, g_open, 0.4),
                                          ("Rut len", "LIN", px, py, 60, g_open, 0)):
        qq, msg = k.ik(x, y, z, -90.0, q_ref=q)
        if qq is None:
            w._log("Demo: %s — %s" % (name, msg), warn=True)
            continue
        qq[4] = g
        q = qq
        pts.append({"name": name, "mode": mode, "q": [round(float(v), 2) for v in q], "dwell": dwell})
    pts.append({"name": "Home", "mode": "PTP", "q": [float(v) for v in w.motion.home], "dwell": 0})
    return pts


def main():
    QtWidgets.QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = TwinWindow()
    w.show()
    if "--demo" in sys.argv:
        w.program = demo_pick_place(w)
        w._refresh_table()
        w._run_program()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
