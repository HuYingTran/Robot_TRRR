# Xuất mesh từng chi tiết của robot.FCStd (toạ độ global, tư thế home) ra thư mục meshes/
# Cách chạy:
#   - Dòng lệnh: freecadcmd export_meshes.py          (đọc robot.FCStd đã lưu)
#   - Trong FreeCAD GUI (đang mở robot): exec(open(r"...\digital_twin\export_meshes.py", encoding="utf-8").read())
#     -> dùng luôn bản đang mở, kể cả thay đổi chưa lưu.
# Link trong EXPAND (tay kẹp SoftGripper) được tách thành từng chi tiết con: <Link>__<Object>.stl
import os, json
import FreeCAD as App
import MeshPart

_f = globals().get("__file__", "")
HERE = os.path.dirname(os.path.abspath(_f)) if os.path.basename(_f) == "export_meshes.py" else r"D:\FreeCad\robot_TRR\digital_twin"
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "meshes")
os.makedirs(OUT, exist_ok=True)

SKIP_SUFFIX = ("Tendons_ref",)   # dây cước: quá mảnh, bỏ qua
EXPAND = ("SoftGripper",)


def write_mesh(name, shp, label, src, info):
    m = MeshPart.meshFromShape(Shape=shp, LinearDeflection=0.3, AngularDeflection=0.35, Relative=False)
    m.write(os.path.join(OUT, name + ".stl"))
    bb = shp.BoundBox
    info[name] = dict(label=label, file=src, facets=m.CountFacets,
                      bbox=[bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax])
    print(name, label, src, m.CountFacets, ["%.1f" % v for v in info[name]["bbox"]])


docs = App.listDocuments()
doc = docs["robot"] if "robot" in docs else App.openDocument(os.path.join(ROOT, "robot.FCStd"))
doc.recompute()
info = {}
for o in doc.Objects:
    if o.TypeId != "App::Link":
        continue
    lo = o.getLinkedObject(True)
    if lo is None or o.Shape.isNull():
        print("SKIP", o.Name)
        continue
    src = os.path.basename(lo.Document.FileName)
    if o.Name in EXPAND and lo.TypeId == "App::Part":
        for c in lo.Group:
            if not hasattr(c, "Shape") or c.Shape.isNull() or c.Name.endswith(SKIP_SUFFIX):
                continue
            s = c.Shape.copy()
            s.Placement = o.Placement.multiply(lo.Placement).multiply(c.Placement)
            write_mesh(o.Name + "__" + c.Name, s, o.Label + "/" + c.Label, src, info)
    else:
        write_mesh(o.Name, o.Shape, o.Label, src, info)
with open(os.path.join(OUT, "parts.json"), "w") as f:
    json.dump(info, f, indent=1)
print("DONE")
