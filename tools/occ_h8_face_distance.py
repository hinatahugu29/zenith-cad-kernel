"""**H8 で切った結果**を、OCC の切った結果と面ごとに突き合わせます（4-685）。

**入口（`linkrods` そのもの）では 4-664〜4-684 でやりました。**
**出口（切ったあと）に、同じ物差しを当てます。**

**OCC の切り手は `tools/occ_h8_reference.py` と同じ規則**
（境界箱の 3% 内、高さ 0.47）で、**`--box` は必ず渡してください**（4-510）。

使い方:
    # ① こちらの結果の面の上の点を書き出す
    ZENITH_FACE_POINTS=<先> bash tools/h8_ports.sh difference
    # ② OCC の結果に当てる
    py tools/occ_h8_face_distance.py <点の一覧> <difference|intersection|union> \
        3.1250000000 2.5000000000 0.0000000000 8.1452847075 4.0000000000 2.0000000000

**`ZENITH_SURFACE_TOO=1`** で「トリムしていない曲面まで」も出します
（遅くなります。4-673）。
"""
import os
import sys

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

freecad_bin = r"C:\Program Files\FreeCAD 1.1\bin"
if freecad_bin not in sys.path:
    sys.path.insert(0, freecad_bin)
if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin)
    except Exception:
        pass

import FreeCAD  # noqa: E402
import Part  # noqa: E402

SAMPLE = "reference/OCCT/data/step/linkrods.step"
STRIP = 1.0e-8

if len(sys.argv) < 9:
    print("**点の一覧と演算名と --box の 6 数を渡してください。**")
    print(__doc__)
    raise SystemExit(2)

points_file, operation = sys.argv[1], sys.argv[2]
box = [float(x) for x in sys.argv[3:9]]
if not os.path.exists(points_file):
    print(f"**{points_file} がありません。**")
    raise SystemExit(2)

shape = Part.Shape()
shape.read(SAMPLE)
solids = shape.Solids
subject = max(solids, key=lambda s: len(s.Faces)) if solids else shape

size = (box[3] - box[0], box[4] - box[1], box[5] - box[2])
inset, height = 0.03, 0.47
cutter = Part.makeBox(
    size[0] * (1.0 - inset * 2.0), size[1] * (1.0 - inset * 2.0), size[2] * height
)
cutter.translate(
    FreeCAD.Vector(
        box[0] + size[0] * inset,
        box[1] + size[1] * inset,
        box[2] + size[2] * (height * 0.5),
    )
)

if operation == "difference":
    result = subject.cut(cutter)
elif operation == "intersection":
    result = subject.common(cutter)
elif operation == "union":
    result = subject.fuse(cutter)
else:
    print(f"**知らない演算です: {operation}**（difference / intersection / union）")
    raise SystemExit(2)

print(f"OCC の {operation}: 体積 {result.Volume:.6f}、面 {len(result.Faces)} 枚")
print(flush=True)

surface_too = os.environ.get("ZENITH_SURFACE_TOO") == "1"
by_face = {}
for line in open(points_file, encoding="utf-8"):
    parts = line.split()
    if len(parts) != 4:
        continue
    by_face.setdefault(int(parts[0]), []).append(
        (float(parts[1]), float(parts[2]), float(parts[3]))
    )

header = "面   点数   帯%     最大距離     中央距離"
if surface_too:
    header += "     最大(曲面)   中央(曲面)"
print(header)
print("-" * (len(header) + 12))
worst_all = 0.0
for index in sorted(by_face):
    points = by_face[index]
    distances = []
    surface = []
    for x, y, z in points:
        point = FreeCAD.Vector(x, y, z)
        distances.append(result.distToShape(Part.Vertex(point))[0])
        if surface_too:
            best = float("inf")
            for face in result.Faces:
                try:
                    got = face.Surface.projectPoint(point, "LowerDistance")
                except Exception:
                    continue
                value = got if isinstance(got, float) else min(got)
                if value < best:
                    best = value
            surface.append(best)
    in_strip = sum(1 for d in distances if d > STRIP)
    distances.sort()
    surface.sort()
    worst_all = max(worst_all, distances[-1])
    row = (
        f"{index:<4} {len(points):<6} {100.0 * in_strip / len(points):<6.2f}  "
        f"{distances[-1]:.4e}  {distances[len(distances) // 2]:.4e}"
    )
    if surface_too and surface:
        row += f"  {surface[-1]:.4e}  {surface[len(surface) // 2]:.4e}"
    print(row, flush=True)
print("-" * (len(header) + 12))
print(f"いちばん離れた点: {worst_all:.4e}")
