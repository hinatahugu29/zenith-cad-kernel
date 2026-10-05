"""**こちらの面の上の点が、OCC の立体からどれだけ離れているか**（4-664）。

**なぜ別の口にしたか**: `tools/occ_face_area_reference.py` は同じことを
しますが、**その前に 37 枚すべてを `tessellate(1.0e-5)` で割ります**
（体積への寄与を出すため）。**この機械ではそこで 1.2 GB 使い、
30 分で終わりません**——**点を 1 つも渡さなくても終わりません。**
**距離だけが要るときに、割る必要はありません。**

**測るのは 4-561 と同じ**——**こちらの面の上の点を OCC に渡して、
内か外かと、立体までの距離を聞く**。**面ごとにまとめます。**

**途中で切られても残るように、面ごとに flush します**（この機械は
連続 30 分で切られます。4-660）。

使い方:
    ZENITH_FACE_POINTS=<点の一覧> py tools/occ_face_distance.py [step ファイル]

点の一覧は `ZENITH_FACE_POINTS=<先> read_volume_convergence_probe` が書きます
（1 行が「面番号 x y z」）。
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

sample_file = os.environ.get("ZENITH_FACE_POINTS")
if not sample_file or not os.path.exists(sample_file):
    print("**ZENITH_FACE_POINTS に点の一覧を渡してください。**")
    print("  ZENITH_FACE_POINTS=<先> ./target/release/examples/read_volume_convergence_probe")
    raise SystemExit(2)

path = sys.argv[1] if len(sys.argv) > 1 else "reference/OCCT/data/step/linkrods.step"
shape = Part.Shape()
shape.read(path)
solids = shape.Solids
if not solids:
    print("立体がありません")
    raise SystemExit(1)
solid = max(solids, key=lambda s: len(s.Faces))

print(f"{path}")
print(f"立体 {len(solids)} 個、いちばん面の多いもの: 面 {len(solid.Faces)} 枚")
print(f"体積 {solid.Volume:.9f}")
print(flush=True)

# **面ごとにまとめてから測ります**——**面 1 枚ぶん測るたびに印字する**
# ので、**途中で切られても、そこまでは読めます。**
by_face = {}
for line in open(sample_file, encoding="utf-8"):
    parts = line.split()
    if len(parts) != 4:
        continue
    by_face.setdefault(int(parts[0]), []).append(
        (float(parts[1]), float(parts[2]), float(parts[3]))
    )

print("面   点数   内    外    最大距離     中央距離")
print("-" * 56)
grand_in = grand_out = 0
for index in sorted(by_face):
    points = by_face[index]
    inside = outside = 0
    distances = []
    for x, y, z in points:
        point = FreeCAD.Vector(x, y, z)
        # **公差 0 で聞きます**——**内か外かだけが要る**ので（4-561）。
        if solid.isInside(point, 0.0, True):
            inside += 1
        else:
            outside += 1
        distances.append(solid.distToShape(Part.Vertex(point))[0])
    distances.sort()
    grand_in += inside
    grand_out += outside
    print(
        f"{index:<4} {len(points):<6} {inside:<5} {outside:<5} "
        f"{distances[-1]:.4e}  {distances[len(distances) // 2]:.4e}",
        flush=True,
    )

print("-" * 56)
print(f"こちらの面の上の点: 内 {grand_in} 個 / 外 {grand_out} 個")
print("（内に偏るなら、こちらの面は OCC の面より内側を通っています。4-561）")
