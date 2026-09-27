"""**OCC に、読んだ立体の面ごとの面積を出させます**（4-560）。

**なぜ要るか**: こちらが読んだ `linkrods.step` の体積は、OCC より
**1.234e-4 小さい**（4-559）。**位相は合っています**（面 37 枚、稜 114 本
すべて 2 面が使用）。**なら、どれかの面の形が違う**はずです。
**面ごとに並べれば、どの面かが分かります。**

使い方: `py tools/occ_face_area_reference.py [step ファイル]`
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
print(f"表面積 {solid.Area:.9f}")
print()
# **こちらが吐いた点が、OCC の立体の内か外か**（4-561）。
#
# **面積は合っていて体積だけ小さい**（4-560）——**なら、面の位置**です。
# **こちらの面の上の点を OCC に渡して、内か外かを聞きます。**
# **一様に「内」なら、こちらの面は OCC の面より内側を通っています。**
#
# 使い方: 先に `ZENITH_FACE_POINTS=<書き出し先> read_volume_convergence_probe`。
sample_file = os.environ.get("ZENITH_FACE_POINTS")
if sample_file and os.path.exists(sample_file):
    import collections

    inside = collections.Counter()
    outside = collections.Counter()
    worst = {}
    for line in open(sample_file, encoding="utf-8"):
        parts = line.split()
        if len(parts) != 4:
            continue
        index = int(parts[0])
        point = FreeCAD.Vector(float(parts[1]), float(parts[2]), float(parts[3]))
        # **公差 0 で聞きます**——**内か外かだけが要る**ので。
        if solid.isInside(point, 0.0, True):
            inside[index] += 1
        else:
            outside[index] += 1
        distance = solid.distToShape(Part.Vertex(point))[0]
        if distance > worst.get(index, 0.0):
            worst[index] = distance
    total_in = sum(inside.values())
    total_out = sum(outside.values())
    print()
    print(f"こちらの面の上の点: 内 {total_in} 個 / 外 {total_out} 個")
    print("  （内に偏るなら、こちらの面は OCC の面より内側を通っています）")
    rows = sorted(worst.items(), key=lambda kv: -kv[1])[:10]
    print("  面までの距離がいちばん大きい 10 枚:")
    for index, distance in rows:
        print(
            f"    面{index:<3} 最大 {distance:.3e}"
            f"  内 {inside.get(index, 0)} / 外 {outside.get(index, 0)}"
        )
    print()

print("面ごとの面積と、体積への寄与（OCC）:")
# **体積への寄与**は発散定理の各面の項 ∫ (1/3) r·n dA です（4-561）。
# **面積では体積の内訳になりません**——**面積が合っていても、面が
# 内側に在れば体積は減ります**（4-560）。**同じ式を、こちらでも
# 同じように三角形から積みます**ので、突き合わせられます。
total = 0.0
for index, face in enumerate(solid.Faces):
    kind = type(face.Surface).__name__
    nodes, triangles = face.tessellate(1.0e-5)
    term = 0.0
    for a, b, c in triangles:
        pa, pb, pc = nodes[a], nodes[b], nodes[c]
        # (1/6) * pa . (pb x pc)
        cross = (
            pb.y * pc.z - pb.z * pc.y,
            pb.z * pc.x - pb.x * pc.z,
            pb.x * pc.y - pb.y * pc.x,
        )
        term += (pa.x * cross[0] + pa.y * cross[1] + pa.z * cross[2]) / 6.0
    if face.Orientation == "Reversed":
        term = -term
    total += term
    print(f"  面{index:<3} {kind:<24} 面積 {face.Area:.9f}  寄与 {term:+.9f}")
print(f"  寄与の合計 {total:.9f}（OCC の体積 {solid.Volume:.9f}）")
