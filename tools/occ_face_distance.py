"""**こちらの面の上の点が、OCC の立体からどれだけ離れているか**（4-664）。

**なぜ別の口にしたか**: `tools/occ_face_area_reference.py` は同じことを
しますが、**その前に 37 枚すべてを `tessellate(1.0e-5)` で割ります**
（体積への寄与を出すため）。**この機械ではそこで 1.2 GB 使い、
30 分で終わりません**——**点を 1 つも渡さなくても終わりません。**
**距離だけが要るときに、割る必要はありません。**

**`ZENITH_SURFACE_TOO=1` を立てると、「トリムしていない曲面まで」も
出します**（4-673）——**4-672 で分かったとおり、立体までの距離は
「曲面のずれ」ではなく「トリムの食い違い」を測っています。**
**曲面までの距離が小さく、立体までが大きければ、それは縁の帯**です。
**遅くなります**（点ごとに全部の曲面へ投影するので）。

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

surface_too = os.environ.get("ZENITH_SURFACE_TOO") == "1"
# **帯の中に入った点の割合**（4-675）——**4-672 で、立体までの距離が
# 測っているのは「トリムの食い違い」だと分かりました。** **なら、
# 「どれだけ離れているか」より「面のどれだけが帯か」が要ります。**
# **床（5.4e-10）の 20 倍を境にします**——**床の揺らぎを拾わない範囲で、
# いちばん低い所**。
STRIP = 1.0e-8
if surface_too:
    print("面   点数   帯%     平均距離     最大距離     中央距離     最大(曲面)   中央(曲面)")
    print("-" * 91)
else:
    print("面   点数   帯%     平均距離     最大距離     中央距離")
    print("-" * 67)
grand_in = grand_out = 0
for index in sorted(by_face):
    points = by_face[index]
    inside = outside = 0
    distances = []
    surface_distances = []
    for x, y, z in points:
        point = FreeCAD.Vector(x, y, z)
        # **公差 0 で聞きます**——**内か外かだけが要る**ので（4-561）。
        if solid.isInside(point, 0.0, True):
            inside += 1
        else:
            outside += 1
        distances.append(solid.distToShape(Part.Vertex(point))[0])
        if surface_too:
            # **トリムしていない曲面まで**（4-673）。**いちばん近い 1 枚**。
            best = float("inf")
            for face in solid.Faces:
                try:
                    got = face.Surface.projectPoint(point, "LowerDistance")
                except Exception:
                    continue
                value = got if isinstance(got, float) else min(got)
                if value < best:
                    best = value
            surface_distances.append(best)
    in_strip = sum(1 for d in distances if d > STRIP)
    # **平均距離 × 面積 = その面の帯が持つ体積**（4-675）。**面積は
    # こちらの数**（`ZENITH_FACE_AREAS=1 read_volume_convergence_probe`）と
    # **外で掛けます**——この口は面積を知りません。
    mean = sum(distances) / len(distances)
    distances.sort()
    surface_distances.sort()
    grand_in += inside
    grand_out += outside
    row = (
        f"{index:<4} {len(points):<6} {100.0 * in_strip / len(points):<6.2f}  "
        f"{mean:.4e}  {distances[-1]:.4e}  {distances[len(distances) // 2]:.4e}"
    )
    if surface_too and surface_distances:
        row += (
            f"  {surface_distances[-1]:.4e}  "
            f"{surface_distances[len(surface_distances) // 2]:.4e}"
        )
    print(row, flush=True)

print("-" * 56)
print(f"こちらの面の上の点: 内 {grand_in} 個 / 外 {grand_out} 個")
print("（内に偏るなら、こちらの面は OCC の面より内側を通っています。4-561）")
print()
print(f"**帯%** は、OCC の立体から {STRIP:.0e} より遠い点の割合です（4-675）。")
print("**面が OCC の面よりはみ出している（あるいは足りない）帯の広さ**の目安。")
