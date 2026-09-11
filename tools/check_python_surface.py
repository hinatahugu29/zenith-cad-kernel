"""**本物の Python から、拡張モジュールを触る**（4-402）。

なぜ要るのか
------------
`crates/zenith_py/tests/binding_gate_test.rs` は、**Rust の関数として**
呼んでいます（`Python::with_gil` を通らないので、インタプリタ無しで
測れます。4-198）。

**それは「Python から使える」ことの証明にはなりません。**

実際、**`volume` は Python では getter** でした。Rust では
`solid.volume()` ですが、**Python では `solid.volume`** です。
`solid.volume()` と書くと `TypeError: 'float' object is not callable`
になります。**Rust の試験では、この違いは 1 度も見えません。**

**4-400 で「カーネルにあると Blender から使えるは別」と書いた**その差が、
**試験の側にも**ありました。

使い方
------
    cargo build --release -p zenith_py
    py tools/check_python_surface.py

**`abi3-py310` なので、3.10 以降ならどの Python でも入ります。**
`--python` で使うインタプリタを指せます。

読み方
------
**閉じた式と突き合わせます。** 断るべきものが断られるかも見ます。
**1 つでも食い違えば非ゼロで終わります。**
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_probe_source():
    """子プロセスで走らせる本体。**別の Python で回せるように**します。"""
    return r'''
import json, math, sys

import zenith_cad as z

def value_of(holder, name):
    """**getter とメソッドの両方を受けます。**

    Python 側が getter か method かは、`#[getter]` の有無で決まります。
    **ここで固定してしまうと、変わったときに気づけません**ので、
    両方を受けて、**どちらだったかを印字**します。
    """
    attribute = getattr(holder, name)
    if callable(attribute):
        return attribute(), "method"
    return attribute, "getter"

rows = []
S = z.Solid

rect = json.dumps({"points": [[0, 0], [40, 0], [40, 30], [0, 30]],
                   "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]})
volume, kind = value_of(S.from_sketch_extruded(rect, 7.0), "volume")
rows.append(("スケッチを押し出す", volume, 40 * 30 * 7.0))

hole = json.dumps({
    "points": [[0, 0], [40, 0], [40, 30], [0, 30], [20, 15],
               [25, 15], [20, 20], [15, 15], [20, 10]],
    "lines": [[0, 1], [1, 2], [2, 3], [3, 0]],
    "arcs": [{"centre": 4, "start": 5, "end": 6},
             {"centre": 4, "start": 6, "end": 7},
             {"centre": 4, "start": 7, "end": 8},
             {"centre": 4, "start": 8, "end": 5}]})
rows.append(("穴のあるスケッチを押し出す",
             value_of(S.from_sketch_extruded(hole, 7.0), "volume")[0],
             (40 * 30 - math.pi * 25) * 7.0))

ring = json.dumps({"points": [[10, 0], [14, 0], [14, 6], [10, 6]],
                   "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]})
rows.append(("スケッチを回す（パップス）",
             value_of(S.from_sketch_revolved(ring, 0.0, 0.0, 0.0, 1.0), "volume")[0],
             2 * math.pi * 12.0 * 24.0))

rows.append(("箱の体積", value_of(S.box(10.0, 20.0, 30.0), "volume")[0], 6000.0))

# **密度は、慣性にだけ効きます**（4-414）。
#
# **2026/09/09 まで、この引数は受け取って捨てられていました**——
# 引数名が `_density` で、本体からは読まれていません。**鋼の 7850 を
# 渡しても密度 1 の慣性が返り、7850 倍ずれた値がもっともらしい数字
# として返っていました。** **Rust の試験では見えません**——
# **この口は Python にしかありません。**
#
# 原点を隅に置いた直方体の、原点まわり: Ixx = ρ V (dy² + dz²) / 3
DX, DY, DZ = 40.0, 20.0, 10.0
BOX_V = DX * DY * DZ
for rho in (1.0, 7850.0):
    vol, _surf, _centre, inertia = z.compute_box_mass_properties(DX, DY, DZ, rho)
    rows.append(("箱の体積（密度 %g）" % rho, vol, BOX_V))
    rows.append(("箱の Ixx（密度 %g）" % rho,
                 inertia[0], rho * BOX_V * (DY * DY + DZ * DZ) / 3.0))
    rows.append(("箱の Izz（密度 %g）" % rho,
                 inertia[2], rho * BOX_V * (DX * DX + DY * DY) / 3.0))

# **形を作る口を、閉じた式と突き合わせます**（4-420）。
#
# **なぜ要るのか**: 4-415 の `check_python_arguments.py` は
# **「その引数が読まれているか」しか見ません**。**読んだうえで
# 渡し先を取り違えていれば、答えは動くので通ります**——
# **`dy` と `dz` を入れ替えても、両方「効く」**のです。
# **閉じた式と、囲み箱**が、そこを押さえます。
#
# **刻みは 64**。**曲がった形は、その刻みのぶんだけずれます**ので、
# **許容はそれぞれに書いてあります**（**一律にすると、緩い所と
# 厳しすぎる所ができます**）。
DIVISIONS = 64


def volume_and_box(mesh):
    """体積と、囲み箱。

    **`measured` という名前にして、1 度つまずきました**（4-420）——
    **上の表のループが `for name, measured, expected in rows` で
    同じ名前を潰します。** **`'float' object is not callable`** に
    なりました。**同じ名前は、離れていても当たります。**
    """
    volume, _ = value_of(mesh, "volume")
    points = getattr(mesh, "vertices")
    if callable(points):
        points = points()
    box = [
        (min(point[axis] for point in points), max(point[axis] for point in points))
        for axis in range(3)
    ]
    return volume, box


HEX = lambda across: 0.5 * math.sqrt(3.0) * across * across

# (名前, 作る式, 閉じた式, 許容, 囲み箱〈None なら見ない〉)
SHAPES = [
    ("箱 10x20x30", lambda: z.make_box(10.0, 20.0, 30.0, DIVISIONS, DIVISIONS),
     6000.0, 1e-12, [(0.0, 10.0), (0.0, 20.0), (0.0, 30.0)]),
    ("円柱 r5 h12", lambda: z.make_cylinder(5.0, 12.0, DIVISIONS, DIVISIONS),
     math.pi * 25.0 * 12.0, 1e-3, [(-5.0, 5.0), (-5.0, 5.0), (0.0, 12.0)]),
    ("円錐台 r6->r2 h10", lambda: z.make_cone(6.0, 2.0, 10.0, DIVISIONS, DIVISIONS),
     math.pi * 10.0 / 3.0 * (36.0 + 12.0 + 4.0), 1e-3,
     [(-6.0, 6.0), (-6.0, 6.0), (0.0, 10.0)]),
    ("球 r7", lambda: z.make_sphere(7.0, DIVISIONS, DIVISIONS),
     4.0 / 3.0 * math.pi * 343.0, 1e-3, [(-7.0, 7.0), (-7.0, 7.0), (-7.0, 7.0)]),
    ("トーラス R12 r4", lambda: z.make_torus(12.0, 4.0, DIVISIONS, DIVISIONS),
     2.0 * math.pi ** 2 * 12.0 * 16.0, 1e-3,
     [(-16.0, 16.0), (-16.0, 16.0), (-4.0, 4.0)]),
    ("正六角柱 r8 h10", lambda: z.make_regular_prism(6, 8.0, 10.0, DIVISIONS, DIVISIONS),
     0.5 * 6.0 * 64.0 * math.sin(2.0 * math.pi / 6.0) * 10.0, 1e-12, None),
    ("穴あき箱 30x30x10 r4",
     lambda: z.make_drilled_box(30.0, 30.0, 10.0, 4.0, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 10.0 - math.pi * 16.0 * 10.0, 1e-3, None),
    ("六角ナット S16 穴4.25 t8",
     lambda: z.make_hex_nut(16.0, 4.25, 8.0, DIVISIONS, DIVISIONS),
     HEX(16.0) * 8.0 - math.pi * 4.25 ** 2 * 8.0, 1e-3, None),
    ("六角ボルト S16 頭6.4 r5 L30",
     lambda: z.make_hex_bolt(16.0, 6.4, 5.0, 30.0, DIVISIONS, DIVISIONS),
     HEX(16.0) * 6.4 + math.pi * 25.0 * 30.0, 1e-3, None),
    # **`open_face_index` を渡しても、開くのは 1 面**です
    # （4-420 で 1 度、2 面ぶん引く式を書いて外しました）。
    ("片面のない箱 30x30x20 t2",
     lambda: z.make_hollow_box(30.0, 30.0, 20.0, 2.0, 1, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 20.0 - 26.0 * 26.0 * 18.0, 1e-12, None),
    ("両面のない箱 30x30x20 t2",
     lambda: z.make_through_hollow_box(30.0, 30.0, 20.0, 2.0, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 20.0 - 26.0 * 26.0 * 20.0, 1e-12, None),
    ("上のない箱 30x30x20 t2",
     lambda: z.make_open_box(30.0, 30.0, 20.0, 2.0, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 20.0 - 26.0 * 26.0 * 18.0, 1e-12, None),
    ("直線に掃いた管 L20 r3",
     lambda: z.make_sweep_pipe([[0.0, 0.0, 0.0], [0.0, 0.0, 20.0]], 3.0, 32,
                               DIVISIONS, DIVISIONS),
     math.pi * 9.0 * 20.0, 1e-3, None),
    ("回した環（パップス）",
     lambda: z.make_revolve_solid([[10.0, 0.0, 0.0], [14.0, 0.0, 0.0],
                                   [14.0, 0.0, 6.0], [10.0, 0.0, 6.0]],
                                  [0.0, 0.0, 0.0], [0.0, 0.0, 1.0],
                                  DIVISIONS, DIVISIONS),
     2.0 * math.pi * 12.0 * 24.0, 1e-3, None),
    ("段付き軸 3段",
     lambda: z.make_stepped_shaft([(10.0, 20.0), (6.0, 15.0), (8.0, 10.0)],
                                  DIVISIONS, DIVISIONS),
     math.pi * (100.0 * 20.0 + 36.0 * 15.0 + 64.0 * 10.0), 1e-3, None),
    ("フランジ",
     lambda: z.make_circular_flange(40.0, 10.0, 15.0, 28.0, 4, 3.5,
                                    DIVISIONS, DIVISIONS),
     math.pi * 10.0 * (1600.0 - 225.0 - 4.0 * 3.5 ** 2), 1e-3, None),
    ("ざぐり穴の箱",
     lambda: z.make_counterbore_hole_box(50.0, 50.0, 25.0, 4.0, 8.0, 5.0,
                                         DIVISIONS, DIVISIONS),
     50.0 * 50.0 * 25.0 - math.pi * 16.0 * 25.0 - math.pi * (64.0 - 16.0) * 5.0,
     1e-3, None),
    ("面取りした箱 30x20x10 c2",
     lambda: z.make_chamfered_box(30.0, 20.0, 10.0, 2.0, DIVISIONS, DIVISIONS),
     30.0 * 20.0 * 10.0 - 4.0 * (0.5 * 2.0 * 2.0 * 10.0), 1e-12, None),
    ("角丸めした箱 30x20x10 r3",
     lambda: z.make_filleted_box(30.0, 20.0, 10.0, 3.0, DIVISIONS, DIVISIONS),
     30.0 * 20.0 * 10.0 - 4.0 * (9.0 - math.pi * 9.0 / 4.0) * 10.0, 1e-3, None),
    ("鏡像の箱 10x20x30",
     lambda: z.make_mirror_box(10.0, 20.0, 30.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                               DIVISIONS, DIVISIONS),
     6000.0, 1e-12, None),

    # **組み合わせる口**（4-421）。**`op_type` は 0=和 / 1=差 / 2=積**
    # ——**既定は 1、つまり差**です（**和ではありません**。
    # 4-421 で 1 度、1 つずらして読んで「33% ずれる」と書きかけました）。
    ("箱の和（厳密）",
     lambda: z.make_exact_box_boolean(20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
                                      10.0, 10.0, 30.0, [5.0, 5.0, -5.0],
                                      0, DIVISIONS, DIVISIONS),
     8000.0 + 3000.0 - 2000.0, 1e-12, None),
    ("箱の差（厳密）",
     lambda: z.make_exact_box_boolean(20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
                                      10.0, 10.0, 30.0, [5.0, 5.0, -5.0],
                                      1, DIVISIONS, DIVISIONS),
     8000.0 - 2000.0, 1e-12, None),
    ("箱の積（厳密）",
     lambda: z.make_exact_box_boolean(20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
                                      10.0, 10.0, 30.0, [5.0, 5.0, -5.0],
                                      2, DIVISIONS, DIVISIONS),
     2000.0, 1e-12, None),
    ("箱に丸穴（厳密）30x30x20 r4",
     lambda: z.make_exact_drill_boolean(30.0, 30.0, 20.0, [0.0, 0.0, 0.0],
                                        4.0, 40.0, [15.0, 15.0, -10.0],
                                        [0.0, 0.0, 1.0], 1, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 20.0 - math.pi * 16.0 * 20.0, 1e-3, None),
    # **プリズマトイドの式**（h/6 (A1 + 4Am + A2)）。**上下が相似な
    # 四角形なら厳密**です。
    ("ロフト 正方形10→8 h12",
     lambda: z.make_loft_solid([[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0],
                                 [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]],
                                [[1.0, 1.0, 12.0], [9.0, 1.0, 12.0],
                                 [9.0, 9.0, 12.0], [1.0, 9.0, 12.0]]],
                               2, DIVISIONS, DIVISIONS),
     12.0 / 6.0 * (100.0 + 4.0 * 81.0 + 64.0), 1e-12, None),
    ("折れ線に掃いた管 L20 r4",
     lambda: z.make_polyline_pipe([[0.0, 0.0, 0.0], [0.0, 0.0, 20.0]], 4.0, 0.0,
                                  DIVISIONS, DIVISIONS),
     math.pi * 16.0 * 20.0, 1e-3, None),
    ("環状溝の軸 r15 L60 W4 T2.5",
     lambda: z.make_shaft_with_annular_groove(15.0, 60.0, 4.0, 2.5, 25.0,
                                              DIVISIONS, DIVISIONS),
     math.pi * 225.0 * 60.0 - math.pi * (225.0 - 12.5 ** 2) * 4.0, 1e-3, None),
    # **皿もみは、穴とは別に円錐台のぶんだけ増えます**——
    # **穴がすでに抜いた円柱を、二重に引かないこと。**
    ("皿もみの箱 40x40x20",
     lambda: z.make_countersink_hole_box(40.0, 40.0, 20.0, 3.0, 6.0, 90.0,
                                         20.0, 20.0, DIVISIONS, DIVISIONS),
     40.0 * 40.0 * 20.0 - math.pi * 9.0 * 20.0
     - (math.pi * 3.0 / 3.0 * (36.0 + 18.0 + 9.0) - math.pi * 9.0 * 3.0),
     1e-3, None),
    # **`extrude_dir` は向きではなく、長さがそのまま高さ**です
    # （4-421 で 1 度、別の引数を高さと読んで外しました）。
    ("抜き勾配の押し出し 10x10 h1 5度",
     lambda: z.make_draft_extrusion([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0],
                                     [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]],
                                    [0.0, 0.0, 1.0], 5.0, DIVISIONS, DIVISIONS),
     (100.0 + math.sqrt(100.0 * (10.0 + 2.0 * math.tan(math.radians(5.0))) ** 2)
      + (10.0 + 2.0 * math.tan(math.radians(5.0))) ** 2) / 3.0, 1e-6, None),
    ("部分回転 90 度",
     lambda: z.make_partial_revolve_solid([[10.0, 0.0, 0.0], [14.0, 0.0, 0.0],
                                           [14.0, 0.0, 6.0], [10.0, 0.0, 6.0]],
                                          [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 90.0,
                                          DIVISIONS, DIVISIONS),
     2.0 * math.pi * 12.0 * 24.0 / 4.0, 1e-3, None),
    ("中空押し出し 10x10 − 4x4 h1",
     lambda: z.make_hollow_extrusion([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0],
                                      [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]],
                                     [[[3.0, 3.0, 0.0], [7.0, 3.0, 0.0],
                                       [7.0, 7.0, 0.0], [3.0, 7.0, 0.0]]],
                                     [0.0, 0.0, 1.0], DIVISIONS, DIVISIONS),
     100.0 - 16.0, 1e-12, None),
    ("平らな板を厚くする 10x10 t2",
     lambda: z.thicken_surface_patch(
         [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [7.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
         [[0.0, 10.0, 0.0], [3.0, 10.0, 0.0], [7.0, 10.0, 0.0], [10.0, 10.0, 0.0]],
         [[0.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 7.0, 0.0], [0.0, 10.0, 0.0]],
         [[10.0, 0.0, 0.0], [10.0, 3.0, 0.0], [10.0, 7.0, 0.0], [10.0, 10.0, 0.0]],
         2.0, DIVISIONS, DIVISIONS),
     200.0, 1e-12, None),
]

# **歯車の軸穴は、閉じた式が「穴なしの歯車 − π r² t」**です（4-421）。
# **歯車そのものの断面積は、ここでは求めません**——**引くほうだけを
# 見れば、穴が開いているかは分かります。**
#
# **`make_spur_gear` の `bore_radius` は穴を開けません**（4-415）。
# **穴が要るのは `make_drilled_spur_gear`** で、**2026/09/09 まで
# Python から呼べませんでした。**
GEAR = (2.0, 18, 20.0, 10.0, 5.0)
GEAR_DIVISIONS = 128
_gear_solid, _ = value_of(
    z.make_spur_gear(GEAR[0], GEAR[1], GEAR[2], GEAR[3], GEAR[4],
                     GEAR_DIVISIONS, GEAR_DIVISIONS), "volume")
SHAPES.append((
    "軸穴の開いた歯車（穴 = π r² t）",
    lambda: z.make_drilled_spur_gear(GEAR[0], GEAR[1], GEAR[2], GEAR[3], GEAR[4],
                                     GEAR_DIVISIONS, GEAR_DIVISIONS),
    _gear_solid - math.pi * GEAR[4] ** 2 * GEAR[3], 1e-4, None))

# **掃く形**（4-422）。**らせんに沿って掃いた立体は、断面積 ×
# らせんの長さ**です——**断面の重心が、らせんの上に乗っているとき**。
#
# **`make_helix_solid` は、断面の原点をらせんに乗せます。**
# **断面を原点中心にしないと、重心がずれた半径を回ります**
# ——実測（4-422）: 一辺 3 の正方形を `0..3` に置くと **1.58% ずれ**、
# **`±1.5` に置くと 2.5e-5**。**大きさを 1/2/3/4 と変えても、
# 比は 0.999975 で動きません。** **ずれていたのは私の式**でした。
def centred_square(side, height):
    half = side / 2.0
    return [[-half, -half, height], [half, -half, height],
            [half, half, height], [-half, half, height]]


SPRING = (10.0, 8.0, 3.0, 1.5)          # 半径・ピッチ・巻数・線径
HELIX = (15.0, 10.0, 2.0, 3.0)          # 半径・ピッチ・巻数・断面の一辺
spring_length = SPRING[2] * math.sqrt((2.0 * math.pi * SPRING[0]) ** 2 + SPRING[1] ** 2)
helix_length = HELIX[2] * math.sqrt((2.0 * math.pi * HELIX[0]) ** 2 + HELIX[1] ** 2)
ARC = [[20.0 * math.cos(step * math.pi / 2.0 / 32.0),
        20.0 * math.sin(step * math.pi / 2.0 / 32.0), 0.0] for step in range(33)]

SHAPES.extend([
    ("ばね（π rw² × らせん長）",
     lambda: z.make_round_wire_spring(SPRING[0], SPRING[1], SPRING[2], SPRING[3],
                                      DIVISIONS, DIVISIONS),
     math.pi * SPRING[3] ** 2 * spring_length, 1e-3, None),
    ("らせんに掃いた角材（A × らせん長）",
     lambda: z.make_helix_solid(centred_square(HELIX[3], 0.0), HELIX[0], HELIX[1],
                                HELIX[2], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 128,
                                DIVISIONS, DIVISIONS),
     HELIX[3] ** 2 * helix_length, 1e-3, None),
    ("円弧に掃いた管（パップス）",
     lambda: z.make_sweep_pipe(ARC, 2.0, 64, DIVISIONS, DIVISIONS),
     math.pi * 4.0 * (20.0 * math.pi / 2.0), 1e-3, None),
    ("折れ線に掃いた角材 直線 L20",
     lambda: z.make_polyline_sweep(centred_square(3.0, 0.0),
                                   [[0.0, 0.0, 0.0], [0.0, 0.0, 20.0]], 0.0,
                                   DIVISIONS, DIVISIONS),
     9.0 * 20.0, 1e-12, None),
    ("掃いた角材 直線 L20",
     lambda: z.make_sweep_wire(centred_square(3.0, 0.0),
                               [[0.0, 0.0, 0.0], [0.0, 0.0, 20.0]], 16,
                               DIVISIONS, DIVISIONS),
     9.0 * 20.0, 1e-12, None),
    # **案内が直線なら、ふつうのロフトと同じ**はずです。
    ("案内付きロフト 正方形10→8 h12",
     lambda: z.make_guided_loft_solid(
         [centred_square(10.0, 0.0), centred_square(8.0, 12.0)],
         [[[0.0, 0.0, 0.0], [0.0, 0.0, 12.0]]], 2, DIVISIONS, DIVISIONS),
     12.0 / 6.0 * (100.0 + 4.0 * 81.0 + 64.0), 1e-12, None),
    # **面取りするのは 1 稜だけ**です（4-422 で 1 度、4 稜と読んで
    # 外しました）。**それが 2 つぶん。**
    ("鏡像の複合ケーシング（面取り 1 稜 × 2）",
     lambda: z.make_mirror_compound_casing(30.0, 50.0, 20.0, 10.0, 6.0,
                                           [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                                           DIVISIONS, DIVISIONS),
     2.0 * (30.0 * 50.0 * 20.0 - 0.5 * 6.0 * 6.0 * 20.0), 1e-12, None),
])

# **置き方を変えても、大きさは変わりません**（4-423）。
#
# **4-421 のメッシュのブーリアンは、高さ 10 のときだけ合っていました**
# ——**面がちょうど三角形の境目に乗るから**です。**「合っている例が
# ある」は、正しいことの証明になりません。**
#
# **上の 42 形は、どれも軸に沿った素直な置き方**です。**ここでは
# 軸・平面・経路を傾けます**——**体積は剛体運動で変わらない**ので、
# **閉じた式はそのまま**使えます（3-N-2 の「同じ形を置き方だけ
# 変えて測る」）。
DIAGONAL = 1.0 / math.sqrt(3.0)
SHAPES.extend([
    ("箱に丸穴：軸 x（貫通 30）",
     lambda: z.make_exact_drill_boolean(30.0, 30.0, 20.0, [0.0, 0.0, 0.0],
                                        4.0, 60.0, [-20.0, 15.0, 10.0],
                                        [1.0, 0.0, 0.0], 1, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 20.0 - math.pi * 16.0 * 30.0, 1e-3, None),
    ("箱に丸穴：軸 y（貫通 30）",
     lambda: z.make_exact_drill_boolean(30.0, 30.0, 20.0, [0.0, 0.0, 0.0],
                                        4.0, 60.0, [15.0, -20.0, 10.0],
                                        [0.0, 1.0, 0.0], 1, DIVISIONS, DIVISIONS),
     30.0 * 30.0 * 20.0 - math.pi * 16.0 * 30.0, 1e-3, None),
    ("回した環：軸 x（パップスは軸によらない）",
     lambda: z.make_revolve_solid([[0.0, 10.0, 0.0], [0.0, 14.0, 0.0],
                                   [6.0, 14.0, 0.0], [6.0, 10.0, 0.0]],
                                  [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                                  DIVISIONS, DIVISIONS),
     2.0 * math.pi * 12.0 * 24.0, 1e-3, None),
    ("直線に掃いた管：斜め (1,1,1)",
     lambda: z.make_sweep_pipe([[0.0, 0.0, 0.0],
                                [20.0 * DIAGONAL, 20.0 * DIAGONAL, 20.0 * DIAGONAL]],
                               3.0, 32, DIVISIONS, DIVISIONS),
     math.pi * 9.0 * 20.0, 1e-3, None),
    ("らせんに掃いた角材：軸 (1,1,1)",
     lambda: z.make_helix_solid(centred_square(HELIX[3], 0.0), HELIX[0], HELIX[1],
                                HELIX[2], [0.0, 0.0, 0.0],
                                [DIAGONAL, DIAGONAL, DIAGONAL], 128,
                                DIVISIONS, DIVISIONS),
     HELIX[3] ** 2 * helix_length, 1e-3, None),
    ("鏡像の箱：平面 (1,1,1)",
     lambda: z.make_mirror_box(10.0, 20.0, 30.0, [0.0, 0.0, 0.0],
                               [DIAGONAL, DIAGONAL, DIAGONAL],
                               DIVISIONS, DIVISIONS),
     6000.0, 1e-12, None),
    ("箱の差：ずらし (-3, 7, -5)",
     lambda: z.make_exact_box_boolean(20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
                                      10.0, 10.0, 30.0, [-3.0, 7.0, -5.0],
                                      1, DIVISIONS, DIVISIONS),
     8000.0 - 7.0 * 10.0 * 20.0, 1e-12, None),
])

# **`Solid` クラスの口**（4-424）。
#
# **2026/09/11 まで、1 つも測っていませんでした。** `check_python_arguments.py`
# は **`inspect.isclass` で飛ばし**、`check_python_surface.py` は
# **`volume` と `from_sketch_*` しか触っていません**でした。
# **`Solid` には 38 個、`Mesh` には 14 個**あります。
#
# **ここの `volume` は B-Rep の厳密値**です——**刻みに依りません**
# （`tessellate(8)` と `tessellate(64)` で 6000.000000 が両方）。
# **上の `make_*` はメッシュを返す**ので、そちらは刻みのぶんずれます。
#
# **`translated` / `rotated` / `mirrored` は出ています**——
# **4-423 で「`BrepTransform` は Python に出ていない」と書いたのは
# 外れ**でした。**`Solid` のメソッドとして、ずっとありました。**
BOX = (30.0, 20.0, 10.0)


def solid_box():
    return z.Solid.box(*BOX)


def vertical_edges(solid):
    """**長さ 10 の縦稜**（4 本）の番号。

    **番号は決め打ちしません**——**長さで拾います**（番号は作りが
    変われば変わります）。

    **⚠ 番号は、その立体のものです**（4-424）。**同じ寸法の箱でも、
    作り直すたびに別の番号**が振られます——**別の箱から取った番号を
    渡すと `Edge N is not in this solid` で断られます**。
    **それが正しい振る舞い**です（**黙って別の稜を丸めるより、
    ずっといい**）。**ここで 1 度、それをやりました。**
    """
    return [edge["edge_id"] for edge in solid.blendable_edges()
            if abs(edge["length"] - BOX[2]) <= 1e-9]


def rounded(radius, count):
    """縦稜を `count` 本だけ半径 `radius` で丸めた体積。"""
    solid = solid_box()
    ids = vertical_edges(solid)[:count]
    return (solid.fillet_edge(ids[0], radius) if count == 1
            else solid.fillet_edges(ids, radius)).volume


def cut(distance, count):
    """縦稜を `count` 本だけ `distance` で面取りした体積。"""
    solid = solid_box()
    ids = vertical_edges(solid)[:count]
    return (solid.chamfer_edge(ids[0], distance) if count == 1
            else solid.chamfer_edges(ids, distance)).volume


AXIS = [1.0 / math.sqrt(3.0)] * 3
ROUND = 3.0
CHAMFER = 2.0
CORNER = ROUND * ROUND - math.pi * ROUND * ROUND / 4.0

SOLID_CHECKS = [
    # **閉じた式**（刻みに依らない）
    ("Solid 箱 30x20x10", lambda: solid_box().volume, 6000.0, 1e-12),
    ("Solid 円柱 r5 h12", lambda: z.Solid.cylinder(5.0, 12.0).volume,
     math.pi * 25.0 * 12.0, 1e-12),
    ("Solid 球 r7", lambda: z.Solid.sphere(7.0).volume,
     4.0 / 3.0 * math.pi * 343.0, 1e-12),
    ("Solid 円錐台 r6->r2 h10", lambda: z.Solid.cone(6.0, 2.0, 10.0).volume,
     math.pi * 10.0 / 3.0 * (36.0 + 12.0 + 4.0), 1e-12),
    ("Solid トーラス R12 r4", lambda: z.Solid.torus(12.0, 4.0).volume,
     2.0 * math.pi ** 2 * 12.0 * 16.0, 1e-12),
    ("Solid 正六角柱 r8 h10",
     lambda: z.Solid.regular_prism(6, 8.0, 10.0).volume,
     0.5 * 6.0 * 64.0 * math.sin(2.0 * math.pi / 6.0) * 10.0, 1e-12),

    # **置き方**——**体積は剛体運動で変わりません。**
    ("Solid 移す (3,-7,11)",
     lambda: solid_box().translated(3.0, -7.0, 11.0).volume, 6000.0, 1e-12),
    ("Solid 回す 軸(1,1,1) 137度",
     lambda: solid_box().rotated([0.0, 0.0, 0.0], AXIS, 137.0).volume,
     6000.0, 1e-12),
    ("Solid 鏡像 平面(1,1,1)",
     lambda: solid_box().mirrored([0.0, 0.0, 0.0], AXIS).volume, 6000.0, 1e-12),
    ("Solid 回して移して鏡像",
     lambda: solid_box().rotated([1.0, 2.0, 3.0], AXIS, 37.0)
     .translated(5.0, -4.0, 2.0).mirrored([0.0, 0.0, 0.0], [0.0, 1.0, 0.0]).volume,
     6000.0, 1e-12),

    # **厳密ブーリアン**——**回してからでも同じ。**
    ("Solid 和", lambda: z.Solid.box(20.0, 20.0, 20.0)
     .union(z.Solid.box(10.0, 10.0, 30.0).translated(5.0, 5.0, -5.0)).volume,
     8000.0 + 3000.0 - 2000.0, 1e-12),
    ("Solid 差", lambda: z.Solid.box(20.0, 20.0, 20.0)
     .difference(z.Solid.box(10.0, 10.0, 30.0).translated(5.0, 5.0, -5.0)).volume,
     8000.0 - 2000.0, 1e-12),
    ("Solid 積", lambda: z.Solid.box(20.0, 20.0, 20.0)
     .intersection(z.Solid.box(10.0, 10.0, 30.0).translated(5.0, 5.0, -5.0)).volume,
     2000.0, 1e-12),
    ("Solid 回してから差",
     lambda: z.Solid.box(20.0, 20.0, 20.0).rotated([0.0, 0.0, 0.0], AXIS, 31.0)
     .difference(z.Solid.box(10.0, 10.0, 30.0).translated(5.0, 5.0, -5.0)
                 .rotated([0.0, 0.0, 0.0], AXIS, 31.0)).volume,
     6000.0, 1e-12),

    # **稜の丸め・面取り**
    ("Solid 縦稜 1 本を r3 で丸める",
     lambda: rounded(ROUND, 1),
     6000.0 - CORNER * BOX[2], 1e-9),
    ("Solid 縦稜 4 本を r3 で丸める",
     lambda: rounded(ROUND, 4),
     6000.0 - 4.0 * CORNER * BOX[2], 1e-9),
    ("Solid 縦稜 1 本を c2 で面取り",
     lambda: cut(CHAMFER, 1),
     6000.0 - 0.5 * CHAMFER * CHAMFER * BOX[2], 1e-12),
    ("Solid 縦稜 4 本を c2 で面取り",
     lambda: cut(CHAMFER, 4),
     6000.0 - 4.0 * 0.5 * CHAMFER * CHAMFER * BOX[2], 1e-12),

    # **面**——**面積の和は表面積**。
    ("Solid faces の面積の和",
     lambda: sum(face["area"] for face in solid_box().faces()),
     2.0 * (30.0 * 20.0 + 30.0 * 10.0 + 20.0 * 10.0), 1e-12),
    # **押した増分は、その面の面積 × 距離**ちょうどです。
    ("Solid 面 0 を 5 押す", lambda: solid_box().push_pull_face(0, 5.0).volume,
     6000.0 + 600.0 * 5.0, 1e-12),
    ("Solid 面 4 を 5 押す", lambda: solid_box().push_pull_face(4, 5.0).volume,
     6000.0 + 200.0 * 5.0, 1e-12),

    # **物性**——**回しても主慣性モーメントは変わりません。**
    ("Solid 主慣性 0（回す前後）",
     lambda: solid_box().rotated([0.0, 0.0, 0.0], AXIS, 37.0)
     .mass_properties(64, 64)["principal_moments"][0],
     solid_box().mass_properties(64, 64)["principal_moments"][0], 1e-9),
    ("Solid 主慣性 2（回す前後）",
     lambda: solid_box().rotated([0.0, 0.0, 0.0], AXIS, 37.0)
     .mass_properties(64, 64)["principal_moments"][2],
     solid_box().mass_properties(64, 64)["principal_moments"][2], 1e-9),

    # **距離**——**50 離した 10 の箱との隙間は 20。**
    ("Solid distance_to（隙間 20）",
     lambda: solid_box().distance_to(
         z.Solid.box(10.0, 10.0, 10.0).translated(50.0, 0.0, 0.0))["distance"],
     20.0, 1e-9),

    # **縫い直し・簡約・STEP 往復**——**どれも大きさを変えません。**
    ("Solid sewn", lambda: solid_box().sewn().volume, 6000.0, 1e-12),
    ("Solid simplified", lambda: solid_box().simplified().volume, 6000.0, 1e-12),
    ("Solid STEP 往復", lambda: _step_round_trip(), 6000.0, 1e-12),
    # **箱は平面だけなので、刻みを変えても体積は動きません。**
    ("Solid tessellate(8) の体積",
     lambda: solid_box().tessellate(8, 8).volume, 6000.0, 1e-12),
    ("Solid tessellate(64) の体積",
     lambda: solid_box().tessellate(64, 64).volume, 6000.0, 1e-12),
]


def _step_round_trip():
    import os
    import tempfile
    path = os.path.join(tempfile.mkdtemp(prefix="zenith-surface-step-"), "solid.step")
    solid_box().to_step(path, "zenith_surface_check")
    return z.Solid.from_step(path).volume


# **表示メッシュの法線**（4-425）。
#
# **2026/09/11 まで、1 度も測っていませんでした。** 測ったら:
#
# * **箱の 8 頂点すべてが `(0, 0, 1)`**——**6 面ある箱で法線が 1 種類**
#   （`FaceGeometry::Plane(_)` で平面を受け取りながら捨て、`(0, 0, 1)` を
#   決め打ちしていた）
# * **溶接した頂点は「最初の 1 面ぶん」だけ**を残していた
# * **球の南極は `(0, 0, 1)`**——**ちょうど真逆**（極では法線が決まらず、
#   既定で埋めていた）
#
# **ここが見るのは 1 つだけ**です——**頂点の法線が、それを使う三角形の
# 面法線と、同じ側を向いているか。** **内積が負のものが 1 つでもあれば
# 赤**です。**これは陰影の質の話ではなく、向きの話**です
# （**真逆の法線は、Blender で裏返って見えます**）。
def inward_normals(mesh):
    """**頂点の法線が、使う三角形の面法線と逆を向いている数。**"""
    points = mesh.vertices
    normals = mesh.normals
    wrong = 0
    for triangle in mesh.faces:
        a, b, c = (points[i] for i in triangle)
        first = [b[k] - a[k] for k in range(3)]
        second = [c[k] - a[k] for k in range(3)]
        cross = [first[1] * second[2] - first[2] * second[1],
                 first[2] * second[0] - first[0] * second[2],
                 first[0] * second[1] - first[1] * second[0]]
        length = math.sqrt(sum(x * x for x in cross))
        if length <= 1e-12:
            continue
        for corner in triangle:
            if sum(cross[k] * normals[corner][k] for k in range(3)) / length < 0.0:
                wrong += 1
    return float(wrong)


# **稜の長さ**（4-428）。
#
# **2026/09/12 まで、どの門も見ていませんでした。** **だから 10% ずれた
# まま通っていました**——`inspect_edge` が**両端だけ**から長さ・中点・
# 接線を作り、**曲線を一度も見ていなかった**からです。
#
# **見るのは 2 つ**です——**長さが弧長か**（弦ではないか）、
# **中点が立体の上に乗っているか**（線分の中点ではないか）。
def edge_lengths(solid):
    return sorted(round(edge["length"], 9) for edge in solid.edges())


def quarter_arc(solid, radius):
    """**半径 `radius` の四半弧**の長さ（1 本）。"""
    target = math.pi * radius / 2.0
    for edge in solid.edges():
        if abs(edge["length"] - target) <= target * 1e-6:
            return edge["length"]
    # **見つからなければ、いちばん近いものを返します**——
    # **「無い」ではなく「違う」と出したい**からです。
    return min((edge["length"] for edge in solid.edges()),
               key=lambda value: abs(value - target))


def quarter_arc_midpoint_radius(solid, radius):
    """**四半弧の中点**が、軸からどれだけ離れているか。

    **弧の上なら `radius`**、**線分の中点なら `radius / √2`**。
    """
    target = math.pi * radius / 2.0
    best = min(solid.edges(), key=lambda edge: abs(edge["length"] - target))
    point = best["midpoint"]
    return math.hypot(point[0], point[1])


EDGE_CHECKS = [
    # **箱は 1 ビットも変わらないこと**（直線の稜）。
    ("稜 箱 30x20x10 の長さの和",
     lambda: sum(z.Solid.box(30.0, 20.0, 10.0).edges()[i]["length"]
                 for i in range(len(z.Solid.box(30.0, 20.0, 10.0).edges()))),
     4.0 * (30.0 + 20.0 + 10.0), 1e-12),
    ("稜 箱 の本数", lambda: float(len(z.Solid.box(30.0, 20.0, 10.0).edges())),
     12.0, 1e-12),
    # **曲がった稜は、弧長**（弦なら 10% 短い）。
    ("稜 円柱 r5 の四半弧",
     lambda: quarter_arc(z.Solid.cylinder(5.0, 12.0), 5.0),
     math.pi * 5.0 / 2.0, 1e-9),
    ("稜 円柱 r10 の四半弧",
     lambda: quarter_arc(z.Solid.cylinder(10.0, 5.0), 10.0),
     math.pi * 10.0 / 2.0, 1e-9),
    ("稜 球 r7 の四半弧",
     lambda: quarter_arc(z.Solid.sphere(7.0), 7.0),
     math.pi * 7.0 / 2.0, 1e-9),
    # **中点は、弧の上**（線分の中点なら r/√2 = 3.5355）。
    ("稜 円柱 r5 の四半弧の中点の半径",
     lambda: quarter_arc_midpoint_radius(z.Solid.cylinder(5.0, 12.0), 5.0),
     5.0, 1e-9),
    # **`blendable_edges` は円 1 周**（`edges()` はそれを 4 本に割る）。
    ("稜 円柱 r5 の blendable は円 1 周",
     lambda: max(edge["length"]
                 for edge in z.Solid.cylinder(5.0, 12.0).blendable_edges()),
     2.0 * math.pi * 5.0, 1e-9),
]


# **空洞のある立体**（4-427）。
#
# **`face_count` と `faces()` は外側シェルだけ**、**`volume` と
# `tessellate` と `validate` は空洞も数えます**——**そこだけ食い違います。**
# **面積を `faces()` で積むと、空洞のぶんが落ちます。**
#
# **空洞があるかを知る手段がありませんでした**ので、
# **`inner_shell_count` と `total_face_count` を出しました。**
def hollow_box():
    return (z.Solid.box(30.0, 30.0, 20.0)
            .difference(z.Solid.box(26.0, 26.0, 16.0).translated(2.0, 2.0, 2.0)))


SHELL_CHECKS = [
    ("中空の箱 volume（空洞あり）", lambda: hollow_box().volume,
     30.0 * 30.0 * 20.0 - 26.0 * 26.0 * 16.0, 1e-12),
    ("中空の箱 face_count（外側だけ）",
     lambda: float(hollow_box().face_count), 6.0, 1e-12),
    ("中空の箱 total_face_count（外 6 + 内 6）",
     lambda: float(hollow_box().total_face_count), 12.0, 1e-12),
    ("中空の箱 inner_shell_count",
     lambda: float(hollow_box().inner_shell_count), 1.0, 1e-12),
    ("中空の箱 faces() の面積の和（外側だけ）",
     lambda: sum(face["area"] for face in hollow_box().faces()),
     2.0 * 30.0 * 30.0 + 4.0 * 30.0 * 20.0, 1e-12),
    ("中空の箱 メッシュの表面積（空洞あり）",
     lambda: hollow_box().tessellate(8, 8).surface_area,
     2.0 * 30.0 * 30.0 + 4.0 * 30.0 * 20.0
     + 2.0 * 26.0 * 26.0 + 4.0 * 26.0 * 16.0, 1e-12),
    ("中空の箱 STEP 往復で空洞が残る",
     lambda: _hollow_round_trip(), 30.0 * 30.0 * 20.0 - 26.0 * 26.0 * 16.0, 1e-12),
    ("ふつうの箱 inner_shell_count",
     lambda: float(z.Solid.box(30.0, 30.0, 20.0).inner_shell_count), 0.0, 1e-12),
]


def _hollow_round_trip():
    import os
    import tempfile
    path = os.path.join(tempfile.mkdtemp(prefix="zenith-hollow-step-"), "hollow.step")
    hollow_box().to_step(path, "zenith_hollow_check")
    return z.Solid.from_step(path).volume


# **uv は、面の媒介変数そのもの**です（4-425）。**正規化していません。**
#
# **ここで留めるのは「約束が 1 つであること」**です——**球が [0, 1] に
# 見えるのは、その曲面の範囲がそれだから**であって、**正規化している
# からではありません。** **「形によって違う」と 1 度書きかけました。**
#
# **テクスチャ座標として使うなら、面ごとに割り直す**——**それを
# 知らずに使うと、箱だけ模型の寸法で流れます。**
def uv_span(mesh):
    values = mesh.uvs
    return (min(t[0] for t in values), max(t[0] for t in values),
            min(t[1] for t in values), max(t[1] for t in values))


UV_CHECKS = [
    # **平面は、平面へ落とした座標**（模型の寸法）。
    ("uv 箱 30x20x10 の u の最大", lambda: uv_span(
        z.Solid.box(30.0, 20.0, 10.0).tessellate(16, 16))[1], 30.0, 1e-12),
    ("uv 箱 3x2x1 の u の最大", lambda: uv_span(
        z.Solid.box(3.0, 2.0, 1.0).tessellate(16, 16))[1], 3.0, 1e-12),
    # **こちらの作りの曲面は、媒介変数が [0, 1]。**
    ("uv 球 r7 の u の最大", lambda: uv_span(
        z.Solid.sphere(7.0).tessellate(16, 16))[1], 1.0, 1e-9),
    ("uv トーラス の v の最大", lambda: uv_span(
        z.Solid.torus(12.0, 4.0).tessellate(16, 16))[3], 1.0, 1e-9),
]

NORMAL_CHECKS = [
    ("法線が裏返っている 箱", lambda: z.Solid.box(30.0, 20.0, 10.0)),
    ("法線が裏返っている 円柱", lambda: z.Solid.cylinder(5.0, 12.0)),
    ("法線が裏返っている 球", lambda: z.Solid.sphere(7.0)),
    ("法線が裏返っている 円錐台", lambda: z.Solid.cone(6.0, 2.0, 10.0)),
    ("法線が裏返っている トーラス", lambda: z.Solid.torus(12.0, 4.0)),
    ("法線が裏返っている 正六角柱",
     lambda: z.Solid.regular_prism(6, 8.0, 10.0)),
]


# **面だけを返す口**（4-422）。**体積はありません**ので、面積で見ます。
AREAS = [
    ("平らなキャップ 10x10",
     lambda: z.cap_planar_wire([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0],
                                [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]],
                               DIVISIONS, DIVISIONS),
     100.0, 1e-12),
    ("4 境界の平らなパッチ 10x10",
     lambda: z.make_curve_patch(
         [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [7.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
         [[0.0, 10.0, 0.0], [3.0, 10.0, 0.0], [7.0, 10.0, 0.0], [10.0, 10.0, 0.0]],
         [[0.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 7.0, 0.0], [0.0, 10.0, 0.0]],
         [[10.0, 0.0, 0.0], [10.0, 3.0, 0.0], [10.0, 7.0, 0.0], [10.0, 10.0, 0.0]],
         DIVISIONS, DIVISIONS),
     100.0, 1e-12),
]

print("本物の Python から拡張モジュールを触る（4-402）")
print()
print("Python %s" % sys.version.split()[0])
print("`volume` は Python では **%s**" % kind)
print()
print("%-34s%>18s%>18s%>12s  %s".replace(">", "") % ("測るもの", "閉じた式", "測った値", "相対差", "結果"))
print("-" * 100)

wrong = 0
for name, measured, expected in rows:
    residual = abs(measured - expected) / max(abs(expected), 1.0)
    ok = residual <= 1e-9
    if not ok:
        wrong += 1
    print("%-34s%18.9f%18.9f%12.3e  %s" % (name, expected, measured, residual, "ok" if ok else "**ちがう**"))

print("-" * 100)
print()

# **形を作る口を、閉じた式と囲み箱で**（4-420）。
print("%-34s%18s%18s%12s  %s" % ("作る形", "閉じた式", "測った値", "相対差", "結果"))
print("-" * 100)
for name, build, expected, allowance, box in SHAPES:
    try:
        volume, measured_box = volume_and_box(build())
    except Exception as error:
        wrong += 1
        print("%-34s%18s%18s%12s  **断られました**: %s"
              % (name, "%.6f" % expected, "-", "-", str(error)[:28]))
        continue
    residual = abs(volume - expected) / max(abs(expected), 1.0)
    ok = residual <= allowance
    if box is not None:
        for axis, (low, high) in enumerate(box):
            if (abs(measured_box[axis][0] - low) > 1e-6
                    or abs(measured_box[axis][1] - high) > 1e-6):
                ok = False
    if not ok:
        wrong += 1
    print("%-34s%18.6f%18.6f%12.3e  %s"
          % (name, expected, volume, residual, "ok" if ok else "**ちがう**"))
print("-" * 100)
print()

# **`Solid` クラスの口**（4-424）。**ここの体積は B-Rep の厳密値**です。
print("%-40s%18s%18s%12s  %s" % ("Solid の口", "閉じた式", "測った値", "相対差", "結果"))
print("-" * 100)
for name, call, expected, allowance in SOLID_CHECKS:
    try:
        got = call()
    except Exception as error:
        wrong += 1
        print("%-40s%18.6f%18s%12s  **断られました**: %s"
              % (name, expected, "-", "-", str(error)[:24]))
        continue
    residual = abs(got - expected) / max(abs(expected), 1.0)
    ok = residual <= allowance
    if not ok:
        wrong += 1
    print("%-40s%18.6f%18.6f%12.3e  %s"
          % (name, expected, got, residual, "ok" if ok else "**ちがう**"))
print("-" * 100)
print()

# **稜の長さ**（4-428）。**弧長か、弦か。**
print("%-40s%18s%18s%12s  %s" % ("稜", "閉じた式", "測った値", "相対差", "結果"))
print("-" * 100)
for name, call, expected, allowance in EDGE_CHECKS:
    try:
        got = call()
    except Exception as error:
        wrong += 1
        print("%-40s%18.6f%18s%12s  **断られました**: %s"
              % (name, expected, "-", "-", str(error)[:24]))
        continue
    residual = abs(got - expected) / max(abs(expected), 1.0)
    if residual > allowance:
        wrong += 1
    print("%-40s%18.6f%18.6f%12.3e  %s"
          % (name, expected, got, residual, "ok" if residual <= allowance else "**ちがう**"))
print("-" * 100)
print()

# **空洞のある立体**（4-427）。**外側だけを数える所と、空洞も数える所**。
print("%-40s%18s%18s%12s  %s" % ("空洞", "あるべき", "測った値", "相対差", "結果"))
print("-" * 100)
for name, call, expected, allowance in SHELL_CHECKS:
    try:
        got = call()
    except Exception as error:
        wrong += 1
        print("%-40s%18.6f%18s%12s  **断られました**: %s"
              % (name, expected, "-", "-", str(error)[:24]))
        continue
    residual = abs(got - expected) / max(abs(expected), 1.0)
    if residual > allowance:
        wrong += 1
    print("%-40s%18.6f%18.6f%12.3e  %s"
          % (name, expected, got, residual, "ok" if residual <= allowance else "**ちがう**"))
print("-" * 100)
print()

# **uv の約束**（4-425）。**正規化していないことを、ここで留めます。**
print("%-40s%18s%18s%12s  %s" % ("uv", "約束", "測った値", "相対差", "結果"))
print("-" * 100)
for name, call, expected, allowance in UV_CHECKS:
    try:
        got = call()
    except Exception as error:
        wrong += 1
        print("%-40s%18.6f%18s%12s  **断られました**: %s"
              % (name, expected, "-", "-", str(error)[:24]))
        continue
    residual = abs(got - expected) / max(abs(expected), 1.0)
    if residual > allowance:
        wrong += 1
    print("%-40s%18.6f%18.6f%12.3e  %s"
          % (name, expected, got, residual, "ok" if residual <= allowance else "**ちがう**"))
print("-" * 100)
print()

# **表示メッシュの法線**（4-425）。**刻みを 2 つとも見ます。**
#
# **外側の `wrong`（誤答の数）と同じ名前を使わないこと**——
# **ここで 1 度、取り違えました**（4-420 の `measured` と同じ形。
# **同じ名前は、離れていても当たります**）。
print("%-40s%18s%18s%12s  %s" % ("法線", "あるべき", "測った値", "", "結果"))
print("-" * 100)
for name, build in NORMAL_CHECKS:
    for divisions in (8, 32):
        label = "%s（刻み %d）" % (name, divisions)
        try:
            inward = inward_normals(build().tessellate(divisions, divisions))
        except Exception as error:
            wrong += 1
            print("%-40s%18s%18s%12s  **断られました**: %s"
                  % (label, "0", "-", "", str(error)[:24]))
            continue
        if inward != 0.0:
            wrong += 1
        print("%-40s%18d%18d%12s  %s"
              % (label, 0, int(inward), "", "ok" if inward == 0.0 else "**ちがう**"))
print("-" * 100)
print()

# **面だけを返す口**は、面積で見ます（4-422）。
print("%-34s%18s%18s%12s  %s" % ("作る面", "閉じた式", "測った値", "相対差", "結果"))
print("-" * 100)
for name, build, expected, allowance in AREAS:
    try:
        area, _ = value_of(build(), "surface_area")
    except Exception as error:
        wrong += 1
        print("%-34s%18.6f%18s%12s  **断られました**: %s"
              % (name, expected, "-", "-", str(error)[:28]))
        continue
    residual = abs(area - expected) / max(abs(expected), 1.0)
    ok = residual <= allowance
    if not ok:
        wrong += 1
    print("%-34s%18.6f%18.6f%12.3e  %s"
          % (name, expected, area, residual, "ok" if ok else "**ちがう**"))
print("-" * 100)
print()

# **断るべきものが断られるか。**
refusals = [
    ("点の番号が範囲の外",
     lambda: S.from_sketch_extruded(json.dumps(
         {"points": [[0, 0], [10, 0], [10, 10]], "lines": [[0, 1], [1, 9], [9, 0]]}), 5.0)),
    ("線も弧も無い",
     lambda: S.from_sketch_extruded(json.dumps(
         {"points": [[0, 0], [10, 0], [10, 10]]}), 5.0)),
    ("軸をまたぐ輪を回す",
     lambda: S.from_sketch_revolved(json.dumps(
         {"points": [[-5, 0], [5, 0], [5, 6], [-5, 6]],
          "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]}), 0.0, 0.0, 0.0, 1.0)),
    ("拘束の種類の綴り違い",
     lambda: z.solve_2d_sketch("[[0,0],[3,4],[9,1]]",
                               '[{"type": "horizontol", "p1": 0, "p2": 1}]')),
    ("distance に value が無い",
     lambda: z.solve_2d_sketch("[[0,0],[3,4],[9,1]]",
                               '[{"type": "distance", "p1": 0, "p2": 1}]')),
    # **負の密度は、慣性の符号を静かに反転させます**（4-414）。
    ("密度が負",
     lambda: z.compute_box_mass_properties(10.0, 10.0, 10.0, -1.0)),
    ("密度が 0",
     lambda: z.compute_box_mass_properties(10.0, 10.0, 10.0, 0.0)),
]
missed = 0
for name, call in refusals:
    try:
        call()
        print("%-34s **通ってしまいました**" % name)
        missed += 1
    except Exception as error:
        print("%-34s 断りました: %s" % (name, str(error)[:52]))

print()
print("誤答 %d 件、断りそこね %d 件" % (wrong, missed))
sys.exit(1 if (wrong or missed) else 0)
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()

    built = os.path.join(ROOT, "target", "release", "zenith_cad.dll")
    if not os.path.exists(built):
        print("target/release/zenith_cad.dll がありません。")
        print("  cargo build --release -p zenith_py")
        return 1

    # **`.pyd` へ写します。** Python は `.dll` を拡張モジュールとして
    # 探しません。**元のファイルは動かしません**——`cargo` の出力です。
    workspace = tempfile.mkdtemp(prefix="zenith_pysurface_")
    try:
        shutil.copy2(built, os.path.join(workspace, "zenith_cad.pyd"))
        probe = os.path.join(workspace, "probe.py")
        with open(probe, "w", encoding="utf-8") as handle:
            handle.write(build_probe_source())

        environment = dict(os.environ)
        environment["PYTHONPATH"] = workspace
        environment["PYTHONIOENCODING"] = "utf-8"
        result = subprocess.run(
            [args.python, "-X", "utf8", probe],
            env=environment,
            cwd=workspace,
        )
        return result.returncode
    finally:
        shutil.rmtree(workspace, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
