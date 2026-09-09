"""**引数を 1 つずつ動かして、答えが動くかを見る**（4-415）。

なぜ要るのか
------------
**4-414 で、`compute_box_mass_properties` が `density` を受け取って
捨てていた**と分かりました。**鋼の 7850 を渡しても、密度 1 の慣性が
返ります**——**閉じた立体が返り、非多様体でもなく、形の検査は全部
通ります。大きさだけが違います。**

**4-409 の `_face` も、同じ型**でした。**2 件とも、見つかったのは
「たまたまそこを読んだから」**です。

**引数を動かして答えが動かないなら、その引数は読まれていません。**
**それは機械で数えられます。** ここがその口です。

**Rust の `_` 付き引数を数えるだけでは足りません**——**受け取ってから
使い損ねる**場合は `_` が付きません。**呼んで測るほうが強い**です。

使い方
------
    cargo build --release -p zenith_py
    py tools/check_python_arguments.py

**説明の付いていない「効かない引数」が 1 つでもあれば、非ゼロで
終わります。**

読み方
------
**「動かない」＝ただちに誤答、ではありません。** 意味として効かない
引数もあります（**その場合は下の `EXPECTED_INERT` に理由付きで
足してください**——**黙って除外しないでください。理由が書けないなら、
それは欠陥です**）。
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_probe_source():
    return r'''
import math, sys, inspect

import zenith_cad as z

# **効かないことが分かっていて、理由があるもの。**
# **理由を書けないなら、それは欠陥です。**
EXPECTED_INERT = {
    # **歯車の `bore_radius` は、穴を開けません**——歯底半径の下限に
    # 効くだけです（`bore_radius + 0.5 * module` より内側に歯底を
    # 置かない）。**既定の 5.0 では、歯底 (16.0) のほうが外側**なので
    # 下限が当たらず、動かしても形が変わりません。
    # **穴が要るなら `make_drilled_spur_gear`**（4-415 で口を足しました）。
    ("make_spur_gear", "bore_radius"):
        "軸穴は開かない。歯底半径の下限に効くだけで、既定では当たらない",
}

# **既定値だけでは呼べない口に、こちらから渡す引数。**
#
# **ここに無い口は「測っていません」**——**「緑」ではありません。**
# 下の出力で名指しします。
# **並びは、名前を付けて 1 か所に置きます**——**同じ形を別々に
# 書くと、次の人が突き合わせられません**（`seam_torus_wall_probe` の
# 註と同じ）。
SQUARE = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]]
SMALL_SQUARE = [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [3.0, 3.0, 0.0], [0.0, 3.0, 0.0]]
PATH = [[0.0, 0.0, 0.0], [0.0, 0.0, 20.0], [15.0, 0.0, 30.0]]
REVOLVE_PROFILE = [[10.0, 0.0, 0.0], [14.0, 0.0, 0.0],
                   [14.0, 0.0, 6.0], [10.0, 0.0, 6.0]]

BASELINE = {
    "make_box": dict(dx=10.0, dy=20.0, dz=30.0),
    "make_cylinder": dict(radius=5.0, height=12.0),
    "make_cone": dict(r_bottom=6.0, r_top=2.0, height=10.0),
    "make_sphere": dict(radius=7.0),
    "make_torus": dict(r_major=12.0, r_minor=4.0),
    "make_regular_prism": dict(num_sides=6, radius=8.0, height=10.0),
    "make_drilled_box": dict(dx=30.0, dy=30.0, dz=10.0, hole_radius=4.0),
    "make_hollow_box": dict(dx=30.0, dy=30.0, dz=20.0, thickness=2.0),
    "make_through_hollow_box": dict(dx=30.0, dy=30.0, dz=20.0, thickness=2.0),
    "make_open_box": dict(dx=30.0, dy=30.0, dz=20.0, thickness=2.0),
    "make_filleted_box": dict(dx=30.0, dy=20.0, dz=10.0, radius=3.0),
    "make_chamfered_box": dict(dx=30.0, dy=20.0, dz=10.0, chamfer=2.0),
    # **対称な箱では、面の番号を動かしても体積が変わりません。**
    # **辺の長さを 3 つとも違えます**（4-415 で 1 度、そこで
    # 偽陽性を出しました）。
    "push_pull_box": [dict(dx=10.0, dy=20.0, dz=30.0, face_index=1, distance=5.0),
                      dict(dx=10.0, dy=20.0, dz=30.0, face_index=0, distance=5.0)],
    "taper_box": dict(dx=10.0, dy=20.0, dz=30.0, face_index=1, angle_deg=10.0),
    # **稜の番号**は、同じ長さの稜どうしでは体積が変わりません。
    "chamfer_box_single_edge":
        dict(dx=30.0, dy=20.0, dz=10.0, edge_index=0, distance=2.0),
    "fillet_box_single_edge":
        dict(dx=30.0, dy=20.0, dz=10.0, edge_index=0, radius=3.0),
    "compute_box_mass_properties": dict(dx=40.0, dy=20.0, dz=10.0),
    "slice_box_by_plane":
        dict(dx=20.0, dy=20.0, dz=20.0,
             plane_origin=[0.0, 0.0, 10.0], plane_normal=[0.0, 0.0, 1.0]),
    # **隙間は、向かい合う 2 つの角だけで決まります。**
    # **B を +x に置くと `min_a` は一生効きません**——**それは
    # 幾何のほうが正しい**ので、**B を -x にも置いた検体**を
    # 並べます（4-415 で 1 度、これで偽陽性を出しました）。
    "check_boxes_interference":
        [dict(dx1=10.0, dy1=12.0, dz1=14.0, offset1=[0.0, 0.0, 0.0],
              dx2=11.0, dy2=13.0, dz2=15.0, offset2=[5.0, 1.0, 2.0]),
         dict(dx1=10.0, dy1=12.0, dz1=14.0, offset1=[0.0, 0.0, 0.0],
              dx2=11.0, dy2=13.0, dz2=15.0, offset2=[-5.0, -1.0, -2.0])],
    "check_exact_boxes_interference":
        [dict(min_a=[0.0, 0.0, 0.0], max_a=[10.0, 11.0, 12.0],
              min_b=[5.0, 1.0, 2.0], max_b=[15.0, 12.0, 14.0]),
         dict(min_a=[0.0, 0.0, 0.0], max_a=[10.0, 11.0, 12.0],
              min_b=[-15.0, -12.0, -14.0], max_b=[5.0, 1.0, 2.0])],
    "compute_boxes_min_distance":
        [dict(min_a=[0.0, 0.0, 0.0], max_a=[10.0, 11.0, 12.0],
              min_b=[20.0, 21.0, 22.0], max_b=[30.0, 31.0, 32.0]),
         dict(min_a=[20.0, 21.0, 22.0], max_a=[30.0, 31.0, 32.0],
              min_b=[0.0, 0.0, 0.0], max_b=[10.0, 11.0, 12.0])],
    "make_exact_box_boolean":
        dict(dx1=20.0, dy1=20.0, dz1=20.0, offset1=[0.0, 0.0, 0.0],
             dx2=10.0, dy2=10.0, dz2=30.0, offset2=[5.0, 5.0, -5.0]),
    "make_sweep_pipe":
        dict(path_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 20.0], [15.0, 0.0, 30.0]],
             radius=3.0),
    "make_revolve":
        dict(profile_points=[[10.0, 0.0], [14.0, 0.0], [14.0, 6.0], [10.0, 6.0]]),
    "make_mirror_box": dict(dx=10.0, dy=20.0, dz=30.0),
    "make_partial_revolve_solid":
        dict(profile=[[10.0, 0.0], [14.0, 0.0], [14.0, 6.0], [10.0, 6.0]]),
    "make_revolve_solid":
        dict(profile=[[10.0, 0.0], [14.0, 0.0], [14.0, 6.0], [10.0, 6.0]]),
    "make_stepped_shaft":
        dict(sections=[(10.0, 20.0), (6.0, 15.0), (8.0, 10.0)]),

    # **回す形は、3 次元の点で渡します**（4-415 で 1 度、2 次元で
    # 渡して「検体が断られました」を出しました）。
    "make_revolve": dict(profile_points=REVOLVE_PROFILE),
    "make_revolve_solid": dict(profile=REVOLVE_PROFILE),
    "make_partial_revolve_solid": dict(profile=REVOLVE_PROFILE),

    "cap_planar_wire": dict(wire_points=SQUARE),
    "cap_dome_wire": dict(wire_points=SQUARE),
    "make_polyline_pipe": dict(path_points=PATH),
    "make_polyline_sweep": dict(profile_points=SMALL_SQUARE, path_points=PATH),
    "make_sweep_wire": dict(profile_points=SMALL_SQUARE,
                            path_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 20.0]]),
    "make_loft": dict(profiles=[SQUARE, [[p[0], p[1], 12.0] for p in SQUARE]]),
    "make_hollow_extrusion":
        dict(outer_profile=SQUARE,
             inner_profiles=[[[3.0, 3.0, 0.0], [7.0, 3.0, 0.0],
                              [7.0, 7.0, 0.0], [3.0, 7.0, 0.0]]]),
    "make_draft_extrusion": dict(profile=SQUARE),
    "make_helix_solid": dict(profile=SMALL_SQUARE),

    # **点が足りないと、次数 3 の B-spline が組めません**——
    # **2026/09/09 まで、そこでプロセスごと落ちていました**（4-415）。
    # **4 点以上を渡します。**
    "thicken_surface_patch":
        dict(c_u0=[[0.0, 0.0, 0.0], [3.0, 0.0, 1.0], [7.0, 0.0, 1.0], [10.0, 0.0, 0.0]],
             c_u1=[[0.0, 10.0, 0.0], [3.0, 10.0, 1.0], [7.0, 10.0, 1.0], [10.0, 10.0, 0.0]],
             c_0v=[[0.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 7.0, 0.0], [0.0, 10.0, 0.0]],
             c_1v=[[10.0, 0.0, 0.0], [10.0, 3.0, 0.0], [10.0, 7.0, 0.0], [10.0, 10.0, 0.0]]),

    "make_drilled_spur_gear": dict(),
    "get_primitive_shader_payload": dict(),
    "solve_2d_sketch":
        dict(points_json="[[0,0],[3,4],[9,1]]",
             constraints_json='[{"type": "horizontal", "p1": 0, "p2": 1}]'),
    "make_exact_drill_boolean":
        dict(dx=30.0, dy=30.0, dz=20.0, box_offset=[0.0, 0.0, 0.0],
             radius=4.0, height=40.0, drill_offset=[15.0, 15.0, -10.0]),
}

# **触らない引数**——ファイルを書く / 形の種類そのものを変える。
SKIP = {"step_path", "dxf_path", "file_path", "prim_type", "op_type",
        "points_json", "constraints_json"}


def fingerprint(value):
    """**返ってきたものを、1 本の数の列にする。**

    **`volume` だけでは足りません**——刻みを変えても体積は変わらない
    のが正しい形があります（B-Rep で返る口）。**面の数・頂点の数・
    囲み箱も一緒に見ます。**
    """
    parts = []
    if isinstance(value, (int, float)):
        return (round(float(value), 9),)
    if isinstance(value, (list, tuple)):
        for item in value:
            parts.extend(fingerprint(item))
        return tuple(parts)
    if isinstance(value, str):
        return (value,)
    # **大きさだけでは足りません**（4-415）——**30x20x10 の箱は、
    # どの縦稜を面取りしても体積も表面積も同じ**です。**位置も見ます。**
    corner = getattr(value, "vertices", None)
    if callable(corner):
        try:
            corner = corner()
        except Exception:
            corner = None
    if corner:
        for axis in range(3):
            column = [float(point[axis]) for point in corner]
            parts.append(round(min(column), 9))
            parts.append(round(max(column), 9))
            parts.append(round(sum(column) / len(column), 9))
    for name in ("volume", "surface_area", "center_of_mass",
                 "num_faces", "num_vertices", "face_count", "vertex_count",
                 "triangle_count", "edge_count"):
        attribute = getattr(value, name, None)
        if attribute is None:
            continue
        if callable(attribute):
            try:
                attribute = attribute()
            except Exception:
                continue
        if isinstance(attribute, (int, float)):
            parts.append(round(float(attribute), 9))
        elif isinstance(attribute, (list, tuple)) and all(
                isinstance(x, (int, float)) for x in attribute):
            parts.extend(round(float(x), 9) for x in attribute)
    if not parts:
        parts.append(repr(type(value)))
    return tuple(parts)


def perturbations(value):
    """**その引数の、動かし方を全部返す。**

    **1 通りでは足りません**——最初に書いたものは
    **1 通りしか試さず、偽陽性を 15 個出しました**（4-415）。

    * **対称な形**では、面や稜の番号を 1 つずらしても体積が変わりません
    * **並びは、要素ごとに動かさないと効きません**（z 法線の平面に
      x を足しても、切り口は同じ所です）

    **どれか 1 つでも答えが動けば、その引数は読まれています。**
    """
    if isinstance(value, bool):
        return [not value]
    if isinstance(value, int):
        # **番号は、対称で潰れます。** 大きくずらすほうも試します。
        return [value + 1, value + 2, value + 3, max(0, value - 1)]
    if isinstance(value, float):
        if abs(value) <= 1e-12:
            return [1.0, 5.0, -1.0]
        return [value * 1.37, value * 0.61, value + 3.0, value * 2.5]
    if isinstance(value, (list, tuple)) and value and all(
            isinstance(x, (int, float)) for x in value):
        # **要素ごとに動かします。**
        moves = []
        for index in range(len(value)):
            for delta in (1.0, -1.0, 7.0):
                moved = [float(x) for x in value]
                moved[index] += delta
                moves.append(type(value)(moved))
        return moves
    return []


rows = []
inert = []
skipped = []

for name in sorted(n for n in dir(z) if not n.startswith("_")):
    function = getattr(z, name)
    if not callable(function) or inspect.isclass(function):
        continue
    signature = getattr(function, "__text_signature__", None)
    if not signature:
        skipped.append((name, "署名がありません"))
        continue

    # **既定値のある引数だけを、そのまま使います。**
    # 既定の無い引数は、こちらが形を知らないと組めません。
    defaults = {}
    ready = True
    for piece in signature.strip("()").split(", "):
        if not piece or piece == "/" or piece == "*":
            continue
        if "=" not in piece:
            ready = False
            break
        key, text = piece.split("=", 1)
        if text == "...":
            ready = False
            break
        try:
            defaults[key] = eval(text, {"__builtins__": {}}, {})
        except Exception:
            ready = False
            break
    if not ready:
        defaults = {}
    supplied = BASELINE.get(name)
    if supplied is None and (not ready or not defaults):
        skipped.append((name, "既定値だけでは呼べません"))
        continue

    # **検体は 1 つとは限りません**——**向きや対称で潰れる引数**は、
    # **置き方を変えた検体を並べないと「効かない」に見えます**
    # （4-415 で 15 個中 10 個がそれでした）。
    if supplied is None:
        cases = [dict(defaults)]
    elif isinstance(supplied, dict):
        merged = dict(defaults)
        merged.update(supplied)
        cases = [merged]
    else:
        cases = []
        for one in supplied:
            merged = dict(defaults)
            merged.update(one)
            cases.append(merged)

    usable = []
    for case in cases:
        try:
            usable.append((case, fingerprint(function(**case))))
        except Exception as error:
            skipped.append((name, "検体が断られました: %s" % str(error)[:40]))
    if not usable:
        continue

    for key, value in usable[0][0].items():
        if key in SKIP:
            continue
        moves = perturbations(value)
        if not moves:
            continue
        # **どの検体でも、どれか 1 つでも動けば、読まれています。**
        moved_answer = False
        for case, base in usable:
            for moved in perturbations(case[key]):
                trial = dict(case)
                trial[key] = moved
                try:
                    after = fingerprint(function(**trial))
                except Exception:
                    # **断られたなら、読まれています。**
                    moved_answer = True
                    break
                if after != base:
                    moved_answer = True
                    break
            if moved_answer:
                break
        if moved_answer:
            rows.append((name, key, "効く"))
        else:
            reason = EXPECTED_INERT.get((name, key))
            if reason:
                rows.append((name, key, "効かない（説明あり）"))
            else:
                inert.append((name, key))
                rows.append((name, key, "**効きません**"))

print("引数を 1 つずつ動かして、答えが動くかを見る（4-415）")
print()
print("Python %s" % sys.version.split()[0])
print()
print("測った口 %d 個 / 引数 %d 個" % (len({r[0] for r in rows}), len(rows)))
print("回せなかった口 %d 個" % len(skipped))
print()

if inert:
    print("**動かない引数**（説明が付いていないもの）")
    print("-" * 60)
    for name, key in inert:
        print("  %-40s %s" % (name, key))
    print("-" * 60)
else:
    print("**説明の付いていない「効かない引数」は 0 個です。**")
print()

print("回せなかった口（**測っていません**——「緑」ではありません）")
for name, why in skipped:
    print("  %-40s %s" % (name, why))
print()
print("効かない引数 %d 個" % len(inert))
sys.exit(1 if inert else 0)
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

    workspace = tempfile.mkdtemp(prefix="zenith_pyargs_")
    try:
        shutil.copy2(built, os.path.join(workspace, "zenith_cad.pyd"))
        probe = os.path.join(workspace, "probe.py")
        with open(probe, "w", encoding="utf-8") as handle:
            handle.write(build_probe_source())

        environment = dict(os.environ)
        environment["PYTHONPATH"] = workspace
        environment["PYTHONIOENCODING"] = "utf-8"
        return subprocess.run([args.python, "-X", "utf8", probe],
                              env=environment, cwd=workspace).returncode
    finally:
        shutil.rmtree(workspace, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
