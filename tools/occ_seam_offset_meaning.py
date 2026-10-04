"""**4-630 の census で出た数が何なのかを測ります**（4-632）。

4-629/4-630 は、全周している面について**「枠の角度 0 の点から、その面の
最も近い頂点までの 3D 距離」**を数えました。**`pipe_bend` の 8.000000 は
「小円の真裏」と確かめました**が、**`plate_with_holes` の 5.000000 と
`sphere` の 14.142136 は未測**のまま残っています。

**距離のままでは意味が分かりません。** **半径と、その頂点の径数（角度）**を
並べれば、**「真裏」なのか「4 分の 1 周」なのか**が決まります。

使い方: `py tools/occ_seam_offset_meaning.py`
"""
import math
import os
import sys

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
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

ROOT = "crates/zenith_algo/tests/fixtures"
TURNING = ("Cylinder", "Sphere", "Toroid", "Cone", "SurfaceOfRevolution")
SUBJECTS = ["pipe_bend", "plate_with_holes", "sphere"]

print("%-18s %-8s %-6s %9s %8s %10s %9s %12s" % (
    "検体", "曲面", "全周", "距離", "半径", "巻く向き", "弦/半径", "巻かない向き"))
print("-" * 92)

for stem in SUBJECTS:
    path = os.path.join(ROOT, "occ_reference_%s.step" % stem)
    if not os.path.exists(path):
        print("%-18s 見つかりません" % stem)
        continue
    shape = Part.Shape()
    shape.read(path)
    best_row = None
    for face in shape.Faces:
        surf = face.Surface
        kind = type(surf).__name__
        if kind not in TURNING:
            continue
        u0, u1, v0, v1 = face.ParameterRange
        full_u = abs((u1 - u0) - 2 * math.pi) < 1e-6
        full_v = abs((v1 - v0) - 2 * math.pi) < 1e-6
        if not (full_u or full_v):
            continue
        origin = surf.value(0.0, 0.0)
        # いちばん近い頂点と、その径数
        pick = None
        for vertex in face.Vertexes:
            d = vertex.Point.sub(origin).Length
            if pick is None or d < pick[0]:
                pick = (d, vertex.Point)
        if pick is None:
            continue
        distance, point = pick
        # **半径**は、その曲面が持っているものを使います（推測しません）。
        radius = getattr(surf, "Radius", None)
        if radius is None:
            radius = getattr(surf, "MajorRadius", None)
        minor = getattr(surf, "MinorRadius", None)
        use_radius = minor if (kind == "Toroid" and full_v) else radius
        # **頂点の径数**——全周している向きの角度
        try:
            uu, vv = surf.parameter(point)
        except Exception:
            uu, vv = (float("nan"), float("nan"))
        angle = vv if full_v else uu
        # **巻く向きの角度差こそが「継ぎ目のずれ」**です（4-632）。
        # **距離は、巻いていない向きのずれも吸い込みます**——
        # **軸に沿って 5 上がった頂点も、極にある頂点も、距離は出ます。**
        other = uu if full_v else vv
        chord_over_r = (distance / use_radius) if use_radius else float("nan")
        row = (stem, kind, "v" if full_v else "u", distance,
               use_radius or float("nan"), math.degrees(angle), chord_over_r, other)
        if best_row is None or distance > best_row[3]:
            best_row = row
    if best_row:
        print("%-18s %-8s %-6s %9.6f %8.4f %9.3f° %9.6f %12.6f" % best_row)

print()
print()
print("**「継ぎ目のずれ」は、巻く向きの角度だけ**です。")
print("**巻く向きが 0° なら、距離が出ていても継ぎ目はずれていません**")
print("——その距離は、巻かない向き（軸・緯度）の位置から来ています。")
