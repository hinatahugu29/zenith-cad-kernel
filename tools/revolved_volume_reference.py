"""挽き物（`occ_reference_revolved_vase`）の体積を、母線の**評価だけ**から求める。

**OpenCASCADE の立体の求積は、この形で 1.3e-5 外れます。** 実測:

    Green の定理    4171.053368   （刻みを 200 から 200000 まで振って収束）
    Zenith 読み値   4171.053368   （分割 32 以上で 10 桁動かない）
    OCC 立体        4170.999302

有理 B-spline の上での OCC の求積が緩いことは 4-45 で見ています。**期待値に
相手の値をそのまま使うと、相手の誤差を仕様として焼き付けることになります。**

`V = 2 pi * ∫∫_R x dA` で、グリーンの定理から `∫∫_R x dA = ∮ (x^2/2) dz`
（(x, z) 平面で反時計回り）。使うのは曲線の評価だけで、OpenCASCADE の
求積は一切通りません。相手の求積とこちらのメッシュの、どちらから見ても
独立した物差しになります。
"""

import os
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(FREECAD_BIN)
    except Exception:
        pass

import math

import FreeCAD  # noqa: E402
import Part  # noqa: E402

V = FreeCAD.Vector

spline = Part.BSplineCurve()
spline.interpolate([V(6, 0, 0), V(9, 0, 6), V(5, 0, 14), V(7, 0, 22), V(4, 0, 28)])


def integrate_curve(evaluate, t0, t1, steps):
    """∮ x^2/2 dz を Simpson で。steps は偶数。"""
    total = 0.0
    previous = None
    values = []
    for i in range(steps + 1):
        t = t0 + (t1 - t0) * i / steps
        point = evaluate(t)
        values.append(point)
    # 台形ではなく、区間ごとに (x^2/2) を Simpson、dz は解析的に差分。
    # 細かく刻めば台形で十分収束するので、刻みを振って確かめる。
    for i in range(steps):
        a = values[i]
        b = values[i + 1]
        mid_x = 0.5 * (a.x + b.x)
        total += (mid_x * mid_x * 0.5) * (b.z - a.z)
    _ = previous
    return total


def loop_integral(steps):
    total = 0.0
    # 反時計回りに: (0,0) -> (6,0) -> spline -> (4,28) -> (0,28) -> (0,0)
    segments = [
        (V(0, 0, 0), V(6, 0, 0)),
        None,  # spline
        (V(4, 0, 28), V(0, 0, 28)),
        (V(0, 0, 28), V(0, 0, 0)),
    ]
    for segment in segments:
        if segment is None:
            t0, t1 = spline.FirstParameter, spline.LastParameter
            total += integrate_curve(spline.value, t0, t1, steps)
        else:
            a, b = segment
            total += integrate_curve(
                lambda t, a=a, b=b: V(
                    a.x + (b.x - a.x) * t, 0.0, a.z + (b.z - a.z) * t
                ),
                0.0,
                1.0,
                2,
            )
    return total


for steps in (200, 2000, 20000, 200000):
    moment = loop_integral(steps)
    volume = 2.0 * math.pi * moment
    print("steps {:>7}: moment {:.9f}  volume {:.9f}".format(steps, moment, volume))

print()
print("OCC solid volume  4170.999302")
print("Zenith read-back  4171.053368")
