# -*- coding: utf-8 -*-
"""**接点の近くで、端の位置がどこまで決まるか**を閉じた式で測る（4-483）。

主半径 6・管半径 2 のトーラス 2 つを **2R = 12** 離すと、
**管の底の円が互いの上をなぞります**。接点から動かしても、
相手の面からの距離がほとんど増えません——**残差では端の位置が
決まらない**ことの根拠です。

**カーネルは要りません。** Python だけで走ります。

    py tools/measure_torus_tangency_blur.py
"""

import math
import sys

# **Windows の既定のコンソールは cp932** なので、日本語で落ちます。
# 測り口が文字の都合で落ちるのは、ばかばかしいので直しておきます。
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

R, r = 6.0, 2.0


def distance_to_other(separation, along):
    """**A の管の底の円**（rho = R, z = -r）の上で `y = along` の点が、
    **B からどれだけ離れているか**（陰関数を勾配で割った近似距離）。"""
    if along > R:
        return None
    x = math.sqrt(R * R - along * along)
    y, z = along, -r
    rho = math.hypot(x - separation, y)
    value = (rho - R) ** 2 + z * z - r * r
    slope = 2.0 * (rho - R)
    gx = slope * (x - separation) / max(rho, 1e-300)
    gy = slope * y / max(rho, 1e-300)
    gz = 2.0 * z
    norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    return abs(value) / max(norm, 1e-300)


def main():
    print("接点 (6, 0, -2) から、A の上を y 方向へ動いたときの「B からの距離」")
    print("（主半径 6・管半径 2 のトーラス 2 つを、2R = 12 離した置き方）")
    print()
    print(f"{'動いた距離':>14}{'B からの距離':>18}")
    print("-" * 34)
    for along in [1e-7, 1e-6, 1e-5, 2.3e-5, 1e-4, 1e-3, 1e-2]:
        print(f"{along:>14.1e}{distance_to_other(12.0, along):>18.3e}")
    print()
    print("**1e-2 動いても、まだ 1e-10 の内側**です。")
    print("**残差では、端の位置が 1e-2 まで決まりません。**")

    print()
    print("=" * 68)
    print()
    print("**なぞるのは、どの隔たりか**")
    print()
    print(f"{'離した距離':>12}{'d=0':>14}{'d=1e-2':>14}  読み")
    print("-" * 62)
    for separation in [2.0, 4.0, 8.0, 10.0, 11.0, 11.9, 12.0, 12.1, 13.0, 14.0, 16.0]:
        here = distance_to_other(separation, 0.0)
        there = distance_to_other(separation, 1e-2)
        if here is None or there is None:
            continue
        if here < 1e-12 and there < 1e-9:
            note = "**なぞっている**（残差で端が決まらない）"
        elif here < 1e-12:
            note = "点で触れている"
        else:
            note = "触れていない"
        print(f"{separation:>12.1f}{here:>14.3e}{there:>14.3e}  {note}")
    print()
    print("**なぞるのは 2R = 12 のときだけ**です。**掃く表に入れたのは偶然**でした。")


if __name__ == "__main__":
    main()
