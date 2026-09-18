#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**接点の近くで、位置がどこまで決まるか**を閉じた式で測る（4-483）。

主半径 6・管半径 2 のトーラス 2 つを 2R = 12 離すと、管の底の円が
互いの上をなぞる。接点から A の上を動かしても、B からの距離が
ほとんど増えない——**残差では端の位置が決まらない**ことの根拠。

    py tools/measure_torus_tangency_blur.py
"""
import math
# A: 中心 (0,0,0)、主半径 6・管半径 2  /  B: 中心 (12,0,0)、同じ
R, r, D = 6.0, 2.0, 12.0
def g_b(x, y, z):
    rho = math.hypot(x - D, y)
    return (rho - R) ** 2 + z * z - r * r      # B の陰関数（0 なら B の上）
def grad_norm_b(x, y, z):
    rho = math.hypot(x - D, y)
    drho = 2.0 * (rho - R)
    gx = drho * (x - D) / rho
    gy = drho * y / rho
    gz = 2.0 * z
    return math.sqrt(gx * gx + gy * gy + gz * gz)

print("接点 (6, 0, -2) から、A の上を y 方向へ動いたときの「B からの隔たり」")
print()
print(f"{'動いた距離 d':>14}{'B からの距離':>16}")
print("-" * 32)
rows = []
for d in [1e-7, 1e-6, 1e-5, 2.3e-5, 1e-4, 1e-3, 1e-2]:
    x = math.sqrt(R * R - d * d)               # A の管の中心円（rho = 6）の上
    y, z = d, -r                               # 管の底なので z = -2、ここは A の上
    gap = abs(g_b(x, y, z)) / max(grad_norm_b(x, y, z), 1e-300)
    rows.append((d, gap))
    print(f"{d:>14.1e}{gap:>16.3e}")
print()
# 残差 1e-10 に落ちる d を二分で求める
lo, hi = 1e-9, 1e-2
for _ in range(200):
    mid = math.sqrt(lo * hi)
    x = math.sqrt(R * R - mid * mid)
    gap = abs(g_b(x, mid, -r)) / max(grad_norm_b(x, mid, -r), 1e-300)
    if gap < 1e-10:
        lo = mid
    else:
        hi = mid
print(f"**B からの距離が 1e-10 に達するのは d = {math.sqrt(lo*hi):.3e}**")
print("（つまり、この距離までは「両方の面の上にある」と残差では区別できません）")
