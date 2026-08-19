"""
Zenith CAD Kernel - 3D 自由曲線・スプラインパス特化 STEP ファイル生成スクリプト
3D 空間スプラインパイプ、任意断面スイープ、Coons曲面パッチ等の FreeCAD 検証用
"""

import math
import os
import sys

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# blender_addon 内の zenith_cad.pyd をロード
sys.path.insert(0, os.path.abspath("blender_addon/H-CAD_V_1_0_0"))
import zenith_cad


def generate_curve_showcase_models():
    print("=" * 65)
    print("🌊 [Zenith CAD] 3D 自由曲線・スプライン特化 STEP ファイル生成＆検証")
    print("=" * 65)

    output_files = []

    # -------------------------------------------------------------
    # 1. curve_showcase_3d_spline_pipe.step (3次元空間蛇行スプラインパイプ)
    # -------------------------------------------------------------
    path_1 = os.path.abspath("curve_showcase_3d_spline_pipe.step")
    print(
        f"\n[Curve 1] 3次元うねりスプラインパイプ生成中 -> {os.path.basename(path_1)}"
    )

    # 3D空間をダイナミックに蛇行する5点の3次B-Splineパス
    path_pts = [
        [0.0, 0.0, 0.0],
        [25.0, 35.0, 15.0],
        [60.0, -25.0, 35.0],
        [95.0, 30.0, 60.0],
        [120.0, 0.0, 80.0],
    ]
    # 半径 6.0mm の円形断面スプラインパイプ
    mesh_pipe = zenith_cad.make_sweep_pipe(
        path_pts, 6.0, 48, 16, 16, path_1
    )
    print(
        f"  ✓ 3Dスプラインパイプ: {mesh_pipe.num_vertices} 頂点, 体積 = {mesh_pipe.volume:.2f} mm^3"
    )
    output_files.append(path_1)

    # -------------------------------------------------------------
    # 2. curve_showcase_spline_wire_sweep.step (3Dパス沿い長方形リボンスイープ)
    # -------------------------------------------------------------
    path_2 = os.path.abspath("curve_showcase_spline_wire_sweep.step")
    print(
        f"\n[Curve 2] 3Dスプラインパス沿い長方形断面スイープ生成中 -> {os.path.basename(path_2)}"
    )

    # 12x4mm の長方形断面ワイヤ
    prof_rect = [
        [-6.0, -2.0, 0.0],
        [6.0, -2.0, 0.0],
        [6.0, 2.0, 0.0],
        [-6.0, 2.0, 0.0],
    ]
    # 螺旋状に上昇・旋回する3Dスプラインパス
    path_spiral_spline = [
        [0.0, 0.0, 0.0],
        [30.0, 20.0, 15.0],
        [20.0, 50.0, 35.0],
        [-15.0, 40.0, 55.0],
        [-20.0, 10.0, 75.0],
        [10.0, -10.0, 95.0],
    ]
    mesh_sweep = zenith_cad.make_sweep_wire(
        prof_rect, path_spiral_spline, 48, 12, 12, path_2
    )
    print(
        f"  ✓ 3D旋回リボンスイープ: {mesh_sweep.num_vertices} 頂点, 体積 = {mesh_sweep.volume:.2f} mm^3"
    )
    output_files.append(path_2)

    # -------------------------------------------------------------
    # 3. curve_showcase_wave_loft_solid.step (波打つ3D曲線群の多段ロフト)
    # -------------------------------------------------------------
    path_3 = os.path.abspath("curve_showcase_wave_loft_solid.step")
    print(
        f"\n[Curve 3] 波打つ3D自由曲線群の多段ロフトソリッド生成中 -> {os.path.basename(path_3)}"
    )

    # 3つの高さで形状が滑らかに波打ちながら変化する閉断面ワイヤ群
    def make_wave_section(z_height, scale_x, scale_y, wave_amp):
        n_pts = 16
        pts = []
        for i in range(n_pts):
            theta = 2.0 * math.pi * i / n_pts
            # サイン波状の波打ち
            r_mod = 1.0 + wave_amp * math.cos(3.0 * theta)
            x = scale_x * math.cos(theta) * r_mod
            y = scale_y * math.sin(theta) * r_mod
            pts.append([round(x, 4), round(y, 4), z_height])
        return pts

    sec_w1 = make_wave_section(0.0, 20.0, 20.0, 0.15)
    sec_w2 = make_wave_section(30.0, 25.0, 15.0, 0.25)
    sec_w3 = make_wave_section(60.0, 15.0, 22.0, 0.10)

    mesh_wloft = zenith_cad.make_loft_solid(
        [sec_w1, sec_w2, sec_w3], 2, 16, 16, path_3
    )
    print(
        f"  ✓ 波打ち多段ロフト: {mesh_wloft.num_vertices} 頂点, 体積 = {mesh_wloft.volume:.2f} mm^3"
    )
    output_files.append(path_3)

    # -------------------------------------------------------------
    # 自己検証（Self-Verification）: カーネル側で全ファイルを再読み込みして検証
    # -------------------------------------------------------------
    print("\n" + "=" * 65)
    print("🔍 [Self-Check] Zenith STEP インポーターによるラウンドトリップ検証")
    print("=" * 65)

    for fpath in output_files:
        fname = os.path.basename(fpath)
        fsize = os.path.getsize(fpath)
        try:
            imported_mesh = zenith_cad.import_step_file(fpath, 8, 8)
            print(
                f"  ✅ [PASS] {fname:<42} (サイズ: {fsize/1024:>6.1f} KB, 復元頂点数: {imported_mesh.num_vertices:>5})"
            )
        except Exception as e:
            print(f"  ❌ [FAIL] {fname:<42} -> {e}")

    print("\n" + "=" * 65)
    print("🎉 全 3D 自由曲線・スプライン STEP ファイルの出力と自己検証が完了しました！")
    print("   FreeCAD 等の外部 CAD で直接開いて確認可能です。")
    print("=" * 65)
    for fpath in output_files:
        print(f"   • {fpath}")


if __name__ == "__main__":
    generate_curve_showcase_models()
