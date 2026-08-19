"""
Zenith CAD Kernel - 端面トポロジー完全修正版 パイプ・配管・フレーム STEP ファイル生成スクリプト
FreeCAD での単一 Manifold Solid（両端面完全閉鎖）検証用
"""

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


def generate_verified_pipes():
    print("=" * 65)
    print("🚀 [Zenith CAD] 端面完全修復版 STEP ファイル生成＆検証 (v2)")
    print("=" * 65)

    output_files = []

    # -------------------------------------------------------------
    # 1. verified_3d_spline_pipe_v2.step (3次元蛇行スプラインパイプ)
    # -------------------------------------------------------------
    path_1 = os.path.abspath("verified_3d_spline_pipe_v2.step")
    print(f"\n[Model 1] 3Dスプラインパイプ生成中 -> {os.path.basename(path_1)}")

    path_pts = [
        [0.0, 0.0, 0.0],
        [25.0, 35.0, 15.0],
        [60.0, -25.0, 35.0],
        [95.0, 30.0, 60.0],
        [120.0, 0.0, 80.0],
    ]
    mesh_pipe = zenith_cad.make_sweep_pipe(
        path_pts, 6.0, 48, 16, 16, path_1
    )
    print(
        f"  ✓ 3Dスプラインパイプ: {mesh_pipe.num_vertices} 頂点, 体積 = {mesh_pipe.volume:.2f} mm^3"
    )
    output_files.append(path_1)

    # -------------------------------------------------------------
    # 2. verified_hydraulic_polyline_pipe_v2.step (3D角丸め油圧配管)
    # -------------------------------------------------------------
    path_2 = os.path.abspath("verified_hydraulic_polyline_pipe_v2.step")
    print(f"\n[Model 2] 3D角丸め油圧配管パイプ生成中 -> {os.path.basename(path_2)}")

    pipe_path = [
        [0.0, 0.0, 0.0],
        [60.0, 0.0, 0.0],
        [60.0, 50.0, 0.0],
        [60.0, 50.0, 40.0],
        [20.0, 80.0, 40.0],
        [20.0, 80.0, 80.0],
    ]
    mesh_poly = zenith_cad.make_polyline_pipe(
        pipe_path, 4.0, 10.0, 16, 16, path_2
    )
    print(
        f"  ✓ 3D角丸め油圧配管: {mesh_poly.num_vertices} 頂点, 体積 = {mesh_poly.volume:.2f} mm^3"
    )
    output_files.append(path_2)

    # -------------------------------------------------------------
    # 3. verified_crank_frame_sweep_v2.step (3D角丸めクランクフレーム)
    # -------------------------------------------------------------
    path_3 = os.path.abspath("verified_crank_frame_sweep_v2.step")
    print(f"\n[Model 3] 3D角丸めクランクフレーム生成中 -> {os.path.basename(path_3)}")

    prof_rect = [
        [-5.0, -3.0, 0.0],
        [5.0, -3.0, 0.0],
        [5.0, 3.0, 0.0],
        [-5.0, 3.0, 0.0],
    ]
    frame_path = [
        [0.0, 0.0, 0.0],
        [0.0, 60.0, 0.0],
        [40.0, 60.0, 30.0],
        [40.0, 0.0, 30.0],
    ]
    mesh_frame = zenith_cad.make_polyline_sweep(
        prof_rect, frame_path, 12.0, 12, 12, path_3
    )
    print(
        f"  ✓ 3D角丸めフレーム: {mesh_frame.num_vertices} 頂点, 体積 = {mesh_frame.volume:.2f} mm^3"
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
                f"  ✅ [PASS] {fname:<44} (サイズ: {fsize/1024:>6.1f} KB, 復元頂点数: {imported_mesh.num_vertices:>5})"
            )
        except Exception as e:
            print(f"  ❌ [FAIL] {fname:<44} -> {e}")

    print("\n" + "=" * 65)
    print("🎉 全 新規検証 STEP ファイルの出力と自己検証が完了しました！")
    print("   FreeCAD 等の外部 CAD で直接開いて確認可能です。")
    print("=" * 65)
    for fpath in output_files:
        print(f"   • {fpath}")


if __name__ == "__main__":
    generate_verified_pipes()
