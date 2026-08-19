"""
Zenith CAD Kernel - 3D ポリライン配管・角丸めフレーム STEP ファイル生成スクリプト
産業用配管・油圧配管・フレーム構造の FreeCAD 検証用
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


def generate_polyline_showcase_models():
    print("=" * 65)
    print("🔧 [Zenith CAD] 3D ポリライン配管・フレーム STEP ファイル生成＆検証")
    print("=" * 65)

    output_files = []

    # -------------------------------------------------------------
    # 1. polyline_showcase_hydraulic_manifold_pipe.step (角丸め油圧配管パイプ)
    # -------------------------------------------------------------
    path_1 = os.path.abspath("polyline_showcase_hydraulic_pipe.step")
    print(
        f"\n[Polyline 1] 3D角丸め油圧配管パイプ生成中 -> {os.path.basename(path_1)}"
    )

    # 3D空間を90度および45度で屈曲する産業用油圧配管パス
    pipe_path = [
        [0.0, 0.0, 0.0],
        [60.0, 0.0, 0.0],
        [60.0, 50.0, 0.0],
        [60.0, 50.0, 40.0],
        [20.0, 80.0, 40.0],
        [20.0, 80.0, 80.0],
    ]
    # 外径 R=4.0mm, コーナー曲げ半径 R=10.0mm
    mesh_pipe = zenith_cad.make_polyline_pipe(
        pipe_path, 4.0, 10.0, 16, 16, path_1
    )
    print(
        f"  ✓ 角丸め油圧配管: {mesh_pipe.num_vertices} 頂点, 体積 = {mesh_pipe.volume:.2f} mm^3"
    )
    output_files.append(path_1)

    # -------------------------------------------------------------
    # 2. polyline_showcase_structural_frame_sweep.step (角丸め角形フレーム)
    # -------------------------------------------------------------
    path_2 = os.path.abspath("polyline_showcase_structural_frame.step")
    print(
        f"\n[Polyline 2] 3D角丸めポリライン沿い角形フレーム生成中 -> {os.path.basename(path_2)}"
    )

    # 10x6mm 長方形断面
    prof_rect = [
        [-5.0, -3.0, 0.0],
        [5.0, -3.0, 0.0],
        [5.0, 3.0, 0.0],
        [-5.0, 3.0, 0.0],
    ]
    # U字・立体クランク状のフレームパス
    frame_path = [
        [0.0, 0.0, 0.0],
        [0.0, 60.0, 0.0],
        [40.0, 60.0, 30.0],
        [40.0, 0.0, 30.0],
    ]
    # コーナー曲げ半径 R=12.0mm
    mesh_frame = zenith_cad.make_polyline_sweep(
        prof_rect, frame_path, 12.0, 12, 12, path_2
    )
    print(
        f"  ✓ 角丸め角形フレーム: {mesh_frame.num_vertices} 頂点, 体積 = {mesh_frame.volume:.2f} mm^3"
    )
    output_files.append(path_2)

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
    print("🎉 全 3D ポリライン配管・フレーム STEP ファイルの出力と自己検証が完了しました！")
    print("   FreeCAD 等の外部 CAD で直接開いて確認可能です。")
    print("=" * 65)
    for fpath in output_files:
        print(f"   • {fpath}")


if __name__ == "__main__":
    generate_polyline_showcase_models()
