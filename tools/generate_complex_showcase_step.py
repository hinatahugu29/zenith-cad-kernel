"""
Zenith CAD Kernel - 複合的実用形状 STEP ファイル生成＆自己検証スクリプト
FreeCAD / 商用CAD でのインポート検証用
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


def generate_showcase_models():
    print("=" * 60)
    print("[Zenith CAD] 複合的実用形状 STEP ファイル生成＆自己検証開始")
    print("=" * 60)


    output_files = []

    # -------------------------------------------------------------
    # 1. complex_mechanical_housing.step (複合機械ハウジング・金型ボス・ミラー)
    # -------------------------------------------------------------
    path_1 = os.path.abspath("complex_mechanical_housing.step")
    print(f"\n[Model 1] 複合機械ハウジング生成中 -> {os.path.basename(path_1)}")

    # 抜き勾配付きドラフト押し出しソリッド (底面 40x40, 高さ 20, 勾配 5度)
    prof_draft = [[-20, -20, 0], [20, -20, 0], [20, 20, 0], [-20, 20, 0]]
    mesh_draft = zenith_cad.make_draft_extrusion(
        prof_draft, [0, 0, 20], 5.0, 8, 8, path_1
    )
    print(
        f"  ✓ ドラフト押し出しボス: {mesh_draft.num_vertices} 頂点, 体積 = {mesh_draft.volume:.2f} mm^3"
    )

    # ミラー反転左右対称ペア (面取り非対称ケーシングが対称平面 X=0 を挟んで左右対称に配置された複合ソリッドペア)
    path_1_mir = os.path.abspath("complex_mirrored_casing.step")
    mesh_mir = zenith_cad.make_mirror_compound_casing(
        30.0, 50.0, 20.0, 10.0, 6.0, [0, 0, 0], [1, 0, 0], 6, 6, path_1_mir
    )
    print(
        f"  ✓ ミラー反転左右対称ペア: {mesh_mir.num_vertices} 頂点, 体積 = {mesh_mir.volume:.2f} mm^3"
    )

    output_files.extend([path_1, path_1_mir])

    # -------------------------------------------------------------
    # 2. complex_helical_spring_assembly.step (3D螺旋スプリング ＆ 部分回転エルボ)
    # -------------------------------------------------------------
    path_2 = os.path.abspath("complex_helical_spring.step")
    print(f"\n[Model 2] 3D螺旋ヘリカルスプリング生成中 -> {os.path.basename(path_2)}")

    # 正方形断面の3Dヘリカルスプリング (半径 25, ピッチ 15, 2.5巻き)
    prof_spring = [[-2, -2, 0], [2, -2, 0], [2, 2, 0], [-2, 2, 0]]
    mesh_spring = zenith_cad.make_helix_solid(
        prof_spring,
        25.0,
        15.0,
        2.5,
        [0, 0, 0],
        [0, 0, 1],
        48,
        8,
        8,
        path_2,
    )
    print(
        f"  ✓ ヘリカルスプリング: {mesh_spring.num_vertices} 頂点, 体積 = {mesh_spring.volume:.2f} mm^3"
    )

    # 90度部分回転エルボ管 (XZ断面をZ軸まわりに90度回転したパイプエルボ立体)
    path_2_elbow = os.path.abspath("complex_elbow_pipe.step")
    prof_elbow = [[15, 0, -3], [21, 0, -3], [21, 0, 3], [15, 0, 3]]
    mesh_elbow = zenith_cad.make_partial_revolve_solid(
        prof_elbow, [0, 0, 0], [0, 0, 1], 90.0, 8, 8, path_2_elbow
    )
    print(
        f"  ✓ 90度部分回転エルボ: {mesh_elbow.num_vertices} 頂点, 体積 = {mesh_elbow.volume:.2f} mm^3"
    )

    output_files.extend([path_2, path_2_elbow])

    # -------------------------------------------------------------
    # 3. complex_guided_loft_casing.step (ガイドレール自由曲面ロフト ＆ 両端開口角パイプ)
    # -------------------------------------------------------------
    path_3 = os.path.abspath("complex_guided_loft.step")
    print(
        f"\n[Model 3] ガイドレール自由曲面ロフト生成中 -> {os.path.basename(path_3)}"
    )

    # ガイドレール付きロフト (底面 30x30, 天面 20x20, 中央が外側に+8mm大きく膨らむガイド曲線)
    sec_bot = [[-15, -15, 0], [15, -15, 0], [15, 15, 0], [-15, 15, 0]]
    sec_top = [[-10, -10, 40], [10, -10, 40], [10, 10, 40], [-10, 10, 40]]
    guide_curve = [[15, -15, 0], [23, -15, 20], [10, -10, 40]]
    mesh_gloft = zenith_cad.make_guided_loft_solid(
        [sec_bot, sec_top], [guide_curve], 2, 12, 12, path_3
    )
    print(
        f"  ✓ ガイド曲面ロフト: {mesh_gloft.num_vertices} 頂点, 体積 = {mesh_gloft.volume:.2f} mm^3"
    )

    # 両端開放角パイプ中空ソリッド (外形 40x30, 長さ 60, 肉厚 t=2.5)
    path_3_tube = os.path.abspath("complex_through_tube.step")
    mesh_tube = zenith_cad.make_through_hollow_box(
        40.0, 30.0, 60.0, 2.5, 6, 6, path_3_tube
    )
    print(
        f"  ✓ 両端開口角パイプ: {mesh_tube.num_vertices} 頂点, 体積 = {mesh_tube.volume:.2f} mm^3"
    )
    output_files.extend([path_3, path_3_tube])

    # -------------------------------------------------------------
    # 4. complex_drilled_hollow_casing.step (貫通穴 ＆ 複数穴あき中空押し出し)
    # -------------------------------------------------------------
    path_4_hole = os.path.abspath("complex_drilled_box.step")
    mesh_hole = zenith_cad.make_drilled_box(
        40.0, 40.0, 25.0, 8.0, 12, 12, path_4_hole
    )
    print(
        f"  ✓ 貫通穴あきボックス: {mesh_hole.num_vertices} 頂点, 体積 = {mesh_hole.volume:.2f} mm^3"
    )

    path_4_hollow = os.path.abspath("complex_hollow_extrusion.step")
    outer_poly = [[-25, -25, 0], [25, -25, 0], [25, 25, 0], [-25, 25, 0]]
    hole1 = [[-15, -15, 0], [-5, -15, 0], [-5, -5, 0], [-15, -5, 0]]
    hole2 = [[5, 5, 0], [15, 5, 0], [15, 15, 0], [5, 15, 0]]
    mesh_hollow = zenith_cad.make_hollow_extrusion(
        outer_poly, [hole1, hole2], [0, 0, 35], 8, 8, path_4_hollow
    )
    print(
        f"  ✓ 2連角穴中空押し出し: {mesh_hollow.num_vertices} 頂点, 体積 = {mesh_hollow.volume:.2f} mm^3"
    )
    output_files.extend([path_4_hole, path_4_hollow])

    # -------------------------------------------------------------
    # 自己検証（Self-Verification）: カーネル側で全ファイルを再読み込みして検証
    # -------------------------------------------------------------
    print("\n" + "=" * 60)
    print("🔍 [Self-Check] Zenith STEP インポーターによるラウンドトリップ検証")
    print("=" * 60)

    for fpath in output_files:
        fname = os.path.basename(fpath)
        fsize = os.path.getsize(fpath)
        try:
            imported_mesh = zenith_cad.import_step_file(fpath, 8, 8)
            print(
                f"  ✅ [PASS] {fname:<32} (サイズ: {fsize/1024:>6.1f} KB, 復元頂点数: {imported_mesh.num_vertices:>5})"
            )
        except Exception as e:
            print(f"  ❌ [FAIL] {fname:<32} -> {e}")

    print("\n" + "=" * 60)
    print("🎉 全複合 STEP ファイルの出力と自己検証が完了しました！")
    print("   FreeCAD 等の外部 CAD で直接開いて確認可能です。")
    print("=" * 60)
    for fpath in output_files:
        print(f"   • {fpath}")


if __name__ == "__main__":
    generate_showcase_models()
