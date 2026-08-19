"""
Zenith CAD Kernel - アドバンスド複合モデル STEP ファイル生成スクリプト
ハイエンド機械部品・自由曲面アセンブリの FreeCAD 検証用
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


def generate_advanced_models():
    print("=" * 65)
    print("🚀 [Zenith CAD] ハイエンド複合モデル STEP ファイル生成＆検証")
    print("=" * 65)

    output_files = []

    # -------------------------------------------------------------
    # 1. showcase_flange_coupling.step (円形パターン 4穴締結 フランジ継手)
    # -------------------------------------------------------------
    path_1 = os.path.abspath("showcase_flange_coupling.step")
    print(
        f"\n[Model 1] 円形パターン4穴締結フランジ継手生成中 -> {os.path.basename(path_1)}"
    )

    # 40x40 角フランジ、中央に直径 16mm の貫通穴、四隅に面取り
    prof_flange = [[-20, -20, 0], [20, -20, 0], [20, 20, 0], [-20, 20, 0]]
    mesh_flange = zenith_cad.make_drilled_box(
        40.0, 40.0, 15.0, 8.0, 16, 16, path_1
    )
    print(
        f"  ✓ 4穴貫通フランジ本体: {mesh_flange.num_vertices} 頂点, 体積 = {mesh_flange.volume:.2f} mm^3"
    )
    output_files.append(path_1)

    # -------------------------------------------------------------
    # 2. showcase_multi_section_guided_loft.step (多段異形自由曲面ロフト)
    # -------------------------------------------------------------
    path_2 = os.path.abspath("showcase_multi_section_guided_loft.step")
    print(
        f"\n[Model 2] 3断面多段ガイド自由曲面ロフト生成中 -> {os.path.basename(path_2)}"
    )

    # 底面 (30x30, Z=0) -> 中間 (40x20, Z=25) -> 天面 (15x15, Z=50)
    sec_1 = [[-15, -15, 0], [15, -15, 0], [15, 15, 0], [-15, 15, 0]]
    sec_2 = [[-20, -10, 25], [20, -10, 25], [20, 10, 25], [-20, 10, 25]]
    sec_3 = [[-7.5, -7.5, 50], [7.5, -7.5, 50], [7.5, 7.5, 50], [-7.5, 7.5, 50]]

    # 側面に大きくS字状に張り出す3Dガイドレール曲線
    guide = [[15, -15, 0], [28, -10, 25], [7.5, -7.5, 50]]

    mesh_mloft = zenith_cad.make_guided_loft_solid(
        [sec_1, sec_2, sec_3], [guide], 2, 12, 12, path_2
    )
    print(
        f"  ✓ 多段ガイドロフト: {mesh_mloft.num_vertices} 頂点, 体積 = {mesh_mloft.volume:.2f} mm^3"
    )
    output_files.append(path_2)

    # -------------------------------------------------------------
    # 3. showcase_spring_elbow_mechanism.step (3Dヘリカルスプリング ＆ エルボ管)
    # -------------------------------------------------------------
    path_3 = os.path.abspath("showcase_spring_mechanism.step")
    print(
        f"\n[Model 3] 3Dヘリカルスプリング（3.5巻き高精度）生成中 -> {os.path.basename(path_3)}"
    )

    # 長方形断面 (3x2mm) の3.5巻きヘリカルスプリング (半径 18mm, ピッチ 12mm)
    prof_spr = [[-1.5, -1, 0], [1.5, -1, 0], [1.5, 1, 0], [-1.5, 1, 0]]
    mesh_spr = zenith_cad.make_helix_solid(
        prof_spr, 18.0, 12.0, 3.5, [0, 0, 0], [0, 0, 1], 56, 8, 8, path_3
    )
    print(
        f"  ✓ 3.5巻きスプリング: {mesh_spr.num_vertices} 頂点, 体積 = {mesh_spr.volume:.2f} mm^3"
    )
    output_files.append(path_3)

    # -------------------------------------------------------------
    # 4. showcase_symmetric_machined_bracket.step (面取り＆ドラフト 左右対称ミラーペア)
    # -------------------------------------------------------------
    path_4 = os.path.abspath("showcase_symmetric_bracket_pair.step")
    print(
        f"\n[Model 4] 面取り非対称ブラケット左右対称ミラーペア生成中 -> {os.path.basename(path_4)}"
    )

    mesh_bracket = zenith_cad.make_mirror_compound_casing(
        35.0, 60.0, 25.0, 15.0, 8.0, [0, 0, 0], [1, 0, 0], 6, 6, path_4
    )
    print(
        f"  ✓ 左右対称ブラケットペア: {mesh_bracket.num_vertices} 頂点, 体積 = {mesh_bracket.volume:.2f} mm^3"
    )
    output_files.append(path_4)

    # -------------------------------------------------------------
    # 5. showcase_aerospace_through_hollow_beam.step (長尺 両端開放角パイプビーム)
    # -------------------------------------------------------------
    path_5 = os.path.abspath("showcase_aerospace_hollow_beam.step")
    print(
        f"\n[Model 5] 航空宇宙向け長尺両端開口角パイプ生成中 -> {os.path.basename(path_5)}"
    )

    # 50x30x100mm, 肉厚 t=3.0mm
    mesh_beam = zenith_cad.make_through_hollow_box(
        50.0, 30.0, 100.0, 3.0, 6, 6, path_5
    )
    print(
        f"  ✓ 両端開口角パイプビーム: {mesh_beam.num_vertices} 頂点, 体積 = {mesh_beam.volume:.2f} mm^3"
    )
    output_files.append(path_5)

    # -------------------------------------------------------------
    # 6. showcase_draft_die_core.step (抜き勾配 8度 金型コアブロック)
    # -------------------------------------------------------------
    path_6 = os.path.abspath("showcase_draft_die_core.step")
    print(
        f"\n[Model 6] 抜き勾配8度金型コアブロック生成中 -> {os.path.basename(path_6)}"
    )

    prof_die = [[-25, -25, 0], [25, -25, 0], [25, 25, 0], [-25, 25, 0]]
    mesh_die = zenith_cad.make_draft_extrusion(
        prof_die, [0, 0, 30], 8.0, 8, 8, path_6
    )
    print(
        f"  ✓ 勾配8度金型コア: {mesh_die.num_vertices} 頂点, 体積 = {mesh_die.volume:.2f} mm^3"
    )
    output_files.append(path_6)

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
                f"  ✅ [PASS] {fname:<38} (サイズ: {fsize/1024:>6.1f} KB, 復元頂点数: {imported_mesh.num_vertices:>5})"
            )
        except Exception as e:
            print(f"  ❌ [FAIL] {fname:<38} -> {e}")

    print("\n" + "=" * 65)
    print("🎉 全ハイエンド複合 STEP ファイルの出力と自己検証が完了しました！")
    print("   FreeCAD 等の外部 CAD で直接開いて確認可能です。")
    print("=" * 65)
    for fpath in output_files:
        print(f"   • {fpath}")


if __name__ == "__main__":
    generate_advanced_models()
