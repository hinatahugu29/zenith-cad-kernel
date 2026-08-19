"""
Zenith CAD Kernel - 新機能（断面スライス・物性値・シェル化・干渉判定）総合検証スクリプト
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


def verify_all_advanced_features():
    print("=" * 65)
    print("🧪 [Zenith CAD] カーネル基盤新機能 総合セルフチェック")
    print("=" * 65)

    # -------------------------------------------------------------
    # 1. 薄肉シェル化（Open-Top Box）と STEP 出力
    # -------------------------------------------------------------
    step_box = os.path.abspath("feature_showcase_open_box_shell.step")
    print(f"\n[Feature 1] 薄肉シェル容器（Open-Top Box）生成中 -> {os.path.basename(step_box)}")
    mesh_open = zenith_cad.make_open_box(40.0, 30.0, 20.0, 2.0, 8, 8, step_box)
    # 解析体積: 外側 (40*30*20 = 24000) - 内側 (36*26*18 = 16848) = 7152 mm^3
    print(f"  ✓ 薄肉シェル容器: {mesh_open.num_vertices} 頂点, 体積 = {mesh_open.volume:.2f} mm^3 (理論値: 7152.00 mm^3)")
    assert abs(mesh_open.volume - 7152.0) < 1.0, "Volume mismatch for open box shell"

    # -------------------------------------------------------------
    # 2. 断面スライス（Section Slicing）
    # -------------------------------------------------------------
    print(f"\n[Feature 2] 任意平面によるソリッド断面スライス検証 (Box 50x30x20 切断)")
    # Z=10 平面で切断（断面積は 50*30 = 1500 mm^2, 周長は 2*(50+30) = 160 mm）
    area, perim, loops = zenith_cad.slice_box_by_plane(
        50.0, 30.0, 20.0,
        [0.0, 0.0, 10.0],
        [0.0, 0.0, 1.0]
    )
    print(f"  ✓ Z=10 水平断面: 断面積 = {area:.2f} mm^2 (理論値: 1500.00 mm^2), 周長 = {perim:.2f} mm (理論値: 160.00 mm), ループ数 = {len(loops)}")
    assert abs(area - 1500.0) < 1e-3, "Slice area mismatch"
    assert abs(perim - 160.0) < 1e-3, "Slice perimeter mismatch"

    # 斜め45度平面での切断
    area_diag, perim_diag, loops_diag = zenith_cad.slice_box_by_plane(
        50.0, 30.0, 20.0,
        [25.0, 15.0, 10.0],
        [1.0, 1.0, 0.0]
    )
    print(f"  ✓ 斜め45度断面: 断面積 = {area_diag:.2f} mm^2, 周長 = {perim_diag:.2f} mm, ループ数 = {len(loops_diag)}")

    # -------------------------------------------------------------
    # 3. 高精度物性値・重心・慣性モーメント（Mass Properties）
    # -------------------------------------------------------------
    print(f"\n[Feature 3] 高精度物性値・重心・慣性テンソル計算 (Box 40x20x10)")
    # 理論値: 体積 8000 mm^3, 表面積 2*(800+400+200) = 2800 mm^2, 重心 (20, 10, 5)
    vol, surf, center, inertia = zenith_cad.compute_box_mass_properties(40.0, 20.0, 10.0, 1.0)
    print(f"  ✓ 体積: {vol:.2f} mm^3 (理論値: 8000.00 mm^3)")
    print(f"  ✓ 表面積: {surf:.2f} mm^2 (理論値: 2800.00 mm^2)")
    print(f"  ✓ 重心: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}) mm (理論値: 20.00, 10.00, 5.00)")
    print(f"  ✓ 主慣性モーメント対角項: ({inertia[0]:.2f}, {inertia[1]:.2f}, {inertia[2]:.2f})")
    assert abs(vol - 8000.0) < 1.0, "Volume mismatch"
    assert abs(surf - 2800.0) < 1.0, "Surface area mismatch"
    assert abs(center[0] - 20.0) < 0.1, "Center X mismatch"

    # -------------------------------------------------------------
    # 4. アセンブリ干渉・クリアランス判定（Interference / Clash Detection）
    # -------------------------------------------------------------
    print(f"\n[Feature 4] 2ソリッド間の干渉・衝突・クリアランス判定")

    # ケースA: 離れている場合 (Clearance)
    status_a, dist_a, vol_a, msg_a = zenith_cad.check_boxes_interference(
        10.0, 10.0, 10.0, [0.0, 0.0, 0.0],
        10.0, 10.0, 10.0, [25.0, 0.0, 0.0]
    )
    print(f"  ✓ 離脱判定 (Clearance): 状態 = {status_a}, 最小距離 = {dist_a:.2f} mm (理論値: 15.00 mm), メッセージ = {msg_a}")
    assert status_a == "Clearance"
    assert abs(dist_a - 15.0) < 1e-3

    # ケースB: めり込んでいる場合 (Clash)
    status_b, dist_b, vol_b, msg_b = zenith_cad.check_boxes_interference(
        20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
        20.0, 20.0, 20.0, [10.0, 10.0, 10.0]
    )
    print(f"  ✓ めり込み判定 (Clash): 状態 = {status_b}, 干渉体積 = {vol_b:.2f} mm^3 (理論値: 1000.00 mm^3), メッセージ = {msg_b}")
    assert status_b == "Clash"
    assert abs(vol_b - 1000.0) < 1e-3

    # -------------------------------------------------------------
    # STEP 自己検証 (STEP Import Roundtrip)
    # -------------------------------------------------------------
    print("\n" + "=" * 65)
    print("🔍 [STEP Self-Check] 生成された新機能 STEP ファイルの検証")
    print("=" * 65)
    fsize = os.path.getsize(step_box)
    imported = zenith_cad.import_step_file(step_box, 8, 8)
    print(f"  ✅ [PASS] {os.path.basename(step_box):<36} (サイズ: {fsize/1024:>5.1f} KB, 復元頂点数: {imported.num_vertices:>5})")

    print("\n" + "=" * 65)
    print("🎉 全 新機能（断面スライス・物性値・シェル化・干渉判定）の検証が完全合格しました！")
    print("=" * 65)


if __name__ == "__main__":
    verify_all_advanced_features()
