"""**スケッチを描いて、押し出して、ブーリアンして、書き出す、を Blender の中で
1 回動かす**（ROADMAP.md 柱③の次の一手）。

なぜ要るのか
------------
4-400 で `Solid.from_sketch_extruded` を足し、「スケッチを解けても
立体にできない」という穴を塞ぎました。**その後、この一連を実際に
Blender の中で通した記録がありませんでした**——`tools/blender_load_addon.py`
は原始立体（箱・円柱・球）とブーリアン 1 件しか見ていません。

**正しさの検証と同じやり方——動かして、測って、壊れた所を直す——を、
入口にも当てます。**

使い方
------
    "C:/Program Files/Blender Foundation/Blender 4.4/blender.exe" \
        --background --factory-startup \
        --python tools/blender_workflow_test.py

**`--factory-startup`** を付けます——利用者の設定やアドオンを
持ち込まないためです（`blender_load_addon.py` と同じ理由）。

何を測るか
----------
1. **直線だけのスケッチ**（矩形）を押し出し、閉じた式（底面積 × 高さ）
   と突き合わせます。
2. **穴あきスケッチ**（外周と内側の島）を押し出し、閉じた式
   （外周面積 − 穴の面積）× 高さと突き合わせます（4-392 の穴）。
3. **ブーリアン**——押し出した板から円柱で穴を開け、閉じた式
   （板の体積 − 円柱の体積）と突き合わせます。
4. **検証**——`validate()` が `valid: True` を言うか。
5. **STEP 書き出し**——ファイルが実在し、空でないか。
6. **表示メッシュ（OBJ）書き出し**——Blender 側で見る用途を想定し、
   ファイルが実在し、空でないか。

**赤にするのは**——どこかが断られる、閉じた式と食い違う、書き出した
ファイルが無いか空、のいずれか。
"""
import math
import os
import sys


def main():
    root = os.environ.get("ZENITH_ROOT")
    if not root:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    package = os.path.join(root, "blender_addon", "H-CAD_V_1_0_0")
    sys.path.insert(0, package)

    print()
    print("スケッチ→押し出し→ブーリアン→書き出し、を Blender の中で 1 回")
    print()

    try:
        import zenith_cad as z
    except Exception as exc:
        print("**読めません**: %r" % (exc,))
        return 1

    wrong = 0

    # 1. スケッチ: 20x10 の矩形（直線だけ）。
    sketch_json = (
        '{"points": [[0,0],[20,0],[20,10],[0,10]],'
        ' "lines": [[0,1],[1,2],[2,3],[3,0]]}'
    )
    try:
        plate = z.Solid.from_sketch_extruded(sketch_json, 5.0)
    except Exception as exc:
        print("**押し出しで断られました**: %r" % (exc,))
        return 1
    expected_plate = 20.0 * 10.0 * 5.0
    got_plate = plate.volume
    residual = abs(got_plate - expected_plate) / expected_plate
    print("1. スケッチ押し出し: 閉じた式 %.6f、測った値 %.6f、相対差 %.3e  %s"
          % (expected_plate, got_plate, residual,
             "ok" if residual < 1e-9 else "**ちがう**"))
    if residual >= 1e-9:
        wrong += 1

    # 2. 穴あきスケッチ: 外周 20x10、内側に 6x6 の穴（島）。
    hole_json = (
        '{"points": ['
        '[0,0],[20,0],[20,10],[0,10],'
        '[7,2],[13,2],[13,8],[7,8]],'
        ' "lines": [[0,1],[1,2],[2,3],[3,0],'
        '[4,5],[5,6],[6,7],[7,4]]}'
    )
    try:
        plate_with_hole = z.Solid.from_sketch_extruded(hole_json, 5.0)
        got_hole = plate_with_hole.volume
        expected_hole = (20.0 * 10.0 - 6.0 * 6.0) * 5.0
        residual_hole = abs(got_hole - expected_hole) / expected_hole
        print("2. 穴あきスケッチ押し出し: 閉じた式 %.6f、測った値 %.6f、相対差 %.3e  %s"
              % (expected_hole, got_hole, residual_hole,
                 "ok" if residual_hole < 1e-6 else "**ちがう**"))
        if residual_hole >= 1e-6:
            wrong += 1
    except Exception as exc:
        print("2. 穴あきスケッチ押し出し: **断られました**: %r" % (exc,))
        wrong += 1

    # 3. ブーリアン: 押し出した板から、円柱で穴を開ける。
    try:
        drill = z.Solid.cylinder(2.0, 10.0).translated(10.0, 5.0, -2.5)
        drilled = plate.difference(drill)
        expected_drilled = expected_plate - math.pi * 2.0 * 2.0 * 5.0
        got_drilled = drilled.volume
        residual_drilled = abs(got_drilled - expected_drilled) / expected_drilled
        print("3. ブーリアン（板 - 円柱）: 閉じた式 %.6f、測った値 %.6f、相対差 %.3e  %s"
              % (expected_drilled, got_drilled, residual_drilled,
                 "ok" if residual_drilled < 1e-6 else "**ちがう**"))
        if residual_drilled >= 1e-6:
            wrong += 1
    except Exception as exc:
        print("3. ブーリアン: **断られました**: %r" % (exc,))
        wrong += 1
        drilled = plate

    # 4. 検証: validate() が問題無しと言うか。
    try:
        report = drilled.validate()
        print("4. 検証: %r" % (dict(report),))
        if not report.get("valid", False):
            wrong += 1
    except Exception as exc:
        print("4. 検証: **断られました**: %r" % (exc,))
        wrong += 1

    # 5. 書き出し: STEP として一時ファイルに保存し、実在と空でないかを見る。
    import tempfile

    out_dir = os.environ.get("ZENITH_WORKFLOW_OUT") or tempfile.gettempdir()
    step_path = os.path.join(out_dir, "blender_workflow_test.step")
    try:
        drilled.to_step(step_path, "ZENITH_WORKFLOW_TEST")
        size = os.path.getsize(step_path)
        print("5. STEP 書き出し: %s（%d バイト）" % (step_path, size))
        if size <= 0:
            wrong += 1
    except Exception as exc:
        print("5. STEP 書き出し: **断られました**: %r" % (exc,))
        wrong += 1

    # 6. 表示メッシュも書き出す（Blender 側で見る用途を想定）。
    try:
        mesh = drilled.tessellate(24, 24)
        obj_path = os.path.join(out_dir, "blender_workflow_test.obj")
        mesh.export_obj(obj_path, "zenith_workflow_test")
        size_obj = os.path.getsize(obj_path)
        print("6. OBJ 書き出し: %s（%d バイト）、三角形 %d 枚"
              % (obj_path, size_obj, mesh.num_faces))
        if size_obj <= 0:
            wrong += 1
    except Exception as exc:
        print("6. OBJ 書き出し: **断られました**: %r" % (exc,))
        wrong += 1

    print()
    print("誤答・断り %d 件" % wrong)
    return 1 if wrong else 0


if __name__ == "__main__":
    sys.exit(main())
