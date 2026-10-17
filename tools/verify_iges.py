"""
Zenith CAD Kernel: IGES 5.3 出力の外部相互検証（FreeCAD / OpenCASCADE 7.8）

`cargo run --release -p zenith_algo --example export_iges_suite` が
`target/iges/` に書いた `.igs` を OpenCASCADE に読ませ、

  1. そもそもエラーなく読めるか
  2. 読めた曲面の枚数が、こちらの面の枚数と一致するか
  3. 読めた形の境界箱が、元の立体の境界箱を覆っているか

を確かめる。

**体積では突き合わせない。** IGES はまだ面を縫合した Shell / Solid の
実体を書いていないので、読み戻すと面がバラの Compound になる——閉じた
立体として体積を測れるのは STEP の側。

**4-769 でトリム（Entity 144 / 142 / 126）を書けるようになった。**
`face.pcurves` を持つ面（自前ビルダーの面はほぼ全部）はトリムされた
境界箱が返るはずで、境界箱は元の立体とほぼ一致する（`grow` が小さい）。
`pcurves` を持たない面（他カーネルから読んだ全周1枚の面など）は依然
支持曲面の生の範囲のままなので、そちらは `grow` が残る。**小さくなる
（`shrink` が正）のは曲面が欠けている印**で、こちらは常に赤にする。

不一致があれば非ゼロ終了するので、リリースゲートに使える。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/verify_iges.py
"""

import io
import json
import os
import sys

# Windows のコンソールは既定で UTF-8 でないことがあり、日本語の出力が
# 文字化けする（4-771）。標準出力を明示的に UTF-8 に差し替える。
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.append(FREECAD_BIN)

import FreeCAD  # noqa: E402
import Part  # noqa: E402

IGES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "target", "iges")

# 曲面はトリム前の土台なので、境界箱は元より大きくなりうる。小さくなるのは欠落。
SHRINK_TOLERANCE = 1e-6


def read_shape(path):
    shape = Part.Shape()
    shape.read(path)
    return shape


def per_face_bound_box(shape):
    """各面の `BoundBox` を合算して箱を作る。

    4-769 で分かったこと: IGES は面を縫合した Shell / Solid の実体を
    まだ書いていないので、読み戻すと面がバラの Compound になる。
    **`shape.BoundBox`（全体）は、この形だと面ごとのトリムを無視して
    支持曲面の生の範囲まで広がる**（実測: 20x30x40 の箱が
    `shape.BoundBox` では (0,0,0)-(40,40,40) になるのに、面ごとの
    `BoundBox` を合算すると正しい (0,0,0)-(20,30,40) になる）。
    **トリムが効いているかどうかを測るには、この合算のほうを使う。**
    """
    box = FreeCAD.BoundBox()
    for face in shape.Faces:
        box.add(face.BoundBox)
    return box


def main():
    manifest_path = os.path.join(IGES_DIR, "manifest.json")
    if not os.path.exists(manifest_path):
        print("manifest.json not found. Run:")
        print("  cargo run --release -p zenith_algo --example export_iges_suite")
        return 1

    with open(manifest_path, "r", encoding="utf-8") as handle:
        subjects = json.load(handle)

    header = "{:<26}{:>7}{:>7}{:>14}{:>14}{:>10}".format(
        "subject", "ours", "occ", "shrink x/y/z", "grow", "verdict"
    )
    print(header)
    print("-" * len(header))

    problems = 0
    for subject in subjects:
        path = os.path.join(IGES_DIR, subject["file"])
        try:
            shape = read_shape(path)
        except Exception as error:  # noqa: BLE001
            print("{:<26}{:>7}{:>7}  READ FAILED: {}".format(
                subject["name"], subject["faces"], "-", error))
            problems += 1
            continue

        faces = len(shape.Faces)
        box = per_face_bound_box(shape)
        low = subject["low"]
        high = subject["high"]

        shrink = max(
            box.XMin - low[0], box.YMin - low[1], box.ZMin - low[2],
            high[0] - box.XMax, high[1] - box.YMax, high[2] - box.ZMax,
        )
        grow = max(
            low[0] - box.XMin, low[1] - box.YMin, low[2] - box.ZMin,
            box.XMax - high[0], box.YMax - high[1], box.ZMax - high[2],
        )

        issues = []
        if faces != subject["faces"]:
            issues.append("face count {} != {}".format(faces, subject["faces"]))
        if shrink > SHRINK_TOLERANCE:
            issues.append("the imported surfaces fall {:.3e} short of the solid".format(shrink))

        # 4-771: 境界箱が揃っても、トリムが実際に効いているとは言えない
        # ——面ごとの面積を合算して、こちらの面積と突き合わせる。
        # `all_pcurves` な検体（全面が `pcurves` を持つ）だけ、ここを
        # 厳しく見る。持たない面がある検体は、トリム前の土台のままで
        # 正しいので、ここでは見ない。
        if subject.get("all_pcurves") and "surface_area" in subject:
            occ_area = sum(f.Area for f in shape.Faces)
            ours_area = subject["surface_area"]
            area_rel = abs(occ_area - ours_area) / max(abs(ours_area), 1.0)
            if area_rel > 1e-3:
                issues.append(
                    "面積が合いません（トリムが効いていない疑い）: ours={:.3f} occ={:.3f} (相対差 {:.3e})".format(
                        ours_area, occ_area, area_rel
                    )
                )

        verdict = "ok" if not issues else "PROBLEM"
        if issues:
            problems += 1
        print("{:<26}{:>7}{:>7}{:>14.3e}{:>14.3e}{:>10}".format(
            subject["name"], subject["faces"], faces, shrink, grow, verdict))
        for issue in issues:
            print("      {}".format(issue))

    print("-" * len(header))
    print("{} of {} IGES files read back with the expected surfaces".format(
        len(subjects) - problems, len(subjects)))
    print()
    print("ours   = faces in our B-Rep")
    print("occ    = surfaces OpenCASCADE recovered from the IGES file")
    print("shrink = how far the imported surfaces fall INSIDE the solid's box")
    print("         (must be <= 0; anything positive means a surface is missing)")
    print("grow   = how far they extend OUTSIDE it (per-face bound boxes,")
    print("         summed). Faces with a stored p-curve are trimmed (144/142/126)")
    print("         and should grow ~0; faces without one still write the raw")
    print("         untrimmed surface (128) and keep growing.")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
