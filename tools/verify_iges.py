"""
Zenith CAD Kernel: IGES 5.3 出力の外部相互検証（FreeCAD / OpenCASCADE 7.8）

`cargo run --release -p zenith_algo --example export_iges_suite` が
`target/iges/` に書いた `.igs` を OpenCASCADE に読ませ、

  1. そもそもエラーなく読めるか
  2. 読めた曲面の枚数が、こちらの面の枚数と一致するか
  3. 読めた形の境界箱が、元の立体の境界箱を覆っているか

を確かめる。

**体積では突き合わせない。** こちらの IGES はトリムを書いていない（Entity 128
の曲面だけで、Entity 144 / 142 / 126 は出していない）ので、読めるのは面の
土台であって閉じた立体ではない。したがって境界箱は元の立体と同じか大きく
なるのが正しく、小さくなったらそれは曲面が欠けている。

不一致があれば非ゼロ終了するので、リリースゲートに使える。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/verify_iges.py
"""

import json
import os
import sys

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
        box = shape.BoundBox
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
    print("grow   = how far they extend OUTSIDE it, which is expected because")
    print("         we do not write the trimming entities yet")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
