"""同じ形を、違う書き方で書き分けさせる。

読み手を測るとき、これまでの検体は10本とも「OpenCASCADE の既定設定」でした。
書き手を増やすことは（有償 CAD が要るので）できませんが、**同じ書き手に
違う書き方をさせる**ことはできます。実務で効くのはむしろそちらです。

- **単位。** インチで書かれたファイル。長さの単位を読み落とすと、答えは
  静かに 25.4 倍ずれます。IGES で実際にやりました（HANDOVER 4-38）。
- **スキーマ。** AP203 / AP214 / AP242。エンティティの綴りと付随情報が変わります。
- **B-spline 変換。** 解析曲面のまま書くか、全部 B-spline に落とすか。
- **1ファイルに複数の立体。** 実務のファイルは1個ではありません。
- **ソリッドではなくシェル。** 閉じていない面の集まりとして書かれたもの。
- **アセンブリ。** 部品ごとに座標系を持つ入れ子。

出力は target/representation/ に置き、manifest.json に**そのファイルが何で
あるか**（期待体積、単位、スキーマ）を書きます。読み手の答えはこれと
突き合わせます。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/occ_representation_suite.py
"""

import json
import os
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(FREECAD_BIN)
    except Exception:
        pass

import FreeCAD  # noqa: E402,F401  (must load before Part)
import Part  # noqa: E402
import Import  # noqa: E402

OUT = os.path.join("target", "representation")

MM_PER_INCH = 25.4


def block(length=20.0, width=30.0, height=40.0):
    """辺の長さが全部違う箱。向きを取り違えたら体積では気づけても、
    境界箱では必ず気づきます。"""
    return Part.makeBox(length, width, height)


def drilled_block():
    """穴あきの箱。内側のループを持つ面があるので、トリムの読み落としが出ます。"""
    solid = Part.makeBox(30.0, 30.0, 15.0)
    drill = Part.makeCylinder(5.0, 40.0, FreeCAD.Vector(15.0, 15.0, -10.0))
    return solid.cut(drill)


def two_solids():
    """1ファイルに離れた2立体。合計だけ見ていると、片方を落としても気づけません。"""
    a = Part.makeBox(10.0, 10.0, 10.0)
    b = Part.makeBox(20.0, 5.0, 5.0, FreeCAD.Vector(40.0, 0.0, 0.0))
    return Part.makeCompound([a, b])


def write(name, shape, schema=None, unit=None, as_bspline=False, expected_volume=None):
    """1本書いて、書いたものの説明を返す。"""
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{name}.step")

    parameters = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Import/hSTEP")
    if schema is not None:
        parameters.SetString("Scheme", schema)
    if unit is not None:
        parameters.SetInt("Unit", unit)
    parameters.SetBool("ExportLegacy", False)

    written = shape
    if as_bspline:
        # 解析曲面を全部 B-spline に落とす。多くの書き出しがこれをやります。
        written = shape.toNurbs()

    Import.export([Part.show(written)], path)
    FreeCAD.ActiveDocument.removeObject(FreeCAD.ActiveDocument.Objects[-1].Name)

    volume = expected_volume if expected_volume is not None else written.Volume
    box = written.BoundBox
    return {
        "name": name,
        "file": path.replace("\\", "/"),
        "schema": schema or "default",
        "unit": {0: "mm", 1: "m", 2: "inch", 3: "foot"}.get(unit, "mm"),
        "as_bspline": as_bspline,
        "expected_volume_mm3": volume,
        "expected_bbox_mm": [
            box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax
        ],
        "solid_count": len(written.Solids),
    }


def main():
    if FreeCAD.ActiveDocument is None:
        FreeCAD.newDocument("representation")

    subjects = []

    # 1. 基準。既定の設定そのまま。他の行はこれとの差だけが違いです。
    subjects.append(write("block_default", block(), expected_volume=24000.0))

    # 2. スキーマ違い。形も単位も同じ。
    subjects.append(write("block_ap203", block(), schema="AP203", expected_volume=24000.0))
    subjects.append(
        write("block_ap214", block(), schema="AP214IS", expected_volume=24000.0)
    )
    subjects.append(
        write("block_ap242", block(), schema="AP242DIS", expected_volume=24000.0)
    )

    # 3. 単位違い。**ここが一番効きます。** 形は同じ 20x30x40 mm ですが、
    #    ファイルはインチで書かれます。単位を読み落とすと体積が 25.4^3 倍
    #    （16387倍）ずれます。読み落としても「もっともらしい立体」が返るので、
    #    形を見ていては気づけません。
    subjects.append(
        write("block_inch", block(), unit=2, expected_volume=24000.0)
    )
    subjects.append(write("block_metre", block(), unit=1, expected_volume=24000.0))

    # 4. 解析曲面を全部 B-spline に落としたもの。
    subjects.append(
        write("drilled_bspline", drilled_block(), as_bspline=True)
    )
    subjects.append(write("drilled_analytic", drilled_block()))

    # 5. 1ファイルに2立体。
    subjects.append(write("two_solids", two_solids(), expected_volume=1000.0 + 500.0))

    manifest = os.path.join(OUT, "manifest.json")
    with open(manifest, "w", encoding="utf-8") as handle:
        json.dump({"subjects": subjects}, handle, indent=2)

    print(f"{'subject':<20} {'schema':<10} {'unit':<6} {'bspline':<8} {'solids':>6} {'volume mm3':>16}")
    print("-" * 74)
    for subject in subjects:
        print(
            f"{subject['name']:<20} {subject['schema']:<10} {subject['unit']:<6} "
            f"{str(subject['as_bspline']):<8} {subject['solid_count']:>6} "
            f"{subject['expected_volume_mm3']:>16.4f}"
        )
    print()
    print(f"wrote {len(subjects)} file(s) and {manifest}")
    print()
    print("Every row is the same geometry written a different way, so the")
    print("reader must give the same answer for all of them. It is the unit")
    print("rows that decide whether a wrong answer can arrive looking plausible.")


if __name__ == "__main__":
    main()
