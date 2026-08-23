"""ミリ以外の単位で書かれた STEP を作り、**それが本物であることを確かめる**。

FreeCAD の headless 書き出しは単位の設定を無視します（実測: `Unit` を 2
にしても `SI_UNIT(.MILLI.,.METRE.)` のまま、座標も同じ）。そこで別の作り方を
します。

1. 形を 1/25.4 に縮めて書き出す。数値はインチの値になり、宣言はミリのまま。
2. 長さの単位の実体だけを、`CONVERSION_BASED_UNIT('INCH', 25.4mm)` に差し替える。
   座標はもう触りません。**触るのは1箇所だけ**なので、書き換えで形が壊れません。
3. **出来たファイルを FreeCAD に読ませ、元の寸法に戻ることを確かめる。**

3 が要ります。自分で作ったファイルを自分の読み手に食わせても、ファイルが
正しいのか読み手が正しいのか分かりません。OpenCASCADE が 24000 mm^3 と
答えたなら、そのファイルはインチのファイルです。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/make_unit_step.py
"""

import json
import os
import re
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(FREECAD_BIN)
    except Exception:
        pass

import FreeCAD  # noqa: E402
import Part  # noqa: E402
import Import  # noqa: E402

OUT = os.path.join("target", "representation")

# 差し替える1行。OpenCASCADE はこの綴りで書き、この綴りで読みます。
MILLIMETRE = "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )"


def converted_unit(name, millimetres, base_id):
    """`name` 単位の実体4つ。`base_id` から始まる番号を使う。"""
    return (
        f"( CONVERSION_BASED_UNIT('{name}',#{base_id}) LENGTH_UNIT() "
        f"NAMED_UNIT(#{base_id + 1}) )",
        [
            f"#{base_id} = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({millimetres}),"
            f"#{base_id + 2});",
            f"#{base_id + 1} = DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);",
            f"#{base_id + 2} = {MILLIMETRE};",
        ],
    )


def write_unit_file(name, unit_name, millimetres, shape_mm):
    """`shape_mm`（ミリでの形）を `unit_name` で書いたファイルを作る。"""
    os.makedirs(OUT, exist_ok=True)
    scratch = os.path.join(OUT, f"_{name}_scratch.step")
    target = os.path.join(OUT, f"{name}.step")

    # 1. 数値をその単位の値にしてから書き出す。
    scaled = shape_mm.copy()
    scaled.scale(1.0 / millimetres)
    Import.export([Part.show(scaled)], scratch)
    FreeCAD.ActiveDocument.removeObject(FreeCAD.ActiveDocument.Objects[-1].Name)

    with open(scratch, "r", encoding="utf-8", errors="replace") as handle:
        text = handle.read()

    # 2. 長さの単位だけを差し替える。DATA 節に無い番号から採る。
    used = [int(value) for value in re.findall(r"^#(\d+)\s*=", text, re.MULTILINE)]
    base_id = max(used) + 100
    replacement, extra = converted_unit(unit_name, millimetres, base_id)

    if text.count(MILLIMETRE) < 1:
        raise RuntimeError(f"{scratch} does not declare millimetres in the expected form")
    # 長さの単位は1つだけ。角度・立体角の SI_UNIT は綴りが違うので当たりません。
    text = text.replace(MILLIMETRE, replacement, 1)
    text = text.replace("ENDSEC;\nEND-ISO-10303-21;", "\n".join(extra) + "\nENDSEC;\nEND-ISO-10303-21;")

    with open(target, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.remove(scratch)
    return target


def read_back(path):
    """OpenCASCADE に読ませて、体積と境界箱を返す。ここが外の物差し。"""
    shape = Part.Shape()
    shape.read(path)
    box = shape.BoundBox
    return shape.Volume, [box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax]


def main():
    if FreeCAD.ActiveDocument is None:
        FreeCAD.newDocument("units")

    # 平らな面だけの形では、**半径のスカラ**を読み落としても気づけません。
    # 座標だけ単位を掛けて半径を掛け忘れると、箱は通って円柱が壊れます。
    # だから曲面を持つ形と、内側のループを持つ形も並べます。
    block = Part.makeBox(20.0, 30.0, 40.0)
    cylinder = Part.makeCylinder(10.0, 40.0)
    drilled = Part.makeBox(30.0, 30.0, 15.0).cut(
        Part.makeCylinder(5.0, 40.0, FreeCAD.Vector(15.0, 15.0, -10.0))
    )

    subjects = []
    for name, unit_name, millimetres, shape in [
        ("block_inch", "INCH", 25.4, block),
        ("block_centimetre", "CENTIMETRE", 10.0, block),
        ("cylinder_inch", "INCH", 25.4, cylinder),
        ("drilled_inch", "INCH", 25.4, drilled),
    ]:
        expected_volume = shape.Volume
        box = shape.BoundBox
        expected_bbox = [box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax]
        path = write_unit_file(name, unit_name, millimetres, shape)
        volume, bbox = read_back(path)
        agrees = abs(volume - expected_volume) / expected_volume <= 1e-9
        subjects.append(
            {
                "name": name,
                "file": path.replace("\\", "/"),
                "unit": unit_name,
                "millimetres_per_unit": millimetres,
                "expected_volume_mm3": expected_volume,
                "expected_bbox_mm": expected_bbox,
                "occ_read_volume_mm3": volume,
                "occ_agrees": agrees,
            }
        )
        print(
            f"{name:<18} {unit_name:<12} OCC reads {volume:>14.6f} mm3  "
            f"(want {expected_volume:.6f})  {'ok' if agrees else 'MISMATCH'}"
        )

    manifest = os.path.join(OUT, "unit_manifest.json")
    with open(manifest, "w", encoding="utf-8") as handle:
        json.dump({"subjects": subjects}, handle, indent=2)
    print()
    print(f"wrote {len(subjects)} file(s) and {manifest}")
    print()
    print("OpenCASCADE reading these back at the right size is what makes them")
    print("real files rather than something shaped to suit our own reader.")
    if not all(subject["occ_agrees"] for subject in subjects):
        raise SystemExit("a file did not read back at the expected size")


if __name__ == "__main__":
    main()
