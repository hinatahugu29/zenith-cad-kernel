"""
OpenCASCADE に、**実務で普通に出てくる形**の STEP を書かせる。

いまの検体は解析曲面（円柱・円錐・球・トーラス）と、掃引したものが少しです。
そこには**フィレット・面取り・複数の穴・ロフト・スロット・中空**が1つも
ありません。部品ファイルを開けばまず出てくる形が、まるごと測られていない
ということです。

こちらで組み立てた検体は、こちらの思い込みをそのまま検査してしまうので、
**相手の実装に書かせます**。書いたあと読み直して、ファイルに入った形を
測ります。渡した形ではなく、入った形が突き合わせの対象です。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/occ_reference_shapes.py

出力は `target/validation/` に置き、OCC が測った体積・表面積・面数を
`shapes_manifest.json` に添えます。検体として使うものは
`crates/zenith_algo/tests/fixtures/` へ複写してください。
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

import FreeCAD  # noqa: E402
import Part  # noqa: E402

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "target", "validation"
)

V = FreeCAD.Vector


def filleted_box():
    """全稜を r3 で丸めた箱。

    **面取りされた角**が入ります。稜のフィレットは円柱面、頂点のところは
    球面になり、それらが3枚ずつ集まる角が12箇所できます。ブレンドを他の
    カーネルから受け取ったことは一度もありません。
    """
    box = Part.makeBox(30.0, 20.0, 12.0)
    return box.makeFillet(3.0, box.Edges)


def chamfered_box():
    """全稜を 2.5 で面取りした箱。

    フィレットと違って面はすべて平面ですが、**1つの頂点に6枚の平面が
    集まります**。頂点まわりの面の集まりを、曲面抜きで測れます。
    """
    box = Part.makeBox(30.0, 20.0, 12.0)
    return box.makeChamfer(2.5, box.Edges)


def plate_with_holes():
    """3つの穴があいた板。半径も位置もそろえません。

    **1枚の面が内側のワイヤを3本持ちます。** これまで穴を2本以上持つ面は
    輪（1本）しかありませんでした。1本は縁に寄せてあり、切り込みが穴に
    かかる配置を作りやすくしてあります。
    """
    plate = Part.makeBox(40.0, 24.0, 6.0)
    holes = [
        Part.makeCylinder(4.0, 30.0, V(10.0, 12.0, -5.0)),
        Part.makeCylinder(2.5, 30.0, V(22.0, 7.0, -5.0)),
        Part.makeCylinder(3.0, 30.0, V(33.0, 17.0, -5.0)),
    ]
    for hole in holes:
        plate = plate.cut(hole)
    return plate


def slotted_block():
    """貫通したスロットのある塊。

    外周のワイヤが**凹**になります。これまでの検体の外周はすべて凸か円で、
    凹んだ境界を持つ面がありません。多角形の内外判定と分割は、凹のところで
    初めて効きます。
    """
    block = Part.makeBox(36.0, 20.0, 10.0)
    slot = Part.makeBox(12.0, 30.0, 6.0, V(12.0, -5.0, 4.0))
    return block.cut(slot)


def lofted_solid():
    """四角から円へのロフト。

    **B-spline 曲面**が、押し出しでも回転でもない形で出ます。u と v の
    どちらにも曲率があり、等パラメータ線が平面にも円にもなりません。
    """
    square = Part.makePolygon(
        [V(-8, -8, 0), V(8, -8, 0), V(8, 8, 0), V(-8, 8, 0), V(-8, -8, 0)]
    )
    circle = Part.Wire([Part.Circle(V(0, 0, 18), V(0, 0, 1), 5.0).toShape()])
    return Part.makeLoft([square, circle], True, False)


def pipe_bend():
    """円断面を円弧に沿って掃引した曲がり管。

    **断面は軌道に直交させます。** 直交していないと、OpenCASCADE は断面を
    軸を含む平面から外れた位置に置いた `SURFACE_OF_REVOLUTION` を書きます。
    最初の版は 0.53 mm ずれていて、それは管ではなく別の回転面でした。
    軌道は中心 (0,0,20)・半径 20 の円弧で、始点 (0,0,0) の接線は X 軸に
    ぴったり乗ります。
    """
    import math

    r = 20.0
    mid = V(r * math.sin(math.pi / 4), 0, r - r * math.cos(math.pi / 4))
    spine = Part.Arc(V(0, 0, 0), mid, V(r, 0, r)).toShape()
    profile = Part.Wire([Part.Circle(V(0, 0, 0), V(1, 0, 0), 4.0).toShape()])
    shell = Part.BRepOffsetAPI.MakePipeShell(Part.Wire([spine]))
    shell.setFrenetMode(True)
    shell.add(profile, False, False)
    shell.build()
    shell.makeSolid()
    return Part.Solid(shell.shape())


def revolved_vase():
    """スプラインの母線を Z 軸まわりに一周させた挽き物。

    **回転面の実務での典型です。** 母線は軸を含む平面（XZ 平面）に乗って
    いて、半径は場所によって変わります。解析曲面に落とせないので
    OpenCASCADE は `SURFACE_OF_REVOLUTION` を書きます。旋盤で挽く部品は
    この形です。
    """
    spline = Part.BSplineCurve()
    spline.interpolate([V(6, 0, 0), V(9, 0, 6), V(5, 0, 14), V(7, 0, 22), V(4, 0, 28)])
    wire = Part.Wire(
        [
            Part.makeLine(V(0, 0, 0), V(6, 0, 0)),
            spline.toShape(),
            Part.makeLine(V(4, 0, 28), V(0, 0, 28)),
            Part.makeLine(V(0, 0, 28), V(0, 0, 0)),
        ]
    )
    face = Part.Face(wire)
    return face.revolve(V(0, 0, 0), V(0, 0, 1), 360)


def hollow_box():
    """壁厚 2 の中空の箱。**内側の殻を持つ立体**です。

    `Solid` は内側の殻を持てますが、他カーネルから受け取ったことは一度も
    ありません。外の殻だけを見ている処理があれば、ここで出ます。
    """
    box = Part.makeBox(30.0, 20.0, 14.0)
    inner = Part.makeBox(26.0, 16.0, 10.0, V(2.0, 2.0, 2.0))
    return box.cut(inner)


def stepped_shaft():
    """段付きの軸。同軸の円柱が3段。

    **同じ軸の上で半径が変わる**ところに、円環の平面が挟まります。旋盤で
    挽く部品はほぼこの形です。
    """
    shaft = Part.makeCylinder(10.0, 12.0)
    shaft = shaft.fuse(Part.makeCylinder(6.5, 14.0, V(0, 0, 12.0)))
    shaft = shaft.fuse(Part.makeCylinder(4.0, 10.0, V(0, 0, 26.0)))
    return shaft.removeSplitter()


SUBJECTS = [
    ("occ_reference_filleted_box", filleted_box),
    ("occ_reference_chamfered_box", chamfered_box),
    ("occ_reference_plate_with_holes", plate_with_holes),
    ("occ_reference_slotted_block", slotted_block),
    ("occ_reference_lofted_solid", lofted_solid),
    ("occ_reference_pipe_bend", pipe_bend),
    ("occ_reference_revolved_vase", revolved_vase),
    ("occ_reference_hollow_box", hollow_box),
    ("occ_reference_stepped_shaft", stepped_shaft),
]

TOKENS = [
    "PLANE",
    "CYLINDRICAL_SURFACE",
    "CONICAL_SURFACE",
    "SPHERICAL_SURFACE",
    "TOROIDAL_SURFACE",
    "B_SPLINE_SURFACE_WITH_KNOTS",
    "SURFACE_OF_REVOLUTION",
    "SURFACE_OF_LINEAR_EXTRUSION",
    "CIRCLE",
    "ELLIPSE",
    "B_SPLINE_CURVE_WITH_KNOTS",
    "SEAM_CURVE",
    "ADVANCED_FACE",
    "VERTEX_LOOP",
]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    manifest = []

    print(
        "{:<34} {:>6} {:>7} {:>14} {:>13}  entities".format(
            "subject", "solids", "faces", "volume", "area"
        )
    )
    print("-" * 110)

    for name, build in SUBJECTS:
        try:
            shape = build()
        except Exception as error:  # noqa: BLE001
            print("{:<34} BUILD FAILED: {}".format(name, error))
            continue

        path = os.path.join(OUT_DIR, name + ".step")
        shape.exportStep(path)

        # **書いたファイルを読み直して測ります。** 渡した形ではなく、
        # ファイルに入った形が突き合わせの対象です。
        back = Part.Shape()
        back.read(path)

        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()
        entities = [token for token in TOKENS if token in text]

        record = {
            "name": name,
            "solids": len(back.Solids),
            "faces": len(back.Faces),
            "volume": back.Volume,
            "area": back.Area,
            "entities": entities,
        }
        manifest.append(record)

        print(
            "{:<34} {:>6} {:>7} {:>14.6f} {:>13.4f}  {}".format(
                name,
                record["solids"],
                record["faces"],
                record["volume"],
                record["area"],
                ", ".join(entities),
            )
        )

    out = os.path.join(OUT_DIR, "shapes_manifest.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
    print("\nmanifest: {}".format(out))


if __name__ == "__main__":
    main()
