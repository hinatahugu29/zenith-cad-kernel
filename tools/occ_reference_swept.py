"""
OpenCASCADE に、回転面・線形押し出し面・楕円を含む STEP を書かせる。

Zenith のインポーターが読めない曲面・曲線の検体を、こちらで作るのではなく
**相手の実装に書かせる**ための道具。こちらで組み立てた検体は、こちらの
思い込みをそのまま検査してしまう。

出力は `target/validation/` に置き、OpenCASCADE が測った体積・表面積を
`swept_manifest.json` に添える。突き合わせは
`crates/zenith_algo/examples/step_import_audit.rs` と
`tools/verify_swept_import.py` で行う。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/occ_reference_swept.py
"""

import json
import os
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.append(FREECAD_BIN)

import FreeCAD  # noqa: E402
import Part  # noqa: E402

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "target", "validation"
)


def revolved_profile():
    """L字の母線を Z 軸まわりに一周させる。OCC は SURFACE_OF_REVOLUTION を選ぶ。"""
    v = FreeCAD.Vector
    wire = Part.makePolygon([v(4, 0, 0), v(10, 0, 0), v(10, 0, 6), v(4, 0, 6), v(4, 0, 0)])
    face = Part.Face(wire)
    return face.revolve(v(0, 0, 0), v(0, 0, 1), 360)


def extruded_spline():
    """スプライン断面を直線に押し出す。OCC は SURFACE_OF_LINEAR_EXTRUSION を選ぶ。"""
    v = FreeCAD.Vector
    spline = Part.BSplineCurve()
    spline.interpolate([v(0, 0, 0), v(10, 4, 0), v(18, -2, 0), v(24, 6, 0)])
    edge = spline.toShape()
    back = Part.makePolygon([v(24, 6, 0), v(24, 20, 0), v(0, 20, 0), v(0, 0, 0)])
    wire = Part.Wire([edge] + back.Edges)
    face = Part.Face(wire)
    return face.extrude(v(0, 0, 12))


def elliptic_prism():
    """楕円断面の柱。OCC は ELLIPSE を書く。"""
    v = FreeCAD.Vector
    ellipse = Part.Ellipse(v(0, 0, 0), 12.0, 7.0)
    wire = Part.Wire([ellipse.toShape()])
    face = Part.Face(wire)
    return face.extrude(v(0, 0, 15))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    subjects = [
        ("occ_reference_revolved_ring", revolved_profile),
        ("occ_reference_extruded_spline", extruded_spline),
        ("occ_reference_elliptic_prism", elliptic_prism),
    ]

    manifest = []
    for name, build in subjects:
        try:
            shape = build()
        except Exception as error:  # noqa: BLE001
            print("{:<34} BUILD FAILED: {}".format(name, error))
            continue

        path = os.path.join(OUT_DIR, name + ".step")
        shape.exportStep(path)

        # 書いたファイルを読み直して測る。測る対象は「こちらが渡した形」では
        # なく「ファイルに入った形」でなければ、突き合わせの意味がない。
        back = Part.Shape()
        back.read(path)

        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()
        entities = sorted(
            {
                token
                for token in (
                    "SURFACE_OF_REVOLUTION",
                    "SURFACE_OF_LINEAR_EXTRUSION",
                    "ELLIPSE",
                    "B_SPLINE_CURVE_WITH_KNOTS",
                    "CIRCLE",
                    "PLANE",
                    "CYLINDRICAL_SURFACE",
                    "CONICAL_SURFACE",
                    "TOROIDAL_SURFACE",
                    "COMPOSITE_CURVE",
                    "SEAM_CURVE",
                    "OFFSET_CURVE_3D",
                )
                if token in text
            }
        )

        manifest.append(
            {
                "name": name,
                "file": name + ".step",
                "faces": len(back.Faces),
                "volume": back.Volume,
                "area": back.Area,
                "entities": entities,
            }
        )
        print(
            "{:<34} faces {:>3}  volume {:>14.6f}  area {:>14.6f}".format(
                name, len(back.Faces), back.Volume, back.Area
            )
        )
        print("    entities: {}".format(", ".join(entities)))

    with open(
        os.path.join(OUT_DIR, "swept_manifest.json"), "w", encoding="utf-8"
    ) as handle:
        json.dump(manifest, handle, indent=2)

    print()
    print("wrote {} file(s) to {}".format(len(manifest), OUT_DIR))
    return 0


if __name__ == "__main__":
    sys.exit(main())
