"""Control experiment: can OpenCASCADE round-trip a PLANE trimmed by spline arcs?

The kernel's cylinder caps are planes bounded by rational quadratic arcs, and
they come back from STEP with no bound at all. This builds the same kind of
face inside OpenCASCADE, exports it, and reads it back, which separates "our
STEP is malformed" from "this representation does not survive STEP".
"""

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

OUT_DIR = os.path.join("target", "validation")


def quarter_arc_bspline(start, middle, end):
    """A rational quadratic arc, the same construction the kernel exports."""
    curve = Part.BSplineCurve()
    curve.buildFromPolesMultsKnots(
        [FreeCAD.Vector(*start), FreeCAD.Vector(*middle), FreeCAD.Vector(*end)],
        [3, 3],
        [0.0, 1.0],
        False,
        2,
        [1.0, 2.0 ** -0.5, 1.0],
    )
    return curve.toShape()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    r = 10.0

    edges = [
        quarter_arc_bspline((r, 0, 0), (r, r, 0), (0, r, 0)),
        quarter_arc_bspline((0, r, 0), (-r, r, 0), (-r, 0, 0)),
        quarter_arc_bspline((-r, 0, 0), (-r, -r, 0), (0, -r, 0)),
        quarter_arc_bspline((0, -r, 0), (r, -r, 0), (r, 0, 0)),
    ]

    wire = Part.Wire(edges)
    print(f"wire closed: {wire.isClosed()}, length {wire.Length:.6f} (2*pi*r = {2 * 3.141592653589793 * r:.6f})")

    face = Part.Face(wire)
    print(f"face surface: {type(face.Surface).__name__}, area {face.Area:.6f} (pi*r^2 = {3.141592653589793 * r * r:.6f})")

    path = os.path.join(OUT_DIR, "occ_plane_spline_trim.step")
    face.exportStep(path)

    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        text = handle.read()
    for token in ["PLANE", "B_SPLINE_CURVE", "SURFACE_CURVE", "PCURVE", "ADVANCED_FACE"]:
        print(f"  written {token:<20} {text.count(token)}")

    reread = Part.read(path)
    print(f"read back: {reread.ShapeType}, faces={len(reread.Faces)}")
    for index, f in enumerate(reread.Faces):
        print(
            f"  face {index}: {type(f.Surface).__name__} area={f.Area} wires={len(f.Wires)}"
        )


if __name__ == "__main__":
    main()
