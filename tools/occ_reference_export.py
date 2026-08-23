"""Writes a cylinder from OpenCASCADE itself and reports which STEP entities it
uses, so the kernel's exporter can be compared against a known-good writer."""

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

TOKENS = [
    "SURFACE_CURVE",
    "SEAM_CURVE",
    "PCURVE",
    "DEFINITIONAL_REPRESENTATION",
    "B_SPLINE_CURVE",
    "CIRCLE",
    "LINE",
    "CYLINDRICAL_SURFACE",
    "CONICAL_SURFACE",
    "SPHERICAL_SURFACE",
    "TOROIDAL_SURFACE",
    "PLANE",
    "ADVANCED_FACE",
    "EDGE_CURVE",
    "VERTEX_LOOP",
]


def report(name, shape):
    out_dir = os.path.join("target", "validation")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"occ_reference_{name}.step")
    shape.exportStep(path)

    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        text = handle.read()

    print(f"=== OCC-written {name} ({os.path.getsize(path) / 1024:.1f} KB)")
    try:
        print(f"    volume {shape.Volume:.4f}, area {shape.Area:.4f}")
    except Exception:
        pass
    for token in TOKENS:
        count = text.count(token)
        if count:
            print(f"    {token:<30} {count}")
    print()


def main():
    report("cylinder", Part.makeCylinder(10.0, 40.0))

    # The analytic surfaces the importer has to size from the face boundary.
    # Volumes are printed so the reader can be checked against the writer.
    report("cone", Part.makeCone(10.0, 4.0, 20.0))
    report("cone_full", Part.makeCone(10.0, 0.0, 20.0))
    report("sphere", Part.makeSphere(10.0))
    report("torus", Part.makeTorus(12.0, 4.0))

    # Bounded analytic faces: a sphere and a torus cut by real edges rather
    # than by a seam. This is the shape an ordinary CAD file gives the reader,
    # and unlike the full versions it carries no degenerate loop.
    box = Part.makeBox(20.0, 20.0, 20.0, FreeCAD.Vector(-10.0, -10.0, 0.0))
    report("sphere_capped", Part.makeSphere(10.0).common(box))
    report("torus_segment", Part.makeTorus(12.0, 4.0, FreeCAD.Vector(0, 0, 0),
                                           FreeCAD.Vector(0, 0, 1), 0, 360, 90))

    # Force the B-spline representation the kernel uses, so the comparison is
    # about the exporter rather than about analytic vs spline surfaces.
    spline_cylinder = Part.makeCylinder(10.0, 40.0)
    try:
        converted = spline_cylinder.toNurbs()
        report("cylinder_nurbs", converted)
    except Exception as exc:
        print(f"toNurbs failed: {exc}")


if __name__ == "__main__":
    main()
