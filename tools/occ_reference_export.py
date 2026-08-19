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
    for token in TOKENS:
        count = text.count(token)
        if count:
            print(f"    {token:<30} {count}")
    print()


def main():
    report("cylinder", Part.makeCylinder(10.0, 40.0))

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
