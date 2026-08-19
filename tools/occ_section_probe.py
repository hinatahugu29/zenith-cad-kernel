"""Isolates how OpenCASCADE should be asked for a cross-section area.

The cross-validation harness and the kernel disagreed on section areas, so
this probe checks the harness against shapes OpenCASCADE built itself, where
the analytic answer is not in question.
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


def section_area(shape, origin, normal, normalize):
    base = FreeCAD.Vector(*origin)
    direction = FreeCAD.Vector(*normal)
    if normalize:
        direction = direction.normalize()
    distance = direction.dot(base)
    wires = shape.slice(direction, distance)
    if not wires:
        return None, 0

    areas = []
    for wire in wires:
        try:
            areas.append(Part.Face(wire).Area)
        except Exception:
            areas.append(None)

    return areas, len(wires)


def main():
    # OCC builds the box itself, so the hexagon area is beyond dispute.
    box = Part.makeBox(20, 30, 40)
    print("OCC-built box 20x30x40, plane x+y+z=45 (analytic 575*sqrt(3) = 995.929)")
    for normalize in (False, True):
        areas, count = section_area(box, (10, 15, 20), (1, 1, 1), normalize)
        print(f"  normalize={normalize!s:<5} wires={count} areas={areas}")

    print()
    print("OCC-built box 20x30x40, plane z=20 (analytic 600)")
    for normalize in (False, True):
        areas, count = section_area(box, (0, 0, 20), (0, 0, 1), normalize)
        print(f"  normalize={normalize!s:<5} wires={count} areas={areas}")

    print()
    drilled = Part.makeBox(30, 30, 15).cut(
        Part.makeCylinder(5, 15, FreeCAD.Vector(15, 15, 0))
    )
    print("OCC-built drilled box 30x30x15 r5, plane z=7.5 (analytic 900-25pi = 821.460)")
    for normalize in (False, True):
        areas, count = section_area(drilled, (0, 0, 7.5), (0, 0, 1), normalize)
        print(f"  normalize={normalize!s:<5} wires={count} areas={areas}")


if __name__ == "__main__":
    main()
