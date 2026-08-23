"""Inspects a planar face whose hole loop is made of spline arcs.

A hollow extrusion, whose hole loop is straight lines, reads back from STEP as
a solid. The boolean-drilled block, whose hole loop is rational arcs, reads
back as a shell that OpenCASCADE refuses to close. This looks at the faces
themselves to find what differs.
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


def describe(path):
    shape = Part.read(path)
    print(f"=== {os.path.basename(path)}: read as {shape.ShapeType}")
    print(f"    faces={len(shape.Faces)} shells={len(shape.Shells)} solids={len(shape.Solids)}")

    for index, face in enumerate(shape.Faces):
        if len(face.Wires) < 2:
            continue
        print(f"    --- face {index}: {type(face.Surface).__name__}, {len(face.Wires)} wires")
        print(f"        area={face.Area:.6f} valid={face.isValid()} orientation={face.Orientation}")
        for wire_index, wire in enumerate(face.Wires):
            kinds = sorted({type(e.Curve).__name__ for e in wire.Edges})
            print(
                f"        wire {wire_index}: closed={wire.isClosed()}"
                f" edges={len(wire.Edges)} length={wire.Length:.6f}"
                f" orientation={wire.Orientation} curves={kinds}"
            )
        try:
            outer = face.OuterWire
            print(f"        outer wire length={outer.Length:.6f}")
        except Exception as exc:
            print(f"        outer wire lookup failed: {exc}")

    try:
        solid = Part.Solid(shape.Shells[0]) if shape.Shells else None
        if solid is not None:
            print(f"    forcing a solid: volume={solid.Volume:.6f} valid={solid.isValid()}")
    except Exception as exc:
        print(f"    forcing a solid failed: {exc}")
    print()


def main():
    directory = os.path.join("target", "validation")
    for name in ("boolean_drilled_block.step", "hollow_extrusion.step"):
        path = os.path.join(directory, name)
        if os.path.isfile(path):
            describe(path)
        else:
            print(f"missing: {path}")


if __name__ == "__main__":
    main()
