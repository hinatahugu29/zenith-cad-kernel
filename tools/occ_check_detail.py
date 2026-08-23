"""Asks OpenCASCADE why it will not accept a shell as a solid.

The drilled block's shell has the right faces, the right holes and the right
volume, yet BRepCheck marks the forced solid invalid. This runs the analyzer
and prints what it objects to, per sub-shape.
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


def analyse(path):
    print(f"=== {os.path.basename(path)}")
    shape = Part.read(path)

    if not shape.Shells:
        print("    no shells")
        return

    shell = shape.Shells[0]
    print(f"    shell closed={shell.isClosed()} valid={shell.isValid()}")

    solid = Part.Solid(shell)
    print(f"    forced solid: volume={solid.Volume:.6f} valid={solid.isValid()}")

    # BRepCheck's own report, which names the offending sub-shape.
    try:
        result = solid.check(True)
        print(f"    check(True) returned: {result!r}")
    except Exception as exc:
        print(f"    check raised: {exc}")

    for index, face in enumerate(solid.Faces):
        if not face.isValid():
            print(f"    face {index} invalid ({type(face.Surface).__name__})")
        for wire_index, wire in enumerate(face.Wires):
            if not wire.isValid():
                print(f"    face {index} wire {wire_index} invalid")
    for index, edge in enumerate(solid.Edges):
        if not edge.isValid():
            print(f"    edge {index} invalid, length {edge.Length}")

    # Does a fix pass rescue it, and what does it change?
    fixed = solid.copy()
    try:
        fixed.fix(1e-7, 1e-7, 1e-7)
        print(f"    after fix: valid={fixed.isValid()} volume={fixed.Volume:.6f}")
    except Exception as exc:
        print(f"    fix raised: {exc}")

    print()


def main():
    directory = os.path.join("target", "validation")
    for name in ("boolean_drilled_block.step", "hollow_extrusion.step"):
        path = os.path.join(directory, name)
        if os.path.isfile(path):
            analyse(path)


if __name__ == "__main__":
    main()
