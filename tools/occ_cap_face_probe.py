"""Looks at the broken planar cap face directly.

The cylinder's cap comes back from OpenCASCADE with an astronomically large
area, which is what drags the whole solid down to a Compound. This inspects
the face OCC actually built and then tries to rebuild the same face from its
own edges, to separate "the STEP is wrong" from "the reader needed a hint".
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


def main():
    path = os.path.join("target", "validation", "cylinder_r10_h40.step")
    shape = Part.read(path)

    for index, face in enumerate(shape.Faces):
        surface = face.Surface
        if type(surface).__name__ != "Plane":
            continue

        print(f"--- planar face {index}")
        print(f"    area            : {face.Area}")
        print(f"    parameter range : {face.ParameterRange}")
        print(f"    wires           : {len(face.Wires)}")
        print(f"    edges           : {len(face.Edges)}")
        print(f"    valid           : {face.isValid()}")
        for wire_index, wire in enumerate(face.Wires):
            print(
                f"    wire {wire_index}: closed={wire.isClosed()}"
                f" edges={len(wire.Edges)} length={wire.Length:.6f}"
            )
            for edge in wire.Edges:
                print(
                    f"        {type(edge.Curve).__name__} length={edge.Length:.6f}"
                    f" degenerated={edge.Degenerated}"
                )

        # Can OCC build a sane face from those very edges?
        try:
            rebuilt = Part.Face(Part.Wire(face.Edges))
            print(f"    rebuilt from edges: area={rebuilt.Area:.6f}")
        except Exception as exc:
            print(f"    rebuilt from edges: failed ({exc})")

        try:
            fixer = face.copy()
            fixer.fix(1e-7, 1e-7, 1e-7)
            print(f"    after Shape.fix   : area={fixer.Area}")
        except Exception as exc:
            print(f"    after Shape.fix   : failed ({exc})")

        print()

    # What does OCC produce for the same construction natively?
    print("--- OCC native reference")
    native = Part.makeCylinder(10.0, 40.0)
    print(f"    native cylinder volume={native.Volume:.6f} type={native.ShapeType}")
    caps = [f for f in native.Faces if type(f.Surface).__name__ == "Plane"]
    for face in caps:
        print(f"    native cap area={face.Area:.6f}")


if __name__ == "__main__":
    main()
