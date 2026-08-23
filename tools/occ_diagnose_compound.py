"""Explains why OpenCASCADE downgrades some exported solids to a Compound.

A STEP file can carry a perfectly well-formed MANIFOLD_SOLID_BREP and still
come back as a Compound if OCC's reader cannot sew the faces into a closed
shell. This reports the free (unshared) edges that stop the sewing, and checks
whether a sew with a looser tolerance recovers the solid - which tells us
whether the problem is connectivity or precision.
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


def edge_key(edge, digits=4):
    a = edge.Vertexes[0].Point
    b = edge.Vertexes[-1].Point
    ka = (round(a.x, digits), round(a.y, digits), round(a.z, digits))
    kb = (round(b.x, digits), round(b.y, digits), round(b.z, digits))
    return tuple(sorted([ka, kb]))


def diagnose(path):
    name = os.path.basename(path)
    shape = Part.read(path)

    print(f"=== {name}")
    print(f"    read as       : {shape.ShapeType}")
    print(
        f"    faces={len(shape.Faces)} shells={len(shape.Shells)}"
        f" solids={len(shape.Solids)} edges={len(shape.Edges)}"
    )

    counts = {}
    for face in shape.Faces:
        for edge in face.Edges:
            key = edge_key(edge)
            counts[key] = counts.get(key, 0) + 1

    free_edges = [key for key, count in counts.items() if count == 1]
    over_shared = [key for key, count in counts.items() if count > 2]
    print(
        f"    edge uses     : {len(counts)} distinct,"
        f" {len(free_edges)} used once, {len(over_shared)} used more than twice"
    )

    for key in free_edges[:4]:
        print(f"      free edge {key}")

    tolerances = [t.Tolerance for t in shape.Faces]
    if tolerances:
        print(
            f"    face tolerance: min={min(tolerances):.3e} max={max(tolerances):.3e}"
        )

    for tolerance in (1e-7, 1e-5, 1e-3, 1e-2):
        try:
            sewn = shape.copy()
            sewn.sewShape(tolerance)
            solid_ok = False
            try:
                shell = sewn.Shells[0] if sewn.Shells else None
                if shell is not None:
                    solid_ok = shell.isClosed()
            except Exception:
                solid_ok = False
            print(
                f"    sew @ {tolerance:<7.0e}: type={sewn.ShapeType}"
                f" shells={len(sewn.Shells)} closed={solid_ok}"
            )
            if solid_ok:
                break
        except Exception as exc:
            print(f"    sew @ {tolerance:<7.0e}: failed ({exc})")

    print()


def main():
    directory = os.path.join("target", "validation")
    targets = sys.argv[1:] or [
        "cylinder_r10_h40.step",
        "cone_r10_r4_h20.step",
        "drilled_box_30x30x15_r5.step",
        "swept_pipe.step",
        "box_20x30x40.step",
        "sphere_r10.step",
    ]

    for target in targets:
        path = target if os.path.isfile(target) else os.path.join(directory, target)
        if not os.path.isfile(path):
            print(f"missing: {path}")
            continue
        diagnose(path)


if __name__ == "__main__":
    main()
