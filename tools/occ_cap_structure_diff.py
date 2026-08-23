"""Compares the kernel's planar cap against the one OpenCASCADE writes itself.

Both files describe a plane trimmed by spline arcs. One reads back as a solid
and one loses its bound entirely, so the difference between them is the defect.
"""

import os
import re
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(FREECAD_BIN)
    except Exception:
        pass

import FreeCAD  # noqa: E402,F401
import Part  # noqa: E402

ENTITY = re.compile(r"^#(\d+)\s*=\s*(.*);\s*$", re.MULTILINE)


def load_entities(path):
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        text = handle.read()
    return {int(m.group(1)): m.group(2) for m in ENTITY.finditer(text)}


def refs(body):
    return [int(x) for x in re.findall(r"#(\d+)", body)]


def describe_planar_faces(path, label):
    entities = load_entities(path)
    print(f"=== {label}: {os.path.basename(path)}")

    shape = Part.read(path)
    print(f"    reads back as {shape.ShapeType}, {len(shape.Faces)} face(s)")
    for index, face in enumerate(shape.Faces):
        kind = type(face.Surface).__name__
        print(
            f"    face {index}: {kind:<16} area={face.Area:<24} wires={len(face.Wires)}"
        )

    for eid, body in sorted(entities.items()):
        if not body.startswith("ADVANCED_FACE"):
            continue
        surface_id = refs(body)[-1]
        surface_body = entities.get(surface_id, "")
        if not surface_body.startswith("PLANE"):
            continue

        print(f"    --- ADVANCED_FACE #{eid}")
        print(f"        {body}")
        print(f"        surface #{surface_id} = {surface_body}")
        placement_id = refs(surface_body)[0]
        print(f"        {placement_id}: {entities.get(placement_id)}")
        for bound_id in refs(body)[:-1]:
            bound_body = entities.get(bound_id, "")
            print(f"        bound #{bound_id} = {bound_body}")
            loop_id = refs(bound_body)[0]
            loop_body = entities.get(loop_id, "")
            print(f"          loop #{loop_id} = {loop_body}")
            for oriented_id in refs(loop_body):
                oriented_body = entities.get(oriented_id, "")
                edge_ids = refs(oriented_body)
                edge_body = entities.get(edge_ids[-1], "") if edge_ids else ""
                curve_ids = refs(edge_body)
                curve_body = entities.get(curve_ids[-1], "") if curve_ids else ""
                curve_kind = curve_body.split("(")[0].strip() or curve_body[:40]
                print(f"            {oriented_body}  -> {edge_body[:60]}")
                print(f"                curve: {curve_kind}")
        break

    print()


def main():
    directory = os.path.join("target", "validation")
    describe_planar_faces(
        os.path.join(directory, "occ_reference_cylinder_nurbs.step"), "OCC writer"
    )
    describe_planar_faces(
        os.path.join(directory, "cylinder_r10_h40.step"), "Zenith writer"
    )


if __name__ == "__main__":
    main()
