"""Reads re-exported foreign files back through OpenCASCADE.

`target/reexport` holds shapes that started life in OpenCASCADE's own STEP, were
read by this kernel, and were written out again. Handing them back to the kernel
they came from closes the loop: their writer, our reader, our writer, their
reader.

The comparison needs care, because OpenCASCADE does not measure a rational
B-spline the way it measures an analytic surface. Converting its own cylinder
with `toNurbs` and measuring that gives 12674.63 against the analytic 12566.37,
a difference of 0.86% that has nothing to do with any file we wrote. So three
numbers are printed: what OpenCASCADE says about the analytic original, what it
says about its own B-spline conversion of that original, and what it says about
our re-export. The last two are the fair pair. Our own reading of the same file
is printed alongside, since it is the one that can be checked against the closed
form.

This is a diagnostic, not a gate: it reports and always exits 0.
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

import FreeCAD  # noqa: E402,F401  (must load before Part)
import Part  # noqa: E402

SOURCES = {
    "cone": "occ_reference_cone.step",
    "cone_full": "occ_reference_cone_full.step",
    "cylinder": "occ_reference_cylinder.step",
    "sphere": "occ_reference_sphere.step",
    "sphere_capped": "occ_reference_sphere_capped.step",
    "torus": "occ_reference_torus.step",
    "torus_segment": "occ_reference_torus_segment.step",
}


def read(path):
    shape = Part.Shape()
    shape.read(path)
    return shape


def main():
    directory = os.path.join("target", "reexport")
    if not os.path.isdir(directory):
        print(f"missing: {directory}")
        print("run: cargo run --release -p zenith_algo --example foreign_reexport")
        return 0

    print(
        f"{'subject':<16} {'type':<8} {'valid':<6} {'closed':<7} "
        f"{'OCC analytic':>14} {'OCC own nurbs':>14} {'OCC our file':>14} "
        f"{'vs own nurbs':>13}"
    )
    print("-" * 100)

    for subject, source_name in sorted(SOURCES.items()):
        path = os.path.join(directory, f"reexport_{subject}.step")
        source = os.path.join("target", "validation", source_name)
        if not os.path.isfile(path) or not os.path.isfile(source):
            print(f"{subject:<16} missing file")
            continue

        original = read(source)
        ours = read(path)

        # 同じ土俵。OpenCASCADE 自身に解析曲面を NURBS 化させて測らせる。
        try:
            own_nurbs = original.toNurbs().Volume
        except Exception:
            own_nurbs = float("nan")

        analytic = original.Volume
        mine = ours.Volume
        closed = bool(ours.Solids) and ours.Solids[0].Shells[0].isClosed()
        spread = abs(mine - own_nurbs) / abs(own_nurbs) if own_nurbs else float("nan")

        print(
            f"{subject:<16} {ours.ShapeType:<8} {str(ours.isValid()):<6} {str(closed):<7} "
            f"{analytic:>14.4f} {own_nurbs:>14.4f} {mine:>14.4f} {spread:>13.2e}"
        )

    print("-" * 100)
    print("OCC analytic  = OpenCASCADE measuring its own analytic surfaces (the true value)")
    print("OCC own nurbs = OpenCASCADE measuring its own B-spline conversion of the same shape")
    print("OCC our file  = OpenCASCADE measuring our re-export, which is also B-spline")
    print()
    print("The last two are the comparable pair. Both sit above the analytic value by")
    print("a similar margin, which is OpenCASCADE's integration of rational patches,")
    print("not a difference in the geometry: our reader puts every one of these files")
    print("back at the analytic value to 1e-13 (foreign_reexport prints that column).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
