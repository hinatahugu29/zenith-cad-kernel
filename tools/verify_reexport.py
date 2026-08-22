"""Reads re-exported foreign files back through OpenCASCADE.

`target/reexport` holds shapes that started life in OpenCASCADE's own STEP, were
read by this kernel, and were written out again. Handing them back to the kernel
they came from closes the loop: their writer, our reader, our writer, their
reader.

For a long time this file said the fair comparison was OpenCASCADE's own
`toNurbs` conversion, on the grounds that OpenCASCADE does not measure a
rational B-spline the way it measures an analytic surface: converting its own
cylinder and measuring that gives 12674.63 against the analytic 12566.37, 0.86%
off. That reading was wrong, and it hid a defect of ours for as long as it
stood. OpenCASCADE measures our own builders' rational patches to 1e-11. What
it cannot measure is a patch wrapped all the way round, which is the form our
importer used to hand straight back out. Once the exporter cuts those
(`zenith_algo::Regularizer`), our re-exports land on the analytic value.

So the comparison is now against the closed form, and OpenCASCADE's own
conversion is printed beside it as context — it is the one that drifts.

This is a gate: it exits non-zero if any re-export misses the analytic value.
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
        f"{'ours vs true':>13}"
    )
    print("-" * 100)

    # 実力を測ってから決めた許容。2026年8月23日の実測で最悪 5.43e-10
    # （sphere_capped）なので、そのすぐ上に置く。
    #
    # ここは長らく 1e-6 でした。当時の実力は 1e-13 〜 1e-11 で、**7桁ぶん
    # 何も見張っていません**。p-curve を書くようにしたとき、球の相対誤差が
    # 1.19e-13 から 1.71e-10 へ動きましたが、1e-6 のままなら気づけません
    # （気づいたのは表の数字を前回と見比べたからです）。
    #
    # 締めるときは、実測してから決めること。先に厳しくすると、実力不足なのか
    # ゲートが過剰なのか区別できません。
    tolerance = 1e-8
    failures = []

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
        error = abs(mine - analytic) / abs(analytic) if analytic else float("nan")
        if not (error <= tolerance) or not ours.isValid() or not closed:
            failures.append((subject, error, ours.isValid(), closed))

        print(
            f"{subject:<16} {ours.ShapeType:<8} {str(ours.isValid()):<6} {str(closed):<7} "
            f"{analytic:>14.4f} {own_nurbs:>14.4f} {mine:>14.4f} {error:>13.2e}"
        )

    print("-" * 100)
    print("OCC analytic  = OpenCASCADE measuring its own analytic surfaces (the true value)")
    print("OCC own nurbs = OpenCASCADE measuring its own B-spline conversion of the same shape")
    print("OCC our file  = OpenCASCADE measuring our re-export, which is also B-spline")
    print("ours vs true  = the last column against the first. This is what is graded.")
    print()
    if failures:
        for subject, error, valid, closed in failures:
            print(f"FAILED {subject}: error {error:.3e}, valid={valid}, closed={closed}")
        print(f"{len(failures)} of {len(SOURCES)} re-exports miss the analytic value")
        return 1
    print(f"{len(SOURCES)} of {len(SOURCES)} re-exports land on the analytic value "
          f"within {tolerance:.0e}, as valid closed solids")
    return 0


if __name__ == "__main__":
    sys.exit(main())
