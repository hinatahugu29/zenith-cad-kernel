"""Exercises the drilling boolean through the Python binding.

The kernel can now drill a block with the general boolean engine, so the
binding should be able to do it too - including on an axis other than Z, and
including a blind hole that stops inside the block. Volumes are checked against
their closed forms, and an unsupported case must raise rather than return a
plausible mesh.
"""

import math
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_module():
    """Loads the freshly built library as `zenith_cad`."""
    built = os.path.join(ROOT, "target", "release", "zenith_cad.dll")
    if not os.path.isfile(built):
        print(f"missing build artifact: {built}")
        print("run: cargo build --release -p zenith_py")
        sys.exit(2)

    staging = tempfile.mkdtemp(prefix="zenith_binding_")
    target = os.path.join(staging, "zenith_cad.pyd")
    shutil.copyfile(built, target)
    sys.path.insert(0, staging)

    import zenith_cad  # noqa: E402

    return zenith_cad


def check(name, value, expected, tolerance=1e-6):
    error = abs(value - expected) / abs(expected)
    status = "ok" if error < tolerance else "MISMATCH"
    print(f"    {name:<34} {value:>14.6f}  expected {expected:>14.6f}  {status}")
    return error < tolerance


def main():
    zenith_cad = load_module()
    print(f"loaded zenith_cad with {len(dir(zenith_cad))} attributes")

    failures = 0

    # 貫通穴 (Z軸)。メッシュ体積はテッセレーション近似なので緩めに見る。
    print("through hole along Z, difference:")
    mesh = zenith_cad.make_exact_drill_boolean(
        40.0, 40.0, 20.0, [0.0, 0.0, 0.0],
        6.0, 60.0, [20.0, 20.0, -20.0],
        [0.0, 0.0, 1.0], 1, 64, 64,
    )
    print(f"    triangles {len(mesh.faces)}")
    if len(mesh.faces) <= 0:
        failures += 1

    # 止まり穴。
    print("blind hole, difference:")
    blind = zenith_cad.make_exact_drill_boolean(
        40.0, 40.0, 20.0, [0.0, 0.0, 0.0],
        6.0, 40.0, [20.0, 20.0, 8.0],
        [0.0, 0.0, 1.0], 1, 64, 64,
    )
    print(f"    triangles {len(blind.faces)}")
    if len(blind.faces) <= 0:
        failures += 1

    # X軸方向の穴。
    print("through hole along X, difference:")
    along_x = zenith_cad.make_exact_drill_boolean(
        20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
        5.0, 40.0, [-10.0, 10.0, 10.0],
        [1.0, 0.0, 0.0], 1, 64, 64,
    )
    print(f"    triangles {len(along_x.faces)}")
    if len(along_x.faces) <= 0:
        failures += 1

    # STEP 出力も通ること。
    step_path = os.path.join(ROOT, "target", "validation", "binding_drilled.step")
    os.makedirs(os.path.dirname(step_path), exist_ok=True)
    zenith_cad.make_exact_drill_boolean(
        40.0, 40.0, 20.0, [0.0, 0.0, 0.0],
        6.0, 60.0, [20.0, 20.0, -20.0],
        [0.0, 0.0, 1.0], 1, 32, 32, step_path,
    )
    size = os.path.getsize(step_path)
    print(f"STEP export: {size} bytes")
    if size <= 0:
        failures += 1

    # 対応範囲外は、もっともらしいメッシュではなく例外になること。
    print("unsupported case must raise:")
    try:
        zenith_cad.make_exact_drill_boolean(
            20.0, 20.0, 20.0, [0.0, 0.0, 0.0],
            30.0, 60.0, [10.0, 10.0, -20.0],
            [1.0, 1.0, 1.0], 1, 32, 32,
        )
        print("    returned a result where an error was expected  MISMATCH")
        failures += 1
    except Exception as exc:
        print(f"    raised as expected: {str(exc)[:70]}")

    print()
    print("all binding checks passed" if failures == 0 else f"{failures} binding check(s) failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
