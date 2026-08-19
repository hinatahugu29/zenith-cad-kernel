"""Reads every showcase STEP back through OpenCASCADE.

The point of the showcase files is that someone else's kernel can open them, so
they are checked with someone else's kernel before being handed over.
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
    directory = os.path.join("target", "showcase")
    if not os.path.isdir(directory):
        print(f"missing: {directory}")
        print("run: cargo run --release -p zenith_algo --example export_showcase")
        return 2

    names = sorted(n for n in os.listdir(directory) if n.endswith(".step"))
    print(f"{'file':<34} {'type':<10} {'valid':<6} {'closed':<7} {'faces':>6} {'edges':>6} {'volume':>14}")
    print("-" * 92)

    failures = 0
    for name in names:
        path = os.path.join(directory, name)
        try:
            shape = Part.read(path)
        except Exception as exc:
            print(f"{name:<34} could not be read: {exc}")
            failures += 1
            continue

        try:
            valid = shape.isValid()
        except Exception:
            valid = None
        try:
            closed = shape.isClosed()
        except Exception:
            closed = None

        volume = 0.0
        try:
            volume = float(shape.Volume)
        except Exception:
            pass

        ok = shape.ShapeType == "Solid" and valid and closed
        if not ok:
            failures += 1

        print(
            f"{name:<34} {shape.ShapeType:<10} {str(valid):<6} {str(closed):<7}"
            f" {len(shape.Faces):>6} {len(shape.Edges):>6} {volume:>14.4f}"
        )

    print("-" * 92)
    print(f"{len(names) - failures} of {len(names)} read back as valid closed solids")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
