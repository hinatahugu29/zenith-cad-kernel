"""整理した立体が、OpenCASCADE で読んでも同じ体積の valid closed solid か。

面を併合し、平面を平面として持ち直すと、STEP に出る面・稜・曲面の種類が
変わります。減ったこと自体は `face_merge_probe` が測っていますが、
**他カーネルが同じ形として読めるか**は外から測らないと分かりません。

    "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/verify_simplified.py
"""

import json
import os
import sys
from pathlib import Path

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

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "target" / "simplified" / "manifest.json"

if not MANIFEST.exists():
    print(f"manifest not found: {MANIFEST}")
    print("run: cargo run --release -p zenith_algo --example export_simplified")
    sys.exit(2)

entries = json.loads(MANIFEST.read_text(encoding="utf-8"))
print(f"{'subject':<34}{'OCC type':<11}{'valid':<7}{'closed':<8}{'faces':>6}{'edges':>7}{'volume':>16}{'vs ours':>12}")
print("-" * 102)

failures = 0
for entry in entries:
    path = ROOT / entry["path"]
    shape = Part.Shape()
    shape.read(str(path))
    solids = shape.Solids
    kind = type(shape).__name__ if not solids else "Solid"
    if len(solids) != 1:
        print(f"{entry['name']:<34}{kind:<11}{'-':<7}{'-':<8} read back as {len(solids)} solid(s)")
        failures += 1
        continue

    solid = solids[0]
    volume = solid.Volume
    ours = entry["volume"]
    drift = abs(volume - ours) / max(abs(ours), 1e-12)
    ok = solid.isValid() and solid.Shells and solid.Shells[0].isClosed() and drift < 1e-6
    print(
        f"{entry['name']:<34}{kind:<11}{str(solid.isValid()):<7}"
        f"{str(bool(solid.Shells) and solid.Shells[0].isClosed()):<8}"
        f"{len(solid.Faces):>6}{len(solid.Edges):>7}{volume:>16.4f}{drift:>12.2e}"
    )
    if not ok:
        failures += 1

print("-" * 102)
if failures:
    print(f"{failures} of {len(entries)} simplified solids did not come back intact")
    sys.exit(1)
print(f"{len(entries)} of {len(entries)} simplified solids read back as valid closed solids")
