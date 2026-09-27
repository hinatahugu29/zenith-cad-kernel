"""**OCC に、読んだ立体の面ごとの面積を出させます**（4-560）。

**なぜ要るか**: こちらが読んだ `linkrods.step` の体積は、OCC より
**1.234e-4 小さい**（4-559）。**位相は合っています**（面 37 枚、稜 114 本
すべて 2 面が使用）。**なら、どれかの面の形が違う**はずです。
**面ごとに並べれば、どの面かが分かります。**

使い方: `py tools/occ_face_area_reference.py [step ファイル]`
"""
import os
import sys

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

freecad_bin = r"C:\Program Files\FreeCAD 1.1\bin"
if freecad_bin not in sys.path:
    sys.path.insert(0, freecad_bin)
if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin)
    except Exception:
        pass

import FreeCAD  # noqa: E402
import Part  # noqa: E402

path = sys.argv[1] if len(sys.argv) > 1 else "reference/OCCT/data/step/linkrods.step"
shape = Part.Shape()
shape.read(path)

solids = shape.Solids
if not solids:
    print("立体がありません")
    raise SystemExit(1)
solid = max(solids, key=lambda s: len(s.Faces))

print(f"{path}")
print(f"立体 {len(solids)} 個、いちばん面の多いもの: 面 {len(solid.Faces)} 枚")
print(f"体積 {solid.Volume:.9f}")
print(f"表面積 {solid.Area:.9f}")
print()
print("面ごとの面積（OCC）:")
for index, face in enumerate(solid.Faces):
    kind = type(face.Surface).__name__
    print(f"  面{index:<3} {kind:<24} 面積 {face.Area:.9f}")
