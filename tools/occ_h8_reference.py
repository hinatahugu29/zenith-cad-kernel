"""H8（`linkrods.step` を切る）の**答え**を、OpenCASCADE に出させる。

# なぜ要るのか

**H8 は 100 本以上の記録がありますが、目標の数がありませんでした。**
「あぶれ 47」「割れない面 9 枚」——**どれも、こちらの途中経過**です。
**切れたときに、それが正しいかを言える数**が要ります。

# 何を出すか

**`foreign_boolean_probe` と同じ規則の切り手**（境界箱の 3% 内、高さ 0.47。
`read_and_cut_probe`・`h8_trim_probe` と同じ）で、**3 演算の体積と面の数**。

**`--box` を渡してください**（4-510）。渡さないと OCC は**厳密な**境界箱から
切り手を置き、**こちらはメッシュの箱から置く**ので、別の配置になります。
箱は `read_and_cut_probe` と同じ作り方で、下のコマンドが出します。

    cargo run --release -q -p zenith_algo --example h8_trim_probe   # 場面の確認
    & "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/occ_h8_reference.py \
        3.1250000000 2.5000000000 0.0000000000 8.1452847075 4.0000000000 2.0000000000

# 実測（2026/09/22。4-511）

    V(A)    3.847002      （こちらは 3.846879。3.2e-5 で一致）
    A - B   1.551124      面 37 枚
    A ^ B   2.295916      面 37 枚
    A u B   7.805669      面 49 枚

**3 演算とも妥当な立体**です。**OCC 自身の恒等式は 3.8e-5 〜 6.9e-5 で
閉じます**——**有理・スプライン面の上では OCC の求積も緩みます**（4-45）。
**目標にするときは、その桁を見込んでください。**

**ファイルは再配布していません。** `reference/OCCT/data/step/linkrods.step`
に置いてください（README に置き方があります）。
"""

import os
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(FREECAD_BIN)
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import FreeCAD  # noqa: E402
import Part  # noqa: E402

SAMPLE = os.path.join("reference", "OCCT", "data", "step", "linkrods.step")


def main():
    if not os.path.exists(SAMPLE):
        raise SystemExit(
            f"{SAMPLE} がありません。OCCT の data/step から置いてください（再配布していません）。"
        )
    shape = Part.Shape()
    shape.read(SAMPLE)
    solids = shape.Solids
    subject = max(solids, key=lambda s: len(s.Faces)) if solids else shape
    box = [float(x) for x in sys.argv[1:7]] if len(sys.argv) >= 7 else None
    if box is None:
        bb = subject.BoundBox
        box = [bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax]
        print("**`--box` が渡されていません。** OCC の厳密な箱で置きます")
        print("——**こちらのメッシュの箱とは別の配置**です（4-510）。")

    size = (box[3] - box[0], box[4] - box[1], box[5] - box[2])
    inset, height = 0.03, 0.47
    cutter = Part.makeBox(
        size[0] * (1.0 - inset * 2.0), size[1] * (1.0 - inset * 2.0), size[2] * height
    )
    cutter.translate(
        FreeCAD.Vector(
            box[0] + size[0] * inset,
            box[1] + size[1] * inset,
            box[2] + size[2] * (height * 0.5),
        )
    )

    print(f"subject {SAMPLE}  面 {len(subject.Faces)} 枚")
    print(f"  V(A)            {subject.Volume:.6f}")
    print(f"  V(B)            {cutter.Volume:.6f}")
    for label, operation in (
        ("A - B", subject.cut),
        ("A ^ B", subject.common),
        ("A u B", subject.fuse),
    ):
        try:
            result = operation(cutter)
        except Exception as exc:  # noqa: BLE001
            print(f"  {label}          OCC も断りました — {exc}")
            continue
        print(
            f"  {label}           {result.Volume:.6f}   面 {len(result.Faces)} 枚、"
            f"立体 {len(result.Solids)} 個、妥当 {result.isValid()}"
        )
    print()
    print("**こちらが切れたときに、この数と突き合わせてください。**")
    print("**OCC 自身の恒等式も 1e-5 の桁で閉じます**（有理・スプライン面）。")


if __name__ == "__main__":
    main()
