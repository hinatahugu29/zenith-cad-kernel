"""**A（読んだ立体そのもの）の、面ごとの面積**を OCC に出させる（4-572）。

# なぜ要るのか

4-560 の宿題でした——**「面ごとの体積への寄与（発散定理の各面の項）を、
OCC とこちらで並べる」**。**やってみて、半分だけ当てになると分かりました。**

# どうやるか、何が分かったか

**OCC には、面ごとの寄与を直接出す API がありません。** そこで、**面を
細かく三角形分割し（`Part.Face.tessellate`）、こちらの `compute_from_mesh`
と同じ式**（原点を頂点とする四面体の符号付き体積の和）で積みます。

**面積は当てになります**——**`ZENITH_OCC_FACES=A`（`occ_h8_reference.py`）
の面積と、こちらの列は一致します。** **体積（寄与）は当てになりません**
——**符号を決める`Face.Orientation` の意味を、こちらが正しく理解できて
いません**（37 枚中 2 枚だけ符号が合わない。関数の docstring に詳細）。

**罠が 1 つありました**（4-572）。**`face.tessellate()` の三角形の頂点
番号は 0 始まり**です——**1 始まりだと思って `-1` していたら、頂点 0 を
含む三角形が Python の負の添字で末尾の点にすり替わり**、**面積が π 倍
近くに膨らむ**という壊れた結果になりました。**添字の始まりは、使う前に
確かめてください。**

    & "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/occ_face_volume_contribution.py

**ファイルは再配布していません。** `reference/OCCT/data/step/linkrods.step`
に置いてください。
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


def face_volume_contribution(face, deflection, solid):
    """面を細かく三角形分割し、原点基準の符号付き四面体体積の和を返す。

    **向きは `Face.Orientation` で決めます**——**ただし、これは確かめた
    結果ではなく、いちばんましだった選択です**（4-572）。**実測**: 37 枚中
    35 枚は、これで我々の実装（`compute_face_integral`）と符号が揃います。
    **面10・面11（ともに平面）だけ符号が合いません。**

    **`solid.isInside` で外側かどうかを確かめる手も試しましたが、もっと
    悪化しました**（合う枚数が減りました）——**この手は外しました。**
    **`Face.Orientation` の意味を、こちらが正しく理解できていません。**
    **これは未解決のまま残します。** 体積の合計（発散定理の総和）は、
    **したがって、いまのこの道具では当てになりません**——**面積の列だけ
    使ってください。**
    """
    points, triangles = face.tessellate(deflection)
    if not points or not triangles:
        return 0.0, 0.0
    volume = 0.0
    area = 0.0
    for tri in triangles:
        # FreeCAD の三角形の頂点番号は 0 始まり（実測で確認。4-572）。
        p0 = points[tri[0]]
        p1 = points[tri[1]]
        p2 = points[tri[2]]
        det = (
            p0.x * (p1.y * p2.z - p1.z * p2.y)
            - p0.y * (p1.x * p2.z - p1.z * p2.x)
            + p0.z * (p1.x * p2.y - p1.y * p2.x)
        )
        volume += det / 6.0
        cross = (p1 - p0).cross(p2 - p0)
        area += 0.5 * cross.Length
    if face.Orientation == "Reversed":
        volume = -volume
    return area, volume


def main():
    if not os.path.exists(SAMPLE):
        raise SystemExit(
            f"{SAMPLE} がありません。OCCT の data/step から置いてください（再配布していません）。"
        )
    shape = Part.Shape()
    shape.read(SAMPLE)
    solids = shape.Solids
    subject = max(solids, key=lambda s: len(s.Faces)) if solids else shape

    deflection = float(sys.argv[1]) if len(sys.argv) >= 2 else 1e-5
    print(f"面ごとの体積への寄与（三角形分割の弦誤差 {deflection:.1e}）:")
    rows = []
    total_area = 0.0
    total_volume = 0.0
    for at, face in enumerate(subject.Faces):
        print(f"  ...面{at} を刻んでいます", flush=True)
        area, volume = face_volume_contribution(face, deflection, subject)
        total_area += area
        total_volume += volume
        centre = face.CenterOfMass
        rows.append((at, area, volume, centre.x, centre.y, centre.z))
        print(f"  面{at:<3} 面積 {area:.6f}  重心 ({centre.x:.4f} {centre.y:.4f} {centre.z:.4f})  寄与 {volume:+.6f}", flush=True)
    rows.sort(key=lambda row: -row[1])
    print(f"  面積の合計 {total_area:.6f}、体積（寄与の合計） {total_volume:.6f}")
    print(f"  真の V(A)（`subject.Volume`） {subject.Volume:.6f}")


if __name__ == "__main__":
    main()
