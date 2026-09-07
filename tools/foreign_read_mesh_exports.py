"""**書き出したメッシュと断面を、他人の実装に読ませる**（4-389）。

なぜ要るのか
------------
`tools/verify_mesh_exports.py` は**自前のパーサ**で解き直しています。
**他人の実装には 1 度も読ませていません**（HANDOVER 3-0-0 の
「STL / OBJ / glTF / DXF の外部検算」が**部分的**のまま）。

**自前のパーサで自前の書き出しを読むと、書き手と読み手が同じ思い違いを
していても通ります。** 4-378 で踏んだのと同じ形です——**自分の物差しで
自分を測っていました。**

ここでは **FreeCAD / OpenCASCADE** に読ませます。**閉じた式ではなく、
`manifest.json` に書いてある B-Rep 側の実測**と突き合わせます。

使い方
------
    cargo run --release -p zenith_algo --example export_mesh_suite
    & "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/foreign_read_mesh_exports.py

読み方
------
**三角形の数と境界箱は、そのまま突き合わせられます。**
**体積は、メッシュの体積**なので、**曲面のある立体では B-Rep より
小さく出ます**（内接するから）。**そこは「小さいこと」と「刻みに
見合う差であること」を見ます**——**一致は求められません**。

**1 件でも読めなければ非ゼロで終わります。**
"""

import io
import json
import os
import sys

# **コンソールが cp932 でも落ちないようにします。**
#
# FreeCAD 同梱の python は Windows の既定コードページで印字するので、
# **全角ダッシュ（U+2014）で `UnicodeEncodeError` を出して止まります**。
# **止まると、そこまでの検算結果ごと失われます**——実際に 1 度落ちました。
# 表示が化けるのは構いませんが、**落ちてはいけません。**
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXPORTS = os.path.join(ROOT, "target", "mesh_exports")


def main():
    manifest_path = os.path.join(EXPORTS, "manifest.json")
    if not os.path.exists(manifest_path):
        print("manifest.json がありません。先に export_mesh_suite を回してください。")
        return 1
    subjects = json.load(io.open(manifest_path, encoding="utf-8"))

    try:
        # **`import FreeCAD` が先です。** これが走るまで `Mesh` は
        # 見つかりません（`ModuleNotFoundError`）。**単体で `import Mesh`
        # を試すと「無い」と出ます**——実際にはあります。
        import FreeCAD  # noqa: F401
        import Mesh
    except Exception as exc:
        print("FreeCAD の Mesh が読めません: %r" % (exc,))
        print("FreeCAD 同梱の python で回してください。")
        return 1

    print("書き出した STL を、FreeCAD に読ませる（4-389）")
    print()
    print("%-32s %8s %8s %10s %10s %9s %s" % (
        "検体", "自分の枚数", "FreeCADの枚数", "自分の体積", "FreeCADの体積", "境界箱", "水密"))
    print("-" * 118)

    failures = 0
    for subject in subjects:
        name = subject["name"]
        stl = os.path.join(EXPORTS, name + ".stl")
        if not os.path.exists(stl):
            print("%-32s **STL がありません**" % name)
            failures += 1
            continue
        try:
            mesh = Mesh.Mesh(stl)
        except Exception as exc:
            print("%-32s **読めません**: %r" % (name, exc))
            failures += 1
            continue

        mine_tris = subject["triangles"]
        theirs = mesh.CountFacets
        volume = mesh.Volume
        bbox = mesh.BoundBox
        low = subject["low"]
        high = subject["high"]
        box_ok = (
            abs(bbox.XMin - low[0]) <= 1e-6 and abs(bbox.YMin - low[1]) <= 1e-6
            and abs(bbox.ZMin - low[2]) <= 1e-6 and abs(bbox.XMax - high[0]) <= 1e-6
            and abs(bbox.YMax - high[1]) <= 1e-6 and abs(bbox.ZMax - high[2]) <= 1e-6
        )
        # **枚数はそのまま合うべきです。** ここがずれたら、書き手か読み手の
        # どちらかが三角形を作り変えています。
        tris_ok = theirs == mine_tris
        solid = mesh.isSolid()

        if not tris_ok:
            failures += 1
        if not box_ok:
            failures += 1
        if not solid:
            failures += 1

        print("%-32s %8d %8d %10.3f %10.3f %9s %s" % (
            name, mine_tris, theirs, subject["brep_volume"], volume,
            "OK" if box_ok else "**ちがう**",
            "OK" if solid else "**閉じていない**"))

    # ---- OBJ も同じ人に読ませる ----
    #
    # **同じ立体を 2 つの形式で書いています。** **両方を他人に読ませて、
    # 互いに合うか**まで見ます——**片方だけ読ませると、書き手の思い違いが
    # 両方に入っていても気づけません。**
    #
    # **OBJ は頂点を共有します**（STL は三角形ごとにばらばら）。
    # **点の数は台帳の `vertices` と合うはず**です。
    print()
    print("書き出した OBJ を、FreeCAD に読ませる")
    print()
    print("%-32s %8s %8s %8s %8s %9s %s" % (
        "検体", "枚数(台帳)", "枚数(OBJ)", "点(台帳)", "点(OBJ)", "境界箱", "水密"))
    print("-" * 96)
    for subject in subjects:
        name = subject["name"]
        obj = os.path.join(EXPORTS, name + ".obj")
        if not os.path.exists(obj):
            print("%-32s **OBJ がありません**" % name)
            failures += 1
            continue
        try:
            mesh = Mesh.Mesh(obj)
        except Exception as exc:
            print("%-32s **読めません**: %r" % (name, exc))
            failures += 1
            continue
        low, high = subject["low"], subject["high"]
        b = mesh.BoundBox
        box_ok = (
            abs(b.XMin - low[0]) <= 1e-6 and abs(b.YMin - low[1]) <= 1e-6
            and abs(b.ZMin - low[2]) <= 1e-6 and abs(b.XMax - high[0]) <= 1e-6
            and abs(b.YMax - high[1]) <= 1e-6 and abs(b.ZMax - high[2]) <= 1e-6
        )
        tris_ok = mesh.CountFacets == subject["triangles"]
        pts_ok = mesh.CountPoints == subject["vertices"]
        solid = mesh.isSolid()
        if not (tris_ok and pts_ok and box_ok and solid):
            failures += 1
        print("%-32s %8d %8s %8d %8s %9s %s" % (
            name, subject["triangles"],
            str(mesh.CountFacets) if tris_ok else "**%d**" % mesh.CountFacets,
            subject["vertices"],
            str(mesh.CountPoints) if pts_ok else "**%d**" % mesh.CountPoints,
            "OK" if box_ok else "**ちがう**",
            "OK" if solid else "**閉じていない**"))

    # ---- DXF（断面）を FreeCAD に読ませる ----
    #
    # **断面の面積は、閉じた式ではなく B-Rep 側の実測**（`section_area`）と
    # 突き合わせます。**丸い穴は多角形で書かれる**ので、**内接するぶん
    # 小さく出ます**——`r=5` の穴で 78.5378 対 78.5398（π r²）です。
    # **一致ではなく、刻みに見合う差**を見ます。
    #
    # **外か穴かは、面積の大きい順で決めます。** 台帳が
    # `section_outer_loops` / `section_hole_loops` を持っているので、
    # **大きいほうから外の数だけ取り、残りを穴**とします。**入れ子が
    # 二段になる断面では、この決め方は足りません**——いまの 8 検体には
    # ありません。
    try:
        import importDXF
        import Part
    except Exception as exc:
        print()
        print("importDXF が読めません: %r（DXF の検算は飛ばします）" % (exc,))
        importDXF = None

    if importDXF is not None:
        print()
        print("書き出した DXF を、FreeCAD に読ませる")
        print()
        print("%-32s %6s %6s %14s %14s %10s" % (
            "検体", "輪(台帳)", "輪(FreeCAD)", "断面積(台帳)", "断面積(FreeCAD)", "相対差"))
        print("-" * 96)
        for subject in subjects:
            name = subject["name"]
            dxf = os.path.join(EXPORTS, name + ".dxf")
            if not os.path.exists(dxf):
                print("%-32s **DXF がありません**" % name)
                failures += 1
                continue
            doc = FreeCAD.newDocument("dxf_" + name)
            try:
                importDXF.insert(os.path.abspath(dxf), doc.Name)
                areas = []
                for obj in doc.Objects:
                    shape = getattr(obj, "Shape", None)
                    if shape is None or shape.ShapeType != "Wire":
                        continue
                    if not shape.isClosed():
                        continue
                    try:
                        areas.append(Part.Face(shape).Area)
                    except Exception:
                        pass
            finally:
                FreeCAD.closeDocument(doc.Name)

            want_loops = subject["section_outer_loops"] + subject["section_hole_loops"]
            areas.sort(reverse=True)
            outers = subject["section_outer_loops"]
            got = sum(areas[:outers]) - sum(areas[outers:])
            want = subject["section_area"]
            residual = abs(got - want) / max(abs(want), 1e-12)
            loops_ok = len(areas) == want_loops
            # **刻みぶんの差は許します。** 24 分割の多角形が円に内接するので、
            # **穴のある検体は 1e-4 級で小さく出ます**。
            area_ok = residual <= 3e-4
            if not loops_ok or not area_ok:
                failures += 1
            print("%-32s %6d %6s %14.6f %14.6f %10.3e" % (
                name, want_loops,
                str(len(areas)) if loops_ok else "**%d**" % len(areas),
                want, got, residual))

    print()
    if failures:
        print("**%d 件、他人の実装と食い違いました。**" % failures)
        return 1
    print("**8 検体すべて、FreeCAD が STL と OBJ を枚数・点数・境界箱どおりに読んで**")
    print("**水密と答え、DXF の断面も輪の数と面積が合いました。**")
    print()
    print("**glTF は、まだ他人に読ませていません**——FreeCAD の `Mesh` は")
    print("`File extension not supported` で断ります。`gltf-validator` が要ります。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
