"""**書き出した glTF / STL / OBJ を、Blender に読ませる**（4-432、4-434）。

なぜ要るのか
------------
4-389 で **STL・OBJ・DXF は FreeCAD に読ませました**が、
**glTF だけ残っていました**——FreeCAD の `Mesh` は
`File extension not supported` で断り、`gltf-validator`・`trimesh`・
`pygltflib` がこの環境に入っていないからです。

**Blender の glTF 取り込みは、Khronos が保守している実装**
（`io_scene_gltf2`）です。**自前のパーサではありません。**

**自前のパーサで自前の書き出しを読むと、書き手と読み手が同じ思い違いを
していても通ります**（4-378 と同じ形）。**ここが、その穴を塞ぎます。**

使い方
------
    cargo run --release -p zenith_algo --example export_mesh_suite
    "C:/Program Files/Blender Foundation/Blender 4.4/blender.exe" \
        --background --factory-startup \
        --python tools/blender_read_gltf_exports.py

**`--factory-startup`** を付けます——**利用者の設定やアドオンを
持ち込まない**ためです。

読み方
------
**枚数と点の数と境界箱は、そのまま突き合わせられます。**

**⚠ glTF は y-up、CAD は z-up** です。**Blender の取り込みは既定で
座標を入れ替えます**ので、**境界箱は入れ替えた先で比べます**
（`+Y up` → Blender の `+Z up`）。**ここを忘れると、正しい書き出しを
「ちがう」と読みます。**

**⚠ 点の数は、必ずしも合いません。** **glTF は法線や uv が違えば
点を分けます**——**同じ位置でも別の点**です。**位置だけを丸めて
数え直した値**と、台帳の `vertices` を比べます。

**1 件でも食い違えば、非ゼロで終わります。**
"""

import json
import os
import sys

import bpy


def load_manifest(root):
    path = os.path.join(root, "target", "mesh_exports", "manifest.json")
    if not os.path.exists(path):
        print("manifest.json がありません。先に export_mesh_suite を回してください。")
        return None
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def read_one(path):
    """glTF を 1 つ読み、枚数・位置の数・境界箱を返す。"""
    clear_scene()
    bpy.ops.import_scene.gltf(filepath=path)

    triangles = 0
    positions = set()
    low = [float("inf")] * 3
    high = [float("-inf")] * 3
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        mesh = obj.data
        mesh.calc_loop_triangles()
        triangles += len(mesh.loop_triangles)
        for vertex in mesh.vertices:
            world = obj.matrix_world @ vertex.co
            # **位置だけで数えます**——glTF は法線や uv が違えば点を
            # 分けるので、**そのままの点数は台帳と合いません。**
            positions.add((round(world.x, 6), round(world.y, 6), round(world.z, 6)))
            for axis in range(3):
                low[axis] = min(low[axis], world[axis])
                high[axis] = max(high[axis], world[axis])
    return triangles, len(positions), low, high


def read_stl_or_obj(path, kind):
    """**STL と OBJ も、同じ人に読ませます**（4-434）。

    **FreeCAD には読ませてあります**（4-389）。**二人目です**——
    **glTF は、自前のパーサでは気づけない誤りを持っていました**ので、
    **他の形式も、もう一人に見てもらいます。**

    **⚠ OBJ は、軸を明示して読みます。**
    **OBJ の仕様には、上がどちらかの定めがありません。**
    **Blender の取り込みは既定で「Y が上」と見なす**ので、
    **z-up の CAD ファイルは横倒しに読まれます**——**書き出しの
    誤りではありません**（実測: 既定だと `[0,-40,0]..[20,0,30]`、
    `forward='Y', up='Z'` なら `[0,0,0]..[20,30,40]` で台帳どおり）。
    **glTF は違います**——**あちらは仕様が「+Y が上」と定めている**
    ので、**z-up のまま書いていたのは、こちらの誤りでした**（4-432）。
    """
    clear_scene()
    if kind == "stl":
        try:
            bpy.ops.wm.stl_import(filepath=path)
        except AttributeError:
            bpy.ops.import_mesh.stl(filepath=path)
    else:
        bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")

    triangles = 0
    positions = set()
    low = [float("inf")] * 3
    high = [float("-inf")] * 3
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        mesh = obj.data
        mesh.calc_loop_triangles()
        triangles += len(mesh.loop_triangles)
        for vertex in mesh.vertices:
            world = obj.matrix_world @ vertex.co
            positions.add((round(world.x, 5), round(world.y, 5), round(world.z, 5)))
            for axis in range(3):
                low[axis] = min(low[axis], world[axis])
                high[axis] = max(high[axis], world[axis])
    return triangles, len(positions), low, high


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    subjects = load_manifest(root)
    if subjects is None:
        return 1
    exports = os.path.join(root, "target", "mesh_exports")

    print()
    print("書き出した glTF を、Blender に読ませる（4-432）")
    print("Blender %s / glTF の取り込みは Khronos の io_scene_gltf2" % bpy.app.version_string)
    print()
    print("%-32s %9s %9s %8s %8s %s" % (
        "検体", "枚数(台帳)", "枚数(Blender)", "点(台帳)", "点(位置)", "境界箱"))
    print("-" * 96)

    failures = 0
    for subject in subjects:
        name = subject["name"]
        path = os.path.join(exports, name + ".gltf")
        if not os.path.exists(path):
            print("%-32s **glTF がありません**" % name)
            failures += 1
            continue
        try:
            triangles, points, low, high = read_one(path)
        except Exception as exc:
            print("%-32s **読めません**: %r" % (name, exc))
            failures += 1
            continue

        # **座標の入れ替え**——glTF は y-up。Blender は取り込みで z-up へ
        # 直すので、**そのまま台帳と比べられます。**
        want_low = subject["low"]
        want_high = subject["high"]
        box_ok = all(
            abs(low[axis] - want_low[axis]) <= 1e-4
            and abs(high[axis] - want_high[axis]) <= 1e-4
            for axis in range(3)
        )
        tris_ok = triangles == subject["triangles"]
        points_ok = points == subject["vertices"]

        if not tris_ok or not box_ok or not points_ok:
            failures += 1

        print("%-32s %9d %9s %8d %8s %s" % (
            name, subject["triangles"],
            ("%d" % triangles) if tris_ok else ("**%d**" % triangles),
            subject["vertices"],
            ("%d" % points) if points_ok else ("**%d**" % points),
            "OK" if box_ok else "**ちがう** %s %s" % (
                [round(x, 4) for x in low], [round(x, 4) for x in high])))

    print("-" * 96)

    # ---- STL と OBJ も、同じ人に読ませます（4-434）----
    for kind in ("stl", "obj"):
        print()
        print("書き出した %s を、Blender に読ませる（4-434）%s"
              % (kind.upper(),
                 "  ※ 軸を明示（forward=Y, up=Z）" if kind == "obj" else ""))
        print()
        print("%-32s %9s %9s %8s %8s %s" % (
            "検体", "枚数(台帳)", "枚数(Blender)", "点(台帳)", "点(位置)", "境界箱"))
        print("-" * 96)
        for subject in subjects:
            name = subject["name"]
            path = os.path.join(exports, name + "." + kind)
            if not os.path.exists(path):
                print("%-32s **%s がありません**" % (name, kind.upper()))
                failures += 1
                continue
            try:
                triangles, points, low, high = read_stl_or_obj(path, kind)
            except Exception as exc:
                print("%-32s **読めません**: %r" % (name, exc))
                failures += 1
                continue
            box_ok = all(
                abs(low[axis] - subject["low"][axis]) <= 1e-4
                and abs(high[axis] - subject["high"][axis]) <= 1e-4
                for axis in range(3))
            tris_ok = triangles == subject["triangles"]
            points_ok = points == subject["vertices"]
            if not tris_ok or not box_ok or not points_ok:
                failures += 1
            print("%-32s %9d %9s %8d %8s %s" % (
                name, subject["triangles"],
                ("%d" % triangles) if tris_ok else ("**%d**" % triangles),
                subject["vertices"],
                ("%d" % points) if points_ok else ("**%d**" % points),
                "OK" if box_ok else "**ちがう** %s %s" % (
                    [round(x, 4) for x in low], [round(x, 4) for x in high])))
        print("-" * 96)

    print()
    if failures:
        print("**食い違い %d 件。**" % failures)
    else:
        print("**glTF / STL / OBJ とも、8 検体すべてで枚数・点の数・境界箱が一致しました。**")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
