"""検体を同じ切り手で切ったときの体積を、OpenCASCADE に出させる。

`foreign_boolean_probe` が断った配置について、「断ったのは正しいのか、
それとも本当は削れるのか」を決めるために使う。切り手の置き方はプローブと
同じ規則（境界箱に対する比）で作るので、同じ配置になる。

**OCC の求積は有理 B-spline 上で緩みます**（4-45）。ここで欲しいのは
「削れるのか、削れないのか」であって桁いっぱいの一致ではないので、その用途
には十分。

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/occ_cut_reference.py revolved_ring corner
"""

import os
import sys

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(FREECAD_BIN)

import FreeCAD  # noqa: E402
import Part  # noqa: E402

FIXTURES = os.path.join("crates", "zenith_algo", "tests", "fixtures")


def cutter(kind, box):
    """プローブと同じ規則で切り手を作る。"""
    size = (box.XMax - box.XMin, box.YMax - box.YMin, box.ZMax - box.ZMin)
    if kind == "slab":
        shape = Part.makeBox(size[0] * 0.6, size[1] * 2.0, size[2] * 2.0)
        shape.translate(
            FreeCAD.Vector(
                box.XMin - size[0] * 0.11,
                box.YMin - size[1] * 0.5,
                box.ZMin - size[2] * 0.5,
            )
        )
        return shape
    if kind == "drill":
        radius = min(size[0], size[1]) * 0.18
        shape = Part.makeCylinder(radius, size[2] * 3.0)
        shape.translate(
            FreeCAD.Vector(
                (box.XMin + box.XMax) * 0.5,
                (box.YMin + box.YMax) * 0.5,
                box.ZMin - size[2],
            )
        )
        return shape
    if kind == "corner":
        shape = Part.makeBox(size[0] * 0.45, size[1] * 0.45, size[2] * 0.45)
        shape.translate(
            FreeCAD.Vector(
                box.XMax - size[0] * 0.30,
                box.YMax - size[1] * 0.30,
                box.ZMax - size[2] * 0.30,
            )
        )
        return shape
    raise SystemExit(f"unknown cutter {kind}")


class ExplicitBox:
    """渡された6数そのものの箱。OCC の `BoundBox` と同じ読み方ができる。"""

    def __init__(self, values):
        (
            self.XMin,
            self.YMin,
            self.ZMin,
            self.XMax,
            self.YMax,
            self.ZMax,
        ) = values


def main():
    if len(sys.argv) < 3:
        raise SystemExit(
            "usage: occ_cut_reference.py <subject> <slab|drill|corner>"
            " [--box xmin ymin zmin xmax ymax zmax]"
        )
    subject, kind = sys.argv[1], sys.argv[2]
    given_box = None
    if "--box" in sys.argv:
        at = sys.argv.index("--box")
        given_box = ExplicitBox([float(v) for v in sys.argv[at + 1 : at + 7]])

    path = os.path.join(FIXTURES, f"occ_reference_{subject}.step")
    solid = Part.Shape()
    solid.read(path)

    # 切り手の置き方はプローブと同じで、**メッシュではなく厳密な境界箱**から
    # 決めます。プローブ側はメッシュの箱を使うので、丸い形ではわずかに違う
    # 箱になりえます。ずれると別の配置を測ることになるので、そこも出します。
    box = given_box if given_box is not None else solid.BoundBox
    tool = cutter(kind, box)

    difference = solid.cut(tool)
    intersection = solid.common(tool)
    union = solid.fuse(tool)

    print(f"subject {subject}  cutter {kind}")
    print(
        f"  bbox            ({box.XMin:.4f} {box.YMin:.4f} {box.ZMin:.4f})"
        f" - ({box.XMax:.4f} {box.YMax:.4f} {box.ZMax:.4f})"
    )
    print(f"  V(A)            {solid.Volume:.6f}")
    print(f"  V(B)            {tool.Volume:.6f}")
    print(f"  V(A - B)        {difference.Volume:.6f}")
    print(f"  V(A ^ B)        {intersection.Volume:.6f}")
    print(f"  V(A u B)        {union.Volume:.6f}")
    removed = solid.Volume - difference.Volume
    print(f"  removed         {removed:.6f}")
    print()
    if abs(removed) <= 1e-9 * max(solid.Volume, 1.0):
        print("  The cutter removes nothing here, so returning the solid")
        print("  unchanged is the right answer, not a wrong one.")
    else:
        print("  The cutter really does remove material. Returning the solid")
        print("  unchanged would be a wrong answer, not a refusal.")


if __name__ == "__main__":
    main()
