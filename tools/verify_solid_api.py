"""Python 側で立体を持ち回れるかを、閉じた式と突き合わせて確かめる。

これまでの Python API はワンショットのビルダーしかなく、作った立体を次の
演算に渡す手段がありませんでした。ここで測るのは `Solid` ハンドル越しに

    作る -> ブーリアン -> 稜を選ぶ -> 丸める -> STEP に書く -> 読み直す

が一周し、**体積が閉じた式と一致したまま**戻ってくるかどうかです。

使い方:
    py tools/verify_solid_api.py
（先に `py tools/build_pyd.py` か `cargo build --release -p zenith_py`）
"""

import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "target" / "release"))

import zenith_cad as z  # noqa: E402

FAILURES = []


def check(name, condition, detail=""):
    if condition:
        print(f"  ok   {name}")
    else:
        print(f"  FAIL {name} :: {detail}")
        FAILURES.append(name)


def close(actual, expected, rel=1e-9):
    if expected == 0:
        return abs(actual) < rel
    return abs(actual - expected) / abs(expected) < rel


def test_primitives_and_measurement():
    print("[primitives]")
    part = z.Solid.box(20.0, 30.0, 40.0)
    check("a box reports six faces", part.face_count == 6, part.face_count)
    check("a box measures dx*dy*dz", close(part.volume, 24000.0), part.volume)

    mass = part.mass_properties()
    check(
        "the centre of mass sits at the middle",
        all(close(a, b, 1e-9) for a, b in zip(mass["center_of_mass"], [10.0, 15.0, 20.0])),
        mass["center_of_mass"],
    )

    cylinder = z.Solid.cylinder(5.0, 12.0)
    expected = math.pi * 25.0 * 12.0
    check(
        "a cylinder measures pi r^2 h",
        close(cylinder.volume, expected, 1e-9),
        f"{cylinder.volume} against {expected}",
    )


def test_boolean_chain():
    print("[boolean]")
    block = z.Solid.box(40.0, 40.0, 20.0)
    corner = z.Solid.box(20.0, 20.0, 20.0).translated(20.0, 20.0, 0.0)

    notched = block.difference(corner)
    expected = (40.0 * 40.0 - 20.0 * 20.0) * 20.0
    check(
        "a difference measures what it removed",
        close(notched.volume, expected, 1e-11),
        f"{notched.volume} against {expected}",
    )

    report = notched.validate()
    check("the difference is a valid closed shell", report["valid"], report["errors"][:2])
    check(
        "every edge of the difference is shared as one entity",
        report["unshared_edge_entity_uses"] == 0,
        report["unshared_edge_entity_uses"],
    )

    stacked = block.union(z.Solid.box(20.0, 20.0, 30.0).translated(10.0, 10.0, 20.0))
    expected = 40.0 * 40.0 * 20.0 + 20.0 * 20.0 * 30.0
    check(
        "a union measures both parts",
        close(stacked.volume, expected, 1e-11),
        f"{stacked.volume} against {expected}",
    )


def test_fillet_on_a_boolean_result():
    print("[fillet on a boolean result]")
    block = z.Solid.box(40.0, 40.0, 20.0)
    corner = z.Solid.box(20.0, 20.0, 20.0).translated(20.0, 20.0, 0.0)
    notched = block.difference(corner)
    start = notched.volume

    # 切り欠きの外側にある凸の縦稜 (x=40, y=20)
    target = None
    for edge in notched.blendable_edges():
        mid = [
            (a + b) * 0.5
            for a, b in zip(
                next(e["start"] for e in notched.edges() if e["edge_id"] == edge["edge_id"]),
                next(e["end"] for e in notched.edges() if e["edge_id"] == edge["edge_id"]),
            )
        ]
        if abs(mid[0] - 40.0) < 1e-9 and abs(mid[1] - 20.0) < 1e-9:
            target = edge
    check("the boolean result offers a blendable upright at (40, 20)", target is not None)
    if target is None:
        return

    check(
        "it is measured as a right angle",
        close(target["dihedral_angle_deg"], 90.0, 1e-9),
        target["dihedral_angle_deg"],
    )

    radius = 3.0
    filleted = notched.fillet_edge(target["edge_id"], radius)
    expected = start - 20.0 * radius * radius * (1.0 - math.pi / 4.0)
    check(
        "filleting removes the closed form volume",
        close(filleted.volume, expected, 1e-10),
        f"{filleted.volume} against {expected}",
    )
    check("the filleted solid is still valid", filleted.validate()["valid"])

    # 凹の稜は列挙にも出ず、指定しても断られる
    concave = [e for e in notched.edges() if e["kind"] == "concave"]
    check("the notch has a concave upright", len(concave) >= 1, len(concave))
    if concave:
        try:
            notched.fillet_edge(concave[0]["edge_id"], 2.0)
            check("a concave edge is refused", False, "it went through")
        except ValueError as err:
            check("a concave edge is refused with a reason", "convex" in str(err), str(err))


def test_hexagonal_prism_uses_its_own_angle():
    print("[non right angle]")
    prism = z.Solid.regular_prism(6, 10.0, 25.0)
    start = prism.volume

    edges = prism.blendable_edges()
    check("only the six uprights qualify", len(edges) == 6, len(edges))
    if not edges:
        return
    check(
        "the dihedral is measured as 120 deg",
        close(edges[0]["dihedral_angle_deg"], 120.0, 1e-9),
        edges[0]["dihedral_angle_deg"],
    )

    theta = math.radians(120.0)
    radius = 2.0
    filleted = prism.fillet_edge(edges[0]["edge_id"], radius)
    removed = 25.0 * radius * radius * (1.0 / math.tan(theta / 2.0) - 0.5 * (math.pi - theta))
    check(
        "the fillet follows the 120 deg form, not the right angle one",
        close(filleted.volume, start - removed, 1e-10),
        f"{filleted.volume} against {start - removed}",
    )


def test_step_round_trip():
    print("[step round trip]")
    part = z.Solid.box(20.0, 30.0, 40.0)
    edge = part.blendable_edges()[0]
    part = part.fillet_edge(edge["edge_id"], 4.0)
    before = part.volume

    with tempfile.TemporaryDirectory() as directory:
        path = str(Path(directory) / "solid_api_probe.step")
        part.to_step(path, "solid_api_probe")
        again = z.Solid.from_step(path)

    check(
        "the volume survives a STEP round trip",
        close(again.volume, before, 1e-9),
        f"{again.volume} against {before}",
    )


def test_mesh_still_available():
    print("[mesh]")
    mesh = z.Solid.box(10.0, 10.0, 10.0).tessellate(8, 8)
    check("tessellation still hands back a Mesh", mesh.num_faces > 0, mesh.num_faces)
    check("the mesh volume agrees", close(mesh.volume, 1000.0, 1e-9), mesh.volume)


def test_inertia():
    print("[inertia]")
    part = z.Solid.box(20.0, 30.0, 40.0)
    mass = part.mass_properties()
    volume = 20.0 * 30.0 * 40.0

    check(
        "the diagonal is about the origin, not the centre of mass",
        close(mass["inertia_diagonal_about_origin"][0], volume * (30.0**2 + 40.0**2) / 3.0, 1e-11),
        mass["inertia_diagonal_about_origin"][0],
    )

    want = sorted(
        [
            volume * (30.0**2 + 40.0**2) / 12.0,
            volume * (20.0**2 + 40.0**2) / 12.0,
            volume * (20.0**2 + 30.0**2) / 12.0,
        ]
    )
    check(
        "the principal moments are about the centre of mass",
        all(close(a, b, 1e-11) for a, b in zip(mass["principal_moments"], want)),
        mass["principal_moments"],
    )

    turned = part.rotated([0.0, 0.0, 0.0], [1.0, 2.0, 3.0], 35.0)
    check(
        "turning the solid does not change the principal moments",
        all(
            close(a, b, 1e-10)
            for a, b in zip(turned.mass_properties()["principal_moments"], want)
        ),
        turned.mass_properties()["principal_moments"],
    )


def test_simplify():
    print("[simplify]")
    block = z.Solid.box(40.0, 40.0, 20.0)
    corner = z.Solid.box(20.0, 20.0, 20.0).translated(20.0, 20.0, 0.0)
    notched = block.difference(corner)
    before = notched.volume

    report = notched.simplify_report()
    check(
        "the report says the split faces can be merged",
        report["faces_before"] > report["faces_after"],
        report,
    )

    simple = notched.simplified()
    check("an L prism comes back as eight faces", simple.face_count == 8, simple.face_count)
    check(
        "simplifying does not move the volume",
        close(simple.volume, before, 1e-13),
        f"{simple.volume} against {before}",
    )
    check("the simplified solid is valid", simple.validate()["valid"])
    check(
        "more edges become blendable after simplifying",
        len(simple.blendable_edges()) >= len(notched.blendable_edges()),
    )


def main():
    print("=" * 60)
    print("Zenith CAD - Solid handle end to end")
    print("=" * 60)
    test_primitives_and_measurement()
    test_boolean_chain()
    test_fillet_on_a_boolean_result()
    test_hexagonal_prism_uses_its_own_angle()
    test_step_round_trip()
    test_mesh_still_available()
    test_simplify()
    test_inertia()

    print("=" * 60)
    if FAILURES:
        print(f"{len(FAILURES)} check(s) failed: {', '.join(FAILURES)}")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
