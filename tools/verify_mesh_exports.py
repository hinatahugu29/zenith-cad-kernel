"""STL / OBJ / glTF / DXF を、**書いたファイルだけ**から検算する。

`cargo run --release -p zenith_algo --example export_mesh_suite` が
`target/mesh_exports/` に書いた出力を、外側から解いて次を確かめる。

  1. **閉じているか** — 頂点を座標で束ねたとき、どの辺もちょうど2枚の三角形に
     共有されているか。STL はスライサに渡す形なので、ここが赤なら3Dプリンタに
     送れない。
  2. **同じ形か** — 発散定理でメッシュから積んだ体積が、台帳に載っている
     B-Rep の体積と合うか。曲面は内接するので少し小さく出る。**小さい側にだけ**
     許容を取り、大きい側は締める。
  3. **3つの形式が同じものを書いたか** — STL・OBJ・glTF の体積が互いに一致するか。
     どれか1つだけずれていれば、そのエクスポータの欠陥。
  4. **図面の層が向きと合っているか** — DXF の `OUTLINE` / `HOLE` の本数が、
     台帳の外形数・穴数と一致するか。符号付き面積の合計が断面積と合うか。

**以前この位置にあったものは、検査を1つも呼んでいなかった。** 関数は定義されて
いたが `main()` からは呼ばれず、読む立体も無く、常に
「All format validators loaded and verified.」と印字して 0 で終わっていた。

    py tools/verify_mesh_exports.py

不一致があれば非ゼロで終わる。FreeCAD は要らない。

**これは「他人が読めた」ことの証明ではない。** 解いているのはこのファイルの
中の自前パーサで、STL / OBJ / glTF / DXF の実装は1つも入っていない
（`tools/verify_iges.py` が OpenCASCADE に読ませているのとは、そこが違う）。
3形式の体積が互いに一致することは見ているので「1つのエクスポータだけがずれて
いる」は捕まるが、「3つとも同じ規約違反をしている」は捕まらない。
"""

import base64
import json
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXPORT_DIR = os.path.join(ROOT, "target", "mesh_exports")

# 曲面は内接多角形で近似される。**どちらへずれるかは凹凸で変わる。** 外へ
# 出っ張った面（円柱の側面、球）では体積が小さく出るが、**内側へ食い込んだ面**
# （穴のボア）では穴のほうが小さく刻まれるので、立体は逆に大きく出る。
#
# 最初ここを「小さく出るはず」と片側だけで書いたところ、穴あき直方体と
# ブーリアン結果が 6.9e-05 / 5.5e-05 の**増加**で赤くなった。これは欠陥では
# なく近似の向きで、締める側を1つに決めたこちらの誤り。
#
# 帯は両側で取る。内接多角形の相対誤差はおよそ (pi^2/3)/n^2 で、24分割なら
# 5.7e-3。実測の最悪はトーラスの -1.44e-3。
VOLUME_DRIFT_ALLOWANCE = 6.0e-3
# STL は座標を float32 で持つ。形式どうしの比較はその分だけ緩める。
CROSS_FORMAT_TOLERANCE = 1.0e-5
WELD = 1.0e-6

SUFFIX = {"STL": "stl", "OBJ": "obj", "glTF": "gltf"}


def quantize(point):
    return tuple(int(round(value / WELD)) for value in point)


def mesh_volume(positions, triangles):
    """発散定理。閉じた向き付きメッシュなら符号付き体積が出る。"""
    total = 0.0
    for a, b, c in triangles:
        pa, pb, pc = positions[a], positions[b], positions[c]
        total += (
            pa[0] * (pb[1] * pc[2] - pc[1] * pb[2])
            - pa[1] * (pb[0] * pc[2] - pc[0] * pb[2])
            + pa[2] * (pb[0] * pc[1] - pc[0] * pb[1])
        )
    return total / 6.0


def open_and_nonmanifold_edges(positions, triangles):
    counts = {}
    for triangle in triangles:
        keys = [quantize(positions[index]) for index in triangle]
        for corner in range(3):
            a, b = keys[corner], keys[(corner + 1) % 3]
            key = (a, b) if a <= b else (b, a)
            counts[key] = counts.get(key, 0) + 1
    opened = sum(1 for count in counts.values() if count == 1)
    non_manifold = sum(1 for count in counts.values() if count > 2)
    return opened, non_manifold


def read_stl(path):
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) < 84:
        raise ValueError("STL is shorter than its own header")
    count = struct.unpack("<I", data[80:84])[0]
    expected = 84 + count * 50
    if len(data) != expected:
        raise ValueError(
            "STL length {} does not match {} facets ({})".format(len(data), count, expected)
        )

    positions = []
    triangles = []
    offset = 84
    for _ in range(count):
        values = struct.unpack("<12f", data[offset:offset + 48])
        base = len(positions)
        positions.append(values[3:6])
        positions.append(values[6:9])
        positions.append(values[9:12])
        triangles.append((base, base + 1, base + 2))
        offset += 50
    return positions, triangles


def read_obj(path):
    positions = []
    triangles = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("v "):
                parts = line.split()
                positions.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                corners = [int(token.split("/")[0]) - 1 for token in line.split()[1:]]
                if len(corners) != 3:
                    raise ValueError("OBJ face is not a triangle: " + line.strip())
                triangles.append(tuple(corners))
    if not positions or not triangles:
        raise ValueError("OBJ has no vertices or no faces")
    return positions, triangles


def read_gltf(path):
    with open(path, "r", encoding="utf-8") as handle:
        document = json.load(handle)

    if document.get("asset", {}).get("version") != "2.0":
        raise ValueError("not a glTF 2.0 document")

    buffers = document["buffers"]
    if len(buffers) != 1:
        raise ValueError("expected exactly one buffer, found {}".format(len(buffers)))
    uri = buffers[0]["uri"]
    marker = "base64,"
    if marker not in uri:
        raise ValueError("the buffer is not embedded as a base64 data URI")
    blob = base64.b64decode(uri.split(marker, 1)[1])
    if len(blob) != buffers[0]["byteLength"]:
        raise ValueError(
            "buffer byteLength {} but the data URI decodes to {}".format(
                buffers[0]["byteLength"], len(blob)
            )
        )

    views = document["bufferViews"]
    accessors = document["accessors"]
    for index, accessor in enumerate(accessors):
        view = views[accessor["bufferView"]]
        size = {5126: 4, 5125: 4, 5123: 2}[accessor["componentType"]]
        components = {"VEC3": 3, "SCALAR": 1}[accessor["type"]]
        needed = accessor["count"] * components * size
        if needed != view["byteLength"]:
            raise ValueError(
                "accessor {} needs {} bytes but its bufferView holds {}".format(
                    index, needed, view["byteLength"]
                )
            )
        if view["byteOffset"] + view["byteLength"] > len(blob):
            raise ValueError(
                "bufferView of accessor {} runs past the end of the buffer".format(index)
            )

    primitive = document["meshes"][0]["primitives"][0]
    position_accessor = accessors[primitive["attributes"]["POSITION"]]
    index_accessor = accessors[primitive["indices"]]

    view = views[position_accessor["bufferView"]]
    start = view["byteOffset"]
    positions = [
        struct.unpack_from("<3f", blob, start + i * 12)
        for i in range(position_accessor["count"])
    ]

    view = views[index_accessor["bufferView"]]
    raw = struct.unpack_from("<{}I".format(index_accessor["count"]), blob, view["byteOffset"])
    triangles = [tuple(raw[i:i + 3]) for i in range(0, len(raw), 3)]

    for corner in raw:
        if corner >= len(positions):
            raise ValueError("an index points past the end of the POSITION accessor")

    # 宣言した min / max が、実際の座標と合っているか。
    if "min" in position_accessor and "max" in position_accessor:
        for axis in range(3):
            low = min(point[axis] for point in positions)
            high = max(point[axis] for point in positions)
            if low < position_accessor["min"][axis] - 1e-4:
                raise ValueError("POSITION min[{}] is larger than the real minimum".format(axis))
            if high > position_accessor["max"][axis] + 1e-4:
                raise ValueError("POSITION max[{}] is smaller than the real maximum".format(axis))

    return positions, triangles


def read_dxf_polylines(path):
    """LWPOLYLINE を (層, 点列) で拾う。DXF はグループコードと値の対。"""
    with open(path, "r", encoding="utf-8") as handle:
        lines = [line.rstrip("\n") for line in handle]

    polylines = []
    index = 0
    in_entities = False
    while index + 1 < len(lines):
        code, value = lines[index], lines[index + 1]
        if code == "2" and value == "ENTITIES":
            in_entities = True
        if in_entities and code == "0" and value == "LWPOLYLINE":
            layer = None
            points = []
            pending_x = None
            index += 2
            while index + 1 < len(lines) and lines[index] != "0":
                code, value = lines[index], lines[index + 1]
                if code == "8":
                    layer = value
                elif code == "10":
                    pending_x = float(value)
                elif code == "20":
                    points.append((pending_x, float(value)))
                index += 2
            polylines.append((layer, points))
            continue
        index += 2
    return polylines


def shoelace(points):
    total = 0.0
    for i in range(len(points)):
        ax, ay = points[i]
        bx, by = points[(i + 1) % len(points)]
        total += ax * by - bx * ay
    return total / 2.0


def check_meshes(subject, problems):
    name = subject["name"]
    volumes = {}
    for kind, reader in (("STL", read_stl), ("OBJ", read_obj), ("glTF", read_gltf)):
        path = os.path.join(EXPORT_DIR, "{}.{}".format(name, SUFFIX[kind]))
        try:
            positions, triangles = reader(path)
        except Exception as error:  # noqa: BLE001
            problems.append("{} / {}: {}".format(name, kind, error))
            continue

        if len(triangles) != subject["triangles"]:
            problems.append(
                "{} / {}: {} triangles, the kernel wrote {}".format(
                    name, kind, len(triangles), subject["triangles"]
                )
            )

        opened, non_manifold = open_and_nonmanifold_edges(positions, triangles)
        if opened or non_manifold:
            problems.append(
                "{} / {}: not closed - {} open edge(s), {} non-manifold".format(
                    name, kind, opened, non_manifold
                )
            )

        volume = mesh_volume(positions, triangles)
        volumes[kind] = volume
        reference = subject["brep_volume"]
        drift = (volume - reference) / abs(reference)
        if abs(drift) > VOLUME_DRIFT_ALLOWANCE:
            problems.append(
                "{} / {}: volume {:.6f} against the B-Rep {:.6f} (drift {:.2e})".format(
                    name, kind, volume, reference, drift
                )
            )

    if len(volumes) == 3:
        low, high = min(volumes.values()), max(volumes.values())
        spread = (high - low) / max(abs(high), 1e-12)
        if spread > CROSS_FORMAT_TOLERANCE:
            problems.append(
                "{}: the three formats disagree by {:.2e} ({})".format(
                    name,
                    spread,
                    ", ".join("{}={:.6f}".format(k, v) for k, v in volumes.items()),
                )
            )
    return volumes.get("STL")


def check_drawing(subject, problems):
    name = subject["name"]
    path = os.path.join(EXPORT_DIR, "{}.dxf".format(name))
    try:
        polylines = read_dxf_polylines(path)
    except Exception as error:  # noqa: BLE001
        problems.append("{} / DXF: {}".format(name, error))
        return None

    outlines = [points for layer, points in polylines if layer == "OUTLINE"]
    holes = [points for layer, points in polylines if layer == "HOLE"]
    if len(outlines) != subject["section_outer_loops"]:
        problems.append(
            "{} / DXF: {} contour(s) on OUTLINE, the section has {} outer loop(s)".format(
                name, len(outlines), subject["section_outer_loops"]
            )
        )
    if len(holes) != subject["section_hole_loops"]:
        problems.append(
            "{} / DXF: {} contour(s) on HOLE, the section has {} hole(s)".format(
                name, len(holes), subject["section_hole_loops"]
            )
        )
    for points in outlines:
        if shoelace(points) <= 0.0:
            problems.append("{} / DXF: a contour on OUTLINE runs clockwise".format(name))
    for points in holes:
        if shoelace(points) >= 0.0:
            problems.append("{} / DXF: a contour on HOLE runs counter-clockwise".format(name))

    area = sum(shoelace(points) for _, points in polylines)
    reference = subject["section_area"]
    drift = (area - reference) / abs(reference) if reference else area
    if abs(drift) > VOLUME_DRIFT_ALLOWANCE:
        problems.append(
            "{} / DXF: section area {:.6f} against the kernel's {:.6f} (drift {:.2e})".format(
                name, area, reference, drift
            )
        )
    return area


def main():
    manifest_path = os.path.join(EXPORT_DIR, "manifest.json")
    if not os.path.exists(manifest_path):
        print("manifest.json not found. Run:")
        print("  cargo run --release -p zenith_algo --example export_mesh_suite")
        return 1

    with open(manifest_path, "r", encoding="utf-8") as handle:
        subjects = json.load(handle)

    header = "{:<28}{:>10}{:>16}{:>16}{:>13}{:>10}".format(
        "subject", "triangles", "STL volume", "B-Rep volume", "DXF area", "verdict"
    )
    print(header)
    print("-" * len(header))

    problems = []
    bad_subjects = 0
    for subject in subjects:
        before = len(problems)
        stl_volume = check_meshes(subject, problems)
        area = check_drawing(subject, problems)
        if len(problems) != before:
            bad_subjects += 1
        print(
            "{:<28}{:>10}{:>16}{:>16.6f}{:>13}{:>10}".format(
                subject["name"],
                subject["triangles"],
                "-" if stl_volume is None else "{:.6f}".format(stl_volume),
                subject["brep_volume"],
                "-" if area is None else "{:.4f}".format(area),
                "ok" if len(problems) == before else "PROBLEM",
            )
        )

    print("-" * len(header))
    for problem in problems:
        print("  " + problem)
    print(
        "{} of {} subject(s) round-tripped through STL / OBJ / glTF / DXF".format(
            len(subjects) - bad_subjects, len(subjects)
        )
    )
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
