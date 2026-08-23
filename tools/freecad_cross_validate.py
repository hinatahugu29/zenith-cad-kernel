"""Cross-validates Zenith kernel results against FreeCAD / OpenCASCADE.

The kernel exports a fixed set of solids to STEP together with the numbers it
computed for them (see `cargo run -p zenith_algo --example
export_validation_suite`). This script re-reads every STEP through
OpenCASCADE and asks the same questions independently: is the solid valid and
closed, what is its volume and surface area, and what is the area of the same
cross-section. Agreement between two unrelated kernels is evidence; a kernel
agreeing with itself is not.

Usage (FreeCAD's bundled interpreter, so the OCC bindings resolve):

    & "C:\\Program Files\\FreeCAD 1.1\\bin\\python.exe" tools/freecad_cross_validate.py

Exit code is non-zero when any subject disagrees beyond tolerance, so this can
gate a release.

A caveat found on 2026年8月21日: `spur_gear_m2_z18` reads **7.18e-05** in the
volume column while every other subject sits at 1e-12. That is not our error.
The gear's surface **area** agrees to 8.2e-08, the re-imported tooth flanks lie
within 7.6e-08 of the true involute, and re-integrating the same imported shape
from its own tessellation converges on the closed form. What differs is
OpenCASCADE's `shape.Volume` on this solid — 146 faces whose flanks are degree-3
B-splines over dozens of knot spans. See `tools/diagnose_gear_reexport.py`.

Read the volume column with that in mind: it is a strong yardstick for analytic
surfaces and for the degree-2 rational patches `verify_reexport` grades, and a
weaker one for spline-heavy solids like this gear.
"""

import json
import os
import sys

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

FREECAD_BIN = r"C:\Program Files\FreeCAD 1.1\bin"
if FREECAD_BIN not in sys.path:
    sys.path.insert(0, FREECAD_BIN)
if hasattr(os, "add_dll_directory") and os.path.isdir(FREECAD_BIN):
    try:
        os.add_dll_directory(FREECAD_BIN)
    except Exception:
        pass

import FreeCAD  # noqa: E402
import Part  # noqa: E402

# A B-Rep read back through a different kernel will not agree bit for bit;
# these are the bands within which the two are considered to be saying the
# same thing.
VOLUME_RELATIVE_TOLERANCE = 1e-4
AREA_RELATIVE_TOLERANCE = 1e-4
SECTION_RELATIVE_TOLERANCE = 1e-3


def relative_error(value, expected):
    if expected is None or value is None:
        return None
    denominator = abs(expected)
    if denominator < 1e-12:
        return abs(value - expected)
    return abs(value - expected) / denominator


def section_area_via_occ(shape, origin, normal):
    """Cross-section area computed by OpenCASCADE, independent of our slicer.

    Each section loop becomes its own face and its sign comes from how deeply
    it nests inside the others, so holes subtract and islands inside holes add
    back. Facing all the loops as one wire silently merges them and inflates
    the answer.
    """
    try:
        base = FreeCAD.Vector(*origin)
        direction = FreeCAD.Vector(*normal).normalize()
        wires = shape.slice(direction, direction.dot(base))
        if not wires:
            return None, "no section wires"

        faces = []
        for wire in wires:
            try:
                faces.append(Part.Face(wire))
            except Exception:
                faces.append(None)

        if all(face is None for face in faces):
            return None, "no wire could be faced"

        total = 0.0
        for index, face in enumerate(faces):
            if face is None:
                continue

            probe = wires[index].Vertexes[0].Point
            depth = 0
            for other_index, other in enumerate(faces):
                if other is None or other_index == index:
                    continue
                try:
                    if other.isInside(probe, 1e-6, True):
                        depth += 1
                except Exception:
                    continue

            total += face.Area if depth % 2 == 0 else -face.Area

        return total, None
    except Exception as exc:
        return None, str(exc)


def check_subject(subject):
    step_file = subject["step_file"]
    name = subject["name"]

    row = {
        "name": name,
        "problems": [],
        "occ": {},
    }

    if not os.path.isfile(step_file):
        row["problems"].append(f"STEP file missing: {step_file}")
        return row

    try:
        shape = Part.read(step_file)
    except Exception as exc:
        row["problems"].append(f"OpenCASCADE could not read the STEP: {exc}")
        return row

    row["occ"]["shape_type"] = shape.ShapeType
    row["occ"]["face_count"] = len(shape.Faces)

    try:
        row["occ"]["is_valid"] = bool(shape.isValid())
    except Exception as exc:
        row["occ"]["is_valid"] = None
        row["problems"].append(f"isValid raised: {exc}")

    try:
        row["occ"]["is_closed"] = bool(shape.isClosed())
    except Exception:
        row["occ"]["is_closed"] = None

    if row["occ"].get("is_valid") is False:
        row["problems"].append("OpenCASCADE reports the shape as invalid")

    try:
        row["occ"]["volume"] = float(shape.Volume)
    except Exception as exc:
        row["occ"]["volume"] = None
        row["problems"].append(f"Volume raised: {exc}")

    try:
        row["occ"]["area"] = float(shape.Area)
    except Exception:
        row["occ"]["area"] = None

    # Volume: kernel vs OCC, and both against the analytic value.
    kernel_volume = subject.get("kernel_volume")
    occ_volume = row["occ"].get("volume")
    analytic_volume = subject.get("analytic_volume")

    row["kernel_volume"] = kernel_volume
    row["analytic_volume"] = analytic_volume

    cross = relative_error(occ_volume, kernel_volume)
    row["volume_cross_error"] = cross
    if cross is not None and cross > VOLUME_RELATIVE_TOLERANCE:
        row["problems"].append(
            f"volume disagreement: kernel {kernel_volume:.6f} vs OCC {occ_volume:.6f}"
            f" (relative {cross:.3e})"
        )

    if analytic_volume is not None:
        kernel_error = relative_error(kernel_volume, analytic_volume)
        occ_error = relative_error(occ_volume, analytic_volume)
        row["kernel_volume_error"] = kernel_error
        row["occ_volume_error"] = occ_error
        if kernel_error is not None and kernel_error > VOLUME_RELATIVE_TOLERANCE:
            row["problems"].append(
                f"kernel volume off the analytic {analytic_volume:.6f} by {kernel_error:.3e}"
            )
        if occ_error is not None and occ_error > VOLUME_RELATIVE_TOLERANCE:
            row["problems"].append(
                f"OCC volume off the analytic {analytic_volume:.6f} by {occ_error:.3e}"
            )

    # Surface area: kernel vs OCC.
    kernel_area = subject.get("kernel_area")
    occ_area = row["occ"].get("area")
    area_cross = relative_error(occ_area, kernel_area)
    row["area_cross_error"] = area_cross
    if area_cross is not None and area_cross > AREA_RELATIVE_TOLERANCE:
        row["problems"].append(
            f"surface area disagreement: kernel {kernel_area:.6f} vs OCC {occ_area:.6f}"
            f" (relative {area_cross:.3e})"
        )

    # Section area, when the manifest asked for one.
    section = subject.get("section")
    if section:
        if section.get("error"):
            row["problems"].append(f"kernel section failed: {section['error']}")
        else:
            occ_section, section_problem = section_area_via_occ(
                shape, section["origin"], section["normal"]
            )
            row["kernel_section_area"] = section.get("area")
            row["occ_section_area"] = occ_section
            row["analytic_section_area"] = section.get("analytic_area")

            if section_problem:
                row["problems"].append(f"OCC section failed: {section_problem}")
            else:
                cross_section = relative_error(occ_section, section.get("area"))
                row["section_cross_error"] = cross_section
                if (
                    cross_section is not None
                    and cross_section > SECTION_RELATIVE_TOLERANCE
                ):
                    row["problems"].append(
                        f"section area disagreement: kernel {section['area']:.6f}"
                        f" vs OCC {occ_section:.6f} (relative {cross_section:.3e})"
                    )

            analytic_section = section.get("analytic_area")
            if analytic_section is not None:
                kernel_section_error = relative_error(
                    section.get("area"), analytic_section
                )
                row["kernel_section_error"] = kernel_section_error
                if (
                    kernel_section_error is not None
                    and kernel_section_error > SECTION_RELATIVE_TOLERANCE
                ):
                    row["problems"].append(
                        f"kernel section off the analytic {analytic_section:.6f}"
                        f" by {kernel_section_error:.3e}"
                    )

    # Closedness: the kernel claims a closed shell, so OCC should agree.
    if subject.get("shell_valid") and row["occ"].get("is_closed") is False:
        row["problems"].append(
            "kernel reports a valid closed shell but OpenCASCADE reports the solid as open"
        )

    return row


def main():
    manifest_path = os.path.join("target", "validation", "manifest.json")
    if len(sys.argv) > 1:
        manifest_path = sys.argv[1]

    if not os.path.isfile(manifest_path):
        print(f"manifest not found: {manifest_path}")
        print("run: cargo run -p zenith_algo --example export_validation_suite")
        return 2

    with open(manifest_path, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)

    print(f"FreeCAD {'.'.join(FreeCAD.Version()[:3])} / OpenCASCADE cross-validation")
    print(f"manifest: {manifest_path}")
    print("-" * 118)
    print(
        f"{'subject':<28} {'OCC type':<10} {'valid':<6} {'closed':<7}"
        f" {'vol cross':>11} {'area cross':>11} {'sect cross':>11}  status"
    )
    print("-" * 118)

    rows = [check_subject(subject) for subject in manifest["subjects"]]

    def fmt(value):
        if value is None:
            return "-"
        return f"{value:.3e}"

    failures = 0
    for row in rows:
        occ = row["occ"]
        status = "ok" if not row["problems"] else f"{len(row['problems'])} PROBLEM(S)"
        if row["problems"]:
            failures += 1
        print(
            f"{row['name']:<28} {str(occ.get('shape_type', '-')):<10}"
            f" {str(occ.get('is_valid', '-')):<6} {str(occ.get('is_closed', '-')):<7}"
            f" {fmt(row.get('volume_cross_error')):>11}"
            f" {fmt(row.get('area_cross_error')):>11}"
            f" {fmt(row.get('section_cross_error')):>11}  {status}"
        )

    print("-" * 118)
    for row in rows:
        for problem in row["problems"]:
            print(f"  [{row['name']}] {problem}")

    print("-" * 118)
    print(f"{len(rows) - failures} of {len(rows)} subjects agree across both kernels")

    report_path = os.path.join("target", "validation", "freecad_cross_report.json")
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(rows, handle, indent=2)
    print(f"detailed report: {report_path}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
