"""Measures whether exported surfaces are the exact quadrics they claim to be.

The kernel's own volume for a sphere and a torus matches the closed form, but
OpenCASCADE reading the exported STEP gets a different number. Either the
surface written out is not the surface the kernel integrated, or one of the two
volume integrations is wrong. Sampling the surface OCC actually loaded settles
it: a true sphere of radius r has every surface point at distance r from the
centre.
"""

import math
import os
import sys

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


def sample_face(face, samples=24):
    surface = face.Surface
    u0, u1, v0, v1 = face.ParameterRange
    points = []
    for i in range(samples + 1):
        for j in range(samples + 1):
            u = u0 + (u1 - u0) * i / samples
            v = v0 + (v1 - v0) * j / samples
            try:
                points.append(surface.value(u, v))
            except Exception:
                continue
    return points


def check_sphere(path, radius):
    shape = Part.read(path)
    print(f"=== {os.path.basename(path)} (expected sphere r={radius})")
    print(f"    OCC volume      : {shape.Volume:.6f}")
    print(f"    analytic volume : {4.0 / 3.0 * math.pi * radius ** 3:.6f}")
    print(f"    OCC area        : {shape.Area:.6f}")
    print(f"    analytic area   : {4.0 * math.pi * radius ** 2:.6f}")

    worst = 0.0
    for face in shape.Faces:
        for point in sample_face(face):
            worst = max(worst, abs(point.Length - radius))
    print(f"    worst |P|-r     : {worst:.9e}")

    reference = Part.makeSphere(radius)
    print(f"    OCC native sphere volume: {reference.Volume:.6f}")
    print()


def check_torus(path, major, minor):
    shape = Part.read(path)
    print(f"=== {os.path.basename(path)} (expected torus R={major} r={minor})")
    print(f"    OCC volume      : {shape.Volume:.6f}")
    print(f"    analytic volume : {2.0 * math.pi ** 2 * major * minor ** 2:.6f}")

    worst = 0.0
    for face in shape.Faces:
        for point in sample_face(face):
            axial = math.hypot(point.x, point.y)
            distance = math.hypot(axial - major, point.z)
            worst = max(worst, abs(distance - minor))
    print(f"    worst tube error: {worst:.9e}")

    reference = Part.makeTorus(major, minor)
    print(f"    OCC native torus volume : {reference.Volume:.6f}")
    print()


def check_cylinder_caps(path, radius, height):
    shape = Part.read(path)
    print(f"=== {os.path.basename(path)} (expected cylinder r={radius} h={height})")
    for index, face in enumerate(shape.Faces):
        surface = face.Surface
        kind = type(surface).__name__
        edge_summary = []
        for edge in face.Edges:
            a = edge.Vertexes[0].Point
            b = edge.Vertexes[-1].Point
            edge_summary.append(
                f"({a.x:.2f},{a.y:.2f},{a.z:.2f})->({b.x:.2f},{b.y:.2f},{b.z:.2f})"
                f" len={edge.Length:.3f}"
            )
        print(f"    face {index}: {kind} area={face.Area:.4f}")
        for summary in edge_summary:
            print(f"        edge {summary}")
    print()


def main():
    directory = os.path.join("target", "validation")
    check_sphere(os.path.join(directory, "sphere_r10.step"), 10.0)
    check_torus(os.path.join(directory, "torus_R12_r4.step"), 12.0, 4.0)
    check_cylinder_caps(os.path.join(directory, "cylinder_r10_h40.step"), 10.0, 40.0)


if __name__ == "__main__":
    main()
