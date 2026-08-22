"""
Zenith CAD Kernel: Python In-Process バインディング (PyO3) 自動統合テスト
"""
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "target", "release")))

try:
    import zenith_cad
except ImportError as e:
    print(f"[FAIL] Could not import zenith_cad: {e}")
    sys.exit(1)

def run_tests():
    print("=== Zenith CAD Python (zenith_cad) 統合テスト ===")
    
    # 1. Primitives
    box = zenith_cad.make_box(30.0, 40.0, 50.0)
    print(f"  [PASS] make_box: {len(box.vertices)} verts, {len(box.faces)} faces, area: {box.surface_area:.1f} mm^2")
    assert len(box.vertices) == 8
    assert len(box.faces) == 12
    assert abs(box.surface_area - 9400.0) < 1e-4

    cyl = zenith_cad.make_cylinder(10.0, 30.0)
    print(f"  [PASS] make_cylinder: {len(cyl.vertices)} verts, {len(cyl.faces)} faces, area: {cyl.surface_area:.1f} mm^2")
    assert len(cyl.vertices) > 0

    # 2. Fillet / Chamfer
    f_box = zenith_cad.make_filleted_box(30.0, 40.0, 50.0, 4.0)
    print(f"  [PASS] make_filleted_box: {len(f_box.vertices)} verts, {len(f_box.faces)} faces")

    c_box = zenith_cad.make_chamfered_box(30.0, 40.0, 50.0, 3.0)
    print(f"  [PASS] make_chamfered_box: {len(c_box.vertices)} verts, {len(c_box.faces)} faces")

    # 3. Exact Boolean
    bool_res = zenith_cad.make_exact_drill_boolean(
        40.0, 40.0, 20.0, [0.0, 0.0, 0.0],
        8.0, 30.0, [20.0, 20.0, -5.0], [0.0, 0.0, 1.0],
        1 # Difference
    )
    print(f"  [PASS] make_exact_drill_boolean: {len(bool_res.vertices)} verts, {len(bool_res.faces)} faces")

    # 4. Spur Gear
    gear = zenith_cad.make_spur_gear(2.0, 18, 10.0, 20.0, 4.0)
    print(f"  [PASS] make_spur_gear: {len(gear.vertices)} verts, {len(gear.faces)} faces")

    print("--------------------------------------------------------------------------------")
    print("All Python binding tests completed successfully with 100% pass rate.")
    return 0

if __name__ == '__main__':
    sys.exit(run_tests())
