"""
Zenith CAD Kernel - Build & Packaging Automation Tool
Usage: py tools/build_pyd.py [--release]
"""

import os
import sys
import shutil
import subprocess
import zipfile
from pathlib import Path

# UTF-8 出力の強制
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8')

def run_cmd(cmd, cwd=None):
    print(f"[RUN] {cmd}")
    res = subprocess.run(cmd, shell=True, cwd=cwd)
    if res.returncode != 0:
        print(f"[ERROR] Command failed with return code {res.returncode}")
        sys.exit(res.returncode)

def main():
    root_dir = Path(__file__).resolve().parent.parent
    target_dir = root_dir / "target" / "release"
    addon_pkg_dir = root_dir / "blender_addon" / "H-CAD_V_1_0_0"
    zip_path = root_dir / "blender_addon" / "H-CAD_V_1_0_0.zip"

    print("==================================================")
    print("[Zenith CAD Kernel] Build & Package Pipeline")
    print("==================================================")

    # 1. Cargo Release Build
    os.environ["PYO3_PYTHON"] = sys.executable
    print(f"🐍 Using Python: {sys.executable}")
    run_cmd(f'cargo build --release -p zenith_py', cwd=root_dir)

    # 2. Copy DLL to PYD in addon directory
    dll_path = target_dir / "zenith_cad.dll"
    if not dll_path.exists():
        # Fallback check for linux .so or mac .dylib
        dll_path = target_dir / "libzenith_cad.so"
        if not dll_path.exists():
            dll_path = target_dir / "libzenith_cad.dylib"

    if not dll_path.exists():
        print(f"❌ Compiled binary not found in {target_dir}")
        sys.exit(1)

    dest_pyd = addon_pkg_dir / "zenith_cad.pyd"
    shutil.copy2(dll_path, dest_pyd)
    print(f"[OK] Copied {dll_path.name} -> {dest_pyd} ({dest_pyd.stat().st_size / 1024:.1f} KB)")

    # 3. Create / Update Addon ZIP Archive
    print(f"[ZIP] Generating ZIP archive: {zip_path.name} ...")
    if zip_path.exists():
        zip_path.unlink()

    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(addon_pkg_dir):
            for file in files:
                if file.endswith('.pyc') or '__pycache__' in root:
                    continue
                abs_f = Path(root) / file
                rel_f = abs_f.relative_to(root_dir / "blender_addon")
                zf.write(abs_f, rel_f)
    print(f"[OK] Created Addon ZIP: {zip_path} ({zip_path.stat().st_size / 1024:.1f} KB)")

    # 4. Verify Import and Functions
    print("\n[VERIFY] Checking Python Extension Integration...")
    sys.path.insert(0, str(addon_pkg_dir))
    try:
        import zenith_cad
        funcs = [f for f in dir(zenith_cad) if not f.startswith('_')]
        print(f"[OK] zenith_cad successfully loaded! Exported symbols ({len(funcs)}): {funcs}")

        # Quick Smoke Test
        mesh = zenith_cad.make_box(10.0, 20.0, 30.0)
        print(f"   [Test] Box mesh: {mesh.num_vertices} vertices, {mesh.num_faces} triangles, Volume: {mesh.volume} mm^3")
        assert abs(mesh.volume - 6000.0) < 1.0, "Volume check failed"

        # Shader Payload Test
        payload_json = zenith_cad.get_primitive_shader_payload("cylinder", 5.0, 20.0, 0.0, 0.0)
        print(f"   [Test] Cylinder Shader Payload: {payload_json[:60]}...")

        # 2D Sketch Solver Test
        pts_in = "[[0.0, 0.0], [9.8, 0.2]]"
        cons_in = '[{"type": "horizontal", "p1": 0, "p2": 1}, {"type": "distance", "p1": 0, "p2": 1, "value": 15.0}]'
        pts_out = zenith_cad.solve_2d_sketch(pts_in, cons_in)
        print(f"   [Test] 2D Sketch Solver result: {pts_out}")

        print("\n[SUCCESS] All Pipeline Steps Completed Successfully!")
    except Exception as e:
        print(f"[ERROR] Verification failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
