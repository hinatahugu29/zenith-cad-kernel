"""**配る包みを、Blender の中で読ませる**（4-433）。

なぜ要るのか
------------
4-400 が**3 つの段**を分けました——

    カーネルにある → Python から呼べる → 包みに入っている

**4 段目があります**——**Blender の中で読める**。
**そこは、一度も測っていませんでした。**

**`py tools/check_python_surface.py` は、こちらの Python で読みます。**
**Blender は自前の Python を積んでいます**（版によって 3.10 〜 3.13）。
**`abi3-py310` なので読めるはず**ですが、**「はず」で通してきました。**

使い方
------
    py tools/build_pyd.py
    "C:/Program Files/Blender Foundation/Blender 4.4/blender.exe" \
        --background --factory-startup \
        --python tools/blender_load_addon.py

**`--factory-startup`** を付けます——**利用者の設定やアドオンを
持ち込まない**ためです。

読み方
------
**読めただけでは足りません。** **閉じた式と突き合わせます**——
**読めても答えが違えば、配る意味がありません。**

**⚠ これはアドオンとしての試験ではありません**（4-426）。
**この包みには `bl_info` も `__init__.py` も無く**、**中身は `.pyd`
1 つ**です。**Blender の「アドオンをインストール」では入りません。**
**ここで見ているのは「Blender の Python から import できるか」**だけ。

**1 つでも食い違えば、非ゼロで終わります。**
"""

import math
import os
import sys


def main():
    root = os.environ.get("ZENITH_ROOT")
    if not root:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    package = os.path.join(root, "blender_addon", "H-CAD_V_1_0_0")

    print()
    print("配る包みを、Blender の中で読ませる（4-433）")
    print("Blender の Python %s" % sys.version.split()[0])
    print("包み %s" % package)
    print()

    if not os.path.isdir(package):
        print("**包みがありません。** py tools/build_pyd.py")
        return 1

    sys.path.insert(0, package)
    try:
        import zenith_cad as z
    except Exception as exc:
        print("**読めません**: %r" % (exc,))
        return 1

    # **読めただけでは足りません。** 閉じた式と突き合わせます。
    checks = [
        ("箱 10x20x30", lambda: z.Solid.box(10.0, 20.0, 30.0).volume, 6000.0, 1e-12),
        ("円柱 r5 h12", lambda: z.Solid.cylinder(5.0, 12.0).volume,
         math.pi * 25.0 * 12.0, 1e-9),
        ("球 r7", lambda: z.Solid.sphere(7.0).volume,
         4.0 / 3.0 * math.pi * 343.0, 1e-9),
        ("箱の差（厳密）", lambda: z.Solid.box(20.0, 20.0, 20.0)
         .difference(z.Solid.box(10.0, 10.0, 30.0).translated(5.0, 5.0, -5.0)).volume,
         6000.0, 1e-9),
        ("回した箱（体積は変わらない）",
         lambda: z.Solid.box(10.0, 20.0, 30.0)
         .rotated([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 37.0).volume, 6000.0, 1e-9),
        ("表示メッシュの三角形",
         lambda: float(z.Solid.box(10.0, 20.0, 30.0).tessellate(8, 8).num_faces),
         12.0, 1e-12),
    ]

    print("%-30s%18s%18s%12s  %s" % ("測るもの", "閉じた式", "測った値", "相対差", "結果"))
    print("-" * 92)
    wrong = 0
    for name, call, expected, allowance in checks:
        try:
            got = call()
        except Exception as exc:
            wrong += 1
            print("%-30s%18.6f%18s%12s  **断られました**: %s"
                  % (name, expected, "-", "-", str(exc)[:24]))
            continue
        residual = abs(got - expected) / max(abs(expected), 1.0)
        if residual > allowance:
            wrong += 1
        print("%-30s%18.6f%18.6f%12.3e  %s"
              % (name, expected, got, residual,
                 "ok" if residual <= allowance else "**ちがう**"))
    print("-" * 92)
    print()

    names = [n for n in dir(z) if not n.startswith("_")]
    print("口の数 %d" % len(names))
    print("誤答 %d 件" % wrong)
    return 1 if wrong else 0


if __name__ == "__main__":
    sys.exit(main())
