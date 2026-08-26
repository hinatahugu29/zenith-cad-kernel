"""Python から呼んだときに、返ってくる形が正しいか。

`zenith_cad`（PyO3 のインプロセス束縛）の**メッシュを返す古い口**を確かめる。
B-Rep ハンドルを返す新しい口は `tools/verify_solid_api.py` の担当。

    cargo build --release -p zenith_py
    py tools/build_pyd.py          # zenith_cad.pyd を target/release へ置く
    py tools/verify_python_binding.py

**以前ここは、呼んで頂点数を印字するだけだった。** フィレット・面取り・
ブーリアン・歯車の4つには表明が1つも無く、返ってきた形が何であっても
「[PASS]」と印字して「100% pass rate」で終わっていた。返ってきた数を
閉じた式か、閉じた式から引ける範囲に当てる。

食い違いがあれば非ゼロで終わる。
"""

import math
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "target", "release")))

try:
    import zenith_cad
except ImportError as error:
    print("[FAIL] could not import zenith_cad: {}".format(error))
    print("       build it first:  cargo build --release -p zenith_py && py tools/build_pyd.py")
    sys.exit(1)

PROBLEMS = []


def check(label, condition, detail):
    if condition:
        print("  ok    {:<34} {}".format(label, detail))
    else:
        print("  WRONG {:<34} {}".format(label, detail))
        PROBLEMS.append("{}: {}".format(label, detail))


def close(value, expected, band):
    if expected == 0.0:
        return abs(value) <= band
    return abs(value - expected) / abs(expected) <= band


def main():
    print("=== zenith_cad (mesh-returning binding) ===")

    # 1. 直方体 — 体積も表面積も厳密に出るはず。
    box = zenith_cad.make_box(30.0, 40.0, 50.0)
    check(
        "box vertices and triangles",
        len(box.vertices) == 8 and len(box.faces) == 12,
        "{} vertices, {} triangles (expected 8 and 12)".format(len(box.vertices), len(box.faces)),
    )
    check(
        "box surface area",
        close(box.surface_area, 9400.0, 1e-12),
        "{:.6f} against 9400".format(box.surface_area),
    )
    check(
        "box volume",
        close(box.volume, 60000.0, 1e-12),
        "{:.6f} against 60000".format(box.volume),
    )

    # 2. 円柱 — メッシュは内接するので少しだけ小さい。**下からも締める。**
    cylinder = zenith_cad.make_cylinder(10.0, 30.0)
    exact = math.pi * 100.0 * 30.0
    drift = (cylinder.volume - exact) / exact
    check(
        "cylinder volume",
        -2.0e-2 <= drift <= 0.0,
        "{:.6f} against {:.6f} (drift {:.2e}; a tessellated cylinder is inscribed, so it may only "
        "fall short)".format(cylinder.volume, exact, drift),
    )

    # 3. フィレット・面取り — 角を削るのだから、体積は必ず減る。削れる量は
    #    12本の稜ぶんが上限で、これを超えていたら形が違う。
    for label, solid, setback in (
        ("filleted box", zenith_cad.make_filleted_box(30.0, 40.0, 50.0, 4.0), 4.0),
        ("chamfered box", zenith_cad.make_chamfered_box(30.0, 40.0, 50.0, 3.0), 3.0),
    ):
        # 稜の総長 4*(30+40+50) から、削れる量の上限は setback^2 * 総長。
        ceiling = setback * setback * 4.0 * (30.0 + 40.0 + 50.0)
        removed = 60000.0 - solid.volume
        check(
            "{} removes material".format(label),
            0.0 < removed < ceiling,
            "{:.6f} removed, must be between 0 and {:.6f}".format(removed, ceiling),
        )

    # 4. 厳密ブーリアン — 40x40x20 の板を半径8で貫通。閉じた式で書ける。
    drilled = zenith_cad.make_exact_drill_boolean(
        40.0, 40.0, 20.0, [0.0, 0.0, 0.0],
        8.0, 30.0, [20.0, 20.0, -5.0], [0.0, 0.0, 1.0],
        1,  # Difference
    )
    exact = 40.0 * 40.0 * 20.0 - math.pi * 64.0 * 20.0
    drift = (drilled.volume - exact) / exact
    check(
        "drilled plate volume",
        abs(drift) <= 2.0e-2,
        "{:.6f} against {:.6f} (drift {:.2e})".format(drilled.volume, exact, drift),
    )

    # 5. 平歯車 — 歯形の体積に初等的な閉じた式は無いが、**歯底円の環と歯先円の
    #    環のあいだ**には必ず入る。
    #
    #    引数は `(module, teeth, 圧力角[deg], 厚み, 軸穴半径)`。**最初これを
    #    (module, teeth, 厚み, 圧力角, 軸穴) と読み違えて、厚みを 10 として
    #    上限を出し、通るはずの 19686.53 を「範囲外」と報告した。** カーネル
    #    ではなくこちらの読み違いだった。
    #
    #    m=2, z=18 なので基準円直径 36、歯先半径 20、歯底半径 15.5、
    #    厚み 20、軸穴半径 4。
    gear = zenith_cad.make_spur_gear(2.0, 18, 10.0, 20.0, 4.0)
    thickness = 20.0
    low = math.pi * (15.5 ** 2 - 4.0 ** 2) * thickness
    high = math.pi * (20.0 ** 2 - 4.0 ** 2) * thickness
    check(
        "spur gear volume",
        low < gear.volume < high,
        "{:.6f} must lie between the root ring {:.6f} and the tip ring {:.6f}".format(
            gear.volume, low, high
        ),
    )

    print("-" * 78)
    if PROBLEMS:
        print("{} check(s) disagreed:".format(len(PROBLEMS)))
        for problem in PROBLEMS:
            print("  " + problem)
        return 1
    print("every check agreed with the closed form or the bound it has to sit inside")
    return 0


if __name__ == "__main__":
    sys.exit(main())
