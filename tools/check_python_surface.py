"""**本物の Python から、拡張モジュールを触る**（4-402）。

なぜ要るのか
------------
`crates/zenith_py/tests/binding_gate_test.rs` は、**Rust の関数として**
呼んでいます（`Python::with_gil` を通らないので、インタプリタ無しで
測れます。4-198）。

**それは「Python から使える」ことの証明にはなりません。**

実際、**`volume` は Python では getter** でした。Rust では
`solid.volume()` ですが、**Python では `solid.volume`** です。
`solid.volume()` と書くと `TypeError: 'float' object is not callable`
になります。**Rust の試験では、この違いは 1 度も見えません。**

**4-400 で「カーネルにあると Blender から使えるは別」と書いた**その差が、
**試験の側にも**ありました。

使い方
------
    cargo build --release -p zenith_py
    py tools/check_python_surface.py

**`abi3-py310` なので、3.10 以降ならどの Python でも入ります。**
`--python` で使うインタプリタを指せます。

読み方
------
**閉じた式と突き合わせます。** 断るべきものが断られるかも見ます。
**1 つでも食い違えば非ゼロで終わります。**
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_probe_source():
    """子プロセスで走らせる本体。**別の Python で回せるように**します。"""
    return r'''
import json, math, sys

import zenith_cad as z

def value_of(holder, name):
    """**getter とメソッドの両方を受けます。**

    Python 側が getter か method かは、`#[getter]` の有無で決まります。
    **ここで固定してしまうと、変わったときに気づけません**ので、
    両方を受けて、**どちらだったかを印字**します。
    """
    attribute = getattr(holder, name)
    if callable(attribute):
        return attribute(), "method"
    return attribute, "getter"

rows = []
S = z.Solid

rect = json.dumps({"points": [[0, 0], [40, 0], [40, 30], [0, 30]],
                   "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]})
volume, kind = value_of(S.from_sketch_extruded(rect, 7.0), "volume")
rows.append(("スケッチを押し出す", volume, 40 * 30 * 7.0))

hole = json.dumps({
    "points": [[0, 0], [40, 0], [40, 30], [0, 30], [20, 15],
               [25, 15], [20, 20], [15, 15], [20, 10]],
    "lines": [[0, 1], [1, 2], [2, 3], [3, 0]],
    "arcs": [{"centre": 4, "start": 5, "end": 6},
             {"centre": 4, "start": 6, "end": 7},
             {"centre": 4, "start": 7, "end": 8},
             {"centre": 4, "start": 8, "end": 5}]})
rows.append(("穴のあるスケッチを押し出す",
             value_of(S.from_sketch_extruded(hole, 7.0), "volume")[0],
             (40 * 30 - math.pi * 25) * 7.0))

ring = json.dumps({"points": [[10, 0], [14, 0], [14, 6], [10, 6]],
                   "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]})
rows.append(("スケッチを回す（パップス）",
             value_of(S.from_sketch_revolved(ring, 0.0, 0.0, 0.0, 1.0), "volume")[0],
             2 * math.pi * 12.0 * 24.0))

rows.append(("箱の体積", value_of(S.box(10.0, 20.0, 30.0), "volume")[0], 6000.0))

print("本物の Python から拡張モジュールを触る（4-402）")
print()
print("Python %s" % sys.version.split()[0])
print("`volume` は Python では **%s**" % kind)
print()
print("%-34s%>18s%>18s%>12s  %s".replace(">", "") % ("測るもの", "閉じた式", "測った値", "相対差", "結果"))
print("-" * 100)

wrong = 0
for name, measured, expected in rows:
    residual = abs(measured - expected) / max(abs(expected), 1.0)
    ok = residual <= 1e-9
    if not ok:
        wrong += 1
    print("%-34s%18.9f%18.9f%12.3e  %s" % (name, expected, measured, residual, "ok" if ok else "**ちがう**"))

print("-" * 100)
print()

# **断るべきものが断られるか。**
refusals = [
    ("点の番号が範囲の外",
     lambda: S.from_sketch_extruded(json.dumps(
         {"points": [[0, 0], [10, 0], [10, 10]], "lines": [[0, 1], [1, 9], [9, 0]]}), 5.0)),
    ("線も弧も無い",
     lambda: S.from_sketch_extruded(json.dumps(
         {"points": [[0, 0], [10, 0], [10, 10]]}), 5.0)),
    ("軸をまたぐ輪を回す",
     lambda: S.from_sketch_revolved(json.dumps(
         {"points": [[-5, 0], [5, 0], [5, 6], [-5, 6]],
          "lines": [[0, 1], [1, 2], [2, 3], [3, 0]]}), 0.0, 0.0, 0.0, 1.0)),
    ("拘束の種類の綴り違い",
     lambda: z.solve_2d_sketch("[[0,0],[3,4],[9,1]]",
                               '[{"type": "horizontol", "p1": 0, "p2": 1}]')),
    ("distance に value が無い",
     lambda: z.solve_2d_sketch("[[0,0],[3,4],[9,1]]",
                               '[{"type": "distance", "p1": 0, "p2": 1}]')),
]
missed = 0
for name, call in refusals:
    try:
        call()
        print("%-34s **通ってしまいました**" % name)
        missed += 1
    except Exception as error:
        print("%-34s 断りました: %s" % (name, str(error)[:52]))

print()
print("誤答 %d 件、断りそこね %d 件" % (wrong, missed))
sys.exit(1 if (wrong or missed) else 0)
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()

    built = os.path.join(ROOT, "target", "release", "zenith_cad.dll")
    if not os.path.exists(built):
        print("target/release/zenith_cad.dll がありません。")
        print("  cargo build --release -p zenith_py")
        return 1

    # **`.pyd` へ写します。** Python は `.dll` を拡張モジュールとして
    # 探しません。**元のファイルは動かしません**——`cargo` の出力です。
    workspace = tempfile.mkdtemp(prefix="zenith_pysurface_")
    try:
        shutil.copy2(built, os.path.join(workspace, "zenith_cad.pyd"))
        probe = os.path.join(workspace, "probe.py")
        with open(probe, "w", encoding="utf-8") as handle:
            handle.write(build_probe_source())

        environment = dict(os.environ)
        environment["PYTHONPATH"] = workspace
        environment["PYTHONIOENCODING"] = "utf-8"
        result = subprocess.run(
            [args.python, "-X", "utf8", probe],
            env=environment,
            cwd=workspace,
        )
        return result.returncode
    finally:
        shutil.rmtree(workspace, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
