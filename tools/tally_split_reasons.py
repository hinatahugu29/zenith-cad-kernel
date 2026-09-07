"""割れなかった交線の断り文を、種類ごとに数える（4-374）。

なぜ要るのか
------------
`linkrods.step` が断られる理由を追うとき、**断り文の内訳**が要ります。
2026/09/07 の実測では、これで**筆頭が入れ替わりました**——
`Only recognized cylinder-side NURBS patches can be split`（126）が
`Split edge start does not lie on the outer boundary`（117）より多い、と。

**それまでは、その場限りのワンライナーで数えていました。** 数え方を
残していないと、**次に数えた人と比べられません**（4-370 では、
**160 文字で切れた文字列**で数えた古い表と食い違いました）。

使い方
------
    ZENITH_READ_CUT_ONLY=linkrods ZENITH_SPLIT_WHY=1 \
      cargo run --release -p zenith_algo --example read_and_cut_probe > out.txt 2>&1
    py tools/tally_split_reasons.py out.txt

読み方
------
**3 演算ののべ**です。1 本の交線に対して**面片ごとに断り文が出る**ので、
**そのまま「何本」とは読めません**。**割合として読んでください。**

**数字が桁で動いていたら、まず「断り文が切れていないか」を疑ってください**
——2026/09/07 まで、診断は理由を **160 文字で切って**いました（4-369）。
"""

import collections
import io
import re
import sys


def tally(path, deciding_only=True):
    """`split nothing` の直後にぶら下がる断り文を数える。

    `deciding_only` なら**最後の段の理由だけ**を数えます。前の段の理由は
    「次の段が引き取った」だけかもしれないので、割れなかった理由では
    ありません（4-383）。
    """
    lines = io.open(path, encoding="utf-8", errors="replace").read().splitlines()
    reasons = collections.Counter()
    counted = 0
    for index, line in enumerate(lines):
        if "split nothing" not in line:
            continue
        # 断り文は 2 字下げでぶら下がります。次の見出しが来たら終わりです。
        for follow in lines[index + 1 : index + 8]:
            if not follow.startswith("  ") or follow.startswith("  BATCH"):
                break
            if follow.lstrip().startswith(("SPLITWHY", "CHAINWHY", "CYLWHY")):
                break
            counted += 1
            parts = follow.strip().split("; ")
            if deciding_only:
                parts = parts[-1:]
            for part in parts:
                # 数字は丸めて、同じ種類をまとめます。
                part = re.sub(r"[0-9]+\.[0-9]+e?[+-]?[0-9]*", "N", part).strip()
                if part:
                    reasons[part] += 1
    return counted, reasons


def main():
    argv = [a for a in sys.argv[1:] if a != "--all"]
    deciding_only = "--all" not in sys.argv[1:]
    if len(argv) != 1:
        print("使い方: py tools/tally_split_reasons.py [--all] <read_and_cut_probe の出力>")
        return 1
    counted, reasons = tally(argv[0], deciding_only)
    if not reasons:
        print("断り文が見つかりません。ZENITH_SPLIT_WHY=1 を付けて回しましたか。")
        return 1
    scope = "決め手だけ" if deciding_only else "**全部の段**（決め手ではありません）"
    print(f"断り文の行 {counted} 本、種類 {len(reasons)}（3 演算ののべ。{scope}）")
    print()
    print(f"| {'断り文':<64} | のべ |")
    print(f"| {'-' * 64} | ---: |")
    for reason, count in reasons.most_common():
        print(f"| {reason[:64]:<64} | {count} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
