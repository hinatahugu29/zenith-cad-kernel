"""**「完了」と書いてある行の指し先が、本当にあるか**（4-440）。

なぜ要るのか
------------
**2026/09/12 に、実体の無い「✅ 完了」が見つかりました**（4-439）
——`ROADMAP.md` が 4 か所で **`blender_addon::zenith_patch_addon.py`**
を指していましたが、**作業ツリーにも git の履歴にも、利用者の Blender
にもありません**でした。

**同じ形が他にもないか**を、ここで数えます。

**指しているのは、たいていファイルではなく Rust の名前**です
（実測: 285 個のうち 281 個は拡張子を持ちません）。**当てる先を
間違えると、空振りを「緑」と読みます**——**1 度そうなりました**
（「名指しされたファイル 0 件が実在 / 全部あります」）。

使い方
------
    py tools/check_doc_claims.py

読み方
------
**分母を必ず見てください。** **「0 件中 0 件」は、何も言っていません。**

**見当たらない名前が 1 つでもあれば、非ゼロで終わります。**

**⚠ これは「名札が合っているか」しか見ません。** **実体があって名札が
違うだけのこと**もあります（4-440 の `zenith_topo::validation` が
そうでした——**検証そのものは `solid.rs` などにあります**）。
**赤が出たら、まず「実体はあるか」を見てください。**
"""

import io
import os
import re
import sys

# **コンソールが cp932 でも落ちないようにします**（4-389 と同じ穴）。
# **全角ダッシュ（U+2014）で `UnicodeEncodeError` を出して止まります**
# ——**止まると、そこまでの結果ごと失われます。** **実際に 1 度
# 落ちました**（4-440 でこの道具を書いた直後）。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

DOCS = ("ROADMAP.md", "KERNEL_SPECS.md", "KERNEL_INVENTORY_SPECS.md")

# **説明のある「無い名前」**。**理由を書けないなら、それは欠陥です。**
EXPECTED_MISSING = {
    # （いまは空です。4-440 で 1 つ直したので、残っていません。）
}

DECLARATION = re.compile(
    r'\b(?:fn|struct|enum|trait|type|const|mod|static|union)\s+([A-Za-z_][A-Za-z0-9_]*)'
)


def collect_names(root):
    """**`crates` 以下の Rust の名前を、1 度だけ集めます。**

    **ディレクトリ名（クレート名・モジュール名）も入れます**——
    **`zenith_geom::nurbs_curve` の前半は宣言ではありません**。
    **入れ忘れると、クレート名が全部「無い」と出ます**（実測: 225 件）。
    """
    names = set()
    for dirpath, _dirs, files in os.walk(os.path.join(root, "crates")):
        if "target" in dirpath:
            continue
        names.add(os.path.basename(dirpath))
        for name in files:
            if not name.endswith(".rs"):
                continue
            names.add(name[:-3])
            path = os.path.join(dirpath, name)
            with io.open(path, encoding="utf-8", errors="replace") as handle:
                for line in handle:
                    for found in DECLARATION.findall(line):
                        names.add(found)
    return names


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)
    names = collect_names(root)

    checked = 0
    missing = []
    for doc in DOCS:
        if not os.path.exists(doc):
            print("**%s がありません。**" % doc)
            return 1
        for number, line in enumerate(io.open(doc, encoding="utf-8"), 1):
            if "完了" not in line and "✅" not in line:
                continue
            for token in re.findall(r'`([^`]+)`', line):
                token = token.strip()
                if "::" not in token:
                    continue
                # **日本語や括弧の混じったものは、名前ではありません。**
                if re.search(r'[ （）、。]', token):
                    continue
                for part in token.split("::"):
                    part = part.split("(")[0].strip()
                    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part):
                        continue
                    checked += 1
                    if part not in names:
                        missing.append((doc, number, token, part))

    print("「完了」の行が指す名前を当てる（4-440）")
    print()
    print("crates から集めた名前 %d 個" % len(names))
    print("**当てた名前 %d 件**" % checked)
    print()

    if checked == 0:
        # **0 件を「緑」と読ませません**（4-410 の落とし穴）。
        print("**1 件も当てていません。** 拾い方が壊れています。")
        return 1

    unexplained = [row for row in missing if row[3] not in EXPECTED_MISSING]
    if unexplained:
        print("**crates に見当たらない名前**")
        print("-" * 76)
        seen = set()
        for doc, number, token, part in unexplained:
            if part in seen:
                continue
            seen.add(part)
            print("  %-30s（%s）  %s:%d" % (part, token, doc, number))
        print("-" * 76)
        print()
        print("のべ %d 件 / 種類 %d 個" % (len(unexplained), len(seen)))
        print()
        print("**まず「実体はあるか」を見てください**——**名札が違うだけ**の")
        print("ことがあります（4-440 の `zenith_topo::validation` がそうでした）。")
        return 1

    print("**名指しされた名前は、全部 crates にあります。**")
    return 0


if __name__ == "__main__":
    sys.exit(main())
