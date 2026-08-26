# 解説書（Treatise）の本文

**解説・理論の本文**を、章ごとの Markdown として置いてあります。**そのまま
読むためのもの**です（2026年8月27日に PDF / HTML の生成をやめました。経緯は
下）。

| ファイル | 載る場所 |
| :--- | :--- |
| [`00-prologue.md`](00-prologue.md) | 巻頭言 |
| [`01-brep-and-nurbs.md`](01-brep-and-nurbs.md) | 第1章 基礎理論 |
| [`02-freeform-and-constraints.md`](02-freeform-and-constraints.md) | 第2章 基礎理論 |
| [`03-boolean-and-ssi.md`](03-boolean-and-ssi.md) | 第3章 基礎理論 |
| [`04-tessellation-and-validation.md`](04-tessellation-and-validation.md) | 第4章 基礎理論 |
| [`05-occt-replacement.md`](05-occt-replacement.md) | 第5章 基礎理論 |
| [`06-step-interchange.md`](06-step-interchange.md) | 第6章 基礎理論 |
| [`a1-glossary.md`](a1-glossary.md) | 付録 A 用語集 |
| [`a2-references.md`](a2-references.md) | 付録 B 常設プローブ一覧 |

## なぜここにあるのか

**2026年8月27日まで、この本文は `tools/generate_unified_pdf.py` の中に
直書きされていました。** 生成スクリプトの中に読み物が埋まっていると、

- 他の文書（`KERNEL_SPECS.md` や `HANDOVER.md`）と食い違っても気づけない
- 差分が「スクリプトの変更」に見えるので、内容のレビューに掛からない
- PDF を作らないと読めない

という状態になります。本文は Markdown として置き、スクリプトは組み立てだけを
します。

## 書くときの注意

**この本文は実測ではありません。** 一般的な CAD 理論の解説と、Zenith の設計
意図を述べたものです。**数字を書くなら、その出どころを併記してください。**
実測値の置き場は決まっています。

- 実測の一覧と経緯: [`../../HANDOVER.md`](../../HANDOVER.md)
- 数字を自分で確かめる手順: [`../../VERIFICATION_PLAYBOOK.md`](../../VERIFICATION_PLAYBOOK.md)
- 機能の棚卸し: [`../../KERNEL_SPECS.md`](../../KERNEL_SPECS.md)

ここに数字を写すと、**写した先が古くなったことに誰も気づけません。** 参照を
書くほうが安全です。

## PDF / HTML の生成はやめました（2026/08/27）

`tools/generate_unified_pdf.py` は削除しました。**本文がここに Markdown として
あるので、束ねた出力を持つ意味がありません。** 生成物は追跡しておらず、
描画段はこの環境で繰り返し実行すると返ってこない問題も抱えていました（4-114）。
束ねたものが要るときは、この節を消して作り直してください。
