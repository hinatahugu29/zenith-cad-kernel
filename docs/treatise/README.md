# 解説書（Treatise）の本文

統合ドキュメント（`Zenith_CAD_Kernel_Documentation.pdf`）に載る**解説・理論の
本文**を、章ごとの Markdown として置いてあります。

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

## 生成

```bash
py tools/generate_unified_pdf.py
```

`tools/unified_docs.html` と `Zenith_CAD_Kernel_Documentation.pdf` を作ります。
どちらも生成物なので追跡していません（`.gitignore`）。`markdown` と `PySide6`
（Qt WebEngine）が要ります。

**章に取り込むリポジトリ文書の切れ目は、見出しで指定してあります**
（`TREATISE_SECTIONS`）。行番号ではありません——行番号で切っていた頃は、文書が
伸びるたびに末尾が黙って落ちていました（`HANDOVER.md` で663行）。見出しを
変えるときは、`tools/generate_unified_pdf.py` の切れ目も一緒に直してください。
見つからなければスクリプトが止まります。
