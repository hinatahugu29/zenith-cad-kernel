# Zenith CAD Kernel — 作業開始案内

このリポジトリを初めて扱う人・AIは、コードを変更する前に次の順で読んでください。

1. [`HANDOVER.md`](HANDOVER.md) — 現在地点、作業ブランチ、次に着手する候補
   （**次の方針は 9-H**。段取りと数値目標 **H1〜H8** がここにあります。
   **H1〜H7 は達成済み**で、いま追うのは **H8 だけ**です。H6 は 2026/09/02 に
   達成しました——射影の収束判定の**単位が合っていなかった**のが原因で、
   `scale_sweep_probe` の門を **0.005 まで**下げて破れ 0 です。4-259）
2. [`VERIFICATION_PLAYBOOK.md`](VERIFICATION_PLAYBOOK.md) — 主張を再測定する手順
3. [`KERNEL_SPECS.md`](KERNEL_SPECS.md) — 現在の仕様と実装範囲
4. [`KERNEL_INVENTORY_SPECS.md`](KERNEL_INVENTORY_SPECS.md) — 機能一覧と制限
5. 必要な課題に対応する `reference/`、Rustコード、その他の設計文書

読み物（理論の解説、用語集、プローブ一覧）は [`docs/treatise/`](docs/treatise/)
にあります。**実測値の正はあくまで `HANDOVER.md` で、解説側の数字は書いた
時点のものです。**

役目を終えた文書は [`docs/archive/`](docs/archive/) にあります（OCCT 置き換えの
段取りなど）。**そこの数字を、いまの実測として引かないでください。**

最初に、文書内のスナップショットより手元のGit状態を優先して確認します。

```bash
git status --short --branch
git log --oneline --decorate -12
git diff main...HEAD --stat
```

欠陥を探すとき、いまいちばん当たる手は**総当たりで掃くこと**です
（置き方を1件ずつ足すより強い、と実測で分かっています）。

```bash
# 恒等式で掃く（|A∪B|+|A∩B| = |A|+|B|、|A\B|+|A∩B| = |A|）
cargo run --release -p zenith_algo --example foreign_cross_pair_probe
```

**掃く軸は4本目まで来ています**——「置き方」（`contact_placement_probe`）、
「大きさの桁」（`scale_sweep_probe`）、「自分の出力を入力に戻す」
（`rechained_boolean_probe`）は枯れました。**4本目の「読んだ立体（STEP）を
切る」は着手済み**です（9-H の H8。検体は `reference/OCCT`、掃き出しは
`read_and_cut_probe`）。

2026/09/02 の時点で、**OCCT が配る実物の STEP が2つとも読めて切れます**
（`screw.step`、`linkrods.step`）。**ただし目標の「恒等式の破れ 0」には
届いていません**——メッシュ非多様体が出ること、`linkrods.step` の演算が
返らないこと（**射影の 9 割が p-curve の導出**。4-271）が残っています。

**閉じた式が無くても、恒等式なら測れます。** 2026/08/28 に直した誤答3件は
**3件とも恒等式でしか見えませんでした**——面は閉じ、非多様体でもなく、
内外判定も通ります（`HANDOVER.md` の 4-143〜4-145）。**形を見ても分かりません。**

作業時の原則:

- いつでも手戻りできるよう、作業ブランチを使い、意味のまとまりごとにコミットする。
- 長時間になりそうなビルド・テスト・調査は、着手前に所要時間を見積もる。
- 作業後は、実装だけでなく引継書・仕様書・検証記録も現在地点へ更新する。
- 過去の測定や判断は履歴として残し、削除は現状理解に必要な最小限にする。
- 文書の主張をそのまま信じず、`VERIFICATION_PLAYBOOK.md`の手順とCIで再確認する。

現在の具体的なブランチ、コミット、未解決事項は
[`HANDOVER.md`](HANDOVER.md)の「0. 作業を引き継ぐときの現在地点」を正とします。

**次に何をするかは [`HANDOVER.md`](HANDOVER.md) の 9-H が正です**（2026/08/30
に置き直しました）。[`ROADMAP.md`](ROADMAP.md) の「今後の実装優先順位」は
その要約です。

---

## ライセンス

**Mozilla Public License 2.0**（[`LICENSE`](LICENSE)）。

**ファイル単位の copyleft** です。

- このリポジトリのファイルを**改変したら、その部分は公開**してください
- **自分のコードと組み合わせて、閉じた製品にするのは自由**です

OCCT が LGPL で広く採用されているのと同じ狙いで選びました——このカーネルは
その代替を目標に置いているので（[`HANDOVER.md`](HANDOVER.md) の 9-0）、
**組み込める**ことが要ります。

**GPL 互換**なので、Blender のアドオン（GPL）から呼んで構いません。
なお、このリポジトリには **Blender の API に触れるコードはありません**
——`zenith_py` が使うのは PyO3 と Python の C API です。

各ファイルに MPL の見出しは付けていません。MPL 2.0 の Exhibit A が
「ファイルに書けない・書きたくない場合は `LICENSE` のような場所に
置いてよい」と定めているためです。
