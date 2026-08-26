# Zenith CAD Kernel — 作業開始案内

このリポジトリを初めて扱う人・AIは、コードを変更する前に次の順で読んでください。

1. [`HANDOVER.md`](HANDOVER.md) — 現在地点、作業ブランチ、次に着手する候補
2. [`VERIFICATION_PLAYBOOK.md`](VERIFICATION_PLAYBOOK.md) — 主張を再測定する手順
3. [`KERNEL_SPECS.md`](KERNEL_SPECS.md) — 現在の仕様と実装範囲
4. [`KERNEL_INVENTORY_SPECS.md`](KERNEL_INVENTORY_SPECS.md) — 機能一覧と制限
5. 必要な課題に対応する `reference/`、Rustコード、その他の設計文書

最初に、文書内のスナップショットより手元のGit状態を優先して確認します。

```bash
git status --short --branch
git log --oneline --decorate -12
git diff main...HEAD --stat
```

作業時の原則:

- いつでも手戻りできるよう、作業ブランチを使い、意味のまとまりごとにコミットする。
- 長時間になりそうなビルド・テスト・調査は、着手前に所要時間を見積もる。
- 作業後は、実装だけでなく引継書・仕様書・検証記録も現在地点へ更新する。
- 過去の測定や判断は履歴として残し、削除は現状理解に必要な最小限にする。
- 文書の主張をそのまま信じず、`VERIFICATION_PLAYBOOK.md`の手順とCIで再確認する。

現在の具体的なブランチ、コミット、未解決事項は
[`HANDOVER.md`](HANDOVER.md)の「0. 作業を引き継ぐときの現在地点」を正とします。
