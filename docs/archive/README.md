# 退避した文書

ここにあるのは、**書かれた当時は正しかったが、いまの作業を導かない**文書です。
消していないのは、なぜそう決めたかの記録として意味があるからです。

**ここの数字を、いまの実測として引かないでください。** 実測の正は
[`../../HANDOVER.md`](../../HANDOVER.md) です。

| 文書 | 書かれた日 | 何だったか | なぜ退避したか |
| :--- | :--- | :--- | :--- |
| `KERNEL_AUDIT.md` | 2026-08-19 | OCCT 置き換えに向けて Rust カーネルを点検した覚書 | 挙げられた指摘（円柱の平面蓋のテッセレーションなど）は解決済み。以後の実測は HANDOVER 4章に集約されている |
| `MIGRATION_MAP.md` | 2026-08-19 | Seamless_CAD の `cad_server.exe`（OCCT 製）を Rust カーネルへ置き換える段取り | 置き換えの境界は決まり、作業はカーネル本体の確度へ移った。現行の連携仕様は [`../../SEAMLESS_PROTOCOL.md`](../../SEAMLESS_PROTOCOL.md) と [`../../SEAMLESS_CAD_ZENITH_INTEGRATION_SPEC.md`](../../SEAMLESS_CAD_ZENITH_INTEGRATION_SPEC.md) |
| `KERNEL_REPLACEMENT_STRATEGY.md` | 2026-08-19 | 「OCCT の小さな複製にしない」という方針表明 | 方針は [`../../ROADMAP.md`](../../ROADMAP.md) と HANDOVER 9章（実用に耐えるには何が要るか）が引き継いでいる |

退避日: 2026-08-30。
