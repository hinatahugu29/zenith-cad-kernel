# 付録 B：Zenith CAD Kernel 常設検証プローブ＆ベンチマーク一覧

Zenith CAD Kernel リポジトリに常設されている自動検証プローブ（Release Gates & Audit Probes）の**抜粋**です。
すべて `cargo run --release -p zenith_algo --example <probe_name>` で実行できます。

> **ここは一覧の抜粋で、常設の全数ではありません。** 全数と、いま実際に出る
> 数字は [`HANDOVER.md`](../../HANDOVER.md) の第1章と
> [`.github/workflows/gates.yml`](../../.github/workflows/gates.yml) にあります。
> **下の表の数値は書いた時点のもの**なので、判断に使う前に自分で回して
> ください（手順は [`VERIFICATION_PLAYBOOK.md`](../../VERIFICATION_PLAYBOOK.md)）。

| プローブ名 | 検証対象・検査内容 | 合格基準 / リリースゲート条件 |
| :--- | :--- | :--- |
| **`builder_audit`** | 全24種ソリッドビルダーの幾何健全性・解析解一致 | 全24ビルダーで体積・表面積が閉じた式と $10^{-12}$ 以内 |
| **`boolean_envelope`** | 自作立体同士の厳密ブーリアン45ケース走査 | 44 supported / 0 wrong-result / 1 既知非多様体明示拒否 |
| **`step_import_audit`** | STEP Part 21 往復と、外部CAD（OpenCASCADE）が書いたファイルの読み込み | 往復で面数が保たれ体積残差が $10^{-13}$ 台であること、外部ファイル側に `FAILED` が1件も無いこと |
| **`mass_convergence`** | ガウス発散定理による質量特性積分の収束性 | 分割数に応じた誤差減衰と解析解との一致 |
| **`slice_probe`** | 任意平面による断面積スライサーと解析解の差 | 断面積・周長が解析解と $1.34\times 10^{-11}$ 以内 |
| **`pcurve_fidelity_probe`** | 面上の p-curve が真に3D空間の稜線の上に乗っているか | 3D曲線と曲面上のUV射影点の偏差が $10^{-12}$ 以内 |
| **`foreign_reexport`** | 他カーネル（OCCT等）のファイルを読み込み再書き出し | 読み書き一周後の体積残差が $10^{-11} \sim 10^{-13}$ 以内 |
| **`face_split_probe`** | 自由曲線による曲面パッチ分割（FaceSplitter） | 分割後の各面片の面積総和が元曲面面積と $1.46\times 10^{-13}$ で一致 |
| **`ssi_probe`** | 曲面間幾何交差（SSI）の追跡精度と残差 | フィット交線が両曲面に $10^{-6}$ 以内で乗ること |
| **`boolean_topology_probe`** | ブーリアン結果におけるエッジ実体共有性 | 共有されていない不正エッジが0本であること（Release Gate） |
| **`mesh_watertight_probe`** | 4〜256分割における出力メッシュの完全密閉性 | open: 0, non-manifold: 0, degenerate: 0 |
| **`foreign_distance_probe`** | 他カーネル立体に対する最近傍点・最短距離探索 | 36チェックすべてで閉じた式と一致（最悪 $3.55\times 10^{-15}$） |
| **`export_mesh_suite` ＋ `tools/verify_mesh_exports.py`** | STL / OBJ / glTF / DXF を書き出し、**書いたファイルだけ**から解き直す | 8検体すべてで、辺がちょうど2枚の三角形に共有され、体積が B-Rep と合い、3形式が互いに一致し、DXF の層と向きが断面と合うこと（FreeCAD 不要・CI 収録） |
