# 引継書 — Zenith CAD Kernel

**最終更新**: 2026年8月20日
**ブランチ**: `kernel-accuracy-hardening`（`main` から分岐、未マージ）

この文書は「次に何から手を付けるか」を1枚で分かるようにしたものです。
仕様の詳細は [`KERNEL_SPECS.md`](KERNEL_SPECS.md)、検証の詳細は
[`FREECAD_VALIDATION_REPORT.md`](FREECAD_VALIDATION_REPORT.md) にあります。

---

## 1. まず動かして状態を確認する

```bash
cargo test --release --workspace --exclude zenith_py
```

`zenith_py` は `pyo3` が PATH から Python を探します。見つからない環境では明示します。

```bash
PYO3_PYTHON="C:/Users/hinat/AppData/Local/Programs/Python/Python311/python.exe" cargo test --release --workspace
```

現在の状態（すべて実測）:

| 指標 | 値 |
| :--- | :--- |
| テストバイナリ | 36 すべてグリーン |
| コンパイラ警告 | 0 |
| ビルダー監査 | 21/21 クリーン |
| ブーリアン対応 | 45ケース中25成功、**誤答ゼロ** |
| FreeCAD 相互検証 | 15/15 一致 |
| ショーケース | 16/16 が OpenCASCADE で valid closed solid |

---

## 2. この作業の考え方（重要）

このカーネルで見つかった欠陥は、ほぼすべて**内部からは正常に見える**種類でした。

- 閉多様体だが答えが違うブーリアン
- 面積がちょうど2倍になる断面
- いくら細分しても収束しない積分
- STEP に書いた瞬間に他カーネルで壊れる立体

**主張ではなく測定で判断してください。** そのための常設ツールが揃っています。

```bash
cargo run --release -p zenith_algo --example builder_audit        # 全ビルダーの健全性と解析解との一致
cargo run --release -p zenith_algo --example boolean_envelope     # ブーリアンの実対応範囲（45ケース表）
cargo run --release -p zenith_algo --example step_import_audit    # STEP 往復と他カーネルファイルの読み込み
cargo run --release -p zenith_algo --example mass_convergence     # 質量積分の収束
cargo run --release -p zenith_algo --example slice_probe          # 断面積と解析解の差
```

外部カーネルとの突き合わせ（不一致で非ゼロ終了するのでリリースゲートに使えます）:

```bash
cargo run --release -p zenith_algo --example export_validation_suite
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/freecad_cross_validate.py

cargo run --release -p zenith_algo --example export_showcase
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/verify_showcase.py
```

ブーリアンには検証ゲート（`BooleanResultVerifier`）が入っています。
①全シェルの閉性 ②演算が含意する体積境界 ③384点の内外一貫性 を確認し、
通らなければ**もっともらしいソリッドではなくエラー**を返します。
踏み込んだ改造を安全に試せるのはこのゲートのおかげです。壊さないでください。

---

## 3. 次にやること（優先順）

### 3-1. インポーター: 円錐・球・トーラスの解析曲面 ★最も確実

**現状**: `CONICAL_SURFACE` / `SPHERICAL_SURFACE` / `TOROIDAL_SURFACE` は、
面の境界を見ずに固定サイズのパッチとして読まれます。境界が曲面から外れるため、
これらを使った他カーネルのファイルは読めません。

**やり方**: 円柱で確立した手法をそのまま適用します。
`crates/zenith_io/src/step_import.rs` の `get_surface_for_boundary` に分岐を足し、
`cylinder_patch_for_boundary` と同じ要領で境界の占める範囲に合わせたパッチを組みます。
面の境界は既に先に読まれているので、配線は済んでいます。

**確認**: OCC に対象形状を書かせてから読みます。

```bash
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/occ_reference_export.py
cargo run --release -p zenith_algo --example step_import_audit
```

### 3-2. インポーター: トリム B-spline 面の境界近似

**現状**: 曲線でトリムされた B-spline 面は、トリム境界をポリゴン近似するため
面積に数%の誤差が出ます（OCC の NURBS 円柱でキャップが 282.47、正しくは 314.16）。

**手がかり**: 曲線自体は厳密に読めています（`imported_curve_probe` で確認済み。
円周 62.8315、半径ちょうど10）。問題は `zenith_tess` 側のトリム領域の三角形分割です。
断面スライサーで解決したのと同じ性質の問題です。

### 3-3. ブーリアン: 曲面同士の交差（SSI）★最も大きい

**現状**: 未対応なのはこれだけになりました。

| 未対応のケース | 必要なもの |
| :--- | :--- |
| 円柱 × 円柱、球 × 球 | NURBS × NURBS の交線 |
| ボックス × 球、円錐 × ボックス、トーラス × ボックス | 平面 × 各解析曲面の交線 |
| 円柱が面に接する配置 | 退化（接線）配置の扱い |

**手がかり**: 分割・選択・縫合・同一平面の各段階は揃っています。
`intersect_face_supports`（`brep_intersection.rs`）が NURBS×NURBS で
`Unsupported` を返しているので、**交線さえ供給できれば下流はそのまま使えるはず**です。
`zenith_geom/src/ssi.rs` に細分割＋Newton marching の実装があります。

平面×球は交線が円になり厳密表現できるので、そこから始めるのが素直です。
ただし球パッチを非パラメータ線で分割する処理が新たに必要になります。

---

## 4. 踏んだ落とし穴（繰り返さないために）

**ブーリアンの回転ボックス対応は、部品を1つずつ入れると必ず別のケースを壊しました。**
4回取り下げています。必要だったのは以下の4つ＋同一平面処理で、**同時に入れないと効きません**。

1. 連結分割（切り込みは複数の交線が内部の角で繋がったもの）
2. 境界沿い交線の除外（それは接触の記録であって切り込みではない）
3. 同一境界辺への着地の許可（両端間の経路はその辺の一区間）
4. 頂点の刻み込み（**2度「無意味」と判断して捨てた**。分割が正しくなって初めて効いた）

教訓として、**単独で効果が測れない部品でも、他が揃うまで判断を保留する**ほうが良い場合があります。
ただし取り下げの判断自体は正しく、そのたびに障壁の位置が1段深く特定できました。

**プローブの測り方を間違えると誤診します。** 境界までの距離をサンプル点への距離で測っていたため、
「交線が面を横断していない」と誤読し、存在しない問題を追いかけました。線分距離で測り直すと
gap は全て 0 でした。

---

## 5. リポジトリの見取り図

| 場所 | 中身 |
| :--- | :--- |
| `crates/zenith_math` | 点・ベクトル・変換・ロバスト述語 |
| `crates/zenith_geom` | NURBS 曲線・曲面、Coons/Gordon、曲率、SSI |
| `crates/zenith_topo` | Vertex/Edge/Wire/Face/Shell/Solid、シェル検証 |
| `crates/zenith_algo` | ブーリアン・押出/回転/ロフト/掃引・穴/フィレット・断面・質量特性 |
| `crates/zenith_tess` | テッセレーション（積分領域はノット区間に整合） |
| `crates/zenith_io` | STEP 読み書き、STL/OBJ/glTF/IGES |
| `crates/zenith_py` | PyO3 バインディング（45関数） |
| `crates/zenith_algo/examples/` | **測定・診断ツール**（19個） |
| `tools/*.py` | FreeCAD ヘッドレス検証（`occ_*` は診断用） |
| `target/showcase/` | 代表16形状の STEP（`export_showcase` で再生成） |

Blender アドオン本体（`__init__.py`・オペレータ・パネル）は未着手です。
`blender_addon/` にはビルド済みの `zenith_cad.pyd` のみが入っています。

---

## 6. 未コミットのもの

リポジトリ直下に生成物の `.step` が多数、`target_*` ディレクトリが多数あります。
いずれも未追跡のまま残してあります（`.gitignore` の見直し余地あり）。
`reference/` は移植元の Seamless CAD と OCCT で、合計 420MB。gitignore 済みです。
