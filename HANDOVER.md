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
| テストバイナリ | 37 すべてグリーン（263テスト） |
| コンパイラ警告 | 0（examples に3件、既存） |
| ビルダー監査 | 21/21 クリーン |
| ブーリアン対応 | 45ケース中25成功、**誤答ゼロ** |
| FreeCAD 相互検証 | 15/15 一致 |
| ショーケース | 16/16 が OpenCASCADE で valid closed solid |
| 他カーネルの解析曲面の読み込み | 7/7 が体積・面積とも OpenCASCADE と一致 |
| 他カーネルのトリム B-spline | 1件、面積が 282.47（正しくは 314.16）。下の 3-2 |

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
cargo run --release -p zenith_algo --example pcurve_fidelity_probe # p-curve が本当に辺の上にあるか
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

### 3-1. 済: インポーターの解析曲面（2026年8月20日）

`CONICAL_SURFACE` / `SPHERICAL_SURFACE` / `TOROIDAL_SURFACE` を、円柱と同じく
面の境界から実寸に合わせて張るようにしました。付随して3つの欠陥が出ました。

| 直したもの | 症状 |
| :--- | :--- |
| `FACE_BOUND` の向きフラグを落としていた | 面の境界が逆回りになり、辺が両隣から同じ向きに使われて弾かれる |
| シェル検証が辺を端点だけで対にしていた | トーラスの2本の縫い目は端点が同じなので、4つの使用が互いに対と判定される |
| 縫い目だけのループを p-curve の面積で判定していた | 縫い目上の点は UV の両端に写るので、面がちょうど半分になる |

`VERTEX_LOOP`（球を1面で書いたときの極）にも対応しました。辺の無いループは
書き落としではなく「トリムするものが無い」という意味なので、縫い目だけの
ループと同じ扱いにしています。

OpenCASCADE が書いた参照ファイルに対する実測（すべて OCC の値と一致）:

| ファイル | 体積 | 以前 |
| :--- | ---: | :--- |
| 円錐 r10/r4 h20 | 3267.2564 | 読めず |
| 頂点まで届く円錐 | 2094.3951 | 読めず |
| 球 r10（VERTEX_LOOP） | 4188.7902 | 読めず |
| 半球（縫い目を往復する境界） | 2094.3951 | 読めず |
| トーラス R12 r4（1面表現） | 3789.9281 | 読めず |
| トーラス90度区分 | 947.4820 | 読めず |

参照ファイルは `crates/zenith_algo/tests/fixtures/` に置いてあり、
`foreign_analytic_surface_test` が体積と面積の両方を見ています。
期待値はこのリポジトリが決めた数ではなく OpenCASCADE 自身が報告した数なので、
ここが落ちたら他カーネルとの不一致です。再生成は次のとおり。

```bash
& "C:\Program Files\FreeCAD 1.1in\python.exe" tools/occ_reference_export.py
cargo run --release -p zenith_algo --example step_import_audit
```

### 3-2. p-curve 検証が、作った点の上でしか測っていない ★最優先

**これは精度の話ではなく、検証が効いていないという話です。**

`Face::new` は NURBS 面の p-curve を `derive_nurbs_boundary_pcurves(tol, 8)` で
作ります。辺を8等分して投影し、その9点を通る1次のポリラインにします。
シェル検証は `face.validate_pcurves(tol, 8)` を呼びます。**同じ8等分**です。
つまり検証は、p-curve が構成上ぴったり通ることが分かっている点だけを見ています。
間のどこも見ていません。

実測してください。

```bash
cargo run --release -p zenith_algo --example pcurve_fidelity_probe
```

```
file                                   face          8          9         16         37         64
occ_reference_cylinder_nurbs.step      face  1 nurbs   5.515e-12   8.344e-1   8.892e-1   8.881e-1   8.892e-1
occ_reference_sphere_capped.step       face  0 nurbs   3.534e-11    7.002e0    2.000e1    1.956e1    2.000e1
```

8のところだけ 1e-11、他はすべて 0.89 と 20.0 です。半球の 20.0 は半径10の球の
**直径**で、投影が裏側に落ちている点があるという意味です。ポリラインが粗いと
いう話ではありません。

**効いていないのは投影経路だけです。** 他の面はすべてどの列でも 1e-15 です。
`match_nurbs_boundary_pcurve` は、まず辺が曲面の等パラメータ境界に乗るかを試し
（`match_nurbs_outer_boundary_pcurve`、これは厳密）、駄目なら
`project_edge_to_nurbs_pcurve` に落ちます。壊れているのは後者だけです。

**やること**（この順で、まとめて）:

1. 投影が裏側に落ちる件。`ExtremumEngine::point_to_surface` が最も近い点を
   返していない、あるいは球の継ぎ目でどちらの端にも写る点を選び損ねている。
   まずここを潰さないと、後の2つを入れても数字は良くなりません。
2. ポリラインを辺に追従させる。今の8等分は、点が曲面に乗っているかしか
   見ておらず、**点と点の間が辺から離れていないかを一度も見ていません**。
   円が八角形として通っているのはこれです（NURBS円柱のキャップ 282.47、
   正しくは 314.16）。
3. 検証のサンプル数を、構成に使った数と**別の数**にする。
   `shell.rs:132` の `validate_pcurves(tol, 8)` を変えるだけで露見します。

**先に3だけ入れないでください。** 1と2が無い状態で 37 に変えると、
いま体積・面積が正しく読めているファイルが軒並み弾かれます（実測済み）。
順序が要ります。

**試して取り下げたもの**: 2 だけを先に入れました（弦の中点が辺から離れて
いる区間を割る適応細分）。p-curve 自体は良くなりました（2D長 61.2091 →
62.8315、囲む面積 282.4690 → 314.1512、いずれも厳密値と一致）。
ですが分割点が等間隔でなくなるため、検証が前提にしている
「p-curve のパラメータ比 = 辺のパラメータ比」が崩れ、半球が弾かれました。
差分は残していません。入れ直すなら、点を辺のパラメータに合わせた
ノットベクトルで張り（1次なら `[t0,t0,t1,...,tn,tn]`）、比例関係を保つこと。

なお面積にはもう一段の損失があります。p-curve を厳密にしても、キャップの
積分面積は 312.10 でした（正しくは 314.16）。残りは
`zenith_tess` の `loop_deflection_target` で、トリムループを
`diagonal / (divisions * 4)` まで粗く折ってよいことにしているためです。
これは全テッセレーションに効くので、触る前に既存の測定値を控えてください。

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

**「面積・体積がちょうど半分／2倍」は、たいてい継ぎ目の取り違えです。**
トーラスを1面で書いたファイルは、体積がちょうど半分になりました。縫い目上の点は
UV 領域の両端どちらにも写るので、p-curve から囲まれた領域を読むと、投影がどちらを
選ぶかで答えが割れます。**p-curve では原理的に決まらない**ので、位相で見ます
（ループ内のどの辺も2度現れるなら、その面は曲面全体）。断面積がちょうど2倍に
なったときと同じ性質の間違いです。

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
