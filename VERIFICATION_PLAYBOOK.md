# 検証手順書 — Zenith CAD Kernel

**対象コミット**: `kernel-accuracy-hardening` ブランチ
**最終確認**: 2026年8月20日（再エクスポート精度の是正まで）

この文書は、**このリポジトリを初めて見る人（または別の AI モデル）が、
主張を信じずに自分で確かめながら作業を進める**ための手順書です。

前提知識は要りません。書いてあるコマンドを順に実行すれば、現在の状態が
本当にその通りかを自分で確認できます。数値はすべて実測値で、貼ってある
値と違う結果が出たら、それは**この文書が古いか、あなたの変更が何かを
壊したか**のどちらかです。

関連文書:

- [`HANDOVER.md`](HANDOVER.md) — 次に何から手を付けるか
- [`KERNEL_SPECS.md`](KERNEL_SPECS.md) — 機能の棚卸しと精度の目安

---

## 0. 最初に読むべき原則

このカーネルで見つかった欠陥は、ほぼすべて**内部からは正常に見える**種類でした。

- 閉多様体だが答えが違うブーリアン
- 面積がちょうど2倍になる断面
- いくら細分しても収束しない積分
- 「ずれは 0」と報告するが、実は**自分が作った点の上でしか測っていない**検査

したがって、この文書の全体を貫く規則は一つです。

> **主張ではなく測定で判断する。測定するときは、測り方そのものを疑う。**

具体的には:

1. **変更の前に測る。** 後で「良くなった」と言うには、前の数字が要ります。
2. **外の物差しで測る。** 解析解（閉じた式）か、OpenCASCADE のような
   別カーネルか。自分の実装で自分を検算しても何も分かりません。
3. **通ったことを、通るはずのない条件でも確かめる。** サンプル数を変える、
   格子を細かくする、別の配置で試す。
4. **落ちたときは、まず自分の期待値を疑う。** 後述の 4-4 に実例があります。

---

## 1. 環境

### 1-1. 必要なもの

| | 用途 | 確認 |
| :--- | :--- | :--- |
| Rust（stable） | 本体のビルド | `cargo --version` |
| FreeCAD 1.1 | OpenCASCADE 7.8 による外部検証 | 下記 |
| Python 3.11 | `zenith_py`（PyO3）のビルドのみ。検証には不要 | `python --version` |

FreeCAD の Python は同梱のものを使います。パスは既定で以下です。

```bash
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" -c "import FreeCAD, Part; print(Part.__name__)"
```

`Part` が表示されれば準備完了です。**別の場所にインストールしている場合**は、
`tools/*.py` の先頭にある `FREECAD_BIN` を書き換えてください。

FreeCAD が無くても本体のテストは全部通ります。ただし**外部カーネルとの
突き合わせができない**ので、その分は「未確認」として扱ってください。

### 1-2. ビルド

```bash
cargo test --release --workspace --exclude zenith_py
```

`zenith_py` を除外しているのは、`pyo3` が PATH から Python を探すためです。
含めたい場合は明示します。

```bash
PYO3_PYTHON="C:/Users/<user>/AppData/Local/Programs/Python/Python311/python.exe" cargo test --release --workspace
```

---

## 2. 現在の状態を自分で確認する

以下を上から順に実行してください。所要時間はビルド込みで 5〜10 分程度です。

### 2-1. テスト

```bash
cargo test --release --workspace --exclude zenith_py
```

**期待**: 43 テストバイナリ、296 テスト、失敗 0。

数を数えるなら:

```bash
cargo test --release --workspace --exclude zenith_py 2>&1 | grep -E "^test result" | awk -F'[ ;]' '{n++;p+=$4;f+=$6} END{print n" binaries, "p" passed, "f" failed"}'
```

### 2-2. ビルダー監査（解析解との一致）

```bash
cargo run --release -p zenith_algo --example builder_audit
```

**期待**: `21 of 21 builder cases clean, 0 with problems`

各行は、シェルの有効性・体積の正値性・**分割数を4倍にしたときの安定性**・
解析解との相対誤差を出します。解析解を持つものは 1e-12 以下です。

ここの「体積の正値性」は**符号付き**で見ています。発散定理は、面が外向きなら
囲む体積を、内向きならその負を返すので、符号がシェルの向きを表します。
`MassProperties` が絶対値を返していた間この検査は一度も発火せず、
球・トーラス・回転体が裏返ったまま通っていました。

### 2-3. ブーリアンの実対応範囲

```bash
cargo run --release -p zenith_algo --example boolean_envelope
```

**期待**: `supported: 30   wrong-result: 0   unsupported/error: 15   (total 45)`

**`wrong-result` が 0 であることが最も重要です。** 対応範囲が狭いのは
仕様ですが、誤答は仕様ではありません。ここが 0 でなくなったら、
その変更は入れてはいけません。

### 2-4. 質量積分の収束

```bash
cargo run --release -p zenith_algo --example mass_convergence
```

**期待**: 分割数を上げても値が動かないこと。96分割から192分割で
`step +7.822e-10` 程度。

### 2-5. 断面と解析解

```bash
cargo run --release -p zenith_algo --example slice_probe
```

**期待**: 円柱・球の z 断面が `area=314.1593`（解析解 314.1592653590 に対して
相対 4.83e-11）。穴あき箱は 2 ループで `area=821.4602`（相対 4.62e-12）。
箱の断面は分割数によらず**厳密**（600.0000、1200.0000）。

断面は表示用メッシュの多角形ではなく、**B-Rep の上で測った点**で積まれます。
輪郭の点を断面へ載せ直し、弦ごとに中点をもう1点測って2次で積むので、誤差は
分割数の**4乗**で縮みます（弦のままなら2乗）。24分割と48分割で比を取ると
16前後になるはずです。ここが4前後なら、B-Rep に当てる段が効いていません。

### 2-6. STEP の往復と他カーネルのファイル

```bash
cargo run --release -p zenith_algo --example step_import_audit
```

**期待（自前ファイルの往復）**: 面数が保たれ、相対誤差 1e-13 台。

```
box       6 ->  6 faces  volume 24000.0000 -> 24000.0000  rel 0.00e0    shell valid
cylinder  6 ->  6 faces  volume 12566.3706 -> 12566.3706  rel 1.46e-13  shell valid
sphere    8 ->  8 faces  volume  4188.7902 ->  4188.7902  rel 1.28e-13  shell valid
cone      6 ->  6 faces  volume  3267.2564 ->  3267.2564  rel 1.17e-13  shell valid
torus    16 -> 16 faces  volume  3789.9281 ->  3789.9281  rel 8.98e-14  shell valid
```

**期待（OpenCASCADE が書いたファイル）**: `FAILED` が 1 件も無いこと。

```
occ_reference_cone.step            1 solid(s), 3 face(s), volume  3267.2564
occ_reference_cone_full.step       1 solid(s), 2 face(s), volume  2094.3951
occ_reference_cylinder.step        1 solid(s), 3 face(s), volume 12566.3706
occ_reference_cylinder_nurbs.step  1 solid(s), 3 face(s), volume 12566.6236
occ_reference_sphere.step          1 solid(s), 1 face(s), volume  4188.7902
occ_reference_sphere_capped.step   1 solid(s), 2 face(s), volume  2094.3951
occ_reference_torus.step           1 solid(s), 1 face(s), volume  3789.9281
occ_reference_torus_segment.step   1 solid(s), 3 face(s), volume   947.4820
```

これらの期待値は**このリポジトリが決めた数ではなく、OpenCASCADE 自身が
同じ形状について報告した数**です（`cylinder_nurbs` のみ、解析値 12566.3706
に対する読み取り 12566.6236 で相対 2.0e-5）。

参照ファイルが無い場合は先に作ります。

```bash
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/occ_reference_export.py
```

### 2-7. p-curve が本当に辺の上にあるか

```bash
cargo run --release -p zenith_algo --example pcurve_fidelity_probe
```

**期待**: どの面も、どの標本数（8, 9, 16, 37, 64）でも **3e-12 以下**
（最悪は円錐の側面で 2.899e-12。半球の球面は 3.407e-13）。

この検査には由来があります。p-curve は辺を8等分して作られ、シェル検証も
8等分で測っていました。**同じ点なので、構成上そこを通ることしか
確かめていませんでした。** 8 の列だけが小さくて他が大きい表が出たら、
それは「検査が効いていない」という意味です。

### 2-8. 外部カーネルとの突き合わせ（リリースゲート）

```bash
cargo run --release -p zenith_algo --example export_validation_suite
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/freecad_cross_validate.py
```

**期待**: `15 of 15 subjects agree across both kernels`、**終了コード 0**。

不一致があれば非ゼロ終了します。CI に置けます。

```bash
cargo run --release -p zenith_algo --example export_showcase
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/verify_showcase.py
```

**期待**: `24 of 24 read back as valid closed solids`、**終了コード 0**。

```bash
cargo run --release -p zenith_algo --example foreign_reexport
& "C:\Program Files\FreeCAD 1.1\bin\python.exe" tools/verify_reexport.py
```

**期待**: `7 of 7 re-exports land on the analytic value within 1e-06`、
**終了コード 0**。実測はすべて 1e-11 以内です。

これは以前は診断でした。比較相手を「OpenCASCADE 自身の NURBS 化」に置いていた
ためで、その置き方が我々の欠陥を隠していました（3章末を参照）。

### 2-9. まとめて実行する

```bash
cargo test --release --workspace --exclude zenith_py \
  && cargo run --release -q -p zenith_algo --example builder_audit | tail -1 \
  && cargo run --release -q -p zenith_algo --example boolean_envelope | tail -1 \
  && cargo run --release -q -p zenith_algo --example export_validation_suite > /dev/null \
  && "/c/Program Files/FreeCAD 1.1/bin/python.exe" tools/freecad_cross_validate.py | tail -1 \
  && cargo run --release -q -p zenith_algo --example export_showcase > /dev/null \
  && "/c/Program Files/FreeCAD 1.1/bin/python.exe" tools/verify_showcase.py | tail -1 \
  && cargo run --release -q -p zenith_algo --example foreign_reexport > /dev/null \
  && "/c/Program Files/FreeCAD 1.1/bin/python.exe" tools/verify_reexport.py | tail -1
```

---

## 3. ゲートと診断の区別

**すべてのツールが合否を判定するわけではありません。** これを取り違えると、
直っていないものを直ったと報告することになります。

| ツール | 種別 | 落ちたら |
| :--- | :--- | :--- |
| `cargo test` | **ゲート** | 直すか、取り下げる |
| `builder_audit` | **ゲート** | 同上 |
| `boolean_envelope` の `wrong-result` | **ゲート（最重要）** | 絶対に入れない |
| `freecad_cross_validate.py` | **ゲート**（非ゼロ終了） | 同上 |
| `verify_showcase.py` | **ゲート**（非ゼロ終了） | 同上 |
| `boolean_envelope` の `supported` 数 | 進捗の指標 | 減っていたら回帰 |
| `step_import_audit` | 診断 | 数字を読む |
| `pcurve_fidelity_probe` | 診断 | 数字を読む |
| `mass_convergence` / `slice_probe` | 診断 | 数字を読む |
| `verify_reexport.py` | **ゲート**（非ゼロ終了） | 解析解と 1e-6 以内かを見る |
| `regularize_probe` | 診断 | 全周を刻んで形が動いていないか |
| `pcurve_derivation_probe` | 診断 | p-curve を導出し直すと答えが変わる面 |

**この表の `verify_reexport.py` の行は、以前は「診断」でした。それが誤りでした。**

かつてこの文書はこう書いていました——OpenCASCADE は有理 B-spline を解析曲面と
同じようには測らず、自分の円柱を `toNurbs` してから測ると 12674.63（解析値
12566.37 に対して +0.86%）になる。だから比較相手は「OCC 自身の NURBS 化」で
あって「OCC の解析値」ではない、と。

**測り直すと成り立ちませんでした。** 自前ビルダーの有理パッチ（円柱の四半周）は
OpenCASCADE が **628.318530712**（解析解 628.318530718、相対 1e-11）で読みます。
有理パッチが測れないのではありません。測れないのは**全周を1枚で巻いたパッチ**
だけで、それを書いていたのは我々でした。

比較相手を「相手も同じくらい外れているもの」に置くと、欠陥は仕様に見えます。
**外れようのないもの（解析解）を相手に置いてください。**

---

## 4. 変更を加えるときの手順

### 4-1. 基本の型

```
1. 変更前に、関係しそうなツールを走らせて数字を控える
2. 変更する
3. 同じツールを走らせて、数字を並べる
4. 全ゲートを走らせる
5. 数字が動いた理由を説明できるなら入れる。できないなら入れない
```

**3 で「変わらない」ことも立派な結果です。** 変わらないはずのものが
変わっていたら、それが本当の発見です。

### 4-2. ブーリアンに触るとき

ブーリアンには検証ゲート（`BooleanResultVerifier`）が入っています。

1. 全シェルの閉性
2. 演算が含意する体積境界
3. **384点の内外一貫性**

通らなければ**もっともらしいソリッドではなくエラー**を返します。
踏み込んだ改造を安全に試せるのはこのゲートのおかげです。**弱めないでください。**

段階ごとの件数を見たいときは:

```bash
cargo run --release -p zenith_algo --example boolean_pipeline_probe
cargo run --release -p zenith_algo --example split_error_probe
cargo run --release -p zenith_algo --example imprint_probe
cargo run --release -p zenith_algo --example coplanar_probe
```

### 4-3. 測り方の落とし穴（実際に踏んだもの）

これらは**すべてこのリポジトリで実際に起きた**誤診です。

**(a) プローブが自分の解像度を測る**

境界が曲面に乗っているかを 80×80 の格子で測って `0.41` を得ました。
これは**格子間隔そのもの**でした（半径10の全周を80分割 → 0.39）。
粗い格子で当たりを付けてから局所的に詰め直すと `5e-13` でした。
存在しない欠陥を追いかけるところでした。

> **対策**: 距離を格子探索で測るなら、必ず段階的に詰める。
> 出た値が「格子間隔くらい」なら、それは測定の限界であって欠陥ではない。

**(b) 探索の粗さがそのまま答えとして残る**

同じ間違いを**カーネル内部もしていました**。最近傍点探索は 16×16 の粗
サンプリングから始めます。ニュートン法が動けなかったとき、そのまま返すと
格子間隔（半径10で 1.8）が答えになります。しかもそれは
**「もっともらしい小さな値」なので検査を通ってしまいます**。

> **対策**: 反復解法が「収束した」と言うとき、出発点より良くなったことを
> 確かめる。悪化した位置を返さない。

同じ形が断面でも出ました。輪郭の点を面へ射影して断面に載せ直すとき、**平面の
パッチでは正しい補正が 0** です。それでも射影は自分の残差ぶんだけ点を動かすので、
そのまま採ると残差が面積の補正として積まれます。**動いた量が残差と同じ桁なら、
動いたのは幾何ではなく探索のほうです。** 現在は残差の8倍を超える動きだけを
採っており、平面だけの断面は4通りの分割数で厳密なままです。

**(c)「いくつか乗っている」は「そういう形である」ではない**

円柱の認識は「最大半径に16点以上乗っていれば円柱」でした。
円錐の底円はちょうど最大半径に乗るので、**円錐が円柱として通ります**。

> **対策**: 「ある条件を満たす部分集合が存在する」ではなく
> 「**すべての標本が**満たす」まで見る。

**(d) 検査が自分の作った点の上でしか測っていない**

p-curve は8等分で作られ、検査も8等分でした。構成上そこを通ることしか
確かめておらず、間では球を一周していました（半径10の球で **20.0** のずれ）。

> **対策**: 検査の標本位置を、構成に使った位置と**互いに素**にする。
> いまは 37（8 と共有するのは両端だけ）。

**(e0) 比較相手が同じくらい外れていると、欠陥が仕様に見える**

書き戻したファイルを OpenCASCADE が 0.86% 高く測るのを、「OCC は有理パッチを
そう測らない」と説明し、比較相手を **OCC 自身の NURBS 化**に置いていました。
両方が同じくらい外れているので、一致しているように見えます。実際は、
**自前ビルダーの有理パッチは 1e-11 で読まれていました**。外れていたのは
有理パッチ全般ではなく、我々が書いていた**全周1枚**の形だけです。

> **対策**: 比較相手には、外れようのないもの（解析解）を置く。
> 「相手も同じくらい外れている」は、自分が正しい証拠ではない。

**(e1) 同じ量を2つの経路で出していると、片方だけ直っていることがある**

`face_uv_triangulation` は面が**保持している** p-curve のフィールドを見て、
無ければトリム前の全矩形に落ちていました。`plane_pcurves()` のほうは無ければ
**導出**します。同じ「その面の p-curve」を指す2つの経路が別々に振る舞い、
トリムされた面が3倍の面積で返っていました（穴の壁で 113.10 対 376.99）。

**導出そのものは正しかった**（結果は保持しているものと UV 上で1点ずつ一致する）
ので、幾何を疑っている限り見つかりません。

> **対策**: 値が食い違ったら、まず両者が**同じ経路**を通っているかを見る。
> 片方がフィールドを、片方が幾何を見ていないか。

**(e) 絶対値を取ると、向きの誤りが見えなくなる**

`MassProperties` は体積の絶対値を返していました。捨てていた符号こそが、
シェルが外を向いているか内を向いているかを示す唯一の量です。おかげで
**3つのビルダーが裏返ったまま**通っていました（球 -4188.79、トーラス -3789.93、
回転体 -1507.96）。`builder_audit` には「体積が正か」の検査が最初からあり、
一度も発火していませんでした。

> **対策**: 量を報告するとき、`abs` や `max(0.0)` で「もっともらしく」しない。
> 符号や範囲外の値は、たいてい情報であって汚れではない。

**(f) 面積・体積がちょうど半分／2倍なら、たいてい継ぎ目の取り違え**

トーラスを1面で書いたファイルは、体積がちょうど半分になりました。
継ぎ目上の点は UV 領域の両端どちらにも写るので、p-curve から囲まれた領域を
読むと投影がどちらを選ぶかで答えが割れます。**p-curve では原理的に
決まらない**ので、位相で見ます（ループ内のどの辺も2度現れるなら、
その面は曲面全体）。

### 4-4. ゲートが落ちたら、まず自分の期待値を疑う

**実例。** トーラス × ボックスで差と積が「解析解と一致」したのに
ゲートが弾き、誤検知だと判断しました。**間違っていたのは私の解析解**でした。
箱は x, y とも ±10 で、トーラスは ρ が最大16なので**横にはみ出します**。
箱の側面もトーラスを切るのに、それを無視した「無限スラブ」の値を
解析解として計算していました。

内外判定を Halton 点で突き合わせ、外れる35点が全て「箱の外だがトーラスの中」の
領域にあると分かって初めて気付きました。

> **ゲートは 384 点の内外一貫性で見ています。体積が合っていても形が違えば
> 落ちます。体積の一致は形の一致より弱い条件です。**

### 4-5. 部品を一つずつ入れて壊れるとき

ブーリアンの回転ボックス対応では、部品を1つずつ入れると**必ず別のケースを
壊しました**。4回取り下げています。必要だったのは4つ＋同一平面処理で、
**同時に入れないと効きません**。

一方で、円錐対応では**幾何を2箇所直しても envelope が動かず**、
3つ目（円柱向けの近道が円錐を誤認していた、既存の欠陥）を見つけて初めて
動きました。

> **教訓**: 単独で効果が測れない部品でも、他が揃うまで判断を保留するほうが
> 良い場合があります。ただし**取り下げの判断自体は正しい**ことが多く、
> そのたびに障壁の位置が1段深く特定できます。取り下げるときは、
> **何をどこまで測ったか**を必ず残してください。

---

## 5. 別の AI モデル／人が引き継ぐときのチェックリスト

作業を始める前に:

- [ ] 2章を上から実行し、貼ってある数字と一致することを確認した
- [ ] 一致しないものがあれば、それが**この文書が古いのか、環境の違いか**を
      切り分けた（FreeCAD が無いだけなら 2-8 は飛ばしてよい）
- [ ] [`HANDOVER.md`](HANDOVER.md) の「次にやること」を読んだ
- [ ] 3章のゲートと診断の区別を理解した

変更を報告するときに:

- [ ] 変更前後の数字を並べた（「良くなった」ではなく「A から B になった」）
- [ ] 外の物差し（解析解か別カーネル）と突き合わせた
- [ ] 全ゲートを走らせ、結果を書いた
- [ ] `wrong-result` が 0 のままであることを確認した
- [ ] **落とした・取り下げたものがあれば、それも書いた**
- [ ] 自分の推測と測定値を、文中で区別して書いた

やってはいけないこと:

- 検証ゲートを緩めて「通った」と報告する
- 対応範囲外を、エラーではなく近似で返すようにする
- STEP を `StepInterop` ではなく `StepExporter` で直接書く（全周1枚のまま出る）
- 測っていないことを「一致している」と書く
- 数字を丸めて都合よく見せる（相対誤差は指数表記のまま出す）

---

## 6. よく使うコマンド一覧

**能力の測定**

| コマンド | 何を測るか |
| :--- | :--- |
| `builder_audit` | 全ビルダーの健全性・解析解との一致・分割数4倍での安定性 |
| `boolean_envelope` | ブーリアンが実際に成功する範囲（45ケース） |
| `chained_boolean_probe` | ブーリアン結果をさらに加工できるか |
| `mass_convergence` | 質量積分が細分に対して収束するか |
| `slice_probe` | 断面積・周長と解析解の差 |
| `step_import_audit` | STEP の往復と、他カーネルのファイルを読めるか |
| `pcurve_fidelity_probe` | p-curve が本当に辺の上にあるか |
| `foreign_reexport` | 他カーネルのファイルを読んで書き戻す一周 |
| `regularize_probe` | 全周を刻んでも体積・面積が動かないか |
| `pcurve_derivation_probe` | p-curve を導出し直すと積分が変わる面 |

**不具合を追うための診断**

| コマンド | 何が見えるか |
| :--- | :--- |
| `boolean_pipeline_probe` | ブーリアンの各段階の件数（交線・分割・選択・縫合） |
| `boolean_selection_probe` | 選ばれた面の一覧と、そのオペランド・領域区分・面積 |
| `split_error_probe` | 面ごと・交線ごとに、なぜ分割が拒否されたか |
| `imprint_probe` | 各面が受け取る交線と、それが面を横断しているか |
| `coplanar_probe` | 同一平面で重なる面のペアと法線の向き |
| `uv_domain_probe` / `surface_smoothness_probe` | テッセレーションの被覆と曲面評価の不連続 |
| `imported_curve_probe` | インポーターが再構成した曲線・面の中身 |

すべて `cargo run --release -p zenith_algo --example <名前>` で実行します。

**外部カーネルとの突き合わせ**

| コマンド | 何を測るか |
| :--- | :--- |
| `export_validation_suite` ＋ `tools/freecad_cross_validate.py` | 体積・表面積・断面積を OpenCASCADE と突き合わせ（ゲート） |
| `export_showcase` ＋ `tools/verify_showcase.py` | 代表24形状が Solid として読めるか（ゲート） |
| `occ_reference_export.py` | OpenCASCADE 自身に解析曲面の STEP を書かせる |
| `foreign_reexport` ＋ `tools/verify_reexport.py` | 読んで書き戻した一周が解析解に乗るか（**ゲート**） |

---

## 7. 出力物の置き場所

```bash
cargo run --release -p zenith_algo --example export_showcase          # target/showcase   24形状
cargo run --release -p zenith_algo --example export_validation_suite  # target/validation 15形状
cargo run --release -p zenith_algo --example foreign_reexport         # target/reexport    7形状
```

| 置き場所 | 中身 | 突き合わせ |
| :--- | :--- | :--- |
| `target/showcase/` | 代表24形状。解析解を持つものは相対誤差付き | `verify_showcase.py`（ゲート） |
| `target/validation/` | 相互検証用15形状＋OCC が書いた参照ファイル | `freecad_cross_validate.py`（ゲート） |
| `target/reexport/` | 他カーネルのファイルを読んで書き戻した7形状 | `verify_reexport.py`（**ゲート**） |
