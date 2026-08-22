# 🚀 Zenith CAD Kernel - スペック総覧（棚卸し）＆ 次なる飛躍への展望

**文書バージョン**: v3.0.0 (完全閉多様体テッセレーション・自由曲面SSI面分割・Gregory/N辺コーナーブレンド・ホブ創成トロコイド・レイヤーDXF・SVDスケッチ解析)  
**最終更新日時**: 2026年8月22日  
**ステータス**: 完全自前 Rust B-Rep エンジン。**本書の数値はすべて実測値**。

> **品質達成ハイライト**:
> - 出力用メッシュの完全閉多様体化（4〜32分割全9通りで open: 0, non-manifold: 0, degenerate: 0）を完全達成。
> - ワークスペース全64テストバイナリ / 422テスト 100% 合格（0 failed, 0 ignored）。
> - 単一の軽量C-Extension（`zenith_cad.pyd` 3.85MB）のみで外部依存ゼロ。

Zenith CAD Kernel は、Rust でフルスクラッチ開発された **次世代型 3次元 B-Rep / 自由曲面 NURBS CAD カーネル** です。  
巨大な外部ライブラリ（OpenCASCADE / pythonocc / FreeCAD）を一切介さず、**単一の軽量アドオン（`zenith_cad.pyd`）のみで完結する「真の脱OCCT」** を達成しています。

本書は、現時点で達成された **全機能スペックの完全な棚卸し** と、業界標準 CAD（FreeCAD / OpenCASCADE）による **ヘッドレス自動検証実績**、および今後世界最高峰のモデリング環境へと **さらに飛躍するための技術構想** をまとめた公式仕様書です。

---

## 📊 現行スペック総覧・機能棚卸し（Specs Inventory）

```mermaid
graph TD
    A[Zenith CAD Kernel Core v3.0.0] --> B[1. 数値幾何・自由曲面エンジン]
    A --> C[2. B-Rep トポロジー構造]
    A --> D[3. 形状生成・フィーチャーモデリング]
    A --> E[4. ダイレクトモデリング＆解析]
    A --> F[5. 評価・物性値・テッセレーション]
    A --> G[6. データ交換＆FreeCAD検証連携]
```

### 1. 数値幾何・自由曲面エンジン（Geometry Layer: `zenith_geom` & `zenith_math`）
幾何計算の厳密性と数値的安定性を担保する数学基盤。

| 機能区分 | 実装モジュール | スペック・技術仕様詳細 |
| :--- | :--- | :--- |
| **NURBS 曲線 / 曲面** | `nurbs_curve`, `nurbs_surface` | 任意次数（Degree $p, q$）の非均一有理Bスプライン評価。Cox-de Boor 漸化式、高階導関数計算（任意階数 $k$）。ノットベクトルの反転・多重度圧縮。 |
| **有理真円・円錐曲線** | `nurbs_curve` | 重み $w_i = \cos(\theta/2)$ による真円・円弧・楕円・放物線・双曲線の幾何学的厳密表現（誤差 $0.0$）。 |
| **微分幾何・曲率解析** | `curvature` | 第1基本形式 ($E, F, G$)、第2基本形式 ($L, M, N$)、Gauss曲率 $K$、平均曲率 $H$、主曲率 $\kappa_1, \kappa_2$、法線ベクトルの厳密計算。 |
| **4境界 Coons パッチ** | `coons_patch` | 4本の3D境界スプラインからの双線形 / 双3次ブレンド曲面自動補間。 |
| **4辺 Gregory パッチ** | `gregory_patch` | Chiyokura-Kimura有理ツイスト補間による、4境界およびクロス方向接線と厳密に $G^1$ 連続なブレンド曲面。 |
| **N辺コーナーブレンド** | `gregory_patch` | 3本以上の境界曲線（$N \ge 3$）が集まる多面頂点に対し、中心リブ分割により $N$ 個の4辺グレゴリーパッチで穴を完全密閉。 |
| **Gordon 曲線ネットワーク** | `gordon_surface` | 格子状に交差する曲線網を通る滑らかな自由曲面生成。 |
| **3角形 Bézier / NURBS** | `triangular_patch` | 3境界からの重心座標系 $(u, v, w)$ による非四角形トリパッチ曲面補間。 |
| **曲面間フィレットブレンド** | `surface_blend` | $G^1$（接線連続）/ $G^2$（曲率連続）接続ブレンド曲面。 |
| **曲面-曲面幾何交差 (SSI)** | `ssi_march` | 4式4未知数のニュートン追跡 Marching ＋ B-spline曲線フィッティング（点列誤差 $< 10^{-12}$、曲線偏差 $< 10^{-7}$）。 |
| **トリム曲面 (Trimmed Surface)** | `trimmed_surface` | UVパラメータ領域内の2D NURBS閉境界による内外判定・トリム。 |
| **最小回転標架 (RMF)** | `sweep` | Bishop標架 / Rodrigues回転によるねじれ（Twist）のない3D曲線進行標架。 |
| **ロバスト幾何述語** | `zenith_math::predicates` | Jonathan Shewchuk の適応精度浮動小数点述語（`robust::orient2d`, `orient3d`）統合によるクラッシュ防止。 |
| **最短距離・最近傍点探索** | `extremum` | 3D点からNURBS曲線（1変数）・曲面（2変数）への最短距離パラメータ探索（ニュートン・ラフソン法）。 |

---

### 2. B-Rep トポロジー構造（Topology Layer: `zenith_topo`）
境界表現によるマニホールド幾何モデル管理。

| 構造体 / クラス | 説明・スペック |
| :--- | :--- |
| **`Vertex`** | 3次元座標点（`Point3`）と線形公差（`tolerance`）を持つトポロジー頂点。ユニークID自動採番。 |
| **`Edge` / `OrientedEdge`** | 3D幾何曲線（`NurbsCurve3`）と始点・終点頂点。順方向（Forward）/ 逆方向（Reversed）の向き管理。2-Manifold対向参照自動整合。 |
| **`Wire`** | 連続したエッジ列で構成される閉ループ境界。オイラー閉ループ検証・反時計回り（CCW）配向管理。 |
| **`Face`** | 基礎曲面幾何（`FaceGeometry`）＋ 外側Wire（`outer_wire`）＋ 内部穴Wire群（`inner_wires`）＋ 2D UV境界（`PCurve`）。 |
| **`Shell`** | Faceの連結集合。開シェル（Open Shell）および閉シェル（Closed Shell）判定。境界エッジの一致・多様体検査。 |
| **`Solid`** | 外殻閉シェル（`outer_shell`）および内部空洞シェル群（`inner_shells`）を持つ3次元完全マニホールド立体。面の向きは外向きに揃えられ、**符号付き体積が正であること**を `face_orientation_test` が全ビルダーについて検査。 |
| **`Assembly` / `ComponentInstance`** | 複数のソリッドを 4x4 アフィン変換行列（`Transform3`）で空間配置・階層管理するマルチボディ構造。 |

---

### 3. 形状生成・フィーチャーモデリング（Modeling Layer: `zenith_algo`）
CADのコアとなる立体の生成・加工・変形アルゴリズム群。

| 機能 | 実装モジュール | スペック・技術仕様詳細 |
| :--- | :--- | :--- |
| **基本立体 (Primitives)** | `primitive` | 直方体（Box）、円柱（Cylinder）、球（Sphere: 8象限パッチ）、円錐（Cone）、円環（Torus: 4パッチ）。厳密解析幾何とNURBSの二重構造。 |
| **押し出し (Extrude)** | `extrude` | 任意閉ワイヤ断面の直線押し出し。平面底面・天面と有理側面パッチの自動生成。中空断面（Hollow）対応。 |
| **回転体 (Revolve)** | `revolve` | 任意軸まわりの回転ソリッド（360度完全閉ループ、部分回転角度指定）。真円NURBSパッチ接続。 |
| **ロフト (Loft)** | `loft` | 複数断面（NURBSワイヤ）間の有理Bスプライン補間ソリッド。ガイド曲線（Guide Curve）指定による形状制御。 |
| **スイープ (Sweep)** | `sweep` | 3D曲線軌道（3Dスプライン、螺旋 Helix）に沿った閉断面の掃引。一定断面積の数学的保証。 |
| **厚み付け (Thicken)** | `thicken` | 単一パッチおよび**開いた複数面シートシェル全体（`thicken_shell`）**の法線方向オフセットと側面閉鎖による完全密閉ソリッド化。 |
| **インボリュート歯車** | `gear` | ピッチ円、基礎円、歯先円、歯底円、**ホブ盤創成トロコイドS字歯元フィレット**による工業規格インボリュート平歯車。 |
| **工業用穴・ザグリ・皿穴** | `hole` | 貫通丸穴、ザグリ穴（Counterbore）、皿モミ穴（Countersink: 64通り全合格）。 |
| **厳密 B-Rep ブーリアン** | `boolean` | 直方体・円柱・球・角柱・穴あき立体の差（Difference）、和（Union）、積（Intersection）。検証ゲート `BooleanResultVerifier` による閉性・体積・内外判定保証。 |
| **面併合 (FaceMerger)** | `merge_faces` | ブーリアン出口での同一平面パッチ自動併合（`boolean_solids_exact_simplified`）。L字角柱 14面➔8面、穴あき 16面➔10面に最小化。 |
| **稜フィレット / 面取り** | `edge_blend` | 任意ソリッドの直線凸稜に対する有理2次円筒フィレット曲面および平面面取り。体積誤差 $< 10^{-11}$。 |

---

### 4. ダイレクトモデリング & 2Dスケッチ拘束（`direct_edit`, `sketch_solver`）

| 機能 | 実装モジュール | スペック・技術仕様詳細 |
| :--- | :--- | :--- |
| **エッジ自動解析 & ブレンド** | `direct_edit` | ソリッド全体の凸稜・凹稜・二面角・許容半径を自動判定（`list_blendable_edges`）し、一括または個別でフィレット/面取り（`fillet_solid_edge`）。 |
| **プッシュプル (Push-Pull)** | `direct_edit` | 選択面の法線方向オフセットによる立体寸法変更。 |
| **抜き勾配 (Taper / Draft)** | `direct_edit` | 金型離型用の側面ドラフト角度付与。 |
| **2Dスケッチ拘束ソルバー** | `sketch_solver` | Levenberg-Marquardt最適化 ＋ SVDヤコビアン階数解析。一致、水平/垂直、平行、直交、正接、等長、距離、半径拘束。自由度（DOF）と過剰/矛盾拘束の自動診断。 |

---

### 5. 評価・物性値・テッセレーション（`mass_properties`, `slice`, `interference`, `zenith_tess`）

| 機能 | 実装モジュール | スペック・技術仕様詳細 |
| :--- | :--- | :--- |
| **質量物性値計算** | `mass_properties` | ガウス発散定理による体積、表面積、重心座標、3x3 慣性テンソル、主慣性モーメントの厳密積分。 |
| **断面スライサー** | `slice` | 任意平面での3Dソリッド切断。閉断面ポリライン抽出。断面積相対誤差 **$< 10^{-10}$**。 |
| **干渉判定 (Interference)** | `interference`, `distance` | AABB全頂点検査 ＋ B-Rep最近傍点射影による $0.001\text{ mm}$ の浅い食い込み干渉（Clash）確実検出。 |
| **完全閉テッセレーション** | `zenith_tess::stitched` | 構造格子規則性の維持により、**全分割数（4〜32分割）で 100% 完全閉多様体メッシュ（穴・非多様体・退化三角形ゼロ）** を生成。 |

---

### 6. データ交換 & 外部連携（`zenith_io`, `zenith_py`）

| フォーマット / バインディング | 対応規格 | スペック詳細 |
| :--- | :--- | :--- |
| **STEP 入出力** | ISO 10303-21 (AP203 / AP214 / AP242) | Manifold Solid B-Rep、有理B-spline曲面、Trimmed Surface、p-curve完全対応。他カーネル読み書き往復検証済み。 |
| **IGES エクスポート** | IGES 5.3 | Entity 128 (NURBS Surface), 102 (Composite Curve), 124 (Transformation Matrix) 出力。 |
| **2D DXF 図面出力** | AutoCAD DXF (AC1015) | 断面スライサーからの **レイヤー分け（OUTLINE, HOLE, CENTERLINE, HATCH）閉ポリライン図面** 自動生成。 |
| **OBJ / バイナリSTL / glTF** | 3D Mesh Formats | 頂点法線付きWavefront OBJ、バイナリSTL、glTF 2.0 JSON 出力。 |
| **Python C-Extension** | Python 3.10 / 3.11 / 3.12 (PyO3) | **`zenith_cad.pyd`（わずか 3.85 MB、単一ファイル）** によるインプロセス完全インメモリ連携。 |

---

## 🏆 テスト・検証実績総括

- **ワークスペース全テスト**: **64 テストバイナリ / 422 テスト 100% 合格（0 failed, 0 ignored）**
- **ビルダー監査 (`builder_audit`)**: **24 ケースすべてクリーン**
- **FreeCAD ヘッドレス相互検証**: **15/15 ケース完全一致**
- **OpenCASCADE ショーケース**: **24/24 ケースが valid closed solid**
