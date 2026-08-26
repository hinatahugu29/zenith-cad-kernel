# 🚀 Zenith CAD Kernel - スペック総覧（棚卸し）＆ 次なる飛躍への展望

**文書バージョン**: v3.4.5 (4-102の稜ブレンド個別診断を反映)
**最終更新日時**: 2026年8月26日
**ステータス**: 完全自前 Rust B-Rep エンジン。**本書の数値はすべて実測値**。

> **v3.1.0 〜 v3.3.0 での訂正と実装について**
>
> v3.0.0 の記述のうち、外の物差しに当てた結果が残っていなかったものを、
> 実測に合わせて書き直しました。誇張していた項目（IGES、Gregory パッチ、
> 歯元トロコイド、DXF のレイヤー、p-curve 出力）は、できていることと
> できていないことを分けて書いてあります。経緯は
> [`HANDOVER.md`](HANDOVER.md) の 4-37 〜 4-41 にあります。
>
> **達成していること（すべて外部検証つき）**:
> - 出力用メッシュの完全閉多様体化。4〜32分割に加え48〜256でも open: 0, non-manifold: 0, degenerate: 0。ブーリアン曲面分割を含む `contact_placement_probe` も7配置・21演算でB-Rep / mesh異常0となり、赤ゲートへ昇格（修正前は5件・9〜126本。HANDOVER 3-N-2b、4-83〜4-89）。**これは常設検体の実測範囲で、任意の全立体の証明ではありません。**
> - 書き出す STEP が ISO 10303-21 の構文に適合。OpenCASCADE が代表25形状すべてを valid closed solid として読む。
> - 他カーネルのファイルを読んで書き戻した7形状が、解析解と 1e-11〜1e-13 で一致（OpenCASCADE 自身の NURBS 変換より高精度）。
> - FreeCAD ヘッドレス相互検証 27/27。
> - 単一の軽量C-Extension（`zenith_cad.pyd` 3.85MB）のみで外部依存ゼロ。

Zenith CAD Kernel は、Rust でフルスクラッチ開発された **次世代型 3次元 B-Rep / 自由曲面 NURBS CAD カーネル** です。  
巨大な外部ライブラリ（OpenCASCADE / pythonocc / FreeCAD）を一切介さず、**単一の軽量アドオン（`zenith_cad.pyd`）のみで完結する「真の脱OCCT」** を達成しています。

本書は、現時点で達成された **全機能スペックの完全な棚卸し** と、業界標準 CAD（FreeCAD / OpenCASCADE）による **ヘッドレス自動検証実績**、および今後世界最高峰のモデリング環境へと **さらに飛躍するための技術構想** をまとめた公式仕様書です。

---

## 📊 現行スペック総覧・機能棚卸し（Specs Inventory）

```mermaid
graph TD
    A[Zenith CAD Kernel Core v3.3.0] --> B[1. 数値幾何・自由曲面エンジン]
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
| **4辺グレゴリーパッチ** | `gregory_patch` | 双3次ベジエの内部4点を**双子**で持ち、`(u,v)` で有理的に混ぜる（`GregoryPatch4::with_ribbons`）。4本の境界曲線と、4辺それぞれのクロス方向接線（`CrossRibbon`）を同時に満たす。隅のツイストが揃っている必要がない。実測: 境界 $2.3\times10^{-16}$、接線の残差は差分の刻み幅に比例して落ちる（刻み $10^{-6}$ で $3.4\times10^{-5}$）。境界は1スパンの非有理ベジエ（次数3以下）であること。 |
| **N辺コーナーブレンド** | `gregory_patch` | 3本以上の境界曲線（$N \ge 3$）が集まる多面頂点を、中心リブと境界の二等分により $N$ 個のグレゴリーパッチで密閉（`CornerBlendN`）。$N = 3, 4, 5$ で $N$ 枚を返す。隣り合うセルは共有するリブの上で**隙間 0.000e0、法線の食い違い 0.0000 rad**。 |
| **Gordon 曲線ネットワーク** | `gordon_surface` | 格子状に交差する曲線網を通る滑らかな自由曲面生成。 |
| **3角形 Bézier / NURBS** | `triangular_patch` | 3境界からの重心座標系 $(u, v, w)$ による非四角形トリパッチ曲面補間。 |
| **曲面間フィレットブレンド** | `surface_blend` | 2曲面の間を埋めるブレンド曲面（`create_g1_blend` / `create_g2_blend`）。**連続性を数で検査したテストは無く、`create_g2_blend` の膨らみは `0.8 * curvature_scale` という経験係数**。`FaceGeometry` に入っていないので B-Rep の面としては使えない。 |
| **曲面-曲面幾何交差 (SSI)** | `ssi_march` | 4式4未知数のニュートン追跡 Marching ＋ B-spline曲線フィッティング（点列誤差 $< 10^{-12}$、曲線偏差 $< 10^{-7}$）。 |
| **トリム曲面 (Trimmed Surface)** | `trimmed_surface` | UVパラメータ領域内の2D NURBS閉境界による内外判定・トリム。**`FaceGeometry` に含まれていない**ので、B-Rep の面としては使えない（面のトリムは境界ワイヤと p-curve が担っている）。 |
| **最小回転標架 (RMF)** | `sweep` | Bishop標架 / Rodrigues回転によるねじれ（Twist）のない3D曲線進行標架。 |
| **ロバスト幾何述語** | `zenith_math::predicates` | Jonathan Shewchuk の適応精度浮動小数点述語（`robust::orient2d`, `orient3d`）統合によるクラッシュ防止。 |
| **最短距離・最近傍点探索** | `extremum` | 3D点からNURBS曲線（1変数）・曲面（2変数）への最短距離パラメータ探索（ニュートン・ラフソン法）。立体の境界への射影は `zenith_algo::nearest_boundary_projection` が担い、トリムの外に落ちた足は面の境界の稜へ寄せる。回転面の極では法線の**極限**を採る（`normal_or_limit`）。**他カーネルから読んだ球・円柱・円錐・トーラス**に対し、軸の上・縁の斜め外・頂点の真上を含む36チェックが閉じた式と一致（最悪 3.55e-15、`foreign_distance_probe`）。 |

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
| **厚み付け (Thicken)** | `thicken` | 単一パッチ（平面 / NURBS / Coons）の法線方向オフセットと側面閉鎖による完全密閉ソリッド化。`thicken_shell` は複数面シェルを受け取るが、**各面を個別に厚み付けして Union するだけ**で、検体は同一平面の長方形2枚のみ。非平面の隣接シートでの保証はない。 |
| **インボリュート歯車** | `gear` | ピッチ円、基礎円、歯先円、歯底円による工業規格インボリュート平歯車。`make_spur_gear_with_root_fillet` の歯元は、**ホブ歯先丸みが転がりながら掃く円の包絡線**（創成トロコイド）。工具円の食い込み 0（厳密）、フィレット上の全点が工具円に乗る（5.08e-7）。外端はインボリュートの開始点と 1e-6 rad 以内で一致し、これは `inv` の式との**独立な**突き合わせ。既定の `make_spur_gear` は歯元が半径方向の直線で、そちらは断面積の閉じた式（`involute_profile_area`、実測 1.99e-9）を持つ。 |
| **工業用穴・ザグリ・皿穴・長穴** | `hole` | 貫通丸穴、ザグリ穴（Counterbore）、皿モミ穴（Countersink: 64通り全合格）、座ぐり長穴（Counterbored Stadium Slot: 19面閉多様体）。 |
| **金型抜き勾配 (Draft)** | `draft` | 基準底面から指定角度 $\theta$ で均等テーパー傾斜させた角錐台ソリッド（`make_drafted_block`、非相似角錐台体積誤差 $3.06 \times 10^{-16}$）。 |
| **剛性補強リブ (Rib)** | `rib` | 直角三角形断面を持つ5面構成のガセットリブソリッド（`make_triangular_rib`、解析解体積誤差 $0.00$ 完全一致）。 |
| **機械締結ファスナー (Fastener)** | `fastener` | JIS/ISO規格準拠の正六角柱（ボルト頭: 誤差 $5.70 \times 10^{-16}$）、六角ナット（誤差 $2.70 \times 10^{-13}$）、六角穴付きボルト（Cap Screw: 誤差 $1.01 \times 10^{-13}$）、平座金（Plain Washer: 誤差 $1.56 \times 10^{-13}$）、フランジ付き六角ボルト（Flanged Hex Bolt: 誤差 $2.06 \times 10^{-13}$）、スプリングワッシャー（Spring Lock Washer: 6面矩形線材ヘリカルスプリットリング）。 |
| **動力伝達軸・キー溝 (Shaft)** | `shaft` | 任意段数の多段円柱結合（`make_stepped_shaft`）、JIS B 1301 準拠の平行キー溝切削（`make_shaft_with_keyway`）、止め輪・Oリング用環状溝（`make_shaft_with_annular_groove`）。 |
| **厳密 B-Rep ブーリアン** | `boolean` | 直方体・円柱・球・角柱・穴あき立体の差（Difference）、和（Union）、積（Intersection）。検証ゲート `BooleanResultVerifier` による閉性・体積・内外判定保証。**他カーネルが書いた立体**についても、軸に平行な切り手30配置・90演算、**27度傾けた切り手を足した60配置・180演算とも、断るものはありません**（WRONG 0・PANIC 0、恒等式の残差 7.88e-9）。**検証つきの口でも 180/180**（HANDOVER 4-61〜4-68）。自作立体どうしの45ケース表（`boolean_envelope`）は **supported 44 / wrong-result 0 / エラー 1**。**残る1件は直す対象ではありません**——`box × cylinder`（接線）の差で、**答えのほうが非多様体**なので場所を名指しして断ります（HANDOVER 4-74、4-80）。 |
| **面併合 (FaceMerger)** | `merge_faces` | ブーリアン出口での同一平面パッチ自動併合（`boolean_solids_exact_simplified`）。L字角柱 14面➔8面、穴あき 16面➔10面に最小化。 |
| **稜フィレット / 面取り** | `edge_blend` | 直線稜×平面2面、純直円柱・純円錐/円錐台の凸円周、貫通円筒穴口、**円筒ボス根元・段付き軸の小径側90度凹円周**、**平面肩×直円錐ボスの非直角凹円周**を、いずれもフィレット/面取りする。円錐系は厳密有理トーラス扇形/円錐台で、上下リム、真円錐の実頂点、OCC全周円、剛体配置、閉形式体積、STEP、mesh 4〜32を検証。非円形根元は個別診断で理由を返して明示拒否（HANDOVER 4-72、4-92〜4-102）。 |

---

### 4. ダイレクトモデリング & 2Dスケッチ拘束（`direct_edit`, `sketch_solver`）

| 機能 | 実装モジュール | スペック・技術仕様詳細 |
| :--- | :--- | :--- |
| **エッジ自動解析 & ブレンド** | `edge_blend` | `EdgeBlender::blendable_edges`で処理可能稜を列挙し、`blendability`で1稜を非破壊に事前照会して二面角・許容寸法または拒否理由を得る。一括または個別でフィレット/面取り。`DirectModeling::fillet_solid_edge` などは薄い別名で、実体は `edge_blend` 側にある。 |
| **プッシュプル (Push-Pull)** | `direct_edit` | 選択面の法線方向オフセットによる立体寸法変更。 |
| **抜き勾配 (Taper / Draft)** | `direct_edit` | 金型離型用の側面ドラフト角度付与。 |
| **2Dスケッチ拘束ソルバー** | `sketch_solver` | Levenberg-Marquardt最適化 ＋ **自由変数に制限した**ヤコビアンのSVD階数解析。一致、水平/垂直、平行、直交、正接、等長、距離、半径拘束。自由度（DOF）と冗長拘束の診断。**「冗長」と「矛盾」は区別していない**（残差を見ていないため）。 |

---

### 5. 評価・物性値・テッセレーション（`mass_properties`, `slice`, `interference`, `zenith_tess`）

| 機能 | 実装モジュール | スペック・技術仕様詳細 |
| :--- | :--- | :--- |
| **質量物性値計算** | `mass_properties` | ガウス発散定理による体積、表面積、重心座標、3x3 慣性テンソル、主慣性モーメントの厳密積分。 **他カーネルから読んだ**球・円柱・円錐・トーラスでも、体積・重心・原点まわりの慣性が閉じた式と **8.49e-11 以内**（32チェック・外れ0、`foreign_inertia_probe`）。面1枚の面積と重心は `DirectModeling::inspect_face` で、平面は Green の定理により**厳密**（読んだ円柱の蓋で 1.45e-14）。 |
| **断面スライサー** | `slice` | 任意平面での3Dソリッド切断。閉断面ポリライン抽出。断面積相対誤差 **$< 10^{-10}$**。輪郭の連結は距離ではなく**メッシュの位相**（頂点添字・辺添字）で行うため、溶接公差に依存しない。4〜256分割の全ケースで閉じる（`slice_robustness_probe`）。 **他カーネルから読んだ**球・円柱・円錐・トーラスの断面も、面積・周長・ループ数とも閉じた式に乗り、64→128→256 で誤差が縮む（24チェック・外れ0、`foreign_slice_probe`）。 |
| **干渉判定 (Interference)** | `interference`, `distance` | AABB全頂点検査 ＋ B-Rep最近傍点射影による $0.001\text{ mm}$ の浅い食い込み干渉（Clash）確実検出。 射影はトリムの外に落ちた足を**面の境界の稜へ寄せ**、回転面の極では法線の**極限**を採ります。**他カーネルから読んだ**球・円柱・円錐・トーラスへの距離と内外が、軸の上・縁の斜め外・頂点の真上を含む36チェックで閉じた式と一致（最悪 3.55e-15、`foreign_distance_probe`）。 |
| **完全閉テッセレーション** | `zenith_tess::stitched` | 構造格子規則性の維持により、`mesh_watertight_probe` の検体では、**測った全分割数（4〜256分割）で100%完全閉多様体メッシュ（穴・非多様体・退化三角形ゼロ）**。ブーリアン曲面分割を含む `contact_placement_probe` も7配置・21演算でB-Rep / mesh異常0となり、再発時はexit 1。**任意の全立体を証明するものではなく常設検体の実測範囲**です（修正前は5件・9〜126本。HANDOVER 3-N-2b、4-83〜4-89）。格子パッチ内では適応細分を掛けないため、弦誤差は分割数で制御する（32分割で体積差1e-3台）。格子から落ちて earcut＋適応細分へ行った面は `grid_fallback_probe` が検体ごとに数えます。 |

---

### 6. データ交換 & 外部連携（`zenith_io`, `zenith_py`）

| フォーマット / バインディング | 対応規格 | スペック詳細 |
| :--- | :--- | :--- |
| **STEP 出力** | ISO 10303-21 (AP214) | `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`、`ADVANCED_FACE`、`PLANE`、有理B-spline曲面・曲線、稜を実体として共有する `EDGE_CURVE`。**2D トリム境界を `SURFACE_CURVE` ＋ 面ごとの `PCURVE` として出力**（1稜につきちょうど2本。書いている p-curve は 3D の辺と 3.2e-15〜2.9e-12 で一致）。複合エンティティ実体の括弧を含め Part 21 の構文に適合。**解析曲面は読めても書き出しは B-spline 化される** — 実装しないと決めた項目で、理由は HANDOVER 4-41 に。 |
| **STEP 入力** | ISO 10303-21 | **曲面**: `PLANE` / `CYLINDRICAL` / `CONICAL` / `SPHERICAL` / `TOROIDAL` / 有理B-spline / `SURFACE_OF_LINEAR_EXTRUSION` / `SURFACE_OF_REVOLUTION`（角度方向は有理2次、断面方向は元の曲線の次数とノットで**厳密**。断面が軸を含む平面から外れていれば断る）。**曲線**: `LINE` / `CIRCLE` / `ELLIPSE` / `TRIMMED_CURVE` / `SURFACE_CURVE` / 有理B-spline。`FACE_BOUND` のみのファイルからの外周ループ自動判定、`BREP_WITH_VOIDS`。**未対応**: `OFFSET_SURFACE`, `COMPOSITE_CURVE`, `POLYLINE`, 開シェル系表現 — 当たると**エンティティ名を名指ししてエラー**になる（既定の平面や直線に差し替えて黙って進むことはしない）。**この一覧は、その実体が検体に入っていることを確かめてから書いてください**——`SURFACE_OF_REVOLUTION` は長らく「対応」と書かれた道具がありながら、検体に1つも入っておらず実際には読めませんでした（HANDOVER 4-60）。 |
| **IGES エクスポート** | IGES 5.3 | 各面の支持曲面を **Entity 128（有理Bスプライン曲面）** として出力。80桁固定レコード、Global 26フィールド、D/P セクション対応。**トリム（Entity 144 / 142 / 126）は未出力**。`tools/verify_iges.py` で OpenCASCADE が5検体すべてを読み、曲面枚数一致・境界箱のずれ 0。 |
| **2D DXF 図面出力** | AutoCAD DXF (AC1015) | 断面スライサーからの閉ポリライン図面（LWPOLYLINE）。レイヤーは OUTLINE / HOLE / CENTERLINE / HATCH を **テーブルに定義**するが、`generate_dxf_string` が自動で割り当てるのは OUTLINE と HOLE のみ。線種は全レイヤー CONTINUOUS で、HATCH エンティティは出力しない。 |
| **OBJ / バイナリSTL / glTF** | 3D Mesh Formats | 頂点法線付きWavefront OBJ、バイナリSTL、glTF 2.0 JSON 出力。 |
| **Python C-Extension** | Python 3.10 / 3.11 / 3.12 (PyO3) | **`zenith_cad.pyd`（わずか 3.85 MB、単一ファイル）** によるインプロセス完全インメモリ連携。 |

---

## 🏆 テスト・検証実績総括

すべて 2026年8月26日までの実測値です。数字の出どころは
[`VERIFICATION_PLAYBOOK.md`](VERIFICATION_PLAYBOOK.md) の手順で再現できます。

| 何を測ったか | 結果 | 再現コマンド |
| :--- | :--- | :--- |
| ワークスペース全テスト | **103 バイナリ（doctest 込み）/ 591 テスト 100% 合格**（0 failed, 0 ignored） | `cargo test --release --workspace --exclude zenith_py` |
| コンパイラ警告 | **0** | `cargo build --release --workspace --exclude zenith_py` |
| ビルダー監査 | **24/24 クリーン**（解析解との差は最悪 6.3e-13、歯車 1.99e-9） | `--example builder_audit` |
| 平面を NURBS で持つ面 | **全23ビルダーで0枚** | `--example planar_face_audit` |
| FreeCAD ヘッドレス相互検証 | **27/27 完全一致**（ゲート、不一致で非ゼロ終了） | `tools/freecad_cross_validate.py` |
| OpenCASCADE ショーケース | **53/53 が valid closed solid（100% 合格）** | `py tools/verify_showcase.py` |
| 他カーネルからの読み書き一周 | **7/7 が解析解と 1e-11〜1e-13** | `tools/verify_reexport.py` |
| IGES 相互検証 | **5/5**（曲面枚数一致、境界箱のずれ 0） | `tools/verify_iges.py` |
| 常設プローブ | **35/35 が exit 0** | 上記手順書の一覧 |
| Python インプロセス往復 | 全合格 | `py tools/verify_solid_api.py` |
