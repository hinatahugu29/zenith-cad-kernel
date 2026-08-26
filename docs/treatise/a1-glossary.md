# 付録 A：CAD 幾何・B-Rep・NURBS 専門用語集 (Glossary)

| 用語 | 英語表記 | 定義・技術的解説 |
| :--- | :--- | :--- |
| **B-Rep** | Boundary Representation | 3次元立体を「境界面の集合」として表現するデータ構造。頂点（Vertex）、稜線（Edge）、面（Face）、殻（Shell）、立体（Solid）の位相関係で幾何を束ねる。 |
| **NURBS** | Non-Uniform Rational B-Spline | 非均一有理Bスプライン。自由曲面から円錐曲線（真円、楕円、放物線）までを統一的に表現可能な業界標準の数学表現。 |
| **p-curve** | Parameter Space Curve | 3次元曲面の2次元パラメータ空間 $(u,v)$ 上に定義された曲線。トリム領域の内外判定や面分割の基準となる。 |
| **SSI** | Surface-Surface Intersection | 2つの3次元曲面が交差する線（交線）を数値解析的（Newton-Raphson等）に求めるアルゴリズム。 |
| **Gregory Patch** | Gregory Patch | 4本の境界スプラインから曲面を補間する際、四隅のツイスト不整合を双子制御点の有理ブレンドで解消し、$G^1$ 連続性を保証するパッチ。 |
| **Watertight Manifold** | 完全閉多様体 | 隙間（Crack）、孤立頂点、非多様体稜線、縮退面が存在せず、完全に密閉された3次元立体メッシュ。3Dプリントや流体解析の必須条件。 |
| **Shewchuk Predicate** | Shewchuk 幾何述語 | Jonathan Shewchuk が考案した適応精度浮動小数点演算。幾何学的判定（Orient2D/Orient3D）での符号逆転や桁落ちによるクラッシュを完全に防ぐ。 |
| **Euler-Poincaré Formula** | オイラー・ポアンカレの公式 | 多様体立体の頂点数・エッジ数・面数・シェル数・穴数・種数の間に成り立つ位相幾何学的恒等式 ($V - E + F = 2(S - G) + H$)。 |
| **$G^0, G^1, G^2$ 連続性** | Geometric Continuity | 幾何学的連続性。$G^0$ は位置の接続、$G^1$ は接線ベクトル（法線方向）の連続、$G^2$ は曲率（Curvature）の滑らかな連続を表す。 |
| **STEP AP214 / AP242** | ISO 10303 Application Protocol | 自動車・航空宇宙等の製造業で標準的に用いられる CAD データ交換規格。厳密な B-Rep ソリッド階層を規定。 |
| **RMF** | Rotation Minimizing Frame | 最小回転標架。3次元曲線をスイープ掃引する際、不自然なねじれ（Twist）を数学的に最小化する直交座標系進行標架。 |
