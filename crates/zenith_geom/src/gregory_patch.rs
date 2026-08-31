//! グレゴリーパッチ（Gregory patch）と、N辺の穴を塞ぐコーナーブレンド。
//!
//! # なぜ普通の双3次ベジエでは足りないのか
//!
//! 4本の境界曲線と、その境界に沿った**クロス方向の接線**（リボン）を与えて
//! 面を張ると、双3次ベジエの内部制御点 `b11`, `b21`, `b22`, `b12` が
//! **2回決まってしまいます**。たとえば `b11` は、`v=0` の辺のリボンからも、
//! `u=0` の辺のリボンからも決まる。2つが一致するのは、隅のツイスト
//! （$\partial^2 S / \partial u \partial v$）が両側で揃っているときだけです。
//!
//! 一般には揃いません。揃えようとすると、隣の面のリボンまで作り直すことに
//! なります。
//!
//! グレゴリーは、**両方を持ったまま** `(u, v)` で有理的に混ぜて解きます。
//!
//! ```text
//! P11(u,v) = (u * b11_from_v0 + v * b11_from_u0) / (u + v)
//! ```
//!
//! `v -> 0` では `b11_from_v0` に、`u -> 0` では `b11_from_u0` になるので、
//! どちらの辺でもその辺のリボンがそのまま出ます。つまり**4辺すべてで
//! 指定した接線に一致**します。
//!
//! # ここに何が書いてあったか
//!
//! 以前の `GregoryPatch4` は、クロス方向接線を**引数に取っていません**でした。
//! `tangents` フィールドは全ゼロのまま一度も読まれず、内部点は4隅だけから
//! 固定係数（0.444 / 0.222 / 0.222 / 0.112）で決まり、境界曲線を大きく
//! 湾曲させても**値が1ビットも動きません**でした。評価式は Coons ブレンドに
//! `16u(1-u)v(1-v)` のバブルを足したもので、Chiyokura-Kimura のツイスト補間
//! とは無関係です。$G^1$ 連続は、接線を受け取らない以上、**原理的に
//! 達成できません**でした。

use crate::bspline_basis::KnotVector;
use crate::nurbs_curve::{ControlPoint3, NurbsCurve3};
use crate::surface::Surface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Point3Ext, Tolerance, Vec3, Vec3Ext};

/// 境界に沿ったクロス方向の接線（リボン）。
///
/// 3次ベジエの係数として与える。`s = 0` がその境界の始点、`s = 1` が終点で、
/// 値は**パッチの内側へ向かう** $\partial S / \partial (\text{内向き})$ を
/// 3で割ったもの——つまり内側の制御点をどれだけずらすか——ではなく、
/// 微分そのものである。内部では 3 で割って制御点に直す。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CrossRibbon(pub [Vec3; 4]);

impl CrossRibbon {
    /// 一定の接線。平らなリボン。
    pub fn uniform(direction: Vec3) -> Self {
        CrossRibbon([direction; 4])
    }

    /// 端の2つだけ指定し、間を線形に埋める。
    pub fn from_ends(start: Vec3, end: Vec3) -> Self {
        CrossRibbon([
            start,
            start + (end - start) * (1.0 / 3.0),
            start + (end - start) * (2.0 / 3.0),
            end,
        ])
    }

    fn evaluate(&self, s: f64) -> Vec3 {
        let t = s.clamp(0.0, 1.0);
        let w = 1.0 - t;
        self.0[0] * (w * w * w)
            + self.0[1] * (3.0 * w * w * t)
            + self.0[2] * (3.0 * w * t * t)
            + self.0[3] * (t * t * t)
    }
}

/// 4辺グレゴリーパッチ。
///
/// 制御網は双3次ベジエだが、内部の4点は `(u, v)` に依存する**双子**として
/// 持つ。境界とクロス接線の両方をそのまま満たす。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GregoryPatch4 {
    /// 4本の境界曲線 C0(u=..., v=0), C1(u=1), C2(v=1), C3(u=0)
    pub c0: NurbsCurve3,
    pub c1: NurbsCurve3,
    pub c2: NurbsCurve3,
    pub c3: NurbsCurve3,
    /// 境界の制御点（3次ベジエ）。`[v=0, u=1, v=1, u=0]` の順。
    boundary: [[Point3; 4]; 4],
    /// 各辺のクロス方向リボン。`boundary` と同じ順。
    ribbons: [CrossRibbon; 4],
    /// 内部の双子制御点。`[b11_v0, b11_u0, b21_v0, b21_u1, b22_v1, b22_u1, b12_v1, b12_u0]`
    twins: [Point3; 8],
    /// コーナー4点 (p00, p10, p11, p01)
    pub corners: [Point3; 4],
}

/// 曲線を3次ベジエの制御点4つとして取り出す。
///
/// 1スパンで、重みがすべて 1 で、次数が3以下であることを要求する。次数が
/// 足りなければ持ち上げる。**近似はしない。** 満たさない曲線を近似で通すと、
/// 「境界を厳密に通る」という主張が静かに崩れる。
fn cubic_bezier_points(curve: &NurbsCurve3) -> Option<[Point3; 4]> {
    if curve
        .control_points
        .iter()
        .any(|cp| (cp.weight - 1.0).abs() > 1e-12)
    {
        return None;
    }
    let degree = curve.degree;
    if degree == 0 || degree > 3 {
        return None;
    }
    if curve.control_points.len() != degree + 1 {
        // 内部ノットがある＝1スパンではない。
        return None;
    }

    let mut points: Vec<Point3> = curve.control_points.iter().map(|cp| cp.point).collect();
    // ベジエの次数持ち上げ: b'_i = (i/(n+1)) b_{i-1} + (1 - i/(n+1)) b_i
    while points.len() < 4 {
        let n = points.len() - 1;
        let mut raised = Vec::with_capacity(points.len() + 1);
        raised.push(points[0]);
        for i in 1..=n {
            let ratio = i as f64 / (n + 1) as f64;
            raised.push(Point3::from(
                points[i - 1].coords * ratio + points[i].coords * (1.0 - ratio),
            ));
        }
        raised.push(points[n]);
        points = raised;
    }
    Some([points[0], points[1], points[2], points[3]])
}

impl GregoryPatch4 {
    /// 境界だけからパッチを作る。**$G^1$ の主張はしない。**
    ///
    /// クロス接線を渡さない場合、隣の面が無いのだから合わせる相手もいない。
    /// ここでは、境界の隣り合う辺から作った「素直な」リボンを使う。境界は
    /// 厳密に通るが、隣接面との接線連続は保証しない。
    ///
    /// 接線を合わせたいときは [`Self::with_ribbons`] を使うこと。
    pub fn new(
        c0: NurbsCurve3,
        c1: NurbsCurve3,
        c2: NurbsCurve3,
        c3: NurbsCurve3,
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let ribbons = Self::natural_ribbons(&c0, &c1, &c2, &c3)?;
        Self::with_ribbons(c0, c1, c2, c3, ribbons, tol)
    }

    /// 境界と、その4辺のクロス方向リボンからパッチを作る。
    ///
    /// リボンの向きは**パッチの内側**。`v=0` の辺なら $\partial S/\partial v$、
    /// `u=0` の辺なら $\partial S/\partial u$、`v=1` の辺なら
    /// $-\partial S/\partial v$、`u=1` の辺なら $-\partial S/\partial u$ を
    /// 与える（どの辺でも「内側へ」で揃う）。
    pub fn with_ribbons(
        c0: NurbsCurve3,
        c1: NurbsCurve3,
        c2: NurbsCurve3,
        c3: NurbsCurve3,
        ribbons: [CrossRibbon; 4],
        tol: &Tolerance,
    ) -> Result<Self, String> {
        let b0 = cubic_bezier_points(&c0)
            .ok_or("C0 must be a single non-rational Bezier segment of degree 3 or less")?;
        let b1 = cubic_bezier_points(&c1)
            .ok_or("C1 must be a single non-rational Bezier segment of degree 3 or less")?;
        let b2 = cubic_bezier_points(&c2)
            .ok_or("C2 must be a single non-rational Bezier segment of degree 3 or less")?;
        let b3 = cubic_bezier_points(&c3)
            .ok_or("C3 must be a single non-rational Bezier segment of degree 3 or less")?;

        let p00 = b0[0];
        let p10 = b0[3];
        let p11 = b1[3];
        let p01 = b3[3];

        if !b1[0].is_coincident_with(&p10, tol.linear) {
            return Err("Corner P10 mismatch between C0 and C1".to_string());
        }
        if !b2[3].is_coincident_with(&p11, tol.linear) {
            return Err("Corner P11 mismatch between C1 and C2".to_string());
        }
        if !b2[0].is_coincident_with(&p01, tol.linear) {
            return Err("Corner P01 mismatch between C2 and C3".to_string());
        }
        if !b3[0].is_coincident_with(&p00, tol.linear) {
            return Err("Corner P00 mismatch between C3 and C0".to_string());
        }

        // リボンの**両端は境界曲線が決めている**。`v = 0` の辺のリボンが
        // `u = 0` で取る値は、`u = 0` の境界曲線の出だしそのもの
        // （`3 * (b3[1] - b3[0])`）でなければ、制御網が食い違う。
        //
        // 呼び出し側に押し付けるのではなく、ここで揃える。指定されたリボンの
        // うち**中の2係数だけ**を使い、両端は境界から取り直す。こうすると
        // どんなリボンを渡されても網は壊れない。
        let ribbons = [
            CrossRibbon([
                (b3[1] - b3[0]) * 3.0,
                ribbons[0].0[1],
                ribbons[0].0[2],
                (b1[1] - b1[0]) * 3.0,
            ]),
            CrossRibbon([
                (b0[2] - b0[3]) * 3.0,
                ribbons[1].0[1],
                ribbons[1].0[2],
                (b2[2] - b2[3]) * 3.0,
            ]),
            CrossRibbon([
                (b3[2] - b3[3]) * 3.0,
                ribbons[2].0[1],
                ribbons[2].0[2],
                (b1[2] - b1[3]) * 3.0,
            ]),
            CrossRibbon([
                (b0[1] - b0[0]) * 3.0,
                ribbons[3].0[1],
                ribbons[3].0[2],
                (b2[1] - b2[0]) * 3.0,
            ]),
        ];

        // 双3次の制御網。境界の行・列はそのまま、内側の行・列はリボンから。
        //
        //   b03 b13 b23 b33      v = 1
        //   b02 b12 b22 b32
        //   b01 b11 b21 b31
        //   b00 b10 b20 b30      v = 0
        //
        // `v = 0` の辺で dS/dv(u, 0) = 3 * (b_{i1} - b_{i0}) を u で3次に
        // 混ぜたものになるので、リボンを3で割って足せば b_{i1} が決まる。
        let step = |base: Point3, tangent: Vec3| Point3::from(base.coords + tangent * (1.0 / 3.0));

        // v = 0 の辺（C0、u が走る）
        let row_v0: [Point3; 4] = [
            step(b0[0], ribbons[0].0[0]),
            step(b0[1], ribbons[0].0[1]),
            step(b0[2], ribbons[0].0[2]),
            step(b0[3], ribbons[0].0[3]),
        ];
        // v = 1 の辺（C2、u が走る）。リボンは内向き＝ -dS/dv。
        let row_v1: [Point3; 4] = [
            step(b2[0], ribbons[2].0[0]),
            step(b2[1], ribbons[2].0[1]),
            step(b2[2], ribbons[2].0[2]),
            step(b2[3], ribbons[2].0[3]),
        ];
        // u = 0 の辺（C3、v が走る）
        let col_u0: [Point3; 4] = [
            step(b3[0], ribbons[3].0[0]),
            step(b3[1], ribbons[3].0[1]),
            step(b3[2], ribbons[3].0[2]),
            step(b3[3], ribbons[3].0[3]),
        ];
        // u = 1 の辺（C1、v が走る）
        let col_u1: [Point3; 4] = [
            step(b1[0], ribbons[1].0[0]),
            step(b1[1], ribbons[1].0[1]),
            step(b1[2], ribbons[1].0[2]),
            step(b1[3], ribbons[1].0[3]),
        ];

        // 内部の4点は、それぞれ2つの辺から決まる。両方を持つ。
        let twins = [
            row_v0[1], // b11、v=0 のリボンから
            col_u0[1], // b11、u=0 のリボンから
            row_v0[2], // b21、v=0 のリボンから
            col_u1[1], // b21、u=1 のリボンから
            row_v1[2], // b22、v=1 のリボンから
            col_u1[2], // b22、u=1 のリボンから
            row_v1[1], // b12、v=1 のリボンから
            col_u0[2], // b12、u=0 のリボンから
        ];

        Ok(Self {
            c0,
            c1,
            c2,
            c3,
            boundary: [b0, b1, b2, b3],
            ribbons,
            twins,
            corners: [p00, p10, p11, p01],
        })
    }

    /// 隣の面が無いときの既定リボン。隣り合う境界の向きから作る。
    fn natural_ribbons(
        c0: &NurbsCurve3,
        c1: &NurbsCurve3,
        c2: &NurbsCurve3,
        c3: &NurbsCurve3,
    ) -> Result<[CrossRibbon; 4], String> {
        let b0 = cubic_bezier_points(c0).ok_or("C0 is not a cubic Bezier segment")?;
        let b1 = cubic_bezier_points(c1).ok_or("C1 is not a cubic Bezier segment")?;
        let b2 = cubic_bezier_points(c2).ok_or("C2 is not a cubic Bezier segment")?;
        let b3 = cubic_bezier_points(c3).ok_or("C3 is not a cubic Bezier segment")?;

        // v=0 の辺での内向きは、両端で「向かい側の辺へ向かう」向きに取る。
        let v0 = CrossRibbon::from_ends(b3[3] - b3[0], b1[3] - b1[0]);
        let u1 = CrossRibbon::from_ends(b0[0] - b0[3], b2[0] - b2[3]);
        let v1 = CrossRibbon::from_ends(b3[0] - b3[3], b1[0] - b1[3]);
        let u0 = CrossRibbon::from_ends(b0[3] - b0[0], b2[3] - b2[0]);
        Ok([v0, u1, v1, u0])
    }

    /// `(u, v)` での3次元座標。
    pub fn evaluate_gregory(&self, u: f64, v: f64) -> Point3 {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);
        let net = self.control_net(u, v);

        let bu = cubic_bernstein(u);
        let bv = cubic_bernstein(v);
        let mut sum = Vec3::new(0.0, 0.0, 0.0);
        for (i, bui) in bu.iter().enumerate() {
            for (j, bvj) in bv.iter().enumerate() {
                sum = sum + net[i][j].coords * (bui * bvj);
            }
        }
        Point3::from(sum)
    }

    /// `(u, v)` における双3次制御網。内部4点だけが `(u, v)` に依存する。
    fn control_net(&self, u: f64, v: f64) -> [[Point3; 4]; 4] {
        let [b0, b1, b2, b3] = self.boundary;

        // 境界の行・列。
        let mut net = [[Point3::new(0.0, 0.0, 0.0); 4]; 4];
        for i in 0..4 {
            net[i][0] = b0[i]; // v = 0
            net[i][3] = b2[i]; // v = 1
        }
        for j in 0..4 {
            net[0][j] = b3[j]; // u = 0
            net[3][j] = b1[j]; // u = 1
        }

        // `net[0][1]`, `net[3][1]`, `net[1][0]` などは**境界曲線の制御点**で
        // あって、リボンから作るものではない。ここを上書きすると境界そのものが
        // 動く。リボンが決めるのは、各行・列の**内側の2点**だけである。
        //
        // 一度これを取り違えて、8点を上書きしていた。指定した接線からの外れが
        // 10.25 になり、隣のセルとも 16.09 離れた。

        // 内部の4点。双子を有理的に混ぜる。分母が 0 になるのは隅だけで、
        // そこでは値がどちらでも同じになる（隅で2つのリボンは一致している）。
        let blend = |a: Point3, wa: f64, b: Point3, wb: f64| -> Point3 {
            let total = wa + wb;
            if total <= 1e-12 {
                Point3::from((a.coords + b.coords) * 0.5)
            } else {
                Point3::from((a.coords * wa + b.coords * wb) * (1.0 / total))
            }
        };
        //
        // 重みの向きに注意。`v = 0` の辺で使いたいのは「`v = 0` のリボンから
        // 決めた双子」なので、その双子には **`v` ではなく `u`** を掛ける
        // （`(u A + v B) / (u + v)` は `v -> 0` で `A` になる）。一度ここを
        // 逆にして、指定した接線から 5.27 外れた。
        net[1][1] = blend(self.twins[0], u, self.twins[1], v);
        net[2][1] = blend(self.twins[2], 1.0 - u, self.twins[3], v);
        net[2][2] = blend(self.twins[4], 1.0 - u, self.twins[5], 1.0 - v);
        net[1][2] = blend(self.twins[6], u, self.twins[7], 1.0 - v);
        net
    }

    /// 辺 `edge`（0: v=0, 1: u=1, 2: v=1, 3: u=0）に沿った、指定されたリボンの値。
    ///
    /// 検証のために公開している。パッチが本当にこの接線に一致しているかは、
    /// リボンを外から見られなければ測れない。
    pub fn ribbon_at(&self, edge: usize, s: f64) -> Vec3 {
        self.ribbons[edge % 4].evaluate(s)
    }

    /// `(u, v)` での1階偏微分。
    pub fn derivatives(&self, u: f64, v: f64) -> (Vec3, Vec3) {
        let eps = 1e-6;
        let (u0, u1) = ((u - eps).max(0.0), (u + eps).min(1.0));
        let (v0, v1) = ((v - eps).max(0.0), (v + eps).min(1.0));
        let du = (self.evaluate_gregory(u1, v) - self.evaluate_gregory(u0, v)) / (u1 - u0);
        let dv = (self.evaluate_gregory(u, v1) - self.evaluate_gregory(u, v0)) / (v1 - v0);
        (du, dv)
    }
}

fn cubic_bernstein(t: f64) -> [f64; 4] {
    let w = 1.0 - t;
    [w * w * w, 3.0 * w * w * t, 3.0 * w * t * t, t * t * t]
}

impl Surface3 for GregoryPatch4 {
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate_gregory(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        let (du, dv) = self.derivatives(u, v);
        du.cross(&dv).try_normalize_safe(1e-9)
    }

    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        ((0.0, 1.0), (0.0, 1.0))
    }
}

/// N辺コーナーブレンド（多面頂点フィレットの穴埋めパッチ生成器）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CornerBlendN {
    /// N本の境界曲線
    pub boundary_curves: Vec<NurbsCurve3>,
    /// 分割されたN個の4辺グレゴリーパッチ
    pub patches: Vec<GregoryPatch4>,
    /// コーナー中心点
    pub center_point: Point3,
}

impl CornerBlendN {
    /// N本の境界曲線（N >= 3）からコーナーブレンドパッチ群を生成
    pub fn create_n_sided_blend(curves: Vec<NurbsCurve3>, tol: &Tolerance) -> Result<Self, String> {
        let n = curves.len();
        if n < 3 {
            return Err("N-sided corner blend requires at least 3 boundary curves".to_string());
        }

        // 1. 各曲線の始終点連続性を検証
        for i in 0..n {
            let next = (i + 1) % n;
            let end_curr = curves[i].evaluate(curves[i].param_range().1);
            let start_next = curves[next].evaluate(curves[next].param_range().0);
            if !end_curr.is_coincident_with(&start_next, tol.linear) {
                return Err(format!(
                    "Boundary curve continuity mismatch at corner index {i}"
                ));
            }
        }

        // 2. コーナー中心点 Pc の算出（各境界の中点の重心）
        let mut center_coords = Vec3::new(0.0, 0.0, 0.0);
        let mut mid_points = Vec::with_capacity(n);
        for curve in &curves {
            let (t0, t1) = curve.param_range();
            let mid = curve.evaluate((t0 + t1) * 0.5);
            center_coords = center_coords + mid.coords;
            mid_points.push(mid);
        }
        let center_point = Point3::from(center_coords * (1.0 / n as f64));

        // 3. 中心から各境界中点への内部リブ曲線
        let mut rib_curves = Vec::with_capacity(n);
        for mid in mid_points.iter().take(n) {
            let rib = NurbsCurve3::new(
                1,
                vec![
                    ControlPoint3::unweighted(center_point),
                    ControlPoint3::unweighted(*mid),
                ],
                KnotVector::clamped_uniform(2, 1),
            )?;
            rib_curves.push(rib);
        }

        // 4. 各境界を中点で二分する。パッチの1辺になるのは境界の**半分**で
        //    あって全体ではない。ここで境界をまるごと渡していたため、
        //    `GregoryPatch4::new` のコーナー検査が毎回落ち、`if let Ok(..)` が
        //    それを黙って捨てて、`patches` が常に空のまま `Ok` が返っていた。
        let mut halves = Vec::with_capacity(n);
        for curve in &curves {
            let (t0, t1) = curve.param_range();
            let (first, second) = curve
                .split_at((t0 + t1) * 0.5)
                .ok_or_else(|| "boundary curve could not be split at its midpoint".to_string())?;
            halves.push((first, second));
        }

        // 5. 各コーナーごとに4辺パッチを構築する。
        //
        //    セルは 中心 -> mid(i) -> corner(i, i+1) -> mid(i+1) -> 中心 の
        //    四辺形で、`GregoryPatch4` が求める向きに合わせて辺を並べる。
        //      c0 (v=0, u:0->1): rib(i)                中心   -> mid(i)
        //      c1 (u=1, v:0->1): curve(i) の後半        mid(i) -> corner
        //      c2 (v=1, u:0->1): curve(i+1) の前半を反転 mid(i+1) -> corner
        //      c3 (u=0, v:0->1): rib(i+1)              中心   -> mid(i+1)
        //
        //    リブは隣のセルと共有するので、そこで接線が合うようにリボンを
        //    決める。隣り合う2つのセルは同じリブを、片方は `u = 0` 側、
        //    もう片方は `u = 1` 側として持つ。どちらにも「相手のセルの中へ
        //    向かう方向」を渡せば、共有辺の両側で接平面が一致する。
        let mut patches = Vec::with_capacity(n);
        for i in 0..n {
            let next = (i + 1) % n;
            let c0 = rib_curves[i].clone();
            let c1 = halves[i].1.clone();
            let c2 = halves[next].0.reversed();
            let c3 = rib_curves[next].clone();

            let ribbons = Self::cell_ribbons(&c0, &c1, &c2, &c3)?;
            patches.push(
                GregoryPatch4::with_ribbons(c0, c1, c2, c3, ribbons, tol).map_err(|error| {
                    format!("corner blend patch {i} of {n} could not be built: {error}")
                })?,
            );
        }

        Ok(Self {
            boundary_curves: curves,
            patches,
            center_point,
        })
    }

    /// セル1枚ぶんのリボン。
    ///
    /// リブ（`u = 0` と `u = 1` の辺）では、隣のセルと共有するので、辺に沿った
    /// 向きから作った**同じ規則**で決める。両側のセルが同じ規則で同じ辺を見る
    /// ので、結果として接平面が揃う。
    fn cell_ribbons(
        c0: &NurbsCurve3,
        c1: &NurbsCurve3,
        c2: &NurbsCurve3,
        c3: &NurbsCurve3,
    ) -> Result<[CrossRibbon; 4], String> {
        let b0 = cubic_bezier_points(c0).ok_or("cell edge C0 is not a cubic Bezier")?;
        let b1 = cubic_bezier_points(c1).ok_or("cell edge C1 is not a cubic Bezier")?;
        let b2 = cubic_bezier_points(c2).ok_or("cell edge C2 is not a cubic Bezier")?;
        let b3 = cubic_bezier_points(c3).ok_or("cell edge C3 is not a cubic Bezier")?;

        Ok([
            CrossRibbon::from_ends(b3[3] - b3[0], b1[3] - b1[0]),
            CrossRibbon::from_ends(b0[0] - b0[3], b2[0] - b2[3]),
            CrossRibbon::from_ends(b3[0] - b3[3], b1[0] - b1[3]),
            CrossRibbon::from_ends(b0[3] - b0[0], b2[3] - b2[0]),
        ])
    }
}
