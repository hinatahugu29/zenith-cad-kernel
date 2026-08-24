use crate::bspline_basis::KnotVector;
use crate::nurbs_curve::ControlPoint3;
use crate::surface::{PlaneSurface3, Surface3};
use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec3, Vec3Ext};

/// 3次元 NURBS / B-Spline 曲面
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsSurface3 {
    pub degree_u: usize,
    pub degree_v: usize,
    /// 制御点グリッド: [row_u][col_v]
    pub control_points: Vec<Vec<ControlPoint3>>,
    pub knots_u: KnotVector,
    pub knots_v: KnotVector,
}

impl NurbsSurface3 {
    /// この曲面が平面そのものなら、その平面を返す。
    ///
    /// **当てるのではなく、決まります。** B-spline 曲面は制御点の凸包に
    /// 含まれるので（重みが正なら有理でも同じ）、制御点が1つの平面に
    /// 乗っているなら曲面はその平面から出られません。標本を見て「平面らしい」
    /// と判断するのとは別物です。5章にある「最大半径に16点乗っていれば円柱」
    /// のような当て方は、円錐を円柱として通しました。ここはそうなりません。
    ///
    /// 向きは曲面自身の法線に合わせます。適当に張ると裏返り、面が支持曲面と
    /// 食い違って立体が無効になります（実測でそうなりました）。
    ///
    /// 重みが正でない制御点が1つでもあれば、凸包の性質が使えないので
    /// `None` を返します。
    pub fn as_plane(&self) -> Option<PlaneSurface3> {
        let mut points: Vec<Point3> = Vec::new();
        for row in &self.control_points {
            for control in row {
                if !(control.weight > 0.0) {
                    return None;
                }
                points.push(control.point);
            }
        }
        if points.len() < 3 {
            return None;
        }

        let origin = points[0];
        // 原点から一番遠い点、次にその向きから一番外れた点。細長い網でも
        // 退化しない選び方。
        let far = *points
            .iter()
            .max_by(|a, b| (**a - origin).norm().total_cmp(&(**b - origin).norm()))?;
        let u_axis = far - origin;
        if u_axis.norm() <= 1e-12 {
            return None;
        }
        let off = *points.iter().max_by(|a, b| {
            (**a - origin)
                .cross(&u_axis)
                .norm()
                .total_cmp(&(**b - origin).cross(&u_axis).norm())
        })?;
        let normal = u_axis.cross(&(off - origin));
        if normal.norm() <= 1e-12 {
            return None;
        }
        let normal = normal / normal.norm();

        // 網の広がりに対する相対で見る。絶対値で切ると、大きい形が落ちます。
        let extent = points
            .iter()
            .map(|point| (*point - origin).norm())
            .fold(0.0f64, f64::max)
            .max(1.0);
        let limit = extent * 1e-12;
        if points
            .iter()
            .any(|point| (*point - origin).dot(&normal).abs() > limit)
        {
            return None;
        }

        let ((u_min, u_max), (v_min, v_max)) = Surface3::param_range(self);
        let wanted = Surface3::normal(self, (u_min + u_max) * 0.5, (v_min + v_max) * 0.5)?;
        let normal = if normal.dot(&wanted) >= 0.0 {
            normal
        } else {
            -normal
        };
        PlaneSurface3::new(origin, u_axis, normal.cross(&u_axis))
    }

    /// 新規作成（バリデーション付き）
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        control_points: Vec<Vec<ControlPoint3>>,
        knots_u: KnotVector,
        knots_v: KnotVector,
    ) -> Result<Self, String> {
        let num_u = control_points.len();
        if num_u < degree_u + 1 {
            return Err(format!(
                "Number of U control points ({}) must be >= degree_u + 1 ({})",
                num_u,
                degree_u + 1
            ));
        }
        let num_v = control_points[0].len();
        if num_v < degree_v + 1 {
            return Err(format!(
                "Number of V control points ({}) must be >= degree_v + 1 ({})",
                num_v,
                degree_v + 1
            ));
        }
        for (i, row) in control_points.iter().enumerate() {
            if row.len() != num_v {
                return Err(format!(
                    "Row {} has length {} but expected {}",
                    i,
                    row.len(),
                    num_v
                ));
            }
        }

        let expected_knots_u = num_u + degree_u + 1;
        if knots_u.knots.len() != expected_knots_u {
            return Err(format!(
                "Knot vector U length ({}) does not match expected ({})",
                knots_u.knots.len(),
                expected_knots_u
            ));
        }

        let expected_knots_v = num_v + degree_v + 1;
        if knots_v.knots.len() != expected_knots_v {
            return Err(format!(
                "Knot vector V length ({}) does not match expected ({})",
                knots_v.knots.len(),
                expected_knots_v
            ));
        }

        Ok(Self {
            degree_u,
            degree_v,
            control_points,
            knots_u,
            knots_v,
        })
    }

    /// パラメータ有効範囲 [u_min, u_max] x [v_min, v_max]
    pub fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        let u_range = (
            self.knots_u.start_param(self.degree_u),
            self.knots_u.end_param(self.control_points.len()),
        );
        let v_range = (
            self.knots_v.start_param(self.degree_v),
            self.knots_v.end_param(self.control_points[0].len()),
        );
        (u_range, v_range)
    }

    /// 格子状に並んだ点を**すべて通る**曲面を作る。
    ///
    /// 行ごと・列ごとに曲線補間を2回かけるテンソル積の大域補間
    /// （The NURBS Book A9.4）。媒介変数は行や列ごとに決めると食い違うので、
    /// **弦長で決めたものを平均して共通**に使う。
    ///
    /// 有理ではない（重みは 1）。曲面のオフセットのように「この位置を通って
    /// ほしい」点の集まりから曲面を起こすための入口である。
    pub fn interpolate_points(
        degree_u: usize,
        degree_v: usize,
        grid: &[Vec<Point3>],
    ) -> Result<Self, String> {
        let rows = grid.len();
        if rows < 2 {
            return Err("surface interpolation needs at least two rows".to_string());
        }
        let columns = grid[0].len();
        if columns < 2 || grid.iter().any(|row| row.len() != columns) {
            return Err("surface interpolation needs a rectangular grid".to_string());
        }
        let degree_u = degree_u.min(rows - 1).max(1);
        let degree_v = degree_v.min(columns - 1).max(1);

        // 1. 媒介変数。列ごとに u 方向の弦長を測り、平均する。行も同様。
        let mut u_parameters = vec![0.0f64; rows];
        let mut counted = 0usize;
        for column in 0..columns {
            let strip: Vec<Point3> = (0..rows).map(|row| grid[row][column]).collect();
            if let Some(parameters) = crate::nurbs_curve::NurbsCurve3::interpolation_parameters(&strip)
            {
                for (index, value) in parameters.iter().enumerate() {
                    u_parameters[index] += value;
                }
                counted += 1;
            }
        }
        if counted == 0 {
            return Err("every column of the grid is degenerate".to_string());
        }
        for value in &mut u_parameters {
            *value /= counted as f64;
        }

        let mut v_parameters = vec![0.0f64; columns];
        let mut counted = 0usize;
        for row in grid.iter() {
            if let Some(parameters) = crate::nurbs_curve::NurbsCurve3::interpolation_parameters(row) {
                for (index, value) in parameters.iter().enumerate() {
                    v_parameters[index] += value;
                }
                counted += 1;
            }
        }
        if counted == 0 {
            return Err("every row of the grid is degenerate".to_string());
        }
        for value in &mut v_parameters {
            *value /= counted as f64;
        }

        let knots_u = crate::nurbs_curve::NurbsCurve3::interpolation_knots(&u_parameters, degree_u);
        let knots_v = crate::nurbs_curve::NurbsCurve3::interpolation_knots(&v_parameters, degree_v);

        // 2. 列ごとに u 方向へ補間して、中間の制御点を得る。
        let mut intermediate: Vec<Vec<Point3>> = vec![vec![Point3::origin(); columns]; rows];
        for column in 0..columns {
            let strip: Vec<Point3> = (0..rows).map(|row| grid[row][column]).collect();
            let curve = crate::nurbs_curve::NurbsCurve3::interpolate_points_with(
                degree_u,
                &strip,
                &u_parameters,
                &knots_u,
            )?;
            for (row, control) in curve.control_points.iter().enumerate() {
                intermediate[row][column] = control.point;
            }
        }

        // 3. その行ごとに v 方向へ補間すると、曲面の制御点になる。
        let mut control_points: Vec<Vec<ControlPoint3>> = Vec::with_capacity(rows);
        for row in intermediate.iter() {
            let curve = crate::nurbs_curve::NurbsCurve3::interpolate_points_with(
                degree_v,
                row,
                &v_parameters,
                &knots_v,
            )?;
            control_points.push(curve.control_points);
        }

        Self::new(degree_u, degree_v, control_points, knots_u, knots_v)
    }

    /// 任意の [`Surface3`] を格子で標本し、その点を通る NURBS 曲面を起こす。
    ///
    /// Coons / Gordon / 三角パッチのように、制御点の格子を持たない曲面を
    /// NURBS しか表せない相手（STEP の `B_SPLINE_SURFACE`）へ渡すための口。
    ///
    /// 標本点の上では厳密に一致し、その間は補間の精度で近づく。**近似である
    /// ことを隠さないために、呼び出し側で偏差を測れるようにしてある**
    /// （`deviation_from` を参照）。格子を細かくすれば偏差は下がる。
    pub fn approximate_surface(
        surface: &dyn crate::surface::Surface3,
        samples_u: usize,
        samples_v: usize,
    ) -> Result<Self, String> {
        let samples_u = samples_u.max(4);
        let samples_v = samples_v.max(4);
        let ((u_min, u_max), (v_min, v_max)) = surface.param_range();
        if !(u_max > u_min && v_max > v_min) {
            return Err("surface has an empty parameter range".to_string());
        }

        let mut grid = Vec::with_capacity(samples_u);
        for i in 0..samples_u {
            let u = u_min + (u_max - u_min) * i as f64 / (samples_u - 1) as f64;
            let mut row = Vec::with_capacity(samples_v);
            for j in 0..samples_v {
                let v = v_min + (v_max - v_min) * j as f64 / (samples_v - 1) as f64;
                row.push(surface.evaluate(u, v));
            }
            grid.push(row);
        }

        Self::interpolate_points(3, 3, &grid)
    }

    /// この曲面と `other` の隔たりを、両者のパラメータ域を正規化して測る。
    ///
    /// [`Self::approximate_surface`] が「どのくらい近いか」を、主張ではなく
    /// 数で言えるようにするためのもの。
    pub fn deviation_from(&self, other: &dyn crate::surface::Surface3, samples: usize) -> f64 {
        let samples = samples.max(2);
        let ((su_min, su_max), (sv_min, sv_max)) = self.param_range();
        let ((ou_min, ou_max), (ov_min, ov_max)) = other.param_range();
        let mut worst: f64 = 0.0;
        for i in 0..samples {
            let t = i as f64 / (samples - 1) as f64;
            for j in 0..samples {
                let s = j as f64 / (samples - 1) as f64;
                let mine = self.evaluate(su_min + (su_max - su_min) * t, sv_min + (sv_max - sv_min) * s);
                let theirs =
                    other.evaluate(ou_min + (ou_max - ou_min) * t, ov_min + (ov_max - ov_min) * s);
                worst = worst.max((mine - theirs).norm());
            }
        }
        worst
    }

    /// `u` の位置で2枚のパッチに分割する。
    ///
    /// 各 v 列を1本の曲線として同じ分割にかけるので、有理の重みも保たれる。
    /// 分割位置が端にあるか、いずれかの列で割れなければ `None`。
    pub fn split_u(&self, u: f64) -> Option<(Self, Self)> {
        let num_v = self.control_points[0].len();
        let mut left_cols = Vec::with_capacity(num_v);
        let mut right_cols = Vec::with_capacity(num_v);
        let mut left_knots = None;
        let mut right_knots = None;

        for j in 0..num_v {
            let column: Vec<ControlPoint3> =
                self.control_points.iter().map(|row| row[j]).collect();
            let curve =
                crate::nurbs_curve::NurbsCurve3::new(self.degree_u, column, self.knots_u.clone())
                    .ok()?;
            let (left, right) = curve.split_at(u)?;
            left_knots = Some(left.knots.clone());
            right_knots = Some(right.knots.clone());
            left_cols.push(left.control_points);
            right_cols.push(right.control_points);
        }

        let transpose = |cols: Vec<Vec<ControlPoint3>>| {
            let rows = cols[0].len();
            (0..rows)
                .map(|i| cols.iter().map(|col| col[i]).collect::<Vec<_>>())
                .collect::<Vec<_>>()
        };

        let left = Self::new(
            self.degree_u,
            self.degree_v,
            transpose(left_cols),
            left_knots?,
            self.knots_v.clone(),
        )
        .ok()?;
        let right = Self::new(
            self.degree_u,
            self.degree_v,
            transpose(right_cols),
            right_knots?,
            self.knots_v.clone(),
        )
        .ok()?;
        Some((left, right))
    }

    /// `v` の位置で2枚のパッチに分割する。`split_u` の行と列を入れ替えた版。
    pub fn split_v(&self, v: f64) -> Option<(Self, Self)> {
        let mut left_rows = Vec::with_capacity(self.control_points.len());
        let mut right_rows = Vec::with_capacity(self.control_points.len());
        let mut left_knots = None;
        let mut right_knots = None;

        for row in &self.control_points {
            let curve =
                crate::nurbs_curve::NurbsCurve3::new(self.degree_v, row.clone(), self.knots_v.clone())
                    .ok()?;
            let (left, right) = curve.split_at(v)?;
            left_knots = Some(left.knots.clone());
            right_knots = Some(right.knots.clone());
            left_rows.push(left.control_points);
            right_rows.push(right.control_points);
        }

        let left = Self::new(
            self.degree_u,
            self.degree_v,
            left_rows,
            self.knots_u.clone(),
            left_knots?,
        )
        .ok()?;
        let right = Self::new(
            self.degree_u,
            self.degree_v,
            right_rows,
            self.knots_u.clone(),
            right_knots?,
        )
        .ok()?;
        Some((left, right))
    }

    /// 曲面上の3次元座標を評価（Algorithm A3.5 / A4.3 from The NURBS Book）
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        crate::work_counter::count_surface_evaluation();
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();

        let span_u = self.knots_u.find_span(num_u, self.degree_u, u);
        let span_v = self.knots_v.find_span(num_v, self.degree_v, v);

        let basis_u = self.knots_u.basis_functions(span_u, self.degree_u, u);
        let basis_v = self.knots_v.basis_functions(span_v, self.degree_v, v);

        let mut s_w = nalgebra::Vector4::zeros();

        for (l, basis_v_value) in basis_v.iter().enumerate().take(self.degree_v + 1) {
            let mut temp = nalgebra::Vector4::zeros();
            let v_idx = span_v - self.degree_v + l;
            for (k, basis_u_value) in basis_u.iter().enumerate().take(self.degree_u + 1) {
                let u_idx = span_u - self.degree_u + k;
                let pw = self.control_points[u_idx][v_idx].to_homogeneous();
                temp += pw * *basis_u_value;
            }
            s_w += temp * *basis_v_value;
        }

        ControlPoint3::from_homogeneous(&s_w).point
    }

    /// The curve this surface traces at a fixed `v`, as an exact rational curve.
    ///
    /// Every control point of the result is the v-direction basis applied to a
    /// column of the control net in homogeneous coordinates, so the curve is the
    /// surface restricted to that line and not an approximation of it. The u
    /// degree and u knots carry over unchanged.
    ///
    /// This is what makes a section of a surface of revolution exact: cut a
    /// cylinder, a cone or a torus square to its axis and the result is one of
    /// these lines, so the intersection curve comes straight out of the control
    /// net rather than being traced.
    pub fn iso_curve_v(&self, v: f64) -> Option<crate::nurbs_curve::NurbsCurve3> {
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();
        let (_, (v_min, v_max)) = self.param_range();
        if !(v_min - 1e-9..=v_max + 1e-9).contains(&v) {
            return None;
        }
        let v = v.clamp(v_min, v_max);

        let span_v = self.knots_v.find_span(num_v, self.degree_v, v);
        let basis_v = self.knots_v.basis_functions(span_v, self.degree_v, v);

        let control_points = (0..num_u)
            .map(|u_index| {
                let mut accumulated = nalgebra::Vector4::zeros();
                for (l, weight) in basis_v.iter().enumerate().take(self.degree_v + 1) {
                    let v_index = span_v - self.degree_v + l;
                    accumulated += self.control_points[u_index][v_index].to_homogeneous() * *weight;
                }
                ControlPoint3::from_homogeneous(&accumulated)
            })
            .collect();

        crate::nurbs_curve::NurbsCurve3::new(
            self.degree_u,
            control_points,
            KnotVector::new(self.knots_u.knots.clone()),
        )
        .ok()
    }

    /// The curve this surface traces at a fixed `u`, the other way round from
    /// [`Self::iso_curve_v`].
    ///
    /// Which of the two carries a shape's axial direction is a matter of how
    /// the builder laid the patch out - a cylinder's runs along v, a torus's
    /// along u - so anything sectioning a surface has to be prepared for both.
    pub fn iso_curve_u(&self, u: f64) -> Option<crate::nurbs_curve::NurbsCurve3> {
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();
        let ((u_min, u_max), _) = self.param_range();
        if !(u_min - 1e-9..=u_max + 1e-9).contains(&u) {
            return None;
        }
        let u = u.clamp(u_min, u_max);

        let span_u = self.knots_u.find_span(num_u, self.degree_u, u);
        let basis_u = self.knots_u.basis_functions(span_u, self.degree_u, u);

        let control_points = (0..num_v)
            .map(|v_index| {
                let mut accumulated = nalgebra::Vector4::zeros();
                for (k, weight) in basis_u.iter().enumerate().take(self.degree_u + 1) {
                    let u_index = span_u - self.degree_u + k;
                    accumulated += self.control_points[u_index][v_index].to_homogeneous() * *weight;
                }
                ControlPoint3::from_homogeneous(&accumulated)
            })
            .collect();

        crate::nurbs_curve::NurbsCurve3::new(
            self.degree_v,
            control_points,
            KnotVector::new(self.knots_v.knots.clone()),
        )
        .ok()
    }

    /// 曲面上の点および U方向・V方向の偏導関数 (Du, Dv) を評価
    pub fn evaluate_derivatives_1st(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        crate::work_counter::count_surface_evaluation();
        let num_u = self.control_points.len();
        let num_v = self.control_points[0].len();

        let span_u = self.knots_u.find_span(num_u, self.degree_u, u);
        let span_v = self.knots_v.find_span(num_v, self.degree_v, v);

        let ders_u = self
            .knots_u
            .ders_basis_functions(span_u, self.degree_u, 1, u);
        let ders_v = self
            .knots_v
            .ders_basis_functions(span_v, self.degree_v, 1, v);

        // a_{k, l} (k in 0..=1, l in 0..=1)
        let mut a = vec![vec![nalgebra::Vector4::zeros(); 2]; 2];

        for k in 0..=1 {
            for l in 0..=1 {
                for (j, ders_v_value) in ders_v[l].iter().enumerate().take(self.degree_v + 1) {
                    let v_idx = span_v - self.degree_v + j;
                    let mut temp = nalgebra::Vector4::zeros();
                    for (i, ders_u_value) in ders_u[k].iter().enumerate().take(self.degree_u + 1) {
                        let u_idx = span_u - self.degree_u + i;
                        let pw = self.control_points[u_idx][v_idx].to_homogeneous();
                        temp += pw * *ders_u_value;
                    }
                    a[k][l] += temp * *ders_v_value;
                }
            }
        }

        let p = ControlPoint3::from_homogeneous(&a[0][0]).point;
        let w = a[0][0].w;

        // dS/du = (a10 - p * w10) / w
        let a10 = Vec3::new(a[1][0].x, a[1][0].y, a[1][0].z);
        let w10 = a[1][0].w;
        let du = if w.abs() > 1e-15 {
            (a10 - p.coords * w10) / w
        } else {
            Vec3::zeros()
        };

        // dS/dv = (a01 - p * w01) / w
        let a01 = Vec3::new(a[0][1].x, a[0][1].y, a[0][1].z);
        let w01 = a[0][1].w;
        let dv = if w.abs() > 1e-15 {
            (a01 - p.coords * w01) / w
        } else {
            Vec3::zeros()
        };

        (p, du, dv)
    }

    /// 曲面の法線ベクトル（正規化）
    pub fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        let (_p, du, dv) = self.evaluate_derivatives_1st(u, v);
        let cross = du.cross(&dv);
        cross.try_normalize_safe(1e-12)
    }

    /// 法線。退化点では**まわりからの極限**を返す。
    ///
    /// 回転面の極では片方の偏微分が消えるので、`normal` はそこで `None` を
    /// 返します。それは点そのものについては正しいのですが、**面はその点で
    /// 滑らかなことが多い**——球の極の接平面は軸に直交します。読んだ球の
    /// 軸上にある点は、最近点がちょうど極になるので、`None` のままだと
    /// 「その面には足が無い」ことになっていました。中心線は実務でいくらでも
    /// 出てきます。
    ///
    /// 極限は、`(u, v)` から領域の内側へ**斜めに4方向**寄せて取ります。
    /// 4方向のうち法線が取れたものが**互いに一致したときだけ**返します。
    /// 円錐の頂点のように、寄せる向きで法線が変わる点では一致しないので、
    /// そこは `None` のままです。**滑らかでない点に法線を作りません。**
    pub fn normal_or_limit(&self, u: f64, v: f64) -> Option<Vec3> {
        if let Some(normal) = self.normal(u, v) {
            return Some(normal);
        }

        let ((u_min, u_max), (v_min, v_max)) = self.param_range();
        let u_span = u_max - u_min;
        let v_span = v_max - v_min;
        if u_span <= 0.0 || v_span <= 0.0 {
            return None;
        }

        for scale in [1e-6, 1e-5, 1e-4, 1e-3] {
            let du = u_span * scale;
            let dv = v_span * scale;
            let mut agreed: Option<Vec3> = None;
            let mut found = 0usize;
            let mut split = false;

            for (su, sv) in [(1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
                let uu = (u + su * du).clamp(u_min, u_max);
                let vv = (v + sv * dv).clamp(v_min, v_max);
                // 寄せた先が同じ点なら、その向きからは何も分かりません。
                if (uu - u).abs() < du * 0.5 && (vv - v).abs() < dv * 0.5 {
                    continue;
                }
                let Some(candidate) = self.normal(uu, vv) else {
                    continue;
                };
                found += 1;
                match agreed {
                    None => agreed = Some(candidate),
                    Some(previous) => {
                        if previous.dot(&candidate) < 1.0 - 1e-6 {
                            split = true;
                        }
                    }
                }
            }

            if split {
                // 寄せる向きで法線が変わる。ここは滑らかではありません。
                return None;
            }
            if found > 0 {
                return agreed;
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::NurbsSurface3;
    use crate::bspline_basis::KnotVector;
    use crate::nurbs_curve::ControlPoint3;
    use zenith_math::Point3;

    /// 半径 `r`、高さ `h` の円柱側面を、全周1枚の有理パッチとして張る。
    /// これは他カーネルの STEP を読んだときに出てくる形そのもの。
    fn full_wrap_cylinder(r: f64, h: f64) -> NurbsSurface3 {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let ring = [
            (r, 0.0, 1.0),
            (r, r, w),
            (0.0, r, 1.0),
            (-r, r, w),
            (-r, 0.0, 1.0),
            (-r, -r, w),
            (0.0, -r, 1.0),
            (r, -r, w),
            (r, 0.0, 1.0),
        ];
        let grid = ring
            .iter()
            .map(|(x, y, weight)| {
                vec![
                    ControlPoint3::new(Point3::new(*x, *y, 0.0), *weight),
                    ControlPoint3::new(Point3::new(*x, *y, h), *weight),
                ]
            })
            .collect();
        NurbsSurface3::new(
            2,
            1,
            grid,
            KnotVector::new(vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ]),
            KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]),
        )
        .unwrap()
    }

    #[test]
    fn splitting_in_u_keeps_every_piece_on_the_original_surface() {
        let surface = full_wrap_cylinder(10.0, 40.0);
        let (left, right) = surface.split_u(0.25).expect("split u");

        for (piece, lo, hi) in [(&left, 0.0, 0.25), (&right, 0.25, 1.0)] {
            let ((u0, u1), _) = piece.param_range();
            assert!((u0 - lo).abs() < 1e-12 && (u1 - hi).abs() < 1e-12);
            let mut worst: f64 = 0.0;
            let mut off_radius: f64 = 0.0;
            // 分割位置 (1/4) と互いに素な刻みで測る。
            for i in 0..=37 {
                for j in 0..=7 {
                    let u = lo + (hi - lo) * (i as f64 / 37.0);
                    let v = j as f64 / 7.0;
                    let p = piece.evaluate(u, v);
                    worst = worst.max((p - surface.evaluate(u, v)).norm());
                    off_radius = off_radius.max(((p.x * p.x + p.y * p.y).sqrt() - 10.0).abs());
                }
            }
            assert!(worst < 1e-12, "u split moved the surface by {worst}");
            assert!(off_radius < 1e-12, "u split left the cylinder by {off_radius}");
        }
    }

    #[test]
    fn splitting_in_v_keeps_every_piece_on_the_original_surface() {
        let surface = full_wrap_cylinder(6.0, 20.0);
        let (bottom, top) = surface.split_v(0.5).expect("split v");

        for (piece, lo, hi) in [(&bottom, 0.0, 0.5), (&top, 0.5, 1.0)] {
            let mut worst: f64 = 0.0;
            for i in 0..=13 {
                for j in 0..=11 {
                    let u = i as f64 / 13.0;
                    let v = lo + (hi - lo) * (j as f64 / 11.0);
                    worst = worst.max((piece.evaluate(u, v) - surface.evaluate(u, v)).norm());
                }
            }
            assert!(worst < 1e-12, "v split moved the surface by {worst}");
        }
    }

    #[test]
    fn quartering_a_full_wrap_patch_leaves_four_pieces_that_do_not_close() {
        let surface = full_wrap_cylinder(10.0, 40.0);
        let (a, rest) = surface.split_u(0.25).expect("first");
        let (b, rest) = rest.split_u(0.5).expect("second");
        let (c, d) = rest.split_u(0.75).expect("third");

        for piece in [&a, &b, &c, &d] {
            let ((u0, u1), _) = piece.param_range();
            // 四半周のパッチは始端と終端が別の点になる。全周1枚ならここが一致する。
            let start = piece.evaluate(u0, 0.0);
            let end = piece.evaluate(u1, 0.0);
            assert!(
                (start - end).norm() > 1.0,
                "piece still wraps onto itself: {start:?} {end:?}"
            );
        }
    }
}
