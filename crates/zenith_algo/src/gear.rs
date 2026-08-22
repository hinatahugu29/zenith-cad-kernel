use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// インボリュート平歯車（Spur Gear）B-Rep ソリッドビルダー
pub struct GearBuilder;

impl GearBuilder {
    /// 歯面のインボリュートを何点で通すか。
    ///
    /// 歯面は3次で補間するので、誤差は点数の**4乗**で減る。
    ///
    /// **体積で測ってはいけない。** 補間の誤差は標本点の間で符号が入れ替わる
    /// ので、面積に積むとほとんど打ち消し合う。モジュール2・歯数18・圧力角20度
    /// で、歯面上の点が真のインボリュートからどれだけ離れているか（`worst`、
    /// モデル単位）と、体積を [`Self::involute_profile_area`] と比べた相対差:
    ///
    /// ```text
    ///  点数        4        6        8       12       16       32       48
    ///  worst   1.86e-3  5.46e-4  2.16e-4  6.82e-5  1.70e-5  4.73e-7  7.18e-8
    ///  体積rel  2.97e-5  1.88e-6  8.61e-7  6.78e-8  4.8e-11  1.99e-9  5.23e-10
    /// ```
    ///
    /// `worst` は素直に落ちるが、体積のほうは 16 で符号が入れ替わって 4.8e-11
    /// まで落ち、そのあと 1e-9 台に戻る。形の忠実さを見たいなら `worst` を
    /// 見ること。既定値の 32 では、歯先円半径 20 の歯車で 5e-7（0.5ナノ
    /// メートル）以内に乗る。
    pub const DEFAULT_FLANK_SAMPLES: usize = 32;

    /// インボリュート平歯車を作る。
    ///
    /// # 形について
    ///
    /// 歯面は**基礎円のインボリュート**である。基礎円 `r_b = r cos(alpha)`、
    /// 歯先円 `r_a = r + m`、歯底円 `r_f = r - 1.25 m`（ただし基礎円より外には
    /// 出さない）。ピッチ円上の歯厚は標準の `pi m / 2`。歯底から基礎円までは
    /// 半径方向の直線で繋ぐ。歯先と歯底の弧は有理2次で厳密、歯面だけが3次の
    /// 補間である。
    ///
    /// 2026年8月20日までは、歯1つにつき4点を直線で結んだ多角形でした。圧力角は
    /// 歯底半径の下限にしか効かず、歯面はただの斜面でした。噛み合いを解く用途
    /// には足りない形だったので、実際にインボリュートを張るようにしました。
    ///
    /// # `bore_radius`
    ///
    /// **軸穴は開きません。** 歯底半径の下限に効くだけです
    /// （`bore_radius + 0.5 * module` より内側には歯底を置かない）。軸穴が要る
    /// なら、円柱との差で開けてください。
    pub fn make_spur_gear(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        thickness: f64,
        bore_radius: f64,
    ) -> Result<Solid, String> {
        Self::make_spur_gear_with_samples(
            module,
            teeth,
            pressure_angle_deg,
            thickness,
            bore_radius,
            Self::DEFAULT_FLANK_SAMPLES,
        )
    }

    /// 歯面の補間に使う点数を指定して歯車を作る。
    pub fn make_spur_gear_with_samples(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        thickness: f64,
        bore_radius: f64,
        flank_samples: usize,
    ) -> Result<Solid, String> {
        let geometry = GearGeometry::new(module, teeth, pressure_angle_deg, bore_radius)?;
        if thickness <= 0.0 {
            return Err("Thickness must be positive".to_string());
        }

        let profile = geometry.profile_curves(flank_samples.max(3))?;
        Self::prism_from_profile(&profile, thickness)
    }

    /// 歯元に滑らかな $G^1$ フィレットを持つインボリュート平歯車を作る。
    pub fn make_spur_gear_with_root_fillet(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        thickness: f64,
        bore_radius: f64,
    ) -> Result<Solid, String> {
        let geometry = GearGeometry::new(module, teeth, pressure_angle_deg, bore_radius)?;
        if thickness <= 0.0 {
            return Err("Thickness must be positive".to_string());
        }

        let profile = geometry.profile_curves_with_root_fillet(Self::DEFAULT_FLANK_SAMPLES)?;
        Self::prism_from_profile(&profile, thickness)
    }

    /// 軸穴（貫通穴）が開いたインボリュート平歯車を作る。
    ///
    /// 歯車ソリッドを生成した後、中心軸に沿った半径 `bore_radius` の円柱との
    /// ブーリアン差分により、正確な円筒貫通穴を開けます。
    pub fn make_drilled_spur_gear(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        thickness: f64,
        bore_radius: f64,
    ) -> Result<Solid, String> {
        let gear = Self::make_spur_gear(module, teeth, pressure_angle_deg, thickness, bore_radius)?;
        if bore_radius <= 0.0 {
            return Ok(gear);
        }
        let tol = zenith_math::Tolerance::default();
        let drill = crate::PrimitiveBuilder::make_cylinder(bore_radius, thickness + 2.0)?;
        let drill = crate::BrepTransform::translate_solid(&drill, Vec3::new(0.0, 0.0, -1.0));
        crate::BooleanEngine::boolean_solids_exact(&gear, &drill, crate::BooleanOpType::Difference, &tol)
    }

    /// インボリュート歯車の断面積。**閉じた式**である。
    ///
    /// 極形式のグリーンの定理 `A = (1/2) ∮ r^2 dθ` を、境界の3種類に分けて積む。
    ///
    /// - 半径方向の直線: `dθ = 0` なので寄与しない
    /// - 半径 `R` の円弧が角 `Δ` を張る: `R^2 Δ / 2`
    /// - 基礎円 `r_b` のインボリュートを `t1` から `t2` まで:
    ///   `r^2 = r_b^2 (1 + t^2)`、`dθ/dt = t^2/(1 + t^2)` なので
    ///   `r_b^2 (t2^3 - t1^3) / 6`
    ///
    /// 1ピッチぶんを足して歯数を掛けると
    ///
    /// ```text
    /// A = z [ r_f^2 (pi/z - psi_b) + r_b^2 t_a^3 / 3 + r_a^2 psi_a ]
    /// ```
    ///
    /// 立体のほうは歯面を3次で補間しているので、この値との差がそのまま補間の
    /// 誤差になる（[`Self::DEFAULT_FLANK_SAMPLES`] に実測がある）。歯先と歯底の
    /// 弧、半径方向の直線は厳密なので、誤差はすべて歯面から来る。
    ///
    /// この式は `builder_audit` の外の物差しでもある。多角形だった頃の
    /// `spur_gear_profile_area`（4点の多角形の面積）はもう当たらない。
    pub fn involute_profile_area(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        bore_radius: f64,
    ) -> Result<f64, String> {
        Ok(GearGeometry::new(module, teeth, pressure_angle_deg, bore_radius)?.profile_area())
    }

    /// 歯元フィレットを創成するホブの運動と、それが描いた曲線。
    ///
    /// 検証のために公開している。フィレットが本当にホブ歯先丸みの**包絡線**に
    /// なっているか（どの転がり角の工具円も、歯の中に食い込まず、かつ必ず1点で
    /// 触れているか）は、この運動を渡さなければ外から測れない。
    ///
    /// 角度は**歯の中心を 0 とした差**で返す。歯の右側のフィレットは負、
    /// 左側はその鏡像になる。
    pub fn root_fillet_generation(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        bore_radius: f64,
    ) -> Result<RootFilletGeneration, String> {
        let geometry = GearGeometry::new(module, teeth, pressure_angle_deg, bore_radius)?;
        geometry.root_fillet_generation()
    }

    /// 閉じたプロファイル曲線の列を `thickness` だけ押し出す。
    ///
    /// 側面は下の曲線と、それを平行移動した上の曲線を1次で結ぶ。平行移動は
    /// アフィンなので、有理曲線でも制御点を動かすだけで厳密に写る。次数もノットも
    /// 変わらないので、[`crate::sweep`] のように揃え直す必要が無い（揃え直しは
    /// 曲線を動かす）。
    fn prism_from_profile(profile: &[NurbsCurve3], thickness: f64) -> Result<Solid, String> {
        if profile.len() < 3 {
            return Err("a profile needs at least three curves".to_string());
        }
        let rise = Vec3::new(0.0, 0.0, thickness);

        let lift = |curve: &NurbsCurve3| -> Result<NurbsCurve3, String> {
            let control_points = curve
                .control_points
                .iter()
                .map(|cp| ControlPoint3::new(cp.point + rise, cp.weight))
                .collect();
            NurbsCurve3::new(curve.degree, control_points, curve.knots.clone())
        };
        let top_profile = profile.iter().map(lift).collect::<Result<Vec<_>, _>>()?;

        let start_of = |curve: &NurbsCurve3| curve.evaluate(curve.param_range().0);

        let count = profile.len();
        let bottom_vertices: Vec<Vertex> = profile
            .iter()
            .map(|curve| Vertex::from_point(start_of(curve)))
            .collect();
        let top_vertices: Vec<Vertex> = top_profile
            .iter()
            .map(|curve| Vertex::from_point(start_of(curve)))
            .collect();

        let mut bottom_edges = Vec::with_capacity(count);
        let mut top_edges = Vec::with_capacity(count);
        let mut rise_edges = Vec::with_capacity(count);
        for index in 0..count {
            let next = (index + 1) % count;
            bottom_edges.push(Edge::new(
                profile[index].clone(),
                bottom_vertices[index].clone(),
                bottom_vertices[next].clone(),
                1e-6,
            ));
            top_edges.push(Edge::new(
                top_profile[index].clone(),
                top_vertices[index].clone(),
                top_vertices[next].clone(),
                1e-6,
            ));
            rise_edges.push(Edge::line_between(
                bottom_vertices[index].clone(),
                top_vertices[index].clone(),
            )?);
        }

        let mut faces = Vec::with_capacity(count + 2);
        for index in 0..count {
            let next = (index + 1) % count;
            let grid: Vec<Vec<ControlPoint3>> = profile[index]
                .control_points
                .iter()
                .map(|cp| vec![*cp, ControlPoint3::new(cp.point + rise, cp.weight)])
                .collect();
            let wall = NurbsSurface3::new(
                profile[index].degree,
                1,
                grid,
                profile[index].knots.clone(),
                KnotVector::clamped_uniform(2, 1),
            )?;
            faces.push(Face::simple(
                FaceGeometry::Nurbs(wall),
                Wire::new(vec![
                    OrientedEdge::forward(bottom_edges[index].clone()),
                    OrientedEdge::forward(rise_edges[next].clone()),
                    OrientedEdge::reversed(top_edges[index].clone()),
                    OrientedEdge::reversed(rise_edges[index].clone()),
                ]),
            ));
        }

        // 底面 (-Z 法線)
        let bottom_plane = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("gear plane bot")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(bottom_plane),
            Wire::new(
                (0..count)
                    .rev()
                    .map(|index| OrientedEdge::reversed(bottom_edges[index].clone()))
                    .collect(),
            ),
        ));

        // 天面 (+Z 法線)
        let top_plane = PlaneSurface3::new(
            Point3::new(0.0, 0.0, thickness),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("gear plane top")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(top_plane),
            Wire::new(
                top_edges
                    .iter()
                    .map(|edge| OrientedEdge::forward(edge.clone()))
                    .collect(),
            ),
        ));

        crate::validated_solid(Shell::closed(faces))
    }
}

/// 標準平歯車の寸法。
struct GearGeometry {
    teeth: usize,
    /// モジュール。ホブの歯先丸み半径を決めるのに要る。
    module: f64,
    /// ピッチ円半径。創成運動の転がり半径そのもの。
    pitch_radius: f64,
    /// 圧力角（ラジアン）。ラックの歯面の傾き。
    pressure_angle: f64,
    base_radius: f64,
    tip_radius: f64,
    root_radius: f64,
    /// 基礎円上での歯の半角。
    half_angle_at_base: f64,
    /// 歯先円上での歯の半角。
    half_angle_at_tip: f64,
    /// 歯先に届くインボリュートの媒介変数（`tan` の歯先圧力角）。
    tip_parameter: f64,
}

/// インボリュート関数 `inv(a) = tan(a) - a`。
fn involute_of(angle: f64) -> f64 {
    angle.tan() - angle
}

/// 歯元フィレットを創成するホブの運動と、それが描いた曲線。
///
/// [`GearBuilder::root_fillet_generation`] が返す。角度はすべて**歯の中心を
/// 0 とした差**で、歯の右側は負。
#[derive(Debug, Clone, PartialEq)]
pub struct RootFilletGeneration {
    /// ピッチ円半径。ラックが転がる半径そのもの。
    pub pitch_radius: f64,
    /// 基礎円半径。
    pub base_radius: f64,
    /// 歯底円半径。フィレットの内端がここに接する。
    pub root_radius: f64,
    /// フィレットが終わり、インボリュートが始まる半径。
    pub form_radius: f64,
    /// ホブ歯先丸みの半径。
    pub tip_round_radius: f64,
    /// ラック座標での丸み中心 `(u, v)`。`u` はピッチ線方向、`v` は歯車の外向き。
    pub tip_round_centre: (f64, f64),
    /// 歯の右側のフィレット。`(半径, 歯の中心からの角度差)`。
    pub fillet: Vec<(f64, f64)>,
}

impl RootFilletGeneration {
    /// 転がり角 `theta` における、ホブ歯先丸みの中心。
    ///
    /// 歯の中心を 0 とした座標系で返すので、[`Self::fillet`] と直接比べられる。
    pub fn tip_round_centre_at(&self, theta: f64, teeth: usize) -> (f64, f64) {
        let (u, v) = self.tip_round_centre;
        let r = self.pitch_radius;
        let (sin, cos) = theta.sin_cos();
        let along = u + r * theta;
        let out = r + v;
        let (x, y) = (along * cos - out * sin, along * sin + out * cos);
        let offset =
            std::f64::consts::FRAC_PI_2 + std::f64::consts::PI / (2.0 * teeth as f64);
        ((x * x + y * y).sqrt(), y.atan2(x) - offset)
    }
}

impl GearGeometry {
    fn new(
        module: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        bore_radius: f64,
    ) -> Result<Self, String> {
        if module <= 0.0 {
            return Err("Module must be positive".to_string());
        }
        if teeth < 4 {
            return Err("Number of teeth must be at least 4".to_string());
        }
        let pressure_angle = pressure_angle_deg.to_radians();
        if pressure_angle <= 0.0 || pressure_angle >= std::f64::consts::FRAC_PI_2 {
            return Err("Pressure angle must be between 0 and 90 degrees".to_string());
        }

        let z = teeth as f64;
        let pitch_radius = module * z * 0.5;
        let base_radius = pitch_radius * pressure_angle.cos();
        let tip_radius = pitch_radius + module;

        // 歯底は基礎円より外には出さない。出すと、歯底から基礎円へ向かう
        // 半径方向の直線が内向きになり、輪郭が自分と交差する。
        let floor = bore_radius + 0.5 * module;
        if floor > base_radius {
            return Err("The bore leaves no room below the base circle".to_string());
        }
        let root_radius = (pitch_radius - 1.25 * module)
            .max(floor)
            .min(base_radius);
        if root_radius <= 0.0 {
            return Err("The gear's radii do not make a usable tooth".to_string());
        }

        let tip_pressure_angle = (base_radius / tip_radius).clamp(-1.0, 1.0).acos();
        let half_angle_at_base = std::f64::consts::PI / (2.0 * z) + involute_of(pressure_angle);
        let half_angle_at_tip = half_angle_at_base - involute_of(tip_pressure_angle);
        if half_angle_at_tip <= 0.0 {
            return Err("The teeth come to a point before the tip circle".to_string());
        }
        if half_angle_at_base >= std::f64::consts::PI / z {
            return Err("The teeth touch each other at the root circle".to_string());
        }

        Ok(Self {
            teeth,
            module,
            pitch_radius,
            pressure_angle,
            base_radius,
            tip_radius,
            root_radius,
            half_angle_at_base,
            half_angle_at_tip,
            tip_parameter: tip_pressure_angle.tan(),
        })
    }

    /// 極形式のグリーンの定理で積んだ断面積。
    fn profile_area(&self) -> f64 {
        let z = self.teeth as f64;
        let root_span = std::f64::consts::PI / z - self.half_angle_at_base;
        z * (self.root_radius * self.root_radius * root_span
            + self.base_radius * self.base_radius * self.tip_parameter.powi(3) / 3.0
            + self.tip_radius * self.tip_radius * self.half_angle_at_tip)
    }

    fn at(&self, radius: f64, angle: f64) -> Point3 {
        Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0)
    }

    /// ホブ（ラック形工具）の歯先丸み半径。
    ///
    /// JIS / ISO の基準ラックは $\rho = 0.38 m$ を採る。ただし工具の歯末たけ
    /// （このギヤの歯元たけ）に収まらなければならないので、そこで頭を抑える。
    /// `root_radius` は軸穴やベース円で切り上げられることがあるので、公称の
    /// $1.25 m$ ではなく**実際に削る深さ**から決める。
    fn hob_tip_radius(&self) -> f64 {
        let dedendum = self.pitch_radius - self.root_radius;
        (0.38 * self.module).min(0.9 * dedendum).max(0.0)
    }

    /// ラック座標での歯先丸みの中心。
    ///
    /// ラック座標は、ピッチ点を原点、ピッチ線方向を `u`、歯車の外向きを `v` と
    /// する。工具の歯面は原点を通り `v` 軸から圧力角だけ傾いた直線である。
    ///
    /// **工具の本体は `u > 0` 側にある。** `theta = 0` で歯面が触れている
    /// ピッチ点は歯車角 `pi/2` にあり、歯の中心は `pi/2 + pi/(2z)`、つまり
    /// **より大きい角**にある。歯車角が大きい側は `u < 0` なので、歯車の歯は
    /// `u < 0`、それを削る工具の歯は `u > 0` である。
    ///
    /// ここを取り違えると、フィレットの最深点が歯溝ではなく**歯の下**に出る。
    /// 実測で、歯の中心 1.65806 rad に対して最深点が 1.65095 rad——歯の右歯面
    /// （1.55589）より内側——に来ていた。
    ///
    /// 丸みは歯面と歯先線 `v = -h` の両方に接するので、中心は両者から
    /// $\rho$ の距離に来る。
    fn hob_tip_centre(&self, rho: f64) -> (f64, f64) {
        let dedendum = self.pitch_radius - self.root_radius;
        let v_c = -(dedendum - rho);
        let u_c = rho / self.pressure_angle.cos() + (dedendum - rho) * self.pressure_angle.tan();
        (u_c, v_c)
    }

    /// 転がり角 `theta` におけるラック点 `(u, v)` の、歯車に固定した座標。
    ///
    /// ラックがピッチ線に沿って `r_p * theta` だけ進む間に歯車が `theta` 回る。
    /// 導出フレームでは、`theta = 0` で工具の歯面が歯車のピッチ点 `(0, r_p)` に
    /// 触れている。
    fn rack_point_in_gear(&self, u: f64, v: f64, theta: f64) -> (f64, f64) {
        let r = self.pitch_radius;
        let (sin, cos) = theta.sin_cos();
        let along = u + r * theta;
        let out = r + v;
        (along * cos - out * sin, along * sin + out * cos)
    }

    /// 歯元フィレットの標本点（導出フレームの極座標 `(radius, angle)`）。
    ///
    /// フィレットは、歯先丸みが掃いた円の**包絡線**である。丸みの中心が描く
    /// トロコイドを `T(theta)` とすると、包絡線は `T` の法線方向に $\rho$ だけ
    /// 離れた点で、歯車の中心側の枝を採る（転がりがいちばん深いところで
    /// 半径が歯底円にちょうど一致する枝）。
    ///
    /// ここは以前、半径を線形・角度を smoothstep で補間して `* 0.2` の係数で
    /// 寄せたものを「トロコイド」と呼んでいた。工具のころがりは計算して
    /// いなかったので、形は似ていても創成運動とは無関係だった。
    fn trochoid_fillet_polar(&self, samples: usize) -> Result<Vec<(f64, f64)>, String> {
        let rho = self.hob_tip_radius();
        if rho <= 1e-12 {
            return Err("the hob tip round has no room below the pitch line".to_string());
        }
        let (u_c, v_c) = self.hob_tip_centre(rho);

        // 中心が歯車軸にいちばん近づく転がり角。ここでフィレットは歯底円に接する。
        let theta_deepest = -u_c / self.pitch_radius;

        // フィレットが終わる半径。工具の歯面と丸みの継ぎ目が削る位置で、
        // ここから外側はインボリュートになる。
        let sin_alpha = self.pressure_angle.sin();
        let depth_form = (self.pitch_radius - self.root_radius) - rho * (1.0 - sin_alpha);
        let offset = self.pitch_radius * sin_alpha - depth_form / sin_alpha;
        let r_form = (self.base_radius * self.base_radius + offset * offset).sqrt();
        if !(r_form > self.root_radius && r_form < self.tip_radius) {
            return Err(format!(
                "the trochoid would end at radius {r_form}, outside the root..tip range"
            ));
        }

        // 包絡線の枝は、いちばん深いところで半径が歯底円になる側。そこで決めて
        // から、法線の向きを最後まで変えない。
        let envelope = |theta: f64, sign: f64| -> (f64, f64) {
            let (x, y) = self.rack_point_in_gear(u_c, v_c, theta);
            let step = 1e-7;
            let (xa, ya) = self.rack_point_in_gear(u_c, v_c, theta - step);
            let (xb, yb) = self.rack_point_in_gear(u_c, v_c, theta + step);
            let (dx, dy) = (xb - xa, yb - ya);
            let length = (dx * dx + dy * dy).sqrt().max(1e-15);
            // 接線を90度回した単位法線。
            let (nx, ny) = (-dy / length, dx / length);
            (x + sign * rho * nx, y + sign * rho * ny)
        };

        let sign = {
            let plus = envelope(theta_deepest, 1.0);
            let minus = envelope(theta_deepest, -1.0);
            let radius = |p: (f64, f64)| (p.0 * p.0 + p.1 * p.1).sqrt();
            if radius(plus) < radius(minus) {
                1.0
            } else {
                -1.0
            }
        };

        // 歯の右側のフィレットになるのはどちらの向きか。半径が歯底から
        // `r_form` まで上がりきるほうを採る。
        let radius_at = |theta: f64| {
            let (x, y) = envelope(theta, sign);
            (x * x + y * y).sqrt()
        };

        // フィレットが `r_form` に届く転がり角を、行き過ぎてから二分法で詰める。
        // 刻んだまま採ると端が `r_form` を越え、インボリュートの始まりと合わない。
        let reach = |direction: f64| -> Option<f64> {
            let mut previous = self.root_radius;
            let mut last = theta_deepest;
            for step in 1..=256 {
                let theta = theta_deepest
                    + direction * (step as f64 / 256.0) * self.pressure_angle * 2.0;
                let radius = radius_at(theta);
                if radius + 1e-12 < previous {
                    return None;
                }
                previous = radius;
                if radius >= r_form {
                    let (mut low, mut high) = (last, theta);
                    for _ in 0..80 {
                        let middle = (low + high) * 0.5;
                        if radius_at(middle) < r_form {
                            low = middle;
                        } else {
                            high = middle;
                        }
                    }
                    return Some((low + high) * 0.5);
                }
                last = theta;
            }
            None
        };

        let theta_form = reach(-1.0)
            .or_else(|| reach(1.0))
            .ok_or_else(|| "the trochoid never reaches the form radius".to_string())?;

        let mut out = Vec::with_capacity(samples + 1);
        for step in 0..=samples {
            let theta =
                theta_deepest + (theta_form - theta_deepest) * step as f64 / samples as f64;
            let (x, y) = envelope(theta, sign);
            out.push(((x * x + y * y).sqrt(), y.atan2(x)));
        }
        // 両端は解析的に分かっているので、標本の丸めを持ち込まない。
        out[0].0 = self.root_radius;
        out[samples].0 = r_form;
        Ok(out)
    }

    /// 歯面のインボリュート上の点。`t` は基礎円で 0、歯先で `tip_parameter`。
    fn flank_point(&self, centre: f64, t: f64, left: bool) -> Point3 {
        let radius = self.base_radius * (1.0 + t * t).sqrt();
        let half = self.half_angle_at_base - (t - t.atan());
        self.at(radius, if left { centre + half } else { centre - half })
    }

    /// 半径 `radius`、`from` から `to` までの円弧を有理2次で厳密に張る。
    fn arc(&self, radius: f64, from: f64, to: f64) -> Result<NurbsCurve3, String> {
        let half = (to - from) * 0.5;
        let weight = half.cos();
        if weight <= 1e-9 {
            return Err("an arc of half a turn or more needs splitting".to_string());
        }
        let middle = self.at(radius / weight, from + half);
        NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(self.at(radius, from)),
                ControlPoint3::new(middle, weight),
                ControlPoint3::unweighted(self.at(radius, to)),
            ],
            KnotVector::clamped_uniform(3, 2),
        )
    }

    /// 断面を、閉じた曲線の列にする。
    ///
    /// 歯1つにつき6本: 歯底から基礎円への直線、右の歯面、歯先の弧、左の歯面、
    /// 基礎円から歯底への直線、次の歯までの歯底の弧。反時計回り。
    fn profile_curves(&self, flank_samples: usize) -> Result<Vec<NurbsCurve3>, String> {
        let z = self.teeth as f64;
        let pitch_angle = 2.0 * std::f64::consts::PI / z;
        let mut curves = Vec::with_capacity(self.teeth * 6);

        for index in 0..self.teeth {
            let centre = index as f64 * pitch_angle;
            let next_centre = (index + 1) as f64 * pitch_angle;

            let flank = |left: bool| -> Result<NurbsCurve3, String> {
                let mut points: Vec<Point3> = (0..=flank_samples)
                    .map(|step| {
                        let t = self.tip_parameter * step as f64 / flank_samples as f64;
                        self.flank_point(centre, t, left)
                    })
                    .collect();
                if left {
                    points.reverse();
                }
                NurbsCurve3::interpolate_points(3, &points)
            };

            curves.push(NurbsCurve3::bspline_from_points(
                1,
                vec![
                    self.at(self.root_radius, centre - self.half_angle_at_base),
                    self.flank_point(centre, 0.0, false),
                ],
            )?);
            curves.push(flank(false)?);
            curves.push(self.arc(
                self.tip_radius,
                centre - self.half_angle_at_tip,
                centre + self.half_angle_at_tip,
            )?);
            curves.push(flank(true)?);
            curves.push(NurbsCurve3::bspline_from_points(
                1,
                vec![
                    self.flank_point(centre, 0.0, true),
                    self.at(self.root_radius, centre + self.half_angle_at_base),
                ],
            )?);
            curves.push(self.arc(
                self.root_radius,
                centre + self.half_angle_at_base,
                next_centre - self.half_angle_at_base,
            )?);
        }

        Ok(curves)
    }

    /// 検証用に、ホブの運動とフィレットをまとめて返す。
    fn root_fillet_generation(&self) -> Result<RootFilletGeneration, String> {
        let rho = self.hob_tip_radius();
        let polar = self.trochoid_fillet_polar(24)?;
        let offset = std::f64::consts::FRAC_PI_2
            + std::f64::consts::PI / (2.0 * self.teeth as f64);
        Ok(RootFilletGeneration {
            pitch_radius: self.pitch_radius,
            base_radius: self.base_radius,
            root_radius: self.root_radius,
            form_radius: polar.last().expect("samples").0,
            tip_round_radius: rho,
            tip_round_centre: self.hob_tip_centre(rho),
            fillet: polar
                .iter()
                .map(|(radius, angle)| (*radius, angle - offset))
                .collect(),
        })
    }

    /// 歯元にホブ創成トロコイドのフィレットを持つ断面曲線列を生成する。
    ///
    /// 導出フレーム（`trochoid_fillet_polar`）は、`theta = 0` で工具の歯面が
    /// 歯車のピッチ点 `(0, r_p)` に触れる置き方をしている。そこは歯の**右歯面**
    /// がピッチ円を通る位置なので、歯の中心は `pi/2 + pi/(2z)` にある。実際の
    /// 歯の中心 `centre` へ移すには、その差だけ回せばよい。
    fn profile_curves_with_root_fillet(&self, flank_samples: usize) -> Result<Vec<NurbsCurve3>, String> {
        let z = self.teeth as f64;
        let pitch_angle = 2.0 * std::f64::consts::PI / z;
        let fillet = self.trochoid_fillet_polar(12)?;
        let frame_offset = std::f64::consts::FRAC_PI_2 + std::f64::consts::PI / (2.0 * z);

        // フィレットの外端が乗る半径。ここからがインボリュートになる。
        let (r_form, _) = *fillet.last().expect("the fillet has samples");
        let form_parameter = ((r_form / self.base_radius).powi(2) - 1.0).max(0.0).sqrt();

        // 歯底円の弧が始まる角度（歯の中心からの差）。フィレットの内端。
        let (_, root_angle) = fillet[0];
        let root_half_angle = frame_offset - root_angle;

        let mut curves = Vec::with_capacity(self.teeth * 6);
        for index in 0..self.teeth {
            let centre = index as f64 * pitch_angle;
            let next_centre = (index + 1) as f64 * pitch_angle;

            // 右のフィレットは歯底からインボリュートの始まりへ向かって上る。
            let fillet_curve = |left: bool| -> Result<NurbsCurve3, String> {
                let mut points: Vec<Point3> = fillet
                    .iter()
                    .map(|(radius, angle)| {
                        let delta = angle - frame_offset;
                        self.at(*radius, if left { centre - delta } else { centre + delta })
                    })
                    .collect();
                if left {
                    points.reverse();
                }
                NurbsCurve3::interpolate_points(3, &points)
            };

            // 歯面はフィレットが終わる半径から歯先まで。基礎円から始めると
            // フィレットと重なる。
            let flank = |left: bool| -> Result<NurbsCurve3, String> {
                let mut points: Vec<Point3> = (0..=flank_samples)
                    .map(|step| {
                        let t = form_parameter
                            + (self.tip_parameter - form_parameter) * step as f64
                                / flank_samples as f64;
                        self.flank_point(centre, t, left)
                    })
                    .collect();
                if left {
                    points.reverse();
                }
                NurbsCurve3::interpolate_points(3, &points)
            };

            curves.push(fillet_curve(false)?);
            curves.push(flank(false)?);
            curves.push(self.arc(
                self.tip_radius,
                centre - self.half_angle_at_tip,
                centre + self.half_angle_at_tip,
            )?);
            curves.push(flank(true)?);
            curves.push(fillet_curve(true)?);
            curves.push(self.arc(
                self.root_radius,
                centre + root_half_angle,
                next_centre - root_half_angle,
            )?);
        }

        Ok(curves)
    }
}
