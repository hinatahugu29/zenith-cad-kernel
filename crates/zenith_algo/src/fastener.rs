//! ファスナー（Fastener / Bolt & Nut）モデリングアルゴリズム
//!
//! 機械設計におけるボルト頭部、六角ナット、ワッシャーなどの締結要素を
//! 正確なB-Rep多様体ソリッドとして構築します。

use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// ファスナー（ボルト・ナット・締結部品）ビルダー
pub struct FastenerBuilder;

impl FastenerBuilder {
    /// 二面幅（across_flats: S）と高さ（height: H）を持つ正六角柱（Hexagonal Prism）を構築
    /// 外接円半径 R = S / sqrt(3)
    pub fn make_hex_prism(
        across_flats: f64,
        height: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if across_flats <= 1e-6 || height <= 1e-6 {
            return Err("Hex prism dimensions must be strictly positive".to_string());
        }

        let s = across_flats;
        let r_outer = s / 3.0_f64.sqrt(); // 外接円半径

        // 底面 (z=0) 6頂点 (CCW: 0度, 60度, 120度, 180度, 240度, 300度)
        let mut pb = Vec::with_capacity(6);
        let mut pt = Vec::with_capacity(6);
        for i in 0..6 {
            let angle = (i as f64) * std::f64::consts::PI / 3.0;
            let x = r_outer * angle.cos();
            let y = r_outer * angle.sin();
            pb.push(Point3::new(x, y, 0.0));
            pt.push(Point3::new(x, y, height));
        }

        let vb: Vec<Vertex> = pb.iter().map(|&p| Vertex::from_point(p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|&p| Vertex::from_point(p)).collect();

        // 底面エッジ（6本）
        let mut eb = Vec::with_capacity(6);
        let mut et = Vec::with_capacity(6);
        let mut ev = Vec::with_capacity(6);

        for i in 0..6 {
            let next = (i + 1) % 6;
            eb.push(Edge::line_between(vb[i].clone(), vb[next].clone())?);
            et.push(Edge::line_between(vt[i].clone(), vt[next].clone())?);
            ev.push(Edge::line_between(vb[i].clone(), vt[i].clone())?);
        }

        let mut faces = Vec::with_capacity(8);

        // 側面 6面
        for i in 0..6 {
            let next = (i + 1) % 6;
            let edge_dir = (pb[next] - pb[i]).normalize();
            let u_axis = edge_dir;
            let v_axis = Vec3::new(0.0, 0.0, 1.0);

            let plane = PlaneSurface3::new(pb[i], u_axis, v_axis).ok_or("plane side")?;
            faces.push(Face::simple(
                FaceGeometry::Plane(plane),
                Wire::new(vec![
                    OrientedEdge::forward(eb[i].clone()),
                    OrientedEdge::forward(ev[next].clone()),
                    OrientedEdge::reversed(et[i].clone()),
                    OrientedEdge::reversed(ev[i].clone()),
                ]),
            ));
        }

        // 底面 (z=0, 法線 -Z, CCW: vb5 -> vb4 -> ... -> vb0)
        let pl_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bot")?;
        let mut bot_edges = Vec::with_capacity(6);
        for i in (0..6).rev() {
            bot_edges.push(OrientedEdge::reversed(eb[i].clone()));
        }
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_bot),
            Wire::new(bot_edges),
        ));

        // 天面 (z=height, 法線 +Z, CCW: vt0 -> vt1 -> ... -> vt5)
        let pl_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, height),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane top")?;
        let mut top_edges = Vec::with_capacity(6);
        for i in 0..6 {
            top_edges.push(OrientedEdge::forward(et[i].clone()));
        }
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_top),
            Wire::new(top_edges),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 二面幅（across_flats）、高さ（height）、内径穴（hole_radius）を持つ六角ナットブランクソリッドを構築
    pub fn make_hex_nut_blank(
        across_flats: f64,
        height: f64,
        hole_radius: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let r_inscribed = across_flats * 0.5;
        if hole_radius >= r_inscribed {
            return Err(
                "Hole radius must be strictly smaller than inscribed radius of hex".to_string(),
            );
        }

        let hex_body = Self::make_hex_prism(across_flats, height, tol)?;
        let drill = crate::PrimitiveBuilder::make_cylinder(hole_radius, height + 2.0)?;
        let positioned_drill =
            crate::BrepTransform::translate_solid(&drill, Vec3::new(0.0, 0.0, -1.0));

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &hex_body,
            &positioned_drill,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }

    /// JIS/ISO規格準拠の六角穴付きボルト（Socket Head Cap Screw）ソリッドを構築
    ///
    /// `shank_radius`: ボルトねじ軸部半径 (M8なら 4.0)
    /// `shank_length`: 軸部長 (30.0)
    /// `head_radius`: 頭部円柱半径 (6.5)
    /// `head_height`: 頭部高さ (8.0)
    /// `socket_across_flats`: 六角穴の二面幅 S (6.0)
    /// `socket_depth`: 六角穴の深さ (4.0)
    pub fn make_socket_head_cap_screw(
        shank_radius: f64,
        shank_length: f64,
        head_radius: f64,
        head_height: f64,
        socket_across_flats: f64,
        socket_depth: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if socket_depth >= head_height {
            return Err("Socket depth must be less than head height".to_string());
        }
        let r_socket_outer = socket_across_flats / 3.0_f64.sqrt();
        if r_socket_outer >= head_radius {
            return Err("Socket size must fit inside screw head radius".to_string());
        }

        // 1. 段付きシャフト（下部: 軸部 shank_radius x shank_length, 上部: 頭部 head_radius x head_height）
        let bolt_blank = crate::ShaftBuilder::make_stepped_shaft(&[
            (shank_radius, shank_length),
            (head_radius, head_height),
        ])?;

        // 2. 六角穴カッターソリッド
        let socket_cutter = Self::make_hex_prism(socket_across_flats, socket_depth + 1.0, tol)?;
        let positioned_socket = crate::BrepTransform::translate_solid(
            &socket_cutter,
            Vec3::new(0.0, 0.0, shank_length + head_height - socket_depth),
        );

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &bolt_blank,
            &positioned_socket,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }

    /// JIS/ISO規格準拠の平座金（Plain Washer）ソリッドを構築
    pub fn make_plain_washer(
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if inner_radius >= outer_radius || inner_radius <= 1e-6 || thickness <= 1e-6 {
            return Err(format!(
                "Invalid plain washer dimensions: inner={inner_radius}, outer={outer_radius}, thickness={thickness}"
            ));
        }

        let outer = crate::PrimitiveBuilder::make_cylinder(outer_radius, thickness)?;
        let inner = crate::PrimitiveBuilder::make_cylinder(inner_radius, thickness + 2.0)?;
        let inner = crate::BrepTransform::translate_solid(&inner, Vec3::new(0.0, 0.0, -1.0));

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &outer,
            &inner,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }

    /// JIS/ISO規格準拠のフランジ付き六角ボルト（Flanged Hex Bolt）ソリッドを構築
    ///
    /// `shank_radius`: ボルト軸部半径 (M8なら 4.0)
    /// `shank_length`: 軸部長 (30.0)
    /// `flange_radius`: フランジ円盤半径 (8.5)
    /// `flange_height`: フランジ厚み (2.0)
    /// `hex_across_flats`: 六角頭二面幅 S (12.0)
    /// `hex_head_height`: 六角頭高さ (6.0)
    pub fn make_flanged_hex_bolt(
        shank_radius: f64,
        shank_length: f64,
        flange_radius: f64,
        flange_height: f64,
        hex_across_flats: f64,
        hex_head_height: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        let r_hex_outer = hex_across_flats / 3.0_f64.sqrt();
        if flange_radius < r_hex_outer || shank_radius >= flange_radius {
            return Err(
                "Flange radius must enclose hex head outer radius and exceed shank radius"
                    .to_string(),
            );
        }

        // 1. 下部段付き円柱（軸部 ＋ フランジ部）
        let base_body = crate::ShaftBuilder::make_stepped_shaft(&[
            (shank_radius, shank_length),
            (flange_radius, flange_height),
        ])?;

        // 2. 上部正六角柱
        // わずかにフランジ部へ食い込ませることで境界面接触を排除して真の交差として安定結合
        let hex_body = Self::make_hex_prism(hex_across_flats, hex_head_height + 0.1, tol)?;
        let hex_body = crate::BrepTransform::translate_solid(
            &hex_body,
            Vec3::new(0.0, 0.0, shank_length + flange_height - 0.1),
        );

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &base_body,
            &hex_body,
            crate::boolean::BooleanOpType::Union,
            tol,
        )
    }

    /// JIS B 1251 規格準拠のスプリングワッシャー（ばね座金 / Spring Lock Washer）ソリッドを構築
    ///
    /// `inner_radius`: 内径半径 (M8なら 4.25)
    /// `outer_radius`: 外径半径 (7.4)
    /// `thickness`: 板厚 (2.0)
    /// `free_height`: 自由状態の高さ (3.5)
    /// `gap_deg`: 切欠きスリット角度 (20.0 度)
    pub fn make_spring_washer(
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
        free_height: f64,
        gap_deg: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if inner_radius >= outer_radius || thickness <= 1e-6 || free_height <= thickness {
            return Err("Invalid spring washer dimensions: outer must exceed inner, free_height must exceed thickness".to_string());
        }
        if gap_deg <= 0.0 || gap_deg >= 90.0 {
            return Err("Gap angle must be between 0 and 90 degrees".to_string());
        }

        let turns = (360.0 - gap_deg) / 360.0;
        let pitch = (free_height - thickness) / turns;
        let mean_radius = (inner_radius + outer_radius) * 0.5;
        let width = outer_radius - inner_radius;

        let profile_wire = crate::ProfileBuilder::make_rectangle(
            width,
            thickness,
            Point3::new(mean_radius, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )?;

        crate::HelixBuilder::sweep_wire_along_helix(
            &profile_wire,
            mean_radius,
            pitch,
            turns,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            64,
            tol,
        )
    }

    /// JIS B 2804 規格準拠のC形止め輪（サークリップ / Retaining Ring / Circlip）ソリッドを構築
    ///
    /// `inner_radius`: 内径半径 (例: 軸用 M10 なら 4.8)
    /// `outer_radius`: 外径半径 (例: 6.2)
    /// `thickness`: 板厚 (例: 1.0)
    /// `gap_angle_deg`: 切欠き開口角度 (例: 45.0 度)
    pub fn make_retaining_ring(
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
        gap_angle_deg: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if inner_radius >= outer_radius || thickness <= 1e-6 {
            return Err("Invalid retaining ring dimensions: outer must exceed inner, thickness must be positive".to_string());
        }
        if gap_angle_deg <= 0.0 || gap_angle_deg >= 180.0 {
            return Err("Gap angle must be between 0 and 180 degrees".to_string());
        }

        let start_angle = (gap_angle_deg * 0.5).to_radians();
        let end_angle = (360.0 - gap_angle_deg * 0.5).to_radians();
        let num_arc_segs = 4;
        let d_theta = (end_angle - start_angle) / num_arc_segs as f64;
        let wm = (d_theta * 0.5).cos();

        let mut edges: Vec<zenith_topo::OrientedEdge> = Vec::with_capacity(num_arc_segs * 2 + 2);

        // 1. 外周有理円弧 4 セグメント (start_angle -> end_angle, CCW)
        let mut outer_v = Vec::with_capacity(num_arc_segs + 1);
        for i in 0..=num_arc_segs {
            let theta = start_angle + i as f64 * d_theta;
            let pt = Point3::new(outer_radius * theta.cos(), outer_radius * theta.sin(), 0.0);
            outer_v.push(zenith_topo::Vertex::from_point(pt));
        }

        for i in 0..num_arc_segs {
            let th_a = start_angle + i as f64 * d_theta;
            let th_b = start_angle + (i + 1) as f64 * d_theta;
            let th_m = (th_a + th_b) * 0.5;

            let p0 = outer_v[i].point;
            let p1 = outer_v[i + 1].point;
            let p_mid = Point3::new(
                (outer_radius / wm) * th_m.cos(),
                (outer_radius / wm) * th_m.sin(),
                0.0,
            );

            let arc_curve = zenith_geom::NurbsCurve3::new(
                2,
                vec![
                    zenith_geom::ControlPoint3::unweighted(p0),
                    zenith_geom::ControlPoint3::new(p_mid, wm),
                    zenith_geom::ControlPoint3::unweighted(p1),
                ],
                zenith_geom::KnotVector::clamped_uniform(3, 2),
            )?;
            let edge = zenith_topo::Edge::new(
                arc_curve,
                outer_v[i].clone(),
                outer_v[i + 1].clone(),
                tol.linear,
            );
            edges.push(zenith_topo::OrientedEdge::forward(edge));
        }

        // 2. 終端直線エッジ (外径 end_angle -> 内径 end_angle)
        let mut inner_v = Vec::with_capacity(num_arc_segs + 1);
        for i in 0..=num_arc_segs {
            let theta = start_angle + i as f64 * d_theta;
            let pt = Point3::new(inner_radius * theta.cos(), inner_radius * theta.sin(), 0.0);
            inner_v.push(zenith_topo::Vertex::from_point(pt));
        }

        let end_line = zenith_topo::Edge::line_between(
            outer_v[num_arc_segs].clone(),
            inner_v[num_arc_segs].clone(),
        )?;
        edges.push(zenith_topo::OrientedEdge::forward(end_line));

        // 3. 内周有理円弧 4 セグメント (end_angle -> start_angle, 逆順 CW)
        for i in (0..num_arc_segs).rev() {
            let th_a = start_angle + (i + 1) as f64 * d_theta;
            let th_b = start_angle + i as f64 * d_theta;
            let th_m = (th_a + th_b) * 0.5;

            let p0 = inner_v[i + 1].point;
            let p1 = inner_v[i].point;
            let p_mid = Point3::new(
                (inner_radius / wm) * th_m.cos(),
                (inner_radius / wm) * th_m.sin(),
                0.0,
            );

            let arc_curve = zenith_geom::NurbsCurve3::new(
                2,
                vec![
                    zenith_geom::ControlPoint3::unweighted(p0),
                    zenith_geom::ControlPoint3::new(p_mid, wm),
                    zenith_geom::ControlPoint3::unweighted(p1),
                ],
                zenith_geom::KnotVector::clamped_uniform(3, 2),
            )?;
            let edge = zenith_topo::Edge::new(
                arc_curve,
                inner_v[i + 1].clone(),
                inner_v[i].clone(),
                tol.linear,
            );
            edges.push(zenith_topo::OrientedEdge::forward(edge));
        }

        // 4. 始端直線エッジ (内径 start_angle -> 外径 start_angle)
        let start_line = zenith_topo::Edge::line_between(inner_v[0].clone(), outer_v[0].clone())?;
        edges.push(zenith_topo::OrientedEdge::forward(start_line));

        let bottom_wire = zenith_topo::Wire::new(edges);
        crate::ExtrudeBuilder::extrude_wire(&bottom_wire, Vec3::new(0.0, 0.0, thickness), tol)
    }

    /// JIS B 1194 / ISO 10642 規格準拠の皿頭六角穴付きボルト（Countersunk Socket Head Cap Screw）ソリッドを構築
    ///
    /// `shank_radius`: ねじ軸半径 (例: M8 なら 4.0)
    /// `shank_length`: 首下ねじ長さ (例: 20.0)
    /// `head_radius`: 皿頭天面半径 (例: 8.0)
    /// `head_height`: 皿頭高さ (例: 4.4)
    /// `socket_across_flats`: 内六角二面幅 (例: 5.0)
    /// `socket_depth`: 六角穴深さ (例: 2.8)
    pub fn make_countersunk_socket_screw(
        shank_radius: f64,
        shank_length: f64,
        head_radius: f64,
        head_height: f64,
        socket_across_flats: f64,
        socket_depth: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if socket_depth >= head_height + shank_length {
            return Err("Socket depth exceeds bolt total height".to_string());
        }
        let r_socket_outer = socket_across_flats / 3.0_f64.sqrt();
        if r_socket_outer >= head_radius {
            return Err("Socket size must fit inside screw head radius".to_string());
        }
        if head_radius <= shank_radius {
            return Err("Head radius must exceed shank radius".to_string());
        }

        // 1. 軸部円柱（皿頭側へわずかに 0.1 食い込ませて配置）
        let shank = crate::PrimitiveBuilder::make_cylinder(shank_radius, shank_length + 0.1)?;

        // 2. 皿頭円錐台（小径 shank_radius -> 大径 head_radius）
        let head_cone = crate::PrimitiveBuilder::make_cone(shank_radius, head_radius, head_height)?;
        let head_cone =
            crate::BrepTransform::translate_solid(&head_cone, Vec3::new(0.0, 0.0, shank_length));

        let blank = crate::boolean::BooleanEngine::boolean_solids_exact(
            &shank,
            &head_cone,
            crate::boolean::BooleanOpType::Union,
            tol,
        )?;

        // 3. 六角穴カッター
        let socket_cutter = Self::make_hex_prism(socket_across_flats, socket_depth + 1.0, tol)?;
        let positioned_socket = crate::BrepTransform::translate_solid(
            &socket_cutter,
            Vec3::new(0.0, 0.0, shank_length + head_height - socket_depth),
        );

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &blank,
            &positioned_socket,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }

    /// JIS B 2220 / ASME B16.5 規格準拠の溶接ネック配管フランジ（Weld Neck Pipe Flange）ソリッドを構築
    ///
    /// `flange_radius`: フランジ外径半径 (例: 25.0)
    /// `flange_thickness`: フランジ厚み (例: 10.0)
    /// `hub_radius`: ハブ首部外径半径 (例: 15.0)
    /// `hub_height`: ハブ首部高さ (例: 15.0)
    /// `pipe_radius`: パイプ貫通内径半径 (例: 8.0)
    /// `pcd_radius`: ボルトピッチ円半径 (例: 19.0)
    /// `bolt_hole_radius`: ボルト穴半径 (例: 3.0)
    /// `num_bolt_holes`: ボルト穴数 (例: 4)
    pub fn make_weld_neck_flange(
        flange_radius: f64,
        flange_thickness: f64,
        hub_radius: f64,
        hub_height: f64,
        pipe_radius: f64,
        pcd_radius: f64,
        bolt_hole_radius: f64,
        num_bolt_holes: usize,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if flange_radius <= hub_radius || hub_radius <= pipe_radius || flange_thickness <= 1e-6 {
            return Err("Invalid flange dimensions: flange > hub > pipe required".to_string());
        }
        if pcd_radius + bolt_hole_radius >= flange_radius
            || pcd_radius - bolt_hole_radius <= hub_radius
        {
            return Err("Bolt holes must fit inside flange outer rim outside hub".to_string());
        }
        if num_bolt_holes == 0 {
            return Err("Must have at least 1 bolt hole".to_string());
        }

        // 1. フランジ本体 + ハブ首部の段付き軸
        let mut solid = crate::ShaftBuilder::make_stepped_shaft(&[
            (flange_radius, flange_thickness),
            (hub_radius, hub_height),
        ])?;

        // 2. 中央パイプ貫通穴
        let total_h = flange_thickness + hub_height;
        let pipe_cutter = crate::PrimitiveBuilder::make_cylinder(pipe_radius, total_h + 2.0)?;
        let pipe_cutter =
            crate::BrepTransform::translate_solid(&pipe_cutter, Vec3::new(0.0, 0.0, -1.0));
        solid = crate::boolean::BooleanEngine::boolean_solids_exact(
            &solid,
            &pipe_cutter,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )?;

        // 3. PCD 上のボルト穴群
        let d_theta = std::f64::consts::TAU / num_bolt_holes as f64;
        for i in 0..num_bolt_holes {
            let theta = i as f64 * d_theta;
            let bx = pcd_radius * theta.cos();
            let by = pcd_radius * theta.sin();

            let bolt_cutter =
                crate::PrimitiveBuilder::make_cylinder(bolt_hole_radius, flange_thickness + 2.0)?;
            let bolt_cutter =
                crate::BrepTransform::translate_solid(&bolt_cutter, Vec3::new(bx, by, -1.0));
            solid = crate::boolean::BooleanEngine::boolean_solids_exact(
                &solid,
                &bolt_cutter,
                crate::boolean::BooleanOpType::Difference,
                tol,
            )?;
        }

        Ok(solid)
    }

    /// JIS B 0203 / ANSI B16.14 規格準拠の六角穴付き管用テーパプラグ（Hexagon Socket Taper Pipe Plug）ソリッドを構築
    ///
    /// `small_radius`: テーパ先端小径半径 (例: PT 1/4 なら 6.0)
    /// `large_radius`: テーパ後端大径半径 (例: PT 1/4 なら 6.6)
    /// `height`: プラグ全長 (例: 10.0)
    /// `socket_across_flats`: 内六角二面幅 (例: 6.0)
    /// `socket_depth`: 六角穴深さ (例: 5.0)
    pub fn make_taper_pipe_plug(
        small_radius: f64,
        large_radius: f64,
        height: f64,
        socket_across_flats: f64,
        socket_depth: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if small_radius >= large_radius || small_radius <= 1e-6 || height <= 1e-6 {
            return Err("Invalid taper plug dimensions: large > small > 0 required".to_string());
        }
        if socket_depth >= height {
            return Err("Socket depth must be less than plug height".to_string());
        }
        let r_socket_outer = socket_across_flats / 3.0_f64.sqrt();
        if r_socket_outer >= large_radius {
            return Err("Socket size must fit inside plug large end".to_string());
        }

        // 1. テーパ円錐台
        let cone_blank = crate::PrimitiveBuilder::make_cone(small_radius, large_radius, height)?;

        // 2. 六角穴カッター
        let socket_cutter = Self::make_hex_prism(socket_across_flats, socket_depth + 1.0, tol)?;
        let positioned_socket = crate::BrepTransform::translate_solid(
            &socket_cutter,
            Vec3::new(0.0, 0.0, height - socket_depth),
        );

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &cone_blank,
            &positioned_socket,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }

    /// JIS B 1173 / DIN 938 規格準拠の中央六角胴スタッドボルト（Hex Center Stud Bolt）ソリッドを構築
    ///
    /// `bottom_shank_radius`: 下部ねじ軸半径 (例: M8 なら 4.0)
    /// `bottom_shank_length`: 下部ねじ軸長さ (例: 15.0)
    /// `hex_across_flats`: 中央六角胴部二面幅 (例: 13.0)
    /// `hex_height`: 中央六角胴部高さ (例: 6.0)
    /// `top_shank_radius`: 上部ねじ軸半径 (例: M8 なら 4.0)
    /// `top_shank_length`: 上部ねじ軸長さ (例: 20.0)
    pub fn make_stud_bolt(
        bottom_shank_radius: f64,
        bottom_shank_length: f64,
        hex_across_flats: f64,
        hex_height: f64,
        top_shank_radius: f64,
        top_shank_length: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if bottom_shank_radius <= 1e-6 || top_shank_radius <= 1e-6 || hex_height <= 1e-6 {
            return Err("Stud bolt shank radii and hex height must be positive".to_string());
        }
        let r_hex_inner = hex_across_flats * 0.5;
        if bottom_shank_radius >= r_hex_inner || top_shank_radius >= r_hex_inner {
            return Err("Hex collar across flats must be larger than shank diameters".to_string());
        }

        // 1. 中央六角胴部
        let hex_body = Self::make_hex_prism(hex_across_flats, hex_height, tol)?;
        let hex_body = crate::BrepTransform::translate_solid(
            &hex_body,
            Vec3::new(0.0, 0.0, bottom_shank_length),
        );

        // 2. 下部ねじ軸（六角胴内へわずかに 0.1 食い込ませる）
        let bot_shank =
            crate::PrimitiveBuilder::make_cylinder(bottom_shank_radius, bottom_shank_length + 0.1)?;

        let solid_part = crate::boolean::BooleanEngine::boolean_solids_exact(
            &hex_body,
            &bot_shank,
            crate::boolean::BooleanOpType::Union,
            tol,
        )?;

        // 3. 上部ねじ軸（六角胴内へわずかに 0.1 食い込ませる）
        let top_shank =
            crate::PrimitiveBuilder::make_cylinder(top_shank_radius, top_shank_length + 0.1)?;
        let top_shank = crate::BrepTransform::translate_solid(
            &top_shank,
            Vec3::new(0.0, 0.0, bottom_shank_length + hex_height - 0.1),
        );

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &solid_part,
            &top_shank,
            crate::boolean::BooleanOpType::Union,
            tol,
        )
    }

    /// JIS B 2706 / DIN 2093 規格準拠の皿ばね（Belleville Disc Spring / Conical Spring Washer）ソリッドを構築
    ///
    /// `inner_radius`: 内径半径 (例: 8.2)
    /// `outer_radius`: 外径半径 (例: 16.0)
    /// `thickness`: 板厚 (例: 0.9)
    /// `cone_height`: テーパー高さ (例: 1.25)
    pub fn make_belleville_spring(
        inner_radius: f64,
        outer_radius: f64,
        thickness: f64,
        cone_height: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if inner_radius >= outer_radius
            || inner_radius <= 1e-6
            || thickness <= 1e-6
            || cone_height <= 1e-6
        {
            return Err("Invalid Belleville spring dimensions".to_string());
        }

        let r_out_bot = outer_radius;
        let r_out_top = outer_radius - 1.5;
        let r_in_bot = inner_radius;
        let r_in_top = inner_radius - 1.5;
        let h = cone_height + thickness;

        let outer_cone = crate::PrimitiveBuilder::make_cone(r_out_bot, r_out_top, h)?;

        let k = 1.5 / h;
        let r_cutter_bot = r_in_bot + 1.0 * k;
        let r_cutter_top = r_in_top - 1.0 * k;
        let inner_cone = crate::PrimitiveBuilder::make_cone(r_cutter_bot, r_cutter_top, h + 2.0)?;
        let inner_cone =
            crate::BrepTransform::translate_solid(&inner_cone, Vec3::new(0.0, 0.0, -1.0));

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &outer_cone,
            &inner_cone,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }
}
