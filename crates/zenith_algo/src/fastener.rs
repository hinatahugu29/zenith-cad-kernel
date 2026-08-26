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
        faces.push(Face::simple(FaceGeometry::Plane(pl_bot), Wire::new(bot_edges)));

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
        faces.push(Face::simple(FaceGeometry::Plane(pl_top), Wire::new(top_edges)));

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
            return Err("Hole radius must be strictly smaller than inscribed radius of hex".to_string());
        }

        let hex_body = Self::make_hex_prism(across_flats, height, tol)?;
        let drill = crate::PrimitiveBuilder::make_cylinder(hole_radius, height + 2.0)?;
        let positioned_drill = crate::BrepTransform::translate_solid(
            &drill,
            Vec3::new(0.0, 0.0, -1.0),
        );

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
}
