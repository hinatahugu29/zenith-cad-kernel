use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 押し出し（Extrude）モデリングアルゴリズム
pub struct ExtrudeBuilder;

impl ExtrudeBuilder {
    /// 閉じた平坦なワイヤ（底面）と押し出し方向ベクトルから、閉じたSolid（側面Face群 + 底面Face + 天面Face）を構築
    pub fn extrude_wire(bottom_wire: &Wire, dir: Vec3, tol: &Tolerance) -> Result<Solid, String> {
        if !bottom_wire.is_closed(tol) {
            return Err("Extrude requires a closed wire".to_string());
        }

        let num_edges = bottom_wire.edges.len();
        if num_edges < 3 {
            return Err("Extrude requires at least 3 edges in the wire".to_string());
        }

        // 1. 底面頂点と天面頂点の生成
        let mut top_vertices = Vec::with_capacity(num_edges);
        let mut bottom_vertices = Vec::with_capacity(num_edges);

        for oe in &bottom_wire.edges {
            let v_bot = oe.start_vertex();
            bottom_vertices.push(v_bot.clone());
            let top_pt = v_bot.point + dir;
            top_vertices.push(Vertex::new(top_pt, tol.linear));
        }

        // 2. 天面エッジ群と天面ワイヤの構築
        let mut top_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let next_i = (i + 1) % num_edges;
            let edge = Edge::line_between(top_vertices[i].clone(), top_vertices[next_i].clone())?;
            top_edges.push(OrientedEdge::forward(edge));
        }
        let top_wire = Wire::new(top_edges);

        // 3. 側面エッジ群（縦方向エッジ柱）の生成
        let mut pillar_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let edge = Edge::line_between(bottom_vertices[i].clone(), top_vertices[i].clone())?;
            pillar_edges.push(edge);
        }

        // 4. 各側面Face（Side Faces）の構築 (底辺、右柱、天辺(rev)、左柱(rev))
        let mut faces = Vec::with_capacity(num_edges + 2);

        for i in 0..num_edges {
            let next_i = (i + 1) % num_edges;

            let bot_edge = bottom_wire.edges[i].edge.clone();
            let right_pillar = pillar_edges[next_i].clone();
            let top_edge = top_wire.edges[i].edge.clone();
            let left_pillar = pillar_edges[i].clone();

            let side_wire = Wire::new(vec![
                OrientedEdge::forward(bot_edge.clone()),
                OrientedEdge::forward(right_pillar),
                OrientedEdge::reversed(top_edge),
                OrientedEdge::reversed(left_pillar),
            ]);

            // 側面のNURBSルールド曲面（底辺カーブから天辺カーブへの押し出し）
            let surf = Self::make_ruled_surface(&bot_edge.curve, dir)?;
            faces.push(Face::simple(FaceGeometry::Nurbs(surf), side_wire));
        }

        // 5. 底面Faceと天面Faceの生成
        let bot_normal = -dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, -1.0));
        let top_normal = dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));

        let bottom_cap_wire = Self::reversed_wire(bottom_wire);
        let bot_face = Self::make_cap_face(&bottom_cap_wire, bottom_vertices[0].point, bot_normal)?;
        let top_face = Self::make_cap_face(&top_wire, top_vertices[0].point, top_normal)?;

        faces.push(bot_face);
        faces.push(top_face);

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 底辺カーブから押し出し方向へのルールドNURBS曲面を生成
    fn make_ruled_surface(curve: &NurbsCurve3, dir: Vec3) -> Result<NurbsSurface3, String> {
        let n_u = curve.control_points.len();
        let degree_u = curve.degree;
        let degree_v = 1;

        let mut row0 = Vec::with_capacity(n_u);
        let mut row1 = Vec::with_capacity(n_u);

        for cp in &curve.control_points {
            row0.push(*cp);
            let top_pt = cp.point + dir;
            row1.push(ControlPoint3::new(top_pt, cp.weight));
        }

        let knots_u = curve.knots.clone();
        let knots_v = KnotVector::clamped_uniform(2, 1);

        NurbsSurface3::new(degree_u, degree_v, vec![row0, row1], knots_u, knots_v)
    }

    /// 平面キャップFaceの生成
    fn make_cap_face(wire: &Wire, origin: Point3, normal: Vec3) -> Result<Face, String> {
        let arb = if normal.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u_axis = normal
            .cross(&arb)
            .try_normalize_safe(1e-12)
            .ok_or("Failed u_axis")?;
        let v_axis = normal
            .cross(&u_axis)
            .try_normalize_safe(1e-12)
            .ok_or("Failed v_axis")?;

        let plane = PlaneSurface3::new(origin, u_axis, v_axis).ok_or("Failed to create plane")?;
        Ok(Face::simple(FaceGeometry::Plane(plane), wire.clone()))
    }

    fn reversed_wire(wire: &Wire) -> Wire {
        let edges = wire
            .edges
            .iter()
            .rev()
            .map(|edge| {
                let mut reversed = edge.clone();
                reversed.orientation = reversed.orientation.reversed();
                reversed
            })
            .collect();
        Wire::new(edges)
    }
}
