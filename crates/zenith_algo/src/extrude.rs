use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 押し出し（Extrude）モデリングアルゴリズム
pub struct ExtrudeBuilder;

impl ExtrudeBuilder {
    /// 閉じた平坦なワイヤ（底面）と押し出し方向ベクトルから、閉じたSolid（側面Face群 + 底面Face + 天面Face）を構築
    pub fn extrude_wire(bottom_wire: &Wire, dir: Vec3, tol: &Tolerance) -> Result<Solid, String> {
        Self::extrude_face_with_holes(bottom_wire, &[], dir, tol)
    }

    /// 外側境界ワイヤと複数の内側境界（穴）ワイヤを持つ平坦なプロファイルから中空・穴あき押し出しSolidを構築
    pub fn extrude_face_with_holes(
        outer_wire: &Wire,
        inner_wires: &[Wire],
        dir: Vec3,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if !outer_wire.is_closed(tol) {
            return Err("Extrude requires a closed outer wire".to_string());
        }
        if outer_wire.edges.len() < 3 {
            return Err("Extrude requires at least 3 edges in outer wire".to_string());
        }

        for (idx, hole) in inner_wires.iter().enumerate() {
            if !hole.is_closed(tol) {
                return Err(format!("Inner wire {} must be closed", idx));
            }
            if hole.edges.len() < 3 {
                return Err(format!("Inner wire {} must have at least 3 edges", idx));
            }
        }

        let mut faces = Vec::new();

        // 1. 外壁側面Face群（Outer Wall Faces）の生成 (is_hole: false)
        let (top_outer_wire, outer_side_faces) = Self::extrude_loop(outer_wire, dir, false, tol)?;
        faces.extend(outer_side_faces);

        // 2. 内壁側面Face群（Inner Hole Wall Faces）の生成 (is_hole: true)
        let mut top_inner_wires = Vec::with_capacity(inner_wires.len());
        for hole_wire in inner_wires {
            let (top_hole_wire, hole_side_faces) = Self::extrude_loop(hole_wire, dir, true, tol)?;
            faces.extend(hole_side_faces);
            top_inner_wires.push(top_hole_wire);
        }

        // 3. 底面キャップFace（-dir 法線）
        let bot_normal = -dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, -1.0));
        let top_normal = dir
            .try_normalize_safe(1e-12)
            .unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));

        let bot_outer_wire = Self::reversed_wire(outer_wire);
        let bot_inner_wires = inner_wires.to_vec();

        let bot_face = Self::make_cap_face_with_holes(
            &bot_outer_wire,
            &bot_inner_wires,
            outer_wire.edges[0].start_vertex().point,
            bot_normal,
        )?;
        faces.push(bot_face);

        // 4. 天面キャップFace（+dir 法線）
        let top_inner_reversed: Vec<Wire> = top_inner_wires.iter().map(Self::reversed_wire).collect();
        let top_face = Self::make_cap_face_with_holes(
            &top_outer_wire,
            &top_inner_reversed,
            top_outer_wire.edges[0].start_vertex().point,
            top_normal,
        )?;
        faces.push(top_face);

        let shell = Shell::closed(faces);
        let report = shell.validate_closed(tol);
        if !report.is_valid() {
            return Err(format!("Extrude hollow validation failed: {:?}", report.errors));
        }
        crate::validated_solid(shell)
    }

    /// 単一ループ（外側または穴）から天面ワイヤと側面Face群を構築
    fn extrude_loop(
        wire: &Wire,
        dir: Vec3,
        is_hole: bool,
        tol: &Tolerance,
    ) -> Result<(Wire, Vec<Face>), String> {
        let num_edges = wire.edges.len();

        let mut bottom_vertices = Vec::with_capacity(num_edges);
        let mut top_vertices = Vec::with_capacity(num_edges);

        for oe in &wire.edges {
            let v_bot = oe.start_vertex();
            bottom_vertices.push(v_bot.clone());
            let top_pt = v_bot.point + dir;
            top_vertices.push(Vertex::new(top_pt, tol.linear));
        }

        let mut top_edges = Vec::with_capacity(num_edges);
        for oriented in &wire.edges {
            top_edges.push(OrientedEdge::new(
                crate::BrepTransform::translate_edge(&oriented.edge, dir),
                oriented.orientation,
            ));
        }
        let top_wire = Wire::new(top_edges);

        let mut pillar_edges = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let edge = Edge::line_between(bottom_vertices[i].clone(), top_vertices[i].clone())?;
            pillar_edges.push(edge);
        }

        let mut side_faces = Vec::with_capacity(num_edges);
        for i in 0..num_edges {
            let next_i = (i + 1) % num_edges;

            let bot_edge = wire.edges[i].edge.clone();
            let right_pillar = pillar_edges[next_i].clone();
            let top_edge = top_wire.edges[i].edge.clone();
            let left_pillar = pillar_edges[i].clone();

            let (side_wire, surf) = if !is_hole {
                // 外壁: 外向き法線
                let w = Wire::new(vec![
                    OrientedEdge::forward(bot_edge.clone()),
                    OrientedEdge::forward(right_pillar),
                    OrientedEdge::reversed(top_edge),
                    OrientedEdge::reversed(left_pillar),
                ]);
                let s = Self::make_ruled_surface(&bot_edge.curve, dir)?;
                (w, s)
            } else {
                // 穴内壁: 穴の中心向き法線（曲線を反転して法線を内向きにする）
                let w = Wire::new(vec![
                    OrientedEdge::reversed(bot_edge.clone()),
                    OrientedEdge::forward(left_pillar),
                    OrientedEdge::forward(top_edge),
                    OrientedEdge::reversed(right_pillar),
                ]);
                let rev_curve = bot_edge.curve.reversed();
                let s = Self::make_ruled_surface(&rev_curve, dir)?;
                (w, s)
            };

            side_faces.push(Face::simple(FaceGeometry::Nurbs(surf), side_wire));
        }

        Ok((top_wire, side_faces))
    }

    /// 底辺カーブから押し出し方向へのルールドNURBS曲面を生成
    fn make_ruled_surface(curve: &NurbsCurve3, dir: Vec3) -> Result<NurbsSurface3, String> {
        let n_u = curve.control_points.len();
        let degree_u = curve.degree;
        let degree_v = 1;

        let mut control_points = Vec::with_capacity(n_u);
        for cp in &curve.control_points {
            control_points.push(vec![*cp, ControlPoint3::new(cp.point + dir, cp.weight)]);
        }

        let knots_u = curve.knots.clone();
        let knots_v = KnotVector::clamped_uniform(2, 1);

        NurbsSurface3::new(degree_u, degree_v, control_points, knots_u, knots_v)
    }

    /// 平面キャップFace（穴あき対応）の生成
    fn make_cap_face_with_holes(
        outer_wire: &Wire,
        inner_wires: &[Wire],
        origin: Point3,
        normal: Vec3,
    ) -> Result<Face, String> {
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
        Ok(Face::new(
            FaceGeometry::Plane(plane),
            outer_wire.clone(),
            inner_wires.to_vec(),
            zenith_topo::Orientation::Forward,
            1e-6,
        ))
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
