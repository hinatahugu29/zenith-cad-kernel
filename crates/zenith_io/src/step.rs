use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_geom::{NurbsCurve2, NurbsCurve3, NurbsSurface3};
use zenith_math::{Point2, Point3, Tolerance};
use zenith_topo::{Face, FaceGeometry, FacePcurveLoop, FacePcurveSegment, Solid, Wire};

/// ISO 10303-21 (STEP AP214 / AP203 / AP242) 完全共有マニホールド B-Rep エクスポーター
pub struct StepExporter;

struct StepContext {
    id_counter: u64,
    lines: Vec<String>,
    vertex_map: HashMap<u64, u64>, // Vertex ID -> STEP Vertex ID
    edge_map: HashMap<u64, u64>,   // Edge ID -> STEP EdgeCurve ID
    pcurve_context_id: Option<u64>,
}

impl StepContext {
    fn new() -> Self {
        Self {
            id_counter: 1,
            lines: Vec::new(),
            vertex_map: HashMap::new(),
            edge_map: HashMap::new(),
            pcurve_context_id: None,
        }
    }

    fn next_id(&mut self) -> u64 {
        let id = self.id_counter;
        self.id_counter += 1;
        id
    }

    fn add_entity(&mut self, entity: &str) -> u64 {
        let id = self.next_id();
        self.lines.push(format!("#{} = {};", id, entity));
        id
    }
}

impl StepExporter {
    /// SolidをSTEP形式（ISO 10303-21）ファイルとして出力
    pub fn export_solid_to_file<P: AsRef<Path>>(
        solid: &Solid,
        path: P,
        product_name: &str,
    ) -> std::io::Result<()> {
        let content = Self::export_solid_to_string(solid, product_name);
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut file = File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }

    /// SolidをSTEP形式の文字列として生成
    pub fn export_solid_to_string(solid: &Solid, product_name: &str) -> String {
        let mut ctx = StepContext::new();

        // 1. プロダクト・コンテキスト定義（FreeCAD / OCCT 必須構造）
        let app_context_id = ctx.add_entity(
            "APPLICATION_CONTEXT('core data for automotive mechanical design processes')",
        );
        let _app_protocol_id = ctx.add_entity(&format!(
            "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{})",
            app_context_id
        ));
        let product_context_id = ctx.add_entity(&format!(
            "PRODUCT_CONTEXT('',#{},'mechanical')",
            app_context_id
        ));
        let product_id = ctx.add_entity(&format!(
            "PRODUCT('{}','{}','',(#{}))",
            product_name, product_name, product_context_id
        ));
        let product_def_formation_id = ctx.add_entity(&format!(
            "PRODUCT_DEFINITION_FORMATION('','',#{})",
            product_id
        ));
        let product_def_context_id = ctx.add_entity(&format!(
            "PRODUCT_DEFINITION_CONTEXT('part definition',#{},'design')",
            app_context_id
        ));
        let product_def_id = ctx.add_entity(&format!(
            "PRODUCT_DEFINITION('design','',#{},#{})",
            product_def_formation_id, product_def_context_id
        ));
        let product_def_shape_id = ctx.add_entity(&format!(
            "PRODUCT_DEFINITION_SHAPE('','',#{})",
            product_def_id
        ));

        // 2. 幾何単位・コンテキスト定義
        let length_unit_id =
            ctx.add_entity("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
        let plane_angle_unit_id =
            ctx.add_entity("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
        let solid_angle_unit_id =
            ctx.add_entity("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
        let uncertainty_id = ctx.add_entity(&format!(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#{},'distance_accuracy_value','confusion accuracy')",
            length_unit_id
        ));

        let geom_context_id = ctx.add_entity(&format!(
            "GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{},#{},#{})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY')",
            uncertainty_id, length_unit_id, plane_angle_unit_id, solid_angle_unit_id
        ));
        ctx.pcurve_context_id = Some(ctx.add_entity(
            "GEOMETRIC_REPRESENTATION_CONTEXT(2) REPRESENTATION_CONTEXT('Parametric Context','2D p-curve Context')",
        ));

        // 原点と座標軸
        let origin_pt_id = Self::write_point(&mut ctx, Point3::new(0.0, 0.0, 0.0));
        let z_axis_id = ctx.add_entity("DIRECTION('',(0.0,0.0,1.0))");
        let x_axis_id = ctx.add_entity("DIRECTION('',(1.0,0.0,0.0))");
        let world_axis_id = ctx.add_entity(&format!(
            "AXIS2_PLACEMENT_3D('',#{},#{},#{})",
            origin_pt_id, z_axis_id, x_axis_id
        ));

        // 3. Shell内の各Faceのエンティティ生成（トポロジー共有）
        let mut advanced_face_ids = Vec::new();
        for face in &solid.outer_shell.faces {
            if let Some(face_id) = Self::write_face(&mut ctx, face) {
                advanced_face_ids.push(face_id);
            }
        }

        let face_list_str = advanced_face_ids
            .iter()
            .map(|id| format!("#{}", id))
            .collect::<Vec<_>>()
            .join(",");

        let closed_shell_id = ctx.add_entity(&format!("CLOSED_SHELL('',({}))", face_list_str));
        let manifold_solid_id = ctx.add_entity(&format!(
            "MANIFOLD_SOLID_BREP('{}',#{})",
            product_name, closed_shell_id
        ));

        let shape_rep_id = ctx.add_entity(&format!(
            "ADVANCED_BREP_SHAPE_REPRESENTATION('{}',(#{},#{}),#{})",
            product_name, manifold_solid_id, world_axis_id, geom_context_id
        ));

        let _shape_def_rep_id = ctx.add_entity(&format!(
            "SHAPE_DEFINITION_REPRESENTATION(#{},#{})",
            product_def_shape_id, shape_rep_id
        ));

        // 4. STEPヘッダーおよびフッターの組み立て
        let mut out = String::new();
        out.push_str("ISO-10303-21;\nHEADER;\n");
        out.push_str("FILE_DESCRIPTION(('Zenith CAD Generated STEP File'),'2;1');\n");
        out.push_str(&format!("FILE_NAME('{}.stp','2026-08-18T10:00:00',('Zenith CAD'),('Zenith CAD Kernel'),'Zenith CAD Kernel 0.1.0','Zenith CAD','');\n", product_name));
        out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
        out.push_str("ENDSEC;\nDATA;\n");

        for line in &ctx.lines {
            out.push_str(line);
            out.push('\n');
        }

        out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
        out
    }

    fn write_point(ctx: &mut StepContext, p: Point3) -> u64 {
        let p_str = format!("CARTESIAN_POINT('',({:.6},{:.6},{:.6}))", p.x, p.y, p.z);
        ctx.add_entity(&p_str)
    }

    fn write_point2(ctx: &mut StepContext, p: Point2) -> u64 {
        let p_str = format!("CARTESIAN_POINT('',({:.6},{:.6}))", p.x, p.y);
        ctx.add_entity(&p_str)
    }

    fn get_or_create_vertex(ctx: &mut StepContext, v: &zenith_topo::Vertex) -> u64 {
        if let Some(&id) = ctx.vertex_map.get(&v.id) {
            return id;
        }
        let pt_id = Self::write_point(ctx, v.point);
        let v_id = ctx.add_entity(&format!("VERTEX_POINT('',#{})", pt_id));
        ctx.vertex_map.insert(v.id, v_id);
        v_id
    }

    fn get_or_create_edge_curve(ctx: &mut StepContext, edge: &zenith_topo::Edge) -> u64 {
        if let Some(&id) = ctx.edge_map.get(&edge.id) {
            return id;
        }
        let start_v_id = Self::get_or_create_vertex(ctx, &edge.start_vertex);
        let end_v_id = Self::get_or_create_vertex(ctx, &edge.end_vertex);
        let curve_id = Self::write_edge_curve_geometry(
            ctx,
            &edge.curve,
            edge.start_vertex.point,
            edge.end_vertex.point,
        );

        let edge_curve_id = ctx.add_entity(&format!(
            "EDGE_CURVE('',#{},#{},#{},.T.)",
            start_v_id, end_v_id, curve_id
        ));
        ctx.edge_map.insert(edge.id, edge_curve_id);
        edge_curve_id
    }

    fn write_edge_curve_geometry(
        ctx: &mut StepContext,
        nurbs: &NurbsCurve3,
        p_start: Point3,
        p_end: Point3,
    ) -> u64 {
        // 1次（直線）の場合: LINE エンティティとして出力（OCCT最適化）
        if nurbs.degree == 1 && nurbs.control_points.len() == 2 {
            let p_id = Self::write_point(ctx, p_start);
            let dir = (p_end - p_start).normalize();
            let dir_id = ctx.add_entity(&format!(
                "DIRECTION('',({:.6},{:.6},{:.6}))",
                dir.x, dir.y, dir.z
            ));
            let vec_id = ctx.add_entity(&format!("VECTOR('',#{},1.0)", dir_id));
            return ctx.add_entity(&format!("LINE('',#{},#{})", p_id, vec_id));
        }

        // 2次有理円弧の場合: 円弧（TRIMMED_CURVE of CIRCLE）として判定・出力可能か確認
        if nurbs.degree == 2
            && nurbs.control_points.len() == 3
            && (nurbs.control_points[1].weight - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-4
        {
            let p0 = nurbs.control_points[0].point;
            let p1 = nurbs.control_points[1].point;
            let p2 = nurbs.control_points[2].point;
            // コーナー制御点 p1 から中心 C を復元: C = p0 + p2 - p1
            let center = Point3::new(p0.x + p2.x - p1.x, p0.y + p2.y - p1.y, p0.z);
            let r0 = (p0 - center).norm();
            let r2 = (p2 - center).norm();
            if (r0 - r2).abs() < 1e-4 && r0 > 1e-6 {
                let v0 = (p0 - center).normalize();
                let v2 = (p2 - center).normalize();
                let normal = v0.cross(&v2).normalize();
                if normal.norm() > 0.5 {
                    let center_id = Self::write_point(ctx, center);
                    let norm_id = ctx.add_entity(&format!(
                        "DIRECTION('',({:.6},{:.6},{:.6}))",
                        normal.x, normal.y, normal.z
                    ));
                    let ref_id = ctx.add_entity(&format!(
                        "DIRECTION('',({:.6},{:.6},{:.6}))",
                        v0.x, v0.y, v0.z
                    ));
                    let axis_id = ctx.add_entity(&format!(
                        "AXIS2_PLACEMENT_3D('',#{},#{},#{})",
                        center_id, norm_id, ref_id
                    ));
                    let circle_id = ctx.add_entity(&format!("CIRCLE('',#{},{:.6})", axis_id, r0));
                    let p0_id = Self::write_point(ctx, p0);
                    let p2_id = Self::write_point(ctx, p2);
                    return ctx.add_entity(&format!(
                        "TRIMMED_CURVE('',#{},(#{},PARAMETER_VALUE(0.0)),(#{},PARAMETER_VALUE(1.570796326795)),.T.,.PARAMETER.)",
                        circle_id, p0_id, p2_id
                    ));
                }
            }
        }

        // 一般のNURBS曲線（B_SPLINE_CURVE_WITH_KNOTS）
        Self::write_nurbs_curve(ctx, nurbs)
    }

    fn write_nurbs_curve(ctx: &mut StepContext, nurbs: &NurbsCurve3) -> u64 {
        let mut is_rational = false;
        let mut pt_ids = Vec::with_capacity(nurbs.control_points.len());
        let mut weights = Vec::with_capacity(nurbs.control_points.len());

        for cp in &nurbs.control_points {
            if (cp.weight - 1.0).abs() > 1e-6 {
                is_rational = true;
            }
            pt_ids.push(format!("#{}", Self::write_point(ctx, cp.point)));
            weights.push(format!("{:.6}", cp.weight));
        }

        let pts_str = format!("({})", pt_ids.join(","));
        let weights_str = format!("({})", weights.join(","));

        let (mults, knots) = Self::compress_knots(&nurbs.knots.knots);
        let mult_str = format!(
            "({})",
            mults
                .iter()
                .map(|m| m.to_string())
                .collect::<Vec<_>>()
                .join(",")
        );
        let knot_str = format!(
            "({})",
            knots
                .iter()
                .map(|k| format!("{:.6}", k))
                .collect::<Vec<_>>()
                .join(",")
        );

        if is_rational {
            ctx.add_entity(&format!(
                "( BOUNDED_CURVE() B_SPLINE_CURVE({},{},.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({},{},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE({}) REPRESENTATION_ITEM('') )",
                nurbs.degree, pts_str, mult_str, knot_str, weights_str
            ))
        } else {
            ctx.add_entity(&format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{},{},.UNSPECIFIED.,.F.,.F.,{},{},.UNSPECIFIED.)",
                nurbs.degree, pts_str, mult_str, knot_str
            ))
        }
    }

    fn write_nurbs_curve2(ctx: &mut StepContext, nurbs: &NurbsCurve2) -> u64 {
        let mut is_rational = false;
        let mut pt_ids = Vec::with_capacity(nurbs.control_points.len());
        let mut weights = Vec::with_capacity(nurbs.control_points.len());

        for cp in &nurbs.control_points {
            if (cp.weight - 1.0).abs() > 1e-6 {
                is_rational = true;
            }
            pt_ids.push(format!("#{}", Self::write_point2(ctx, cp.point)));
            weights.push(format!("{:.6}", cp.weight));
        }

        let pts_str = format!("({})", pt_ids.join(","));
        let weights_str = format!("({})", weights.join(","));

        let (mults, knots) = Self::compress_knots(&nurbs.knots.knots);
        let mult_str = format!(
            "({})",
            mults
                .iter()
                .map(|m| m.to_string())
                .collect::<Vec<_>>()
                .join(",")
        );
        let knot_str = format!(
            "({})",
            knots
                .iter()
                .map(|k| format!("{:.6}", k))
                .collect::<Vec<_>>()
                .join(",")
        );

        if is_rational {
            ctx.add_entity(&format!(
                "( BOUNDED_CURVE() B_SPLINE_CURVE({},{},.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({},{},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE({}) REPRESENTATION_ITEM('') )",
                nurbs.degree, pts_str, mult_str, knot_str, weights_str
            ))
        } else {
            ctx.add_entity(&format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{},{},.UNSPECIFIED.,.F.,.F.,{},{},.UNSPECIFIED.)",
                nurbs.degree, pts_str, mult_str, knot_str
            ))
        }
    }

    fn write_pcurve(
        ctx: &mut StepContext,
        surface_id: u64,
        segment: &FacePcurveSegment,
    ) -> Option<u64> {
        let pcurve_context_id = ctx.pcurve_context_id?;
        let curve2d_id = Self::write_nurbs_curve2(ctx, &segment.curve);
        let rep_id = ctx.add_entity(&format!(
            "DEFINITIONAL_REPRESENTATION('',(#{}),#{})",
            curve2d_id, pcurve_context_id
        ));
        Some(ctx.add_entity(&format!("PCURVE('',#{},#{})", surface_id, rep_id)))
    }

    fn write_edge_loop_on_surface(
        ctx: &mut StepContext,
        wire: &Wire,
        pcurve_loop: Option<&FacePcurveLoop>,
        surface_id: u64,
    ) -> u64 {
        let mut oriented_edge_ids = Vec::new();
        let pcurve_segments =
            pcurve_loop.filter(|loop_data| loop_data.segments.len() == wire.edges.len());

        for (edge_index, oe) in wire.edges.iter().enumerate() {
            let oriented_id = if let Some(loop_data) = pcurve_segments {
                Self::write_oriented_edge_on_surface(
                    ctx,
                    oe,
                    &loop_data.segments[edge_index],
                    surface_id,
                )
            } else {
                let edge_curve_id = Self::get_or_create_edge_curve(ctx, &oe.edge);
                let same_sense = if oe.orientation.is_forward() {
                    ".T."
                } else {
                    ".F."
                };
                ctx.add_entity(&format!(
                    "ORIENTED_EDGE('',*,*,#{},{})",
                    edge_curve_id, same_sense
                ))
            };
            oriented_edge_ids.push(format!("#{}", oriented_id));
        }

        let edge_list_str = oriented_edge_ids.join(",");
        ctx.add_entity(&format!("EDGE_LOOP('',({}))", edge_list_str))
    }

    fn write_oriented_edge_on_surface(
        ctx: &mut StepContext,
        oe: &zenith_topo::OrientedEdge,
        pcurve_segment: &FacePcurveSegment,
        surface_id: u64,
    ) -> u64 {
        let start_v_id = Self::get_or_create_vertex(ctx, oe.start_vertex());
        let end_v_id = Self::get_or_create_vertex(ctx, oe.end_vertex());
        let curve = if oe.orientation.is_forward() {
            oe.edge.curve.clone()
        } else {
            oe.edge.curve.reversed()
        };
        let curve_3d_id = Self::write_edge_curve_geometry(
            ctx,
            &curve,
            oe.start_vertex().point,
            oe.end_vertex().point,
        );
        let surface_curve_id =
            if let Some(pcurve_id) = Self::write_pcurve(ctx, surface_id, pcurve_segment) {
                ctx.add_entity(&format!(
                    "SURFACE_CURVE('',#{},(#{}),.PCURVE_S1.)",
                    curve_3d_id, pcurve_id
                ))
            } else {
                curve_3d_id
            };
        let edge_curve_id = ctx.add_entity(&format!(
            "EDGE_CURVE('',#{},#{},#{},.T.)",
            start_v_id, end_v_id, surface_curve_id
        ));
        ctx.add_entity(&format!("ORIENTED_EDGE('',*,*,#{},.T.)", edge_curve_id))
    }

    fn write_face(ctx: &mut StepContext, face: &Face) -> Option<u64> {
        let surface_id = match &face.geometry {
            FaceGeometry::Plane(plane) => {
                let loc_id = Self::write_point(ctx, plane.origin);
                let n = plane.normal.normalize();
                let u = plane.u_axis.normalize();
                let z_dir_id =
                    ctx.add_entity(&format!("DIRECTION('',({:.6},{:.6},{:.6}))", n.x, n.y, n.z));
                let x_dir_id =
                    ctx.add_entity(&format!("DIRECTION('',({:.6},{:.6},{:.6}))", u.x, u.y, u.z));
                let axis2_id = ctx.add_entity(&format!(
                    "AXIS2_PLACEMENT_3D('',#{},#{},#{})",
                    loc_id, z_dir_id, x_dir_id
                ));
                ctx.add_entity(&format!("PLANE('',#{})", axis2_id))
            }
            FaceGeometry::Nurbs(nurbs) => Self::write_nurbs_surface(ctx, nurbs),
            _ => return None,
        };

        // B-Rep EDGE_LOOP トポロジーの構築
        let mut bound_ids = Vec::new();
        let pcurves = face.pcurves(&Tolerance::default()).ok();

        // 1. 外側境界 (FACE_OUTER_BOUND)
        let outer_loop_id = Self::write_edge_loop_on_surface(
            ctx,
            &face.outer_wire,
            pcurves.as_ref().map(|p| &p.outer_loop),
            surface_id,
        );
        let outer_bound_id =
            ctx.add_entity(&format!("FACE_OUTER_BOUND('',#{},.T.)", outer_loop_id));
        bound_ids.push(format!("#{}", outer_bound_id));

        // 2. 内側穴境界群 (FACE_BOUND)
        for (inner_index, inner_wire) in face.inner_wires.iter().enumerate() {
            let inner_loop_id = Self::write_edge_loop_on_surface(
                ctx,
                inner_wire,
                pcurves
                    .as_ref()
                    .and_then(|p| p.inner_loops.get(inner_index)),
                surface_id,
            );
            let inner_bound_id = ctx.add_entity(&format!("FACE_BOUND('',#{},.T.)", inner_loop_id));
            bound_ids.push(format!("#{}", inner_bound_id));
        }

        let bounds_str = bound_ids.join(",");
        let same_sense = if face.orientation.is_forward() {
            ".T."
        } else {
            ".F."
        };
        let adv_face_id = ctx.add_entity(&format!(
            "ADVANCED_FACE('',({}) ,#{},{})",
            bounds_str, surface_id, same_sense
        ));

        Some(adv_face_id)
    }

    fn compress_knots(raw_knots: &[f64]) -> (Vec<u32>, Vec<f64>) {
        let mut mults = Vec::new();
        let mut unique_knots = Vec::new();

        if raw_knots.is_empty() {
            return (mults, unique_knots);
        }

        let mut current_k = raw_knots[0];
        let mut count = 1;

        for &k in &raw_knots[1..] {
            if (k - current_k).abs() < 1e-7 {
                count += 1;
            } else {
                mults.push(count);
                unique_knots.push(current_k);
                current_k = k;
                count = 1;
            }
        }
        mults.push(count);
        unique_knots.push(current_k);

        (mults, unique_knots)
    }

    fn write_nurbs_surface(ctx: &mut StepContext, nurbs: &NurbsSurface3) -> u64 {
        let num_u = nurbs.control_points.len();
        let num_v = nurbs.control_points[0].len();

        let mut is_rational = false;
        let mut weights = Vec::with_capacity(num_u);
        let mut row_ids = Vec::with_capacity(num_u);

        for row in &nurbs.control_points {
            let mut pt_ids = Vec::with_capacity(num_v);
            let mut w_row = Vec::with_capacity(num_v);
            for cp in row {
                if (cp.weight - 1.0).abs() > 1e-6 {
                    is_rational = true;
                }
                w_row.push(format!("{:.6}", cp.weight));
                pt_ids.push(format!("#{}", Self::write_point(ctx, cp.point)));
            }
            weights.push(format!("({})", w_row.join(",")));
            row_ids.push(format!("({})", pt_ids.join(",")));
        }
        let grid_str = format!("({})", row_ids.join(","));
        let weights_str = format!("({})", weights.join(","));

        let (u_mults, u_knots) = Self::compress_knots(&nurbs.knots_u.knots);
        let (v_mults, v_knots) = Self::compress_knots(&nurbs.knots_v.knots);

        let u_mult_str = format!(
            "({})",
            u_mults
                .iter()
                .map(|m| m.to_string())
                .collect::<Vec<_>>()
                .join(",")
        );
        let v_mult_str = format!(
            "({})",
            v_mults
                .iter()
                .map(|m| m.to_string())
                .collect::<Vec<_>>()
                .join(",")
        );
        let u_knots_str = format!(
            "({})",
            u_knots
                .iter()
                .map(|k| format!("{:.6}", k))
                .collect::<Vec<_>>()
                .join(",")
        );
        let v_knots_str = format!(
            "({})",
            v_knots
                .iter()
                .map(|k| format!("{:.6}", k))
                .collect::<Vec<_>>()
                .join(",")
        );

        if is_rational {
            ctx.add_entity(&format!(
                "( BOUNDED_SURFACE() B_SPLINE_SURFACE({},{},{},.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS({},{},{},{},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE({}) REPRESENTATION_ITEM('') SURFACE() )",
                nurbs.degree_u, nurbs.degree_v, grid_str,
                u_mult_str, v_mult_str, u_knots_str, v_knots_str,
                weights_str
            ))
        } else {
            ctx.add_entity(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},{},.UNSPECIFIED.,.F.,.F.,.F.,{},{},{},{},.UNSPECIFIED.)",
                nurbs.degree_u, nurbs.degree_v, grid_str,
                u_mult_str, v_mult_str, u_knots_str, v_knots_str
            ))
        }
    }
}
