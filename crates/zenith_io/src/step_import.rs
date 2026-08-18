use std::collections::HashMap;
use std::fs;
use std::path::Path;
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{
    Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shape, Shell, Solid, Vertex, Wire,
};

/// STEP (ISO 10303-21) インポーター
pub struct StepImporter;

#[derive(Debug, Clone)]
struct RawEntity {
    name: String,
    args: String,
}

struct ImportContext {
    raw_entities: HashMap<u64, RawEntity>,
    points: HashMap<u64, Point3>,
    directions: HashMap<u64, Vec3>,
    vertices: HashMap<u64, Vertex>,
    edges: HashMap<u64, Edge>,
    curves: HashMap<u64, NurbsCurve3>,
    wires: HashMap<u64, Wire>,
    surfaces: HashMap<u64, FaceGeometry>,
    faces: HashMap<u64, Face>,
}

impl ImportContext {
    fn new() -> Self {
        Self {
            raw_entities: HashMap::new(),
            points: HashMap::new(),
            directions: HashMap::new(),
            vertices: HashMap::new(),
            edges: HashMap::new(),
            curves: HashMap::new(),
            wires: HashMap::new(),
            surfaces: HashMap::new(),
            faces: HashMap::new(),
        }
    }
}

impl StepImporter {
    /// STEPファイルから Solid（B-Repソリッド）をインポート
    pub fn import_solid_from_file<P: AsRef<Path>>(path: P) -> Result<Solid, String> {
        let content =
            fs::read_to_string(path).map_err(|e| format!("Failed to read STEP file: {}", e))?;
        Self::import_solid_from_str(&content)
    }

    /// STEPファイルから複数の Solid（B-Repソリッド）をインポート
    pub fn import_solids_from_file<P: AsRef<Path>>(path: P) -> Result<Vec<Solid>, String> {
        let content =
            fs::read_to_string(path).map_err(|e| format!("Failed to read STEP file: {}", e))?;
        Self::import_solids_from_str(&content)
    }

    /// STEPファイルから Shape（単一 Solid または Compound）をインポート
    pub fn import_shape_from_file<P: AsRef<Path>>(path: P) -> Result<Shape, String> {
        let solids = Self::import_solids_from_file(path)?;
        Ok(Shape::compound_solids(solids))
    }

    /// STEPテキストから Solid（B-Repソリッド）をインポート
    pub fn import_solid_from_str(content: &str) -> Result<Solid, String> {
        Self::import_solids_from_str(content)?
            .into_iter()
            .next()
            .ok_or("MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS not found in STEP data".to_string())
    }

    /// STEPテキストから複数の Solid（B-Repソリッド）をインポート
    pub fn import_solids_from_str(content: &str) -> Result<Vec<Solid>, String> {
        let mut ctx = ImportContext::new();

        // 1. DATAセクションのエンティティ辞書を構築
        Self::parse_data_section(content, &mut ctx)?;

        // 2. Solid B-Rep を探索
        let solid_ids = Self::solid_brep_ids(&ctx);
        if solid_ids.is_empty() {
            return Err(
                "MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS not found in STEP data".to_string(),
            );
        }

        solid_ids
            .into_iter()
            .map(|solid_id| Self::resolve_solid(&mut ctx, solid_id))
            .collect()
    }

    /// STEPテキストから Shape（単一 Solid または Compound）をインポート
    pub fn import_shape_from_str(content: &str) -> Result<Shape, String> {
        let solids = Self::import_solids_from_str(content)?;
        Ok(Shape::compound_solids(solids))
    }

    fn parse_data_section(content: &str, ctx: &mut ImportContext) -> Result<(), String> {
        let mut in_data = false;
        let mut buffer = String::new();

        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with("DATA;") {
                in_data = true;
                continue;
            }
            if trimmed.starts_with("ENDSEC;") && in_data {
                break;
            }
            if in_data {
                buffer.push_str(trimmed);
                buffer.push(' ');
            }
        }

        // ';' で分割して各エンティティをパース
        for statement in buffer.split(';') {
            let stmt = statement.trim();
            if stmt.starts_with('#') {
                if let Some(eq_idx) = stmt.find('=') {
                    let id_str = &stmt[1..eq_idx].trim();
                    if let Ok(id) = id_str.parse::<u64>() {
                        let rest = stmt[eq_idx + 1..].trim();
                        if let Some(paren_idx) = rest.find('(') {
                            let name = rest[..paren_idx].trim().to_uppercase();
                            let args = rest[paren_idx + 1..rest.len().saturating_sub(1)]
                                .trim()
                                .to_string();
                            ctx.raw_entities.insert(id, RawEntity { name, args });
                        }
                    }
                }
            }
        }

        Ok(())
    }

    fn parse_entity_ref(arg: &str) -> Option<u64> {
        let trimmed = arg.trim();
        if let Some(stripped) = trimmed.strip_prefix('#') {
            stripped.parse::<u64>().ok()
        } else {
            None
        }
    }

    fn split_top_level_args(args: &str) -> Vec<&str> {
        let mut parts = Vec::new();
        let mut start = 0;
        let mut depth = 0usize;
        let mut in_string = false;

        for (i, ch) in args.char_indices() {
            match ch {
                '\'' => in_string = !in_string,
                '(' if !in_string => depth += 1,
                ')' if !in_string => depth = depth.saturating_sub(1),
                ',' if !in_string && depth == 0 => {
                    parts.push(args[start..i].trim());
                    start = i + 1;
                }
                _ => {}
            }
        }

        parts.push(args[start..].trim());
        parts
    }

    fn parse_ref_list(arg: &str) -> Vec<u64> {
        let trimmed = arg.trim();
        let list = trimmed
            .strip_prefix('(')
            .and_then(|s| s.strip_suffix(')'))
            .unwrap_or(trimmed);

        Self::split_top_level_args(list)
            .into_iter()
            .filter_map(Self::parse_entity_ref)
            .collect()
    }

    fn solid_brep_ids(ctx: &ImportContext) -> Vec<u64> {
        let mut ids = Vec::new();
        let mut shape_reps: Vec<(u64, &RawEntity)> = ctx
            .raw_entities
            .iter()
            .filter_map(|(&id, ent)| {
                (ent.name == "ADVANCED_BREP_SHAPE_REPRESENTATION").then_some((id, ent))
            })
            .collect();
        shape_reps.sort_by_key(|(id, _)| *id);

        for (_, shape_rep) in shape_reps {
            let parts = Self::split_top_level_args(&shape_rep.args);
            if parts.len() < 2 {
                continue;
            }
            for item_id in Self::parse_ref_list(parts[1]) {
                if Self::is_solid_brep_entity(ctx, item_id) && !ids.contains(&item_id) {
                    ids.push(item_id);
                }
            }
        }

        if ids.is_empty() {
            ids = ctx
                .raw_entities
                .iter()
                .filter_map(|(&id, _)| Self::is_solid_brep_entity(ctx, id).then_some(id))
                .collect();
            ids.sort_unstable();
        }

        ids
    }

    fn is_solid_brep_entity(ctx: &ImportContext, id: u64) -> bool {
        ctx.raw_entities
            .get(&id)
            .map(|ent| ent.name == "MANIFOLD_SOLID_BREP" || ent.name == "BREP_WITH_VOIDS")
            .unwrap_or(false)
    }

    fn get_point(ctx: &mut ImportContext, id: u64) -> Result<Point3, String> {
        if let Some(&p) = ctx.points.get(&id) {
            return Ok(p);
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();
        if raw.name == "CARTESIAN_POINT" {
            // CARTESIAN_POINT('',(x,y,z))
            if let Some(start) = raw.args.find('(') {
                if let Some(end) = raw.args.rfind(')') {
                    let coords_str = &raw.args[start + 1..end];
                    let parts: Vec<&str> = coords_str.split(',').collect();
                    if parts.len() >= 3 {
                        let x = parts[0].trim().parse::<f64>().map_err(|e| e.to_string())?;
                        let y = parts[1].trim().parse::<f64>().map_err(|e| e.to_string())?;
                        let z = parts[2].trim().parse::<f64>().map_err(|e| e.to_string())?;
                        let p = Point3::new(x, y, z);
                        ctx.points.insert(id, p);
                        return Ok(p);
                    }
                }
            }
        }
        Err(format!("Invalid CARTESIAN_POINT #{}", id))
    }

    fn get_direction(ctx: &mut ImportContext, id: u64) -> Result<Vec3, String> {
        if let Some(&d) = ctx.directions.get(&id) {
            return Ok(d);
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();
        if raw.name == "DIRECTION" {
            if let Some(start) = raw.args.find('(') {
                if let Some(end) = raw.args.rfind(')') {
                    let coords_str = &raw.args[start + 1..end];
                    let parts: Vec<&str> = coords_str.split(',').collect();
                    if parts.len() >= 3 {
                        let x = parts[0].trim().parse::<f64>().map_err(|e| e.to_string())?;
                        let y = parts[1].trim().parse::<f64>().map_err(|e| e.to_string())?;
                        let z = parts[2].trim().parse::<f64>().map_err(|e| e.to_string())?;
                        let d = Vec3::new(x, y, z);
                        ctx.directions.insert(id, d);
                        return Ok(d);
                    }
                }
            }
        }
        Err(format!("Invalid DIRECTION #{}", id))
    }

    fn get_vertex(ctx: &mut ImportContext, id: u64) -> Result<Vertex, String> {
        if let Some(v) = ctx.vertices.get(&id) {
            return Ok(v.clone());
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();
        if raw.name == "VERTEX_POINT" {
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 2 {
                let pt_id = Self::parse_entity_ref(parts[1]).ok_or("Invalid vertex point ref")?;
                let pt = Self::get_point(ctx, pt_id)?;
                let v = Vertex::from_point(pt);
                ctx.vertices.insert(id, v.clone());
                return Ok(v);
            }
        }
        Err(format!("Invalid VERTEX_POINT #{}", id))
    }

    fn get_edge(ctx: &mut ImportContext, id: u64) -> Result<Edge, String> {
        if let Some(e) = ctx.edges.get(&id) {
            return Ok(e.clone());
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();
        if raw.name == "EDGE_CURVE" {
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 4 {
                let v_start_id = Self::parse_entity_ref(parts[1]).ok_or("Invalid v_start ref")?;
                let v_end_id = Self::parse_entity_ref(parts[2]).ok_or("Invalid v_end ref")?;
                let curve_id = Self::parse_entity_ref(parts[3]).ok_or("Invalid curve ref")?;
                let same_sense = parts
                    .get(4)
                    .map(|sense| sense.trim() == ".T.")
                    .unwrap_or(true);

                let v_start = Self::get_vertex(ctx, v_start_id)?;
                let v_end = Self::get_vertex(ctx, v_end_id)?;
                let mut curve = Self::get_curve(ctx, curve_id, v_start.point, v_end.point)?;
                if !same_sense {
                    curve = curve.reversed();
                }

                let edge = Edge::new(curve, v_start, v_end, 1e-6);
                ctx.edges.insert(id, edge.clone());
                return Ok(edge);
            }
        }
        Err(format!("Invalid EDGE_CURVE #{}", id))
    }

    fn get_curve(
        ctx: &mut ImportContext,
        id: u64,
        p_start: Point3,
        p_end: Point3,
    ) -> Result<NurbsCurve3, String> {
        if let Some(c) = ctx.curves.get(&id) {
            return Ok(c.clone());
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();

        if raw.name == "LINE" {
            let c = NurbsCurve3::bspline_from_points(1, vec![p_start, p_end])?;
            ctx.curves.insert(id, c.clone());
            return Ok(c);
        }

        if raw.name == "TRIMMED_CURVE" {
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 4 {
                if let Some(base_curve_id) = Self::parse_entity_ref(parts[1]) {
                    let same_sense = parts
                        .get(4)
                        .map(|sense| sense.trim() == ".T.")
                        .unwrap_or(true);
                    if let Ok(Some((trim_start, trim_end))) =
                        Self::trimmed_circle_points(ctx, base_curve_id, parts[2], parts[3])
                    {
                        let (trim_start, trim_end) = if same_sense {
                            (trim_start, trim_end)
                        } else {
                            (trim_end, trim_start)
                        };
                        if let Ok(Some(c)) =
                            Self::arc_from_circle_curve(ctx, base_curve_id, trim_start, trim_end)
                        {
                            ctx.curves.insert(id, c.clone());
                            return Ok(c);
                        }
                    }
                    let (fallback_start, fallback_end) = if same_sense {
                        (p_start, p_end)
                    } else {
                        (p_end, p_start)
                    };
                    if let Ok(Some(c)) = Self::arc_from_circle_curve(
                        ctx,
                        base_curve_id,
                        fallback_start,
                        fallback_end,
                    ) {
                        ctx.curves.insert(id, c.clone());
                        return Ok(c);
                    }
                }
            }
        }

        if raw.name == "CIRCLE" {
            if let Some(c) = Self::arc_from_circle_entity(ctx, &raw, p_start, p_end)? {
                ctx.curves.insert(id, c.clone());
                return Ok(c);
            }
        }

        if raw.name == "SURFACE_CURVE" {
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 2 {
                if let Some(curve_3d_id) = Self::parse_entity_ref(parts[1]) {
                    let c = Self::get_curve(ctx, curve_3d_id, p_start, p_end)?;
                    ctx.curves.insert(id, c.clone());
                    return Ok(c);
                }
            }
        }

        if raw.name == "B_SPLINE_CURVE_WITH_KNOTS" || raw.args.contains("B_SPLINE_CURVE_WITH_KNOTS")
        {
            if let Some(c) = Self::parse_nurbs_curve(ctx, &raw)? {
                ctx.curves.insert(id, c.clone());
                return Ok(c);
            }
        }

        // デフォルト直線補間
        let c = NurbsCurve3::bspline_from_points(1, vec![p_start, p_end])?;
        Ok(c)
    }

    fn parse_nurbs_curve(
        ctx: &mut ImportContext,
        raw: &RawEntity,
    ) -> Result<Option<NurbsCurve3>, String> {
        let (degree, point_refs_arg, mult_arg, knot_arg) =
            if raw.name == "B_SPLINE_CURVE_WITH_KNOTS" {
                let parts = Self::split_top_level_args(&raw.args);
                if parts.len() < 8 {
                    return Ok(None);
                }
                (
                    parts[1].parse::<usize>().map_err(|e| e.to_string())?,
                    parts[2],
                    parts[6],
                    parts[7],
                )
            } else {
                let Some(curve_args) = extract_entity_args(&raw.args, "B_SPLINE_CURVE") else {
                    return Ok(None);
                };
                let Some(knot_args) = extract_entity_args(&raw.args, "B_SPLINE_CURVE_WITH_KNOTS")
                else {
                    return Ok(None);
                };
                let curve_parts = Self::split_top_level_args(curve_args);
                let knot_parts = Self::split_top_level_args(knot_args);
                if curve_parts.len() < 2 || knot_parts.len() < 2 {
                    return Ok(None);
                }
                (
                    curve_parts[0].parse::<usize>().map_err(|e| e.to_string())?,
                    curve_parts[1],
                    knot_parts[0],
                    knot_parts[1],
                )
            };

        let point_ids = Self::parse_ref_list(point_refs_arg);
        if point_ids.is_empty() {
            return Ok(None);
        }

        let mut control_points = Vec::with_capacity(point_ids.len());
        for point_id in point_ids {
            control_points.push(ControlPoint3::unweighted(Self::get_point(ctx, point_id)?));
        }

        if let Some(weight_args) = extract_entity_args(&raw.args, "RATIONAL_B_SPLINE_CURVE") {
            let weights = parse_f64_list(weight_args)?;
            if weights.len() == control_points.len() {
                for (cp, weight) in control_points.iter_mut().zip(weights) {
                    cp.weight = weight;
                }
            }
        }

        let knots = expand_knot_vector(mult_arg, knot_arg)?;
        Ok(Some(NurbsCurve3::new(degree, control_points, knots)?))
    }

    fn trimmed_circle_points(
        ctx: &mut ImportContext,
        circle_id: u64,
        trim_1: &str,
        trim_2: &str,
    ) -> Result<Option<(Point3, Point3)>, String> {
        let raw = ctx
            .raw_entities
            .get(&circle_id)
            .ok_or_else(|| format!("Circle entity #{} not found", circle_id))?
            .clone();
        if raw.name != "CIRCLE" {
            return Ok(None);
        }

        let circle_parts = Self::split_top_level_args(&raw.args);
        if circle_parts.len() < 3 {
            return Ok(None);
        }

        let axis_id = match Self::parse_entity_ref(circle_parts[1]) {
            Some(id) => id,
            None => return Ok(None),
        };
        let radius = match circle_parts[2].parse::<f64>() {
            Ok(r) if r > 1e-9 => r,
            _ => return Ok(None),
        };
        let (center, normal, ref_dir) = Self::get_axis2_placement(ctx, axis_id)?;

        let start = Self::trim_point_or_parameter(ctx, trim_1, center, normal, ref_dir, radius)?;
        let end = Self::trim_point_or_parameter(ctx, trim_2, center, normal, ref_dir, radius)?;

        Ok(start.zip(end))
    }

    fn trim_point_or_parameter(
        ctx: &mut ImportContext,
        trim_arg: &str,
        center: Point3,
        normal: Vec3,
        ref_dir: Vec3,
        radius: f64,
    ) -> Result<Option<Point3>, String> {
        if let Some(point_id) = Self::first_entity_ref(trim_arg) {
            if let Ok(point) = Self::get_point(ctx, point_id) {
                return Ok(Some(point));
            }
        }

        if let Some(parameter) = extract_parameter_value(trim_arg) {
            let x_axis = ref_dir.normalize();
            let y_axis = normal.cross(&x_axis).normalize();
            return Ok(Some(
                center + x_axis * (radius * parameter.cos()) + y_axis * (radius * parameter.sin()),
            ));
        }

        Ok(None)
    }

    fn first_entity_ref(arg: &str) -> Option<u64> {
        let bytes = arg.as_bytes();
        let mut i = 0;
        while i < bytes.len() {
            if bytes[i] == b'#' {
                let start = i + 1;
                let mut end = start;
                while end < bytes.len() && bytes[end].is_ascii_digit() {
                    end += 1;
                }
                if end > start {
                    return arg[start..end].parse::<u64>().ok();
                }
            }
            i += 1;
        }

        None
    }

    fn arc_from_circle_curve(
        ctx: &mut ImportContext,
        circle_id: u64,
        p_start: Point3,
        p_end: Point3,
    ) -> Result<Option<NurbsCurve3>, String> {
        let raw = ctx
            .raw_entities
            .get(&circle_id)
            .ok_or_else(|| format!("Circle entity #{} not found", circle_id))?
            .clone();
        if raw.name != "CIRCLE" {
            return Ok(None);
        }

        Self::arc_from_circle_entity(ctx, &raw, p_start, p_end)
    }

    fn arc_from_circle_entity(
        ctx: &mut ImportContext,
        raw: &RawEntity,
        p_start: Point3,
        p_end: Point3,
    ) -> Result<Option<NurbsCurve3>, String> {
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 3 {
            return Ok(None);
        }

        let axis_id = match Self::parse_entity_ref(parts[1]) {
            Some(id) => id,
            None => return Ok(None),
        };
        let radius = match parts[2].parse::<f64>() {
            Ok(r) if r > 1e-9 => r,
            _ => return Ok(None),
        };
        let (center, normal, _) = Self::get_axis2_placement(ctx, axis_id)?;

        let v0 = p_start - center;
        let v1 = p_end - center;
        if (v0.norm() - radius).abs() > 1e-3 || (v1.norm() - radius).abs() > 1e-3 {
            return Ok(None);
        }

        Self::make_circular_arc(center, normal, p_start, p_end)
    }

    fn make_circular_arc(
        center: Point3,
        normal: Vec3,
        p_start: Point3,
        p_end: Point3,
    ) -> Result<Option<NurbsCurve3>, String> {
        let v0 = p_start - center;
        let v1 = p_end - center;
        let r0 = v0.norm();
        let r1 = v1.norm();
        if r0 <= 1e-9 || r1 <= 1e-9 || (r0 - r1).abs() > 1e-3 {
            return Ok(None);
        }

        let u0 = v0 / r0;
        let u1 = v1 / r1;
        let dot = u0.dot(&u1).clamp(-1.0, 1.0);
        let angle = dot.acos();
        if angle <= 1e-9 || angle > std::f64::consts::FRAC_PI_2 + 1e-6 {
            return Ok(None);
        }

        let tangent0 = normal.cross(&u0);
        let tangent1 = normal.cross(&u1);
        let control = line_intersection_closest(p_start, tangent0, p_end, tangent1)
            .unwrap_or_else(|| center + (v0 + v1));
        let weight = (angle * 0.5).cos();

        let curve = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(p_start),
                ControlPoint3::new(control, weight),
                ControlPoint3::unweighted(p_end),
            ],
            KnotVector::clamped_uniform(3, 2),
        )?;
        Ok(Some(curve))
    }

    fn get_axis2_placement(
        ctx: &mut ImportContext,
        axis_id: u64,
    ) -> Result<(Point3, Vec3, Vec3), String> {
        let raw_axis = ctx
            .raw_entities
            .get(&axis_id)
            .ok_or("AXIS2 not found")?
            .clone();
        if raw_axis.name != "AXIS2_PLACEMENT_3D" {
            return Err(format!("Invalid AXIS2_PLACEMENT_3D #{}", axis_id));
        }

        let axis_parts = Self::split_top_level_args(&raw_axis.args);
        if axis_parts.len() < 4 {
            return Err(format!("Invalid AXIS2_PLACEMENT_3D #{}", axis_id));
        }

        let orig_id = Self::parse_entity_ref(axis_parts[1]).ok_or("orig ref")?;
        let z_id = Self::parse_entity_ref(axis_parts[2]).ok_or("z ref")?;
        let x_id = Self::parse_entity_ref(axis_parts[3]).ok_or("x ref")?;

        let origin = Self::get_point(ctx, orig_id)?;
        let z_dir = Self::get_direction(ctx, z_id)?.normalize();
        let x_dir = Self::get_direction(ctx, x_id)?.normalize();
        Ok((origin, z_dir, x_dir))
    }

    fn get_wire(ctx: &mut ImportContext, loop_id: u64) -> Result<Wire, String> {
        if let Some(w) = ctx.wires.get(&loop_id) {
            return Ok(w.clone());
        }
        let raw = ctx
            .raw_entities
            .get(&loop_id)
            .ok_or_else(|| format!("Entity #{} not found", loop_id))?
            .clone();
        if raw.name == "EDGE_LOOP" {
            // EDGE_LOOP('',(#1,#2,#3...))
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 2 {
                let mut oriented_edges = Vec::new();
                for oe_id in Self::parse_ref_list(parts[1]) {
                    let oe = Self::get_oriented_edge(ctx, oe_id)?;
                    oriented_edges.push(oe);
                }
                let wire = Wire::new(oriented_edges);
                ctx.wires.insert(loop_id, wire.clone());
                return Ok(wire);
            }
        }
        Err(format!("Invalid EDGE_LOOP #{}", loop_id))
    }

    fn get_oriented_edge(ctx: &mut ImportContext, id: u64) -> Result<OrientedEdge, String> {
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();
        if raw.name == "ORIENTED_EDGE" {
            // ORIENTED_EDGE('',*,*,#edge_curve,.T.)
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 5 {
                let edge_id =
                    Self::parse_entity_ref(parts[3]).ok_or("Invalid edge ref in ORIENTED_EDGE")?;
                let edge = Self::get_edge(ctx, edge_id)?;
                let same_sense = parts[4].trim() == ".T.";
                if same_sense {
                    return Ok(OrientedEdge::forward(edge));
                } else {
                    return Ok(OrientedEdge::reversed(edge));
                }
            }
        }
        Err(format!("Invalid ORIENTED_EDGE #{}", id))
    }

    fn get_surface(ctx: &mut ImportContext, id: u64) -> Result<FaceGeometry, String> {
        if let Some(s) = ctx.surfaces.get(&id) {
            return Ok(s.clone());
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();

        if raw.name == "PLANE" {
            // PLANE('',#axis2)
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 2 {
                let axis2_id =
                    Self::parse_entity_ref(parts[1]).ok_or("Invalid axis2 ref in PLANE")?;
                let raw_axis = ctx
                    .raw_entities
                    .get(&axis2_id)
                    .ok_or("AXIS2 not found")?
                    .clone();
                // AXIS2_PLACEMENT_3D('',#origin,#z_dir,#x_dir)
                let axis_parts = Self::split_top_level_args(&raw_axis.args);
                if axis_parts.len() >= 4 {
                    let orig_id = Self::parse_entity_ref(axis_parts[1]).ok_or("orig ref")?;
                    let z_id = Self::parse_entity_ref(axis_parts[2]).ok_or("z ref")?;
                    let x_id = Self::parse_entity_ref(axis_parts[3]).ok_or("x ref")?;

                    let origin = Self::get_point(ctx, orig_id)?;
                    let z_dir = Self::get_direction(ctx, z_id)?;
                    let x_dir = Self::get_direction(ctx, x_id)?;
                    let y_dir = z_dir.cross(&x_dir).normalize();

                    let plane =
                        PlaneSurface3::new(origin, x_dir, y_dir).ok_or("Plane creation failed")?;
                    let geom = FaceGeometry::Plane(plane);
                    ctx.surfaces.insert(id, geom.clone());
                    return Ok(geom);
                }
            }
        }

        if raw.name == "B_SPLINE_SURFACE_WITH_KNOTS"
            || raw.args.contains("B_SPLINE_SURFACE_WITH_KNOTS")
        {
            if let Some(nurbs) = Self::parse_nurbs_surface(ctx, &raw)? {
                let geom = FaceGeometry::Nurbs(nurbs);
                ctx.surfaces.insert(id, geom.clone());
                return Ok(geom);
            }
        }

        // NURBS曲面等のフォールバック
        let default_plane = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        Ok(FaceGeometry::Plane(default_plane))
    }

    fn parse_nurbs_surface(
        ctx: &mut ImportContext,
        raw: &RawEntity,
    ) -> Result<Option<NurbsSurface3>, String> {
        let (degree_u, degree_v, grid_arg, u_mult_arg, v_mult_arg, u_knot_arg, v_knot_arg) =
            if raw.name == "B_SPLINE_SURFACE_WITH_KNOTS" {
                let parts = Self::split_top_level_args(&raw.args);
                if parts.len() < 12 {
                    return Ok(None);
                }
                (
                    parts[1].parse::<usize>().map_err(|e| e.to_string())?,
                    parts[2].parse::<usize>().map_err(|e| e.to_string())?,
                    parts[3],
                    parts[8],
                    parts[9],
                    parts[10],
                    parts[11],
                )
            } else {
                let Some(surface_args) = extract_entity_args(&raw.args, "B_SPLINE_SURFACE") else {
                    return Ok(None);
                };
                let Some(knot_args) = extract_entity_args(&raw.args, "B_SPLINE_SURFACE_WITH_KNOTS")
                else {
                    return Ok(None);
                };
                let surface_parts = Self::split_top_level_args(surface_args);
                let knot_parts = Self::split_top_level_args(knot_args);
                if surface_parts.len() < 3 || knot_parts.len() < 4 {
                    return Ok(None);
                }
                (
                    surface_parts[0]
                        .parse::<usize>()
                        .map_err(|e| e.to_string())?,
                    surface_parts[1]
                        .parse::<usize>()
                        .map_err(|e| e.to_string())?,
                    surface_parts[2],
                    knot_parts[0],
                    knot_parts[1],
                    knot_parts[2],
                    knot_parts[3],
                )
            };

        let mut control_points = Self::parse_control_point_grid(ctx, grid_arg)?;
        if let Some(weight_args) = extract_entity_args(&raw.args, "RATIONAL_B_SPLINE_SURFACE") {
            let weights = parse_nested_f64_grid(weight_args)?;
            if weights.len() == control_points.len()
                && weights
                    .iter()
                    .zip(control_points.iter())
                    .all(|(w_row, cp_row)| w_row.len() == cp_row.len())
            {
                for (cp_row, w_row) in control_points.iter_mut().zip(weights) {
                    for (cp, weight) in cp_row.iter_mut().zip(w_row) {
                        cp.weight = weight;
                    }
                }
            }
        }

        let knots_u = expand_knot_vector(u_mult_arg, u_knot_arg)?;
        let knots_v = expand_knot_vector(v_mult_arg, v_knot_arg)?;
        let surface = NurbsSurface3::new(degree_u, degree_v, control_points, knots_u, knots_v)?;
        Ok(Some(surface))
    }

    fn parse_control_point_grid(
        ctx: &mut ImportContext,
        grid_arg: &str,
    ) -> Result<Vec<Vec<ControlPoint3>>, String> {
        parse_nested_list(grid_arg)
            .into_iter()
            .map(|row| {
                StepImporter::split_top_level_args(row)
                    .into_iter()
                    .map(|point_ref| {
                        let point_id = StepImporter::parse_entity_ref(point_ref)
                            .ok_or_else(|| format!("Invalid control point ref: {}", point_ref))?;
                        StepImporter::get_point(ctx, point_id).map(ControlPoint3::unweighted)
                    })
                    .collect()
            })
            .collect()
    }

    fn resolve_solid(ctx: &mut ImportContext, solid_id: u64) -> Result<Solid, String> {
        let raw = ctx
            .raw_entities
            .get(&solid_id)
            .ok_or("Solid entity not found")?
            .clone();

        match raw.name.as_str() {
            "MANIFOLD_SOLID_BREP" => Self::resolve_manifold_solid_brep(ctx, &raw),
            "BREP_WITH_VOIDS" => Self::resolve_brep_with_voids(ctx, &raw),
            _ => Err(format!("Unsupported solid entity {}", raw.name)),
        }
    }

    fn resolve_manifold_solid_brep(
        ctx: &mut ImportContext,
        raw: &RawEntity,
    ) -> Result<Solid, String> {
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 2 {
            return Err("Invalid MANIFOLD_SOLID_BREP format".to_string());
        }

        let shell_id = Self::parse_entity_ref(parts[1]).ok_or("Invalid shell ref")?;
        let outer_shell = Self::resolve_closed_shell(ctx, shell_id)?;
        Solid::try_simple(outer_shell, &Tolerance::default()).map_err(|err| err.to_string())
    }

    fn resolve_brep_with_voids(ctx: &mut ImportContext, raw: &RawEntity) -> Result<Solid, String> {
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 3 {
            return Err("Invalid BREP_WITH_VOIDS format".to_string());
        }

        let outer_shell_id = Self::parse_entity_ref(parts[1]).ok_or("Invalid outer shell ref")?;
        let outer_shell = Self::resolve_closed_shell(ctx, outer_shell_id)?;
        let mut inner_shells = Vec::new();
        for oriented_shell_id in Self::parse_ref_list(parts[2]) {
            inner_shells.push(Self::resolve_oriented_closed_shell(ctx, oriented_shell_id)?);
        }

        Solid::try_new(outer_shell, inner_shells, &Tolerance::default())
            .map_err(|err| err.to_string())
    }

    fn resolve_oriented_closed_shell(
        ctx: &mut ImportContext,
        oriented_shell_id: u64,
    ) -> Result<Shell, String> {
        let raw = ctx
            .raw_entities
            .get(&oriented_shell_id)
            .ok_or("Oriented closed shell not found")?
            .clone();
        if raw.name != "ORIENTED_CLOSED_SHELL" {
            return Err(format!(
                "Expected ORIENTED_CLOSED_SHELL but found {}",
                raw.name
            ));
        }

        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 4 {
            return Err("Invalid ORIENTED_CLOSED_SHELL format".to_string());
        }
        let shell_id = Self::parse_entity_ref(parts[2]).ok_or("Invalid oriented shell ref")?;
        Self::resolve_closed_shell(ctx, shell_id)
    }

    fn resolve_closed_shell(ctx: &mut ImportContext, shell_id: u64) -> Result<Shell, String> {
        let shell_raw = ctx
            .raw_entities
            .get(&shell_id)
            .ok_or("Shell not found")?
            .clone();

        // CLOSED_SHELL('',(#face1,#face2...))
        let mut faces = Vec::new();
        let shell_parts = Self::split_top_level_args(&shell_raw.args);
        if shell_parts.len() >= 2 {
            for face_id in Self::parse_ref_list(shell_parts[1]) {
                let face = Self::resolve_face(ctx, face_id)?;
                faces.push(face);
            }
        }

        Ok(Shell::closed(faces))
    }

    fn resolve_face(ctx: &mut ImportContext, face_id: u64) -> Result<Face, String> {
        if let Some(f) = ctx.faces.get(&face_id) {
            return Ok(f.clone());
        }

        let raw = ctx
            .raw_entities
            .get(&face_id)
            .ok_or("Face not found")?
            .clone();
        if raw.name == "ADVANCED_FACE" {
            // ADVANCED_FACE('',(#bound1,#bound2...),#surface_id,.T.)
            let mut outer_wire = None;
            let mut inner_wires = Vec::new();

            // 1. サーフェス参照の取得
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() < 4 {
                return Err(format!("Invalid ADVANCED_FACE #{}", face_id));
            }
            let same_sense = parts[3].trim() == ".T.";

            let surface_id =
                Self::parse_entity_ref(parts[2]).ok_or("Invalid surface ref in ADVANCED_FACE")?;
            let geom = Self::get_surface(ctx, surface_id)?;

            // 2. Bounds の取得
            for b_id in Self::parse_ref_list(parts[1]) {
                let bound_raw = ctx
                    .raw_entities
                    .get(&b_id)
                    .ok_or("Bound not found")?
                    .clone();
                let b_parts = Self::split_top_level_args(&bound_raw.args);
                if b_parts.len() >= 2 {
                    let loop_id = Self::parse_entity_ref(b_parts[1]).ok_or("loop ref")?;
                    let wire = Self::get_wire(ctx, loop_id)?;
                    if bound_raw.name == "FACE_OUTER_BOUND" {
                        outer_wire = Some(wire);
                    } else {
                        inner_wires.push(wire);
                    }
                }
            }

            let outer =
                outer_wire.ok_or_else(|| format!("No outer bound for face #{}", face_id))?;
            let orientation = if same_sense {
                Orientation::Forward
            } else {
                Orientation::Reversed
            };
            let face = Face::new(geom, outer, inner_wires, orientation, 1e-6);
            ctx.faces.insert(face_id, face.clone());
            return Ok(face);
        }

        Err(format!("Invalid ADVANCED_FACE #{}", face_id))
    }
}

fn line_intersection_closest(p0: Point3, d0: Vec3, p1: Point3, d1: Vec3) -> Option<Point3> {
    let a = d0.dot(&d0);
    let b = d0.dot(&d1);
    let c = d1.dot(&d1);
    let r = p0 - p1;
    let d = d0.dot(&r);
    let e = d1.dot(&r);
    let denom = a * c - b * b;

    if denom.abs() <= 1e-12 {
        return None;
    }

    let s = (b * e - c * d) / denom;
    let t = (a * e - b * d) / denom;
    let q0 = p0 + d0 * s;
    let q1 = p1 + d1 * t;

    if (q0 - q1).norm() > 1e-4 {
        return None;
    }

    Some(q0 + (q1 - q0) * 0.5)
}

fn extract_entity_args<'a>(text: &'a str, entity_name: &str) -> Option<&'a str> {
    let mut search_start = 0;
    let open = loop {
        let found = text[search_start..].find(entity_name)? + search_start;
        let before = text[..found].chars().next_back();
        let after_name = found + entity_name.len();
        let after = text[after_name..].chars().next();

        let has_name_boundary_before = before.map_or(true, |ch| !is_step_identifier_char(ch));
        let has_name_boundary_after = after.map_or(true, |ch| !is_step_identifier_char(ch));
        if has_name_boundary_before && has_name_boundary_after {
            let open = text[after_name..].find('(')? + after_name;
            break open;
        }

        search_start = after_name;
    };

    let mut depth = 0usize;
    let mut in_string = false;

    for (i, ch) in text[open..].char_indices() {
        match ch {
            '\'' => in_string = !in_string,
            '(' if !in_string => depth += 1,
            ')' if !in_string => {
                depth = depth.saturating_sub(1);
                if depth == 0 {
                    return Some(&text[open + 1..open + i]);
                }
            }
            _ => {}
        }
    }

    None
}

fn extract_parameter_value(text: &str) -> Option<f64> {
    let args = extract_entity_args(text, "PARAMETER_VALUE")?;
    StepImporter::split_top_level_args(args)
        .first()
        .and_then(|value| value.parse::<f64>().ok())
}

fn is_step_identifier_char(ch: char) -> bool {
    ch.is_ascii_alphanumeric() || ch == '_'
}

fn parse_nested_list(arg: &str) -> Vec<&str> {
    let trimmed = arg.trim();
    let list = trimmed
        .strip_prefix('(')
        .and_then(|s| s.strip_suffix(')'))
        .unwrap_or(trimmed);

    StepImporter::split_top_level_args(list)
        .into_iter()
        .map(|part| {
            part.strip_prefix('(')
                .and_then(|s| s.strip_suffix(')'))
                .unwrap_or(part)
        })
        .collect()
}

fn parse_f64_list(arg: &str) -> Result<Vec<f64>, String> {
    let trimmed = arg.trim();
    let list = trimmed
        .strip_prefix('(')
        .and_then(|s| s.strip_suffix(')'))
        .unwrap_or(trimmed);

    StepImporter::split_top_level_args(list)
        .into_iter()
        .map(|value| value.parse::<f64>().map_err(|e| e.to_string()))
        .collect()
}

fn parse_usize_list(arg: &str) -> Result<Vec<usize>, String> {
    let trimmed = arg.trim();
    let list = trimmed
        .strip_prefix('(')
        .and_then(|s| s.strip_suffix(')'))
        .unwrap_or(trimmed);

    StepImporter::split_top_level_args(list)
        .into_iter()
        .map(|value| value.parse::<usize>().map_err(|e| e.to_string()))
        .collect()
}

fn parse_nested_f64_grid(arg: &str) -> Result<Vec<Vec<f64>>, String> {
    parse_nested_list(arg)
        .into_iter()
        .map(parse_f64_list)
        .collect()
}

fn expand_knot_vector(mult_arg: &str, knot_arg: &str) -> Result<KnotVector, String> {
    let multiplicities = parse_usize_list(mult_arg)?;
    let knots = parse_f64_list(knot_arg)?;
    if multiplicities.len() != knots.len() {
        return Err("Knot multiplicities and knot values have different lengths".to_string());
    }

    let mut expanded = Vec::new();
    for (multiplicity, knot) in multiplicities.into_iter().zip(knots) {
        expanded.extend(std::iter::repeat_n(knot, multiplicity));
    }

    Ok(KnotVector::new(expanded))
}

#[cfg(test)]
mod tests {
    use super::{ImportContext, RawEntity, StepImporter};
    use zenith_math::{Point3, Vec3};
    use zenith_topo::FaceGeometry;

    fn point_entity(x: f64, y: f64, z: f64) -> RawEntity {
        RawEntity {
            name: "CARTESIAN_POINT".to_string(),
            args: format!("'',({},{},{})", x, y, z),
        }
    }

    #[test]
    fn split_top_level_args_keeps_nested_step_lists() {
        let args = "'',(#10,#11,#12),#20,.T.";
        let parts = StepImporter::split_top_level_args(args);

        assert_eq!(parts, vec!["''", "(#10,#11,#12)", "#20", ".T."]);
        assert_eq!(StepImporter::parse_ref_list(parts[1]), vec![10, 11, 12]);
    }

    #[test]
    fn split_top_level_args_ignores_commas_inside_strings_and_points() {
        let args = "'name, with comma',(1.0,2.0,3.0),#7";
        let parts = StepImporter::split_top_level_args(args);

        assert_eq!(parts, vec!["'name, with comma'", "(1.0,2.0,3.0)", "#7"]);
    }

    #[test]
    fn extract_entity_args_uses_exact_step_entity_names() {
        let text = "B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.0,1.0),.UNSPECIFIED.) B_SPLINE_CURVE(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.)";

        assert_eq!(
            super::extract_entity_args(text, "B_SPLINE_CURVE_WITH_KNOTS"),
            Some("(3,3),(0.0,1.0),.UNSPECIFIED.")
        );
        assert_eq!(
            super::extract_entity_args(text, "B_SPLINE_CURVE"),
            Some("2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.")
        );
    }

    #[test]
    fn solid_brep_ids_preserve_shape_representation_order() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(
            10,
            RawEntity {
                name: "MANIFOLD_SOLID_BREP".to_string(),
                args: "'SECOND',#2".to_string(),
            },
        );
        ctx.raw_entities.insert(
            20,
            RawEntity {
                name: "BREP_WITH_VOIDS".to_string(),
                args: "'FIRST',#3,(#4)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            30,
            RawEntity {
                name: "AXIS2_PLACEMENT_3D".to_string(),
                args: "'',#1,#2,#3".to_string(),
            },
        );
        ctx.raw_entities.insert(
            40,
            RawEntity {
                name: "ADVANCED_BREP_SHAPE_REPRESENTATION".to_string(),
                args: "'BODY',(#20,#30,#10),#50".to_string(),
            },
        );

        assert_eq!(StepImporter::solid_brep_ids(&ctx), vec![20, 10]);
    }

    #[test]
    fn imports_trimmed_circle_as_rational_quarter_arc() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(
            1,
            RawEntity {
                name: "CARTESIAN_POINT".to_string(),
                args: "'',(0.0,0.0,0.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            2,
            RawEntity {
                name: "DIRECTION".to_string(),
                args: "'',(0.0,0.0,1.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            3,
            RawEntity {
                name: "DIRECTION".to_string(),
                args: "'',(1.0,0.0,0.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            4,
            RawEntity {
                name: "AXIS2_PLACEMENT_3D".to_string(),
                args: "'',#1,#2,#3".to_string(),
            },
        );
        ctx.raw_entities.insert(
            5,
            RawEntity {
                name: "CIRCLE".to_string(),
                args: "'',#4,10.0".to_string(),
            },
        );
        ctx.raw_entities.insert(
            6,
            RawEntity {
                name: "TRIMMED_CURVE".to_string(),
                args: "'',#5,(#7,PARAMETER_VALUE(0.0)),(#8,PARAMETER_VALUE(1.570796326795)),.T.,.PARAMETER.".to_string(),
            },
        );

        let curve = StepImporter::get_curve(
            &mut ctx,
            6,
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        )
        .expect("trimmed circle import should succeed");

        assert_eq!(curve.degree, 2);
        assert!((curve.control_points[1].weight - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6);
        assert!((curve.evaluate(0.5) - Point3::new(7.0710678119, 7.0710678119, 0.0)).norm() < 1e-6);
        assert!((Vec3::new(10.0, 10.0, 0.0) - curve.control_points[1].point.coords).norm() < 1e-6);
    }

    #[test]
    fn imports_trimmed_circle_from_parameter_values_without_point_refs() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(
            2,
            RawEntity {
                name: "DIRECTION".to_string(),
                args: "'',(0.0,0.0,1.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            3,
            RawEntity {
                name: "DIRECTION".to_string(),
                args: "'',(1.0,0.0,0.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            4,
            RawEntity {
                name: "AXIS2_PLACEMENT_3D".to_string(),
                args: "'',#1,#2,#3".to_string(),
            },
        );
        ctx.raw_entities.insert(
            5,
            RawEntity {
                name: "CIRCLE".to_string(),
                args: "'',#4,10.0".to_string(),
            },
        );
        ctx.raw_entities.insert(
            6,
            RawEntity {
                name: "TRIMMED_CURVE".to_string(),
                args:
                    "'',#5,(PARAMETER_VALUE(0.0)),(PARAMETER_VALUE(1.570796326795)),.T.,.PARAMETER."
                        .to_string(),
            },
        );

        let curve = StepImporter::get_curve(
            &mut ctx,
            6,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        )
        .expect("parameter-only trimmed circle import should succeed");

        assert!((curve.evaluate(0.0) - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-6);
        assert!((curve.evaluate(1.0) - Point3::new(0.0, 10.0, 0.0)).norm() < 1e-6);
        assert!((curve.evaluate(0.5) - Point3::new(7.0710678119, 7.0710678119, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn trimmed_circle_false_sense_reverses_trim_direction() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(
            2,
            RawEntity {
                name: "DIRECTION".to_string(),
                args: "'',(0.0,0.0,1.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            3,
            RawEntity {
                name: "DIRECTION".to_string(),
                args: "'',(1.0,0.0,0.0)".to_string(),
            },
        );
        ctx.raw_entities.insert(
            4,
            RawEntity {
                name: "AXIS2_PLACEMENT_3D".to_string(),
                args: "'',#1,#2,#3".to_string(),
            },
        );
        ctx.raw_entities.insert(
            5,
            RawEntity {
                name: "CIRCLE".to_string(),
                args: "'',#4,10.0".to_string(),
            },
        );
        ctx.raw_entities.insert(
            6,
            RawEntity {
                name: "TRIMMED_CURVE".to_string(),
                args:
                    "'',#5,(PARAMETER_VALUE(0.0)),(PARAMETER_VALUE(1.570796326795)),.F.,.PARAMETER."
                        .to_string(),
            },
        );

        let curve = StepImporter::get_curve(
            &mut ctx,
            6,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        )
        .expect("false-sense trimmed circle import should succeed");

        assert!((curve.evaluate(0.0) - Point3::new(0.0, 10.0, 0.0)).norm() < 1e-6);
        assert!((curve.evaluate(1.0) - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn imports_direct_bspline_curve_with_knots() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(5.0, 10.0, 0.0));
        ctx.raw_entities.insert(3, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(
            10,
            RawEntity {
                name: "B_SPLINE_CURVE_WITH_KNOTS".to_string(),
                args: "'',2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,3),(0.0,1.0),.UNSPECIFIED."
                    .to_string(),
            },
        );

        let curve = StepImporter::get_curve(
            &mut ctx,
            10,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        )
        .expect("B-spline curve import");

        assert_eq!(curve.degree, 2);
        assert_eq!(curve.control_points.len(), 3);
        assert_eq!(curve.knots.knots, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
        assert_eq!(curve.control_points[1].point, Point3::new(5.0, 10.0, 0.0));
    }

    #[test]
    fn edge_curve_false_same_sense_reverses_imported_curve_direction() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(
            3,
            RawEntity {
                name: "VERTEX_POINT".to_string(),
                args: "'',#1".to_string(),
            },
        );
        ctx.raw_entities.insert(
            4,
            RawEntity {
                name: "VERTEX_POINT".to_string(),
                args: "'',#2".to_string(),
            },
        );
        ctx.raw_entities.insert(
            5,
            RawEntity {
                name: "B_SPLINE_CURVE_WITH_KNOTS".to_string(),
                args: "'',1,(#2,#1),.UNSPECIFIED.,.F.,.F.,(2,2),(0.0,1.0),.UNSPECIFIED."
                    .to_string(),
            },
        );
        ctx.raw_entities.insert(
            6,
            RawEntity {
                name: "EDGE_CURVE".to_string(),
                args: "'',#3,#4,#5,.F.".to_string(),
            },
        );

        let edge = StepImporter::get_edge(&mut ctx, 6).expect("edge import");

        assert_eq!(edge.start_vertex.point, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(edge.end_vertex.point, Point3::new(10.0, 0.0, 0.0));
        assert!((edge.evaluate(0.0) - edge.start_vertex.point).norm() < 1e-9);
        assert!((edge.evaluate(1.0) - edge.end_vertex.point).norm() < 1e-9);
    }

    #[test]
    fn imports_complex_rational_bspline_curve_with_knots() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(10.0, 10.0, 0.0));
        ctx.raw_entities.insert(3, point_entity(0.0, 10.0, 0.0));
        ctx.raw_entities.insert(
            11,
            RawEntity {
                name: "".to_string(),
                args: " BOUNDED_CURVE() B_SPLINE_CURVE(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.0,1.0),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.0,0.707106781187,1.0)) REPRESENTATION_ITEM('') ".to_string(),
            },
        );

        let curve = StepImporter::get_curve(
            &mut ctx,
            11,
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        )
        .expect("rational B-spline curve import");

        assert_eq!(curve.degree, 2);
        assert_eq!(curve.control_points.len(), 3);
        assert!((curve.control_points[1].weight - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6);
        assert!((curve.evaluate(0.5) - Point3::new(7.0710678119, 7.0710678119, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn imports_complex_bspline_curve_when_knot_entity_precedes_basis_entity() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(5.0, 10.0, 0.0));
        ctx.raw_entities.insert(3, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(
            12,
            RawEntity {
                name: "".to_string(),
                args: " B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.0,1.0),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() B_SPLINE_CURVE(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.) REPRESENTATION_ITEM('') ".to_string(),
            },
        );

        let curve = StepImporter::get_curve(
            &mut ctx,
            12,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        )
        .expect("B-spline curve import");

        assert_eq!(curve.degree, 2);
        assert_eq!(curve.control_points.len(), 3);
        assert_eq!(curve.control_points[1].point, Point3::new(5.0, 10.0, 0.0));
    }

    #[test]
    fn imports_direct_bspline_surface_with_knots() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(0.0, 10.0, 0.0));
        ctx.raw_entities.insert(3, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(4, point_entity(10.0, 10.0, 5.0));
        ctx.raw_entities.insert(
            20,
            RawEntity {
                name: "B_SPLINE_SURFACE_WITH_KNOTS".to_string(),
                args: "'',1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.0,1.0),(0.0,1.0),.UNSPECIFIED.".to_string(),
            },
        );

        let geom = StepImporter::get_surface(&mut ctx, 20).expect("surface import");
        let FaceGeometry::Nurbs(surface) = geom else {
            panic!("expected NURBS surface");
        };

        assert_eq!(surface.degree_u, 1);
        assert_eq!(surface.degree_v, 1);
        assert_eq!(surface.control_points.len(), 2);
        assert_eq!(surface.control_points[0].len(), 2);
        assert_eq!(surface.knots_u.knots, vec![0.0, 0.0, 1.0, 1.0]);
        assert_eq!(surface.evaluate(0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
        assert_eq!(surface.evaluate(1.0, 1.0), Point3::new(10.0, 10.0, 5.0));
    }

    #[test]
    fn imports_complex_rational_bspline_surface_with_knots() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(10.0, 0.0, 30.0));
        ctx.raw_entities.insert(3, point_entity(10.0, 10.0, 0.0));
        ctx.raw_entities.insert(4, point_entity(10.0, 10.0, 30.0));
        ctx.raw_entities.insert(5, point_entity(0.0, 10.0, 0.0));
        ctx.raw_entities.insert(6, point_entity(0.0, 10.0, 30.0));
        ctx.raw_entities.insert(
            30,
            RawEntity {
                name: "".to_string(),
                args: " BOUNDED_SURFACE() B_SPLINE_SURFACE(2,1,((#1,#2),(#3,#4),(#5,#6)),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.0,1.0),(0.0,1.0),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((1.0,1.0),(0.707106781187,0.707106781187),(1.0,1.0))) REPRESENTATION_ITEM('') SURFACE() ".to_string(),
            },
        );

        let geom = StepImporter::get_surface(&mut ctx, 30).expect("surface import");
        let FaceGeometry::Nurbs(surface) = geom else {
            panic!("expected rational NURBS surface");
        };

        assert_eq!(surface.degree_u, 2);
        assert_eq!(surface.degree_v, 1);
        assert_eq!(surface.control_points.len(), 3);
        assert_eq!(surface.control_points[0].len(), 2);
        assert!(
            (surface.control_points[1][0].weight - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6
        );
        assert_eq!(surface.knots_u.knots, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
        assert_eq!(surface.knots_v.knots, vec![0.0, 0.0, 1.0, 1.0]);
    }

    #[test]
    fn imports_complex_bspline_surface_when_knot_entity_precedes_basis_entity() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, point_entity(0.0, 10.0, 0.0));
        ctx.raw_entities.insert(3, point_entity(10.0, 0.0, 0.0));
        ctx.raw_entities.insert(4, point_entity(10.0, 10.0, 5.0));
        ctx.raw_entities.insert(
            31,
            RawEntity {
                name: "".to_string(),
                args: " B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),(0.0,1.0),(0.0,1.0),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() B_SPLINE_SURFACE(1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.) REPRESENTATION_ITEM('') SURFACE() ".to_string(),
            },
        );

        let geom = StepImporter::get_surface(&mut ctx, 31).expect("surface import");
        let FaceGeometry::Nurbs(surface) = geom else {
            panic!("expected NURBS surface");
        };

        assert_eq!(surface.degree_u, 1);
        assert_eq!(surface.degree_v, 1);
        assert_eq!(surface.evaluate(1.0, 1.0), Point3::new(10.0, 10.0, 5.0));
    }
}
