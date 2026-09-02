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
    /// ファイルの長さの単位1つが何ミリか。
    ///
    /// **これを読まないと、答えは静かに間違います。** インチで書かれた
    /// STEP は珍しくなく、数値をそのままミリとして読むと体積が 25.4^3 =
    /// 16387 倍ずれます。返ってくるのは**閉じていて多様体で形も正しい立体**で、
    /// 大きさだけが違う。閉性の検査も面の検査も恒等式も全部通ります。
    ///
    /// 実測（`step_unit_probe`、20x30x40 mm の箱）: インチのファイルで
    /// 1.464570 mm^3、センチのファイルで 24.0 mm^3。どちらも解析解 24000 から
    /// **ちょうど単位の3乗ぶん**外れていました。
    length_scale: f64,
    /// ファイルが自分で申告している**不確かさ**（`UNCERTAINTY_MEASURE_WITH_UNIT`）。
    /// 単位を掛けたあとのミリ。申告が無ければ `None`。
    ///
    /// **これを読まないと、正しいファイルを断ります。** 読み込んだ立体は
    /// `Tolerance::default()`（1e-6）で検査していましたが、**実物のデータは
    /// もっと粗い精度で書かれていることがあります**——実測（4-228）:
    /// OCCT が配る `linkrods.step` は **2e-5 と申告**しており、境界の点が面から
    /// 5〜9e-6 外れます。**そのファイルの中では正しい**のに、こちらの 1e-6 で
    /// 断っていました。
    declared_uncertainty: Option<f64>,
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
            length_scale: 1.0,
            declared_uncertainty: None,
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

    /// Reads one EDGE_CURVE out of a STEP text, without needing a whole solid
    /// around it.
    ///
    /// Diagnostic entry point: a malformed curve is easier to pin down on its
    /// own than through the shell that failed to validate because of it.
    pub fn import_edge_from_str(content: &str, edge_id: u64) -> Result<Edge, String> {
        let mut ctx = ImportContext::new();
        Self::parse_data_section(content, &mut ctx)?;
        Self::get_edge(&mut ctx, edge_id)
    }

    /// Reads one ADVANCED_FACE out of a STEP text, without needing a shell
    /// around it. Diagnostic counterpart to [`Self::import_edge_from_str`].
    pub fn import_face_from_str(content: &str, face_id: u64) -> Result<Face, String> {
        let mut ctx = ImportContext::new();
        Self::parse_data_section(content, &mut ctx)?;
        Self::resolve_face(&mut ctx, face_id)
    }

    /// STEPテキストから複数の Solid（B-Repソリッド）をインポート
    pub fn import_solids_from_str(content: &str) -> Result<Vec<Solid>, String> {
        let mut ctx = ImportContext::new();

        // 1. DATAセクションのエンティティ辞書を構築
        Self::parse_data_section(content, &mut ctx)?;

        // 2. 長さの単位。座標を読む前に決めておく必要がある。
        ctx.length_scale = Self::resolve_length_scale(&ctx)?;

        // 2b. ファイルが申告している不確かさ。**検査の物差しに使います**（4-228）。
        ctx.declared_uncertainty = Self::resolve_declared_uncertainty(&ctx);

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

    /// `TOKEN( ... )` の中身を、括弧の対応を数えて取り出す。
    ///
    /// **トークンと `(` の間には空白が入りえます。** STEP のファイルは 80 桁で
    /// 折り返され、その折り返しがちょうどそこに来ることがあります。実測:
    ///
    /// ```text
    /// #165 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3)
    /// GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#169)) GLOBAL_UNIT_ASSIGNED_CONTEXT
    /// ((#166,#167,#168)) REPRESENTATION_CONTEXT('Context #1', ...) );
    /// ```
    ///
    /// 行は空白で連結されるので、`GLOBAL_UNIT_ASSIGNED_CONTEXT(` を探しても
    /// 当たりません。最初に単位を読もうとしたとき、これで**単位の文脈が無い
    /// ファイル**に見え、係数 1 のまま素通りしました。
    fn token_args<'a>(text: &'a str, token: &str) -> Option<&'a str> {
        let mut from = 0usize;
        while let Some(at) = text[from..].find(token) {
            let start = from + at;
            // 前が英数字なら、別の名前の末尾に当たっている（LENGTH_UNIT と
            // PLANE_ANGLE_UNIT のような取り違えを避ける）。
            let preceded_by_name = text[..start]
                .chars()
                .next_back()
                .is_some_and(|c| c.is_ascii_alphanumeric() || c == '_');
            let rest = &text[start + token.len()..];
            let open = rest.find(|c: char| !c.is_whitespace());
            from = start + token.len();
            if preceded_by_name {
                continue;
            }
            let Some(open) = open else { continue };
            if rest.as_bytes()[open] != b'(' {
                continue;
            }
            let body = &rest[open + 1..];
            let mut depth = 1usize;
            for (index, ch) in body.char_indices() {
                match ch {
                    '(' => depth += 1,
                    ')' => {
                        depth -= 1;
                        if depth == 0 {
                            return Some(&body[..index]);
                        }
                    }
                    _ => {}
                }
            }
        }
        None
    }

    /// ファイルの長さの単位1つが何ミリかを返す。
    ///
    /// 単位は `GLOBAL_UNIT_ASSIGNED_CONTEXT((#a,#b,#c))` が指しています。
    /// そのうち長さのものを選び、係数に直します。
    ///
    /// - `( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )` → 1
    /// - `SI_UNIT($,.METRE.)`（接頭辞なし） → 1000
    /// - `( CONVERSION_BASED_UNIT('INCH',#m) ... )` で
    ///   `#m = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#mm)` → 25.4
    ///
    /// **単位の文脈があるのに読めないときは、断ります。** そこで既定の
    /// ミリに落とすと、インチのファイルが 16387 倍小さい立体として通ります。
    /// 文脈そのものが無いファイルはミリとして読みます（省略できるのは
    /// 最小限のファイルだけで、AP203/214/242 は必ず持っています）。
    fn resolve_length_scale(ctx: &ImportContext) -> Result<f64, String> {
        let Some(assigned) = ctx
            .raw_entities
            .values()
            .find(|raw| Self::token_args(&raw.args, "GLOBAL_UNIT_ASSIGNED_CONTEXT").is_some())
        else {
            return Ok(1.0);
        };

        // GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) の中身。
        let list = Self::token_args(&assigned.args, "GLOBAL_UNIT_ASSIGNED_CONTEXT")
            .ok_or_else(|| "unit context has no unit list".to_string())?;

        let mut seen: Vec<u64> = Vec::new();
        for arg in list
            .trim()
            .trim_start_matches('(')
            .trim_end_matches(')')
            .split(',')
        {
            if let Some(id) = Self::parse_entity_ref(arg) {
                seen.push(id);
            }
        }
        if seen.is_empty() {
            return Err(
                "STEP unit context lists no units; cannot tell what the numbers mean".to_string(),
            );
        }

        for id in &seen {
            let Some(raw) = ctx.raw_entities.get(id) else {
                continue;
            };
            if Self::token_args(&raw.args, "LENGTH_UNIT").is_none() {
                continue;
            }
            return Self::length_unit_scale(ctx, *id, 0);
        }

        Err(format!(
            "STEP unit context (#{:?}) names no length unit; cannot tell what the numbers mean",
            seen
        ))
    }

    /// ファイルが申告している不確かさを読む（`UNCERTAINTY_MEASURE_WITH_UNIT`）。
    ///
    /// 申告が無い・読めないときは `None` を返します。**推測はしません。**
    fn resolve_declared_uncertainty(ctx: &ImportContext) -> Option<f64> {
        let mut best: Option<f64> = None;
        for raw in ctx.raw_entities.values() {
            // **実体名で来る場合と、複合実体の中に埋まっている場合があります。**
            // 前者は `#18622 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.E-5),...)`、
            // 後者は `( ... UNCERTAINTY_MEASURE_WITH_UNIT(...) ... )` です。
            let args = if raw.name == "UNCERTAINTY_MEASURE_WITH_UNIT" {
                raw.args.as_str()
            } else if let Some(inner) = Self::token_args(&raw.args, "UNCERTAINTY_MEASURE_WITH_UNIT")
            {
                inner
            } else {
                continue;
            };
            let Some(measure) = Self::token_args(args, "LENGTH_MEASURE") else {
                continue;
            };
            let Ok(value) = measure.trim().parse::<f64>() else {
                continue;
            };
            if !(value.is_finite() && value > 0.0) {
                continue;
            }
            let millimetres = value * ctx.length_scale;
            best = Some(best.map_or(millimetres, |worst: f64| worst.max(millimetres)));
        }
        best
    }

    /// 読み込んだ立体を検査するときの物差し。
    ///
    /// **ファイルの申告のほうが粗ければ、そちらに合わせます。** 申告より
    /// 厳しく測っても、こちらが正しくなるわけではありません——**書いた側の
    /// 精度を超える主張はできない**からです。
    ///
    /// **緩めっぱなしにはしません。** 天井は `1e-3` mm です。それを超える
    /// 申告は、もう「不確かさ」ではなく別の問題なので、既定のまま断ります。
    /// 読み込みで受ける粗さの天井。これを超えたら公差の話ではありません。
    fn import_ceiling() -> f64 {
        1e-3
    }

    fn import_tolerance(ctx: &ImportContext) -> Tolerance {
        const CEILING: f64 = 1e-3;
        let mut tol = Tolerance::default();
        if let Some(declared) = ctx.declared_uncertainty {
            if declared > tol.linear && declared <= CEILING {
                tol.linear = declared;
            }
        }
        // **読み込みの公差を上書きする口**（`ZENITH_IMPORT_TOL`。4-265）。
        //
        // **答えが変わります**——速くするための口ではありません。
        // 「他所のファイルの粗さを、どこまで受けるか」を**決める前に測る**
        // ためのものです。実測: `linkrods.step` は既定（1e-6）では読めま
        // せんが、**1e-3 まで受けると読めます**（面 37、体積 3.897551）。
        // そのとき殻の**位相は無傷**で（非多様体 0、相手のいない稜 0）、
        // 残るのは**境界が曲面から外れる 255 件（最大 4.145e-4）**だけです。
        //
        // **申告された不確かさは上限にはなりません**——`screw.step` は
        // 1e-6 と申告しながら、自分の境界が 2.946e-4 ずれています（294 倍）。
        if let Some(value) = std::env::var_os("ZENITH_IMPORT_TOL") {
            if let Some(parsed) = value.to_str().and_then(|v| v.parse::<f64>().ok()) {
                tol.linear = parsed;
            }
        }
        tol
    }

    /// 長さの単位の実体1つを、ミリへの係数に直す。`depth` は循環参照よけ。
    fn length_unit_scale(ctx: &ImportContext, id: u64, depth: usize) -> Result<f64, String> {
        if depth > 8 {
            return Err(format!("STEP length unit #{id} refers to itself"));
        }
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("STEP length unit #{id} is not in the file"))?;

        // 換算単位: 別の単位で測った長さ1つぶんが、この単位1つ。
        if let Some(inner) = Self::token_args(&raw.args, "CONVERSION_BASED_UNIT") {
            let measure_id = inner
                .split(',')
                .filter_map(Self::parse_entity_ref)
                .next()
                .ok_or_else(|| format!("STEP unit #{id} names no conversion factor"))?;
            let measure = ctx
                .raw_entities
                .get(&measure_id)
                .ok_or_else(|| format!("STEP unit factor #{measure_id} is not in the file"))?;
            // LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#mm)
            let value = Self::token_args(&measure.args, "LENGTH_MEASURE")
                .and_then(|text| text.trim().parse::<f64>().ok())
                .ok_or_else(|| format!("STEP unit factor #{measure_id} has no length measure"))?;
            let base_id = Self::split_top_level_args(&measure.args)
                .into_iter()
                .filter_map(Self::parse_entity_ref)
                .next()
                .ok_or_else(|| format!("STEP unit factor #{measure_id} names no base unit"))?;
            return Ok(value * Self::length_unit_scale(ctx, base_id, depth + 1)?);
        }

        // SI 単位: 接頭辞つきのメートル。
        if let Some(body) = Self::token_args(&raw.args, "SI_UNIT") {
            let mut fields = body.split(',');
            let prefix = fields.next().unwrap_or("$").trim();
            let name = fields.next().unwrap_or("").trim();
            if name != ".METRE." {
                return Err(format!("STEP length unit #{id} is not a metre: {name}"));
            }
            // メートルを基準に、ミリでの大きさ。
            let scale = match prefix {
                "$" => 1000.0,
                ".KILO." => 1_000_000.0,
                ".HECTO." => 100_000.0,
                ".DECA." => 10_000.0,
                ".DECI." => 100.0,
                ".CENTI." => 10.0,
                ".MILLI." => 1.0,
                ".MICRO." => 1.0e-3,
                ".NANO." => 1.0e-6,
                ".PICO." => 1.0e-9,
                other => {
                    return Err(format!(
                        "STEP length unit #{id} uses the prefix {other}, which Zenith does not know"
                    ))
                }
            };
            return Ok(scale);
        }

        Err(format!(
            "STEP length unit #{id} is neither an SI unit nor a conversion-based one"
        ))
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
                        // 座標はここが唯一の入口。単位はここで掛ける。
                        let scale = ctx.length_scale;
                        let p = Point3::new(x * scale, y * scale, z * scale);
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
                // **稜の頂点で切り詰めます**（4-228）。丸ごと使う稜では
                // 何も起きません。
                curve = Self::trim_curve_to_vertices(
                    curve,
                    v_start.point,
                    v_end.point,
                    Self::import_tolerance(ctx).linear,
                );
                if !same_sense {
                    curve = curve.reversed();
                }

                // **稜ごとに、曲線の端が頂点とどれだけ離れているかを出す口**
                // （`ZENITH_STEP_WHY=1`）。読めなかったファイルで、どの曲線の
                // 種類が悪いのかを推測せずに見るためです（4-228）。
                if std::env::var_os("ZENITH_STEP_WHY").is_some() {
                    let (t_min, t_max) = curve.param_range();
                    let gap_start = (curve.evaluate(t_min) - v_start.point).norm();
                    let gap_end = (curve.evaluate(t_max) - v_end.point).norm();
                    if gap_start.max(gap_end) > 1e-6 {
                        let kind = ctx
                            .raw_entities
                            .get(&curve_id)
                            .map(|raw| raw.name.clone())
                            .unwrap_or_else(|| "?".to_string());
                        eprintln!(
                            "STEPWHY EDGE_CURVE #{id} 曲線 #{curve_id} {kind} same_sense={same_sense} 端のずれ 始 {gap_start:.6e} 終 {gap_end:.6e}"
                        );
                    }
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

        if raw.name == "ELLIPSE" {
            if let Some(c) = Self::arc_from_ellipse_entity(ctx, &raw, p_start, p_end)? {
                ctx.curves.insert(id, c.clone());
                return Ok(c);
            }
        }

        // `SEAM_CURVE` は ISO 10303-42 で `surface_curve` の subtype です。
        // **属性の並びは同じ**（name, curve_3d, associated_geometry,
        // master_representation）で、違うのは「閉じた曲面の継ぎ目に乗って
        // いて、同じ曲面を2回指す」という制約だけです。**3D 曲線の取り出し方は
        // 変わりません。**
        //
        // 実測（4-228）: OCCT が配っている `screw.step` と `linkrods.step` は
        // **2つともこれで読めませんでした**。閉じた曲面（円柱・球・トーラス）の
        // 継ぎ目にある稜はみなこれなので、実物のデータには普通に出ます。
        if raw.name == "SURFACE_CURVE" || raw.name == "SEAM_CURVE" {
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

        // ここは端点を結ぶ直線を返していた。楕円弧や複合曲線をそのまま弦に
        // 置き換えるということで、平面の上に載る境界なら**面からは外れない**
        // ので、下流の p-curve 検証を素通りしうる。閉じた円のように両端が
        // 同じ点になる曲線は退化として捕まるが、それは捕まる側の運が良い
        // だけである。読めなかったことは、読めなかったと言う。
        Err(format!(
            "Unsupported curve entity {} (#{}). Zenith reads LINE, CIRCLE, \
             TRIMMED_CURVE, SURFACE_CURVE, SEAM_CURVE and (rational) B_SPLINE_CURVE_WITH_KNOTS",
            raw.name, id
        ))
    }

    /// **稜の頂点で、曲線を切り詰める**（4-228）。
    ///
    /// `EDGE_CURVE` の頂点は、曲線の**どこからどこまでを使うか**を決めます。
    /// 自分が書いた STEP では稜が曲線を丸ごと使うので、これまで切り詰めなくても
    /// 合っていました。**実物のデータはそうではありません**——実測: OCCT の
    /// `screw.step` で、B スプラインの端が頂点から **10.18** 離れていました
    /// （`EDGE_CURVE #336`。曲線 `#340` を途中まで使う稜）。
    ///
    /// 端のずれが公差の中なら**何もしません**（丸ごと使う稜がほとんどです）。
    /// 曲線に無い点を指している稜は、切り詰めても直らないので、そのまま返して
    /// 下流の検査に任せます——**黙って形を変えるより、断るほうが安全です**。
    fn trim_curve_to_vertices(
        curve: NurbsCurve3,
        p_start: Point3,
        p_end: Point3,
        limit: f64,
    ) -> NurbsCurve3 {
        let (t_min, t_max) = curve.param_range();
        let gap_start = (curve.evaluate(t_min) - p_start).norm();
        let gap_end = (curve.evaluate(t_max) - p_end).norm();
        if gap_start <= limit && gap_end <= limit {
            return curve;
        }

        // 曲線の上でいちばん近いところを探す。**粗く撒いてから詰めます**
        // （`ExtremumEngine` は `zenith_algo` にあり、ここからは使えません）。
        let nearest = |point: Point3| -> (f64, f64) {
            const SAMPLES: usize = 512;
            let mut best = (t_min, f64::INFINITY);
            for step in 0..=SAMPLES {
                let t = t_min + (t_max - t_min) * (step as f64 / SAMPLES as f64);
                let distance = (curve.evaluate(t) - point).norm();
                if distance < best.1 {
                    best = (t, distance);
                }
            }
            let mut window = (t_max - t_min) / SAMPLES as f64;
            for _ in 0..48 {
                for side in [-1.0, 1.0] {
                    let t = (best.0 + side * window).clamp(t_min, t_max);
                    let distance = (curve.evaluate(t) - point).norm();
                    if distance < best.1 {
                        best = (t, distance);
                    }
                }
                window *= 0.5;
            }
            best
        };

        let (t_start, near_start) = nearest(p_start);
        let (t_end, near_end) = nearest(p_end);
        // **頂点が曲線の上に無いなら、切り詰めても直りません。**
        if near_start > limit || near_end > limit || !(t_start < t_end) {
            return curve;
        }

        let trimmed = curve
            .split_at(t_start)
            .map(|(_, back)| back)
            .unwrap_or_else(|| curve.clone());
        trimmed
            .split_at(t_end)
            .map(|(front, _)| front)
            .unwrap_or(trimmed)
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
        // 半径は長さ。単位を掛ける。
        let radius = match circle_parts[2].parse::<f64>() {
            Ok(r) if r > 1e-9 => r * ctx.length_scale,
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
        // 半径は長さ。単位を掛ける。
        let radius = match parts[2].parse::<f64>() {
            Ok(r) if r > 1e-9 => r * ctx.length_scale,
            _ => return Ok(None),
        };
        let (center, normal, ref_dir) = Self::get_axis2_placement(ctx, axis_id)?;

        let v0 = p_start - center;
        let v1 = p_end - center;
        if (v0.norm() - radius).abs() > 1e-3 || (v1.norm() - radius).abs() > 1e-3 {
            return Ok(None);
        }

        // 円弧は円の自前の座標系から角度で組む。端点から幾何を推測すると、
        // 始点と終点が同じ完全円（円柱の縁として必ず出てくる形）を角度0の
        // 退化として捨ててしまう。
        arc_from_angles(center, normal, ref_dir, radius, p_start, p_end).map(Some)
    }

    /// `SURFACE_OF_LINEAR_EXTRUSION('', #swept_curve, #extrusion_axis)`
    ///
    /// 断面曲線を一方向にまっすぐ掃いた曲面。OpenCASCADE は、スプライン断面や
    /// 楕円断面の押し出しをこの形で書く（解析曲面に落とせる円や直線の押し出しは
    /// `CYLINDRICAL_SURFACE` / `PLANE` になるので、ここに来るのは自由曲線の
    /// 押し出しに限られる）。
    ///
    /// `S(u, v) = C(u) + v * V` は `v` について1次なので、`v` 方向の次数を1に
    /// して両端の制御点列を置けば**厳密**に表せる。有理曲線でも、重みはそのまま
    /// で制御点だけ平行移動すればよい（同次座標で見ると分母が `v` に依らない）。
    ///
    /// `v` の範囲は境界から取る。解析曲面と同じで、STEP の掃引面は範囲を
    /// 持たないからである。ここでは制御点の凸包を使って**必ず覆う**側に丸める
    /// （面のトリムは境界ワイヤが担うので、広いぶんには困らない）。
    fn linear_extrusion_patch(
        ctx: &mut ImportContext,
        raw: &RawEntity,
        boundary_points: &[Point3],
    ) -> Result<Option<NurbsSurface3>, String> {
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 3 {
            return Ok(None);
        }
        let (Some(curve_id), Some(vector_id)) = (
            Self::parse_entity_ref(parts[1]),
            Self::parse_entity_ref(parts[2]),
        ) else {
            return Ok(None);
        };

        let Some(basis) = Self::swept_basis_curve(ctx, curve_id)? else {
            return Ok(None);
        };
        let Some(extrusion) = Self::get_vector(ctx, vector_id)? else {
            return Ok(None);
        };
        let length = extrusion.norm();
        if length <= 1e-12 {
            return Ok(None);
        }
        let direction = extrusion / length;

        // 曲線は制御点の凸包に収まるので、掃引方向の成分もその範囲に収まる。
        let mut curve_low = f64::INFINITY;
        let mut curve_high = f64::NEG_INFINITY;
        for cp in &basis.control_points {
            let s = cp.point.coords.dot(&direction);
            curve_low = curve_low.min(s);
            curve_high = curve_high.max(s);
        }
        let mut boundary_low = f64::INFINITY;
        let mut boundary_high = f64::NEG_INFINITY;
        for point in boundary_points {
            let s = point.coords.dot(&direction);
            boundary_low = boundary_low.min(s);
            boundary_high = boundary_high.max(s);
        }
        if !(curve_low.is_finite() && boundary_low.is_finite()) {
            return Ok(None);
        }

        // 安全のためのマージンは足さない。凸包の範囲だけで必ず覆えるうえ、
        // **この面の境界はパラメータ矩形そのもの**なので、はみ出したぶんが
        // そのまま積分に乗る。実測で 2e-6 だけ v を広げたところ、側面の面積が
        // 3.0e-6、立体の体積が 1.3e-6 だけ大きく出て、しかも分割数を振っても
        // 動かなかった（平面キャップのほうは 1.5e-14 で厳密なままなので、
        // 楕円そのものは正しい）。動かない差は求積の粗さではなく、測っている
        // 対象が違うという印である。
        let v_low = (boundary_low - curve_high) / length;
        let v_high = (boundary_high - curve_low) / length;

        let control_points: Vec<Vec<ControlPoint3>> = basis
            .control_points
            .iter()
            .map(|cp| {
                vec![
                    ControlPoint3::new(cp.point + extrusion * v_low, cp.weight),
                    ControlPoint3::new(cp.point + extrusion * v_high, cp.weight),
                ]
            })
            .collect();

        NurbsSurface3::new(
            basis.degree,
            1,
            control_points,
            basis.knots.clone(),
            KnotVector::clamped_uniform(2, 1),
        )
        .map(Some)
    }

    /// `AXIS1_PLACEMENT('', #origin, #direction)`
    ///
    /// `AXIS2_PLACEMENT_3D` と違い、横向きの基準方向を持たない。回転面の軸は
    /// これで書かれる。
    fn get_axis1_placement(
        ctx: &mut ImportContext,
        axis_id: u64,
    ) -> Result<Option<(Point3, Vec3)>, String> {
        let raw = ctx
            .raw_entities
            .get(&axis_id)
            .ok_or_else(|| format!("Entity #{} not found", axis_id))?
            .clone();
        if raw.name != "AXIS1_PLACEMENT" {
            return Ok(None);
        }
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 3 {
            return Ok(None);
        }
        let (Some(origin_id), Some(direction_id)) = (
            Self::parse_entity_ref(parts[1]),
            Self::parse_entity_ref(parts[2]),
        ) else {
            return Ok(None);
        };
        let origin = Self::get_point(ctx, origin_id)?;
        let direction = Self::get_direction(ctx, direction_id)?;
        let Some(direction) = direction.try_normalize(1e-12) else {
            return Ok(None);
        };
        Ok(Some((origin, direction)))
    }

    /// `SURFACE_OF_REVOLUTION('', #swept_curve, #axis1_placement)`
    ///
    /// 断面曲線を軸まわりに回した曲面。OpenCASCADE は、曲がった軌道に沿った
    /// 掃引（曲がり管）や、円錐・円柱・トーラスに落ちない回転体をこの形で書く。
    ///
    /// **既存の検体には1つも入っていませんでした。** インポーターは
    /// 「読める曲面」の一覧にこれを挙げておらず、実際に断ります。回転体は
    /// 旋盤部品でもパイプでも普通に出てくる形です。
    ///
    /// 厳密に表せます。回転は角度方向に**有理2次**（円弧の標準の組み方）、
    /// 断面方向は元の曲線の次数とノットをそのまま使えばよく、これは
    /// `revolve_profile` が既に持っている組み方そのものです。解析曲面
    /// （円柱・円錐・トーラス）も同じ関数を通っています。
    ///
    /// 断面は**軸を含む平面の上に乗っていなければなりません**。そうでない
    /// 曲線を回すと、できるのは回転面ではなく螺旋面です。制御点が軸を含む
    /// 半平面から外れていたら断ります——近いところを通るもっともらしい面を
    /// 返すと、その先の分割と選別が静かに間違います。
    ///
    /// 角度の範囲は境界から取ります。解析曲面と同じで、STEP の掃引面は範囲を
    /// 持たないからです。
    fn revolution_patch(
        ctx: &mut ImportContext,
        raw: &RawEntity,
        boundary_points: &[Point3],
    ) -> Result<Option<NurbsSurface3>, String> {
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 3 {
            return Ok(None);
        }
        let (Some(curve_id), Some(axis_id)) = (
            Self::parse_entity_ref(parts[1]),
            Self::parse_entity_ref(parts[2]),
        ) else {
            return Ok(None);
        };

        let Some(basis) = Self::swept_basis_curve(ctx, curve_id)? else {
            return Ok(None);
        };
        let Some((origin, axis)) = Self::get_axis1_placement(ctx, axis_id)? else {
            return Ok(None);
        };
        if basis.control_points.len() < 2 {
            return Ok(None);
        }

        // 断面がどの半平面に乗っているかを、いちばん軸から離れた制御点で
        // 決めます。軸の上に乗った点（半径 0）は向きを決められません。
        let mut reference: Option<Vec3> = None;
        let mut widest = 0.0;
        for control_point in &basis.control_points {
            let (_, radial) = axis_frame_coords(control_point.point, origin, axis);
            if radial.norm() > widest {
                widest = radial.norm();
                reference = radial.try_normalize(1e-12);
            }
        }
        let Some(x_axis) = reference else {
            return Ok(None);
        };
        let y_axis = axis.cross(&x_axis);

        // 断面を (半径, 軸方向, 重み) に直します。**半平面から外れていたら
        // 断ります。**
        let scale = widest.max(1.0);
        let mut profile: Vec<(f64, f64, f64)> = Vec::with_capacity(basis.control_points.len());
        for control_point in &basis.control_points {
            let (axial, radial) = axis_frame_coords(control_point.point, origin, axis);
            if radial.dot(&y_axis).abs() > scale * 1e-9 {
                return Ok(None);
            }
            profile.push((radial.dot(&x_axis), axial, control_point.weight));
        }

        // 角度の範囲は境界から。軸の上に乗った点は角度を持たないので外します。
        let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
        for point in boundary_points {
            let (_, radial) = axis_frame_coords(*point, origin, axis);
            if radial.norm() <= scale * 1e-9 {
                continue;
            }
            angles.push(radial.dot(&y_axis).atan2(radial.dot(&x_axis)));
        }
        let (start_angle, sweep) = angular_span(&mut angles);

        Ok(revolve_profile(
            origin,
            axis,
            x_axis,
            y_axis,
            &profile,
            basis.knots.clone(),
            basis.degree,
            start_angle,
            sweep,
        ))
    }
    /// 掃引面の断面になっている曲線。端点を必要としない形だけを受け取る。
    fn swept_basis_curve(
        ctx: &mut ImportContext,
        curve_id: u64,
    ) -> Result<Option<NurbsCurve3>, String> {
        let raw = ctx
            .raw_entities
            .get(&curve_id)
            .ok_or_else(|| format!("Entity #{} not found", curve_id))?
            .clone();

        if raw.name == "B_SPLINE_CURVE_WITH_KNOTS" || raw.args.contains("B_SPLINE_CURVE_WITH_KNOTS")
        {
            return Self::parse_nurbs_curve(ctx, &raw);
        }

        // 円・楕円は全周で組む。掃引面の断面としては閉じた1本なので、端点から
        // 角度を測る必要がない。
        let parts = Self::split_top_level_args(&raw.args);
        if (raw.name == "CIRCLE" && parts.len() >= 3) || (raw.name == "ELLIPSE" && parts.len() >= 4)
        {
            let Some(axis_id) = Self::parse_entity_ref(parts[1]) else {
                return Ok(None);
            };
            let Ok(semi_x) = parts[2].trim().parse::<f64>() else {
                return Ok(None);
            };
            let semi_y = if raw.name == "ELLIPSE" {
                match parts[3].trim().parse::<f64>() {
                    Ok(value) => value,
                    Err(_) => return Ok(None),
                }
            } else {
                semi_x
            };
            if !(semi_x > 1e-9 && semi_y > 1e-9) {
                return Ok(None);
            }
            let (center, normal, ref_dir) = Self::get_axis2_placement(ctx, axis_id)?;
            let axis = normal
                .try_normalize(1e-12)
                .ok_or_else(|| "Swept curve axis is degenerate".to_string())?;
            let x_axis = {
                let projected = ref_dir - axis * ref_dir.dot(&axis);
                projected
                    .try_normalize(1e-12)
                    .ok_or_else(|| "Swept curve reference direction is degenerate".to_string())?
            };
            let y_axis = axis.cross(&x_axis);
            return rational_elliptic_arc(
                center,
                x_axis,
                y_axis,
                semi_x,
                semi_y,
                0.0,
                std::f64::consts::PI * 2.0,
            )
            .map(Some);
        }

        Ok(None)
    }

    /// `VECTOR('', #direction, magnitude)` を、向きと長さを掛けた1本のベクトルで返す。
    fn get_vector(ctx: &mut ImportContext, id: u64) -> Result<Option<Vec3>, String> {
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();
        if raw.name != "VECTOR" {
            return Ok(None);
        }
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 3 {
            return Ok(None);
        }
        let Some(direction_id) = Self::parse_entity_ref(parts[1]) else {
            return Ok(None);
        };
        let Ok(magnitude) = parts[2].trim().parse::<f64>() else {
            return Ok(None);
        };
        // 大きさは長さ。向きは無次元なので掛けない。
        let magnitude = magnitude * ctx.length_scale;
        let direction = Self::get_direction(ctx, direction_id)?;
        Ok(Some(direction * magnitude))
    }

    /// `ELLIPSE('', #axis2_placement_3d, semi_axis_1, semi_axis_2)`
    ///
    /// 楕円は、押し出し断面や斜め切りの円柱の縁として実務のファイルによく出る。
    /// 対応していなかったときは、端点を結ぶ**直線**に置き換わっていた。平面の
    /// 上に載る境界ならその弦も面から外れないので、下流の検証を素通りしうる。
    fn arc_from_ellipse_entity(
        ctx: &mut ImportContext,
        raw: &RawEntity,
        p_start: Point3,
        p_end: Point3,
    ) -> Result<Option<NurbsCurve3>, String> {
        let parts = Self::split_top_level_args(&raw.args);
        if parts.len() < 4 {
            return Ok(None);
        }
        let Some(axis_id) = Self::parse_entity_ref(parts[1]) else {
            return Ok(None);
        };
        // 半軸は長さ。単位を掛ける。
        let (Ok(semi_x), Ok(semi_y)) = (
            parts[2].trim().parse::<f64>().map(|v| v * ctx.length_scale),
            parts[3].trim().parse::<f64>().map(|v| v * ctx.length_scale),
        ) else {
            return Ok(None);
        };
        if !(semi_x > 1e-9 && semi_y > 1e-9) {
            return Ok(None);
        }

        let (center, normal, ref_dir) = Self::get_axis2_placement(ctx, axis_id)?;
        let axis = normal
            .try_normalize(1e-12)
            .ok_or_else(|| "Ellipse axis is degenerate".to_string())?;
        let x_axis = {
            let projected = ref_dir - axis * ref_dir.dot(&axis);
            projected
                .try_normalize(1e-12)
                .ok_or_else(|| "Ellipse reference direction is parallel to its axis".to_string())?
        };
        let y_axis = axis.cross(&x_axis);

        // 楕円の媒介変数角は幾何的な角度ではない。点を各軸の半径で割って
        // 単位円に戻してから角度を測る。
        let angle_of = |point: Point3| {
            let offset = point - center;
            (offset.dot(&y_axis) / semi_y).atan2(offset.dot(&x_axis) / semi_x)
        };
        let on_ellipse = |point: Point3| {
            let offset = point - center;
            let u = offset.dot(&x_axis) / semi_x;
            let v = offset.dot(&y_axis) / semi_y;
            ((u * u + v * v).sqrt() - 1.0).abs() <= 1e-3
        };
        if !on_ellipse(p_start) || !on_ellipse(p_end) {
            return Ok(None);
        }

        let start_angle = angle_of(p_start);
        let full_turn = std::f64::consts::PI * 2.0;
        let mut sweep = angle_of(p_end) - start_angle;
        while sweep <= 1e-9 {
            sweep += full_turn;
        }
        if (p_end - p_start).norm() <= 1e-9 {
            sweep = full_turn;
        }

        rational_elliptic_arc(center, x_axis, y_axis, semi_x, semi_y, start_angle, sweep).map(Some)
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

    /// The point a VERTEX_LOOP stands at, or `None` for any other loop.
    fn vertex_loop_point(ctx: &mut ImportContext, loop_id: u64) -> Option<Point3> {
        let raw = ctx.raw_entities.get(&loop_id)?.clone();
        if raw.name != "VERTEX_LOOP" {
            return None;
        }
        let parts = Self::split_top_level_args(&raw.args);
        let vertex_id = Self::parse_entity_ref(parts.get(1)?)?;
        Self::get_vertex(ctx, vertex_id)
            .ok()
            .map(|vertex| vertex.point)
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
        if raw.name == "VERTEX_LOOP" {
            // VERTEX_LOOP('',#vertex)
            //
            // 頂点ひとつだけのループ。辺が無いのは書き落としではなく、
            // 面が曲面全体を覆っていて囲むものが無いという意味。球を1面で
            // 書くと極がこれになる。空のワイヤで表し、境界の広がりは
            // 頂点のほうから渡す。
            let wire = Wire::new(Vec::new());
            ctx.wires.insert(loop_id, wire.clone());
            return Ok(wire);
        }

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

    /// Builds the surface a face sits on, sized to that face.
    ///
    /// STEP's analytic surfaces are unbounded: a CYLINDRICAL_SURFACE states an
    /// axis and a radius and nothing about how far it runs, and the face's
    /// bounds are what pick out the piece in use. Building a fixed patch and
    /// hoping it covers the boundary does not work, so the extent is taken from
    /// the boundary itself. Surfaces that are already bounded, such as
    /// B-splines, ignore the boundary and go through the ordinary path.
    fn get_surface_for_boundary(
        ctx: &mut ImportContext,
        id: u64,
        boundary_points: &[Point3],
    ) -> Result<FaceGeometry, String> {
        let raw = ctx
            .raw_entities
            .get(&id)
            .ok_or_else(|| format!("Entity #{} not found", id))?
            .clone();

        if boundary_points.is_empty() {
            return Self::get_surface(ctx, id);
        }

        if raw.name == "SURFACE_OF_LINEAR_EXTRUSION" {
            if let Some(nurbs) = Self::linear_extrusion_patch(ctx, &raw, boundary_points)? {
                let geom = FaceGeometry::Nurbs(nurbs);
                ctx.surfaces.insert(id, geom.clone());
                return Ok(geom);
            }
        }

        if raw.name == "SURFACE_OF_REVOLUTION" {
            if let Some(nurbs) = Self::revolution_patch(ctx, &raw, boundary_points)? {
                let geom = FaceGeometry::Nurbs(nurbs);
                ctx.surfaces.insert(id, geom.clone());
                return Ok(geom);
            }
        }

        // どの解析曲面も AXIS2_PLACEMENT_3D を第2引数に取り、その後に半径類が続く。
        let parts = Self::split_top_level_args(&raw.args);
        let placement = parts
            .get(1)
            .and_then(|arg| Self::parse_entity_ref(arg))
            .and_then(|axis2_id| Self::get_axis2_placement(ctx, axis2_id).ok());
        let number = |index: usize| -> Option<f64> {
            parts
                .get(index)
                .and_then(|arg| arg.trim().parse::<f64>().ok())
        };
        // 半径類は長さ。単位を掛ける。角度（円錐の半頂角）には掛けない。
        let unit_scale = ctx.length_scale;
        let length = |index: usize| -> Option<f64> { number(index).map(|v| v * unit_scale) };

        if let Some((origin, z_dir, x_dir)) = placement {
            let patch = match raw.name.as_str() {
                "CYLINDRICAL_SURFACE" => length(2).and_then(|radius| {
                    cylinder_patch_for_boundary(origin, z_dir, x_dir, radius, boundary_points)
                }),
                "CONICAL_SURFACE" => length(2).zip(number(3)).and_then(|(radius, semi_angle)| {
                    cone_patch_for_boundary(
                        origin,
                        z_dir,
                        x_dir,
                        radius,
                        semi_angle,
                        boundary_points,
                    )
                }),
                "SPHERICAL_SURFACE" => length(2).and_then(|radius| {
                    sphere_patch_for_boundary(origin, z_dir, x_dir, radius, boundary_points)
                }),
                "TOROIDAL_SURFACE" => length(2).zip(length(3)).and_then(|(major, minor)| {
                    torus_patch_for_boundary(origin, z_dir, x_dir, major, minor, boundary_points)
                }),
                _ => None,
            };
            if let Some(nurbs) = patch {
                return Ok(FaceGeometry::Nurbs(nurbs));
            }
            // **境界に合わせた解析パッチを作れませんでした**（`ZENITH_STEP_WHY=1`）。
            //
            // ここで落ちると `get_surface` の**決め打ちのパッチ**（トーラスなら
            // 90度 x 90度）に落ちます。面がそこに乗っていなければ、境界の点は
            // **全部**曲面から外れます——実測（4-228）: `screw.step` の
            // トーラス面3枚と円錐面2枚が、27/27・117/117 点とも外れました。
            if std::env::var_os("ZENITH_STEP_WHY").is_some() {
                eprintln!(
                    "STEPWHY 曲面 #{id} {} は境界に合わせたパッチを作れず、決め打ちのパッチに落ちました（境界の点 {} 個）",
                    raw.name,
                    boundary_points.len()
                );
            }
        }

        // **決め打ちに落ちたら、そこに面が乗っているか測ります**（4-263）。
        //
        // 落ちた先は 90度 × 90度の決め打ちパッチです。面がそこに乗っていな
        // ければ、**境界が丸ごと外れた曲面**が黙って出来ます——実測
        // （4-260）: `screw.step` のトーラス面で **87** 離れていました。
        // それを下流へ渡すと、検証が「境界が曲面から外れている」と 72 件
        // 並べます。**読めなかったことが、読めなかったとして出てきません。**
        //
        // **もっともらしい答えより、できないと言う**（3-2）。ここで測って、
        // 乗っていなければ**その曲面を名指しして断ります**。
        //
        // **測ってから断るので、いま通っているものは動きません**——決め打ち
        // でも面が乗っているなら、これまでどおり通ります。
        let surface = Self::get_surface(ctx, id)?;
        if let Some(distance) = Self::boundary_off_surface(&surface, boundary_points) {
            let extent = boundary_extent_of(boundary_points).max(1.0);
            if distance > extent * 1e-3 {
                return Err(format!(
                    "Surface #{id} {} could not be rebuilt to cover its face: the boundary is off the fallback patch by {distance:.6e} (allowed {:.6e} for an extent of {extent:.6})",
                    raw.name,
                    extent * 1e-3
                ));
            }
        }
        Ok(surface)
    }

    /// 境界の点が、その曲面からどれだけ離れているか。曲面でなければ `None`。
    fn boundary_off_surface(surface: &FaceGeometry, boundary_points: &[Point3]) -> Option<f64> {
        let FaceGeometry::Nurbs(nurbs) = surface else {
            return None;
        };
        let mut worst = 0.0f64;
        for point in boundary_points {
            let projection =
                zenith_geom::ExtremumEngine::point_to_surface(*point, nurbs, 32, 1e-9).ok()?;
            worst = worst.max(projection.distance);
        }
        Some(worst)
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

        if raw.name == "CYLINDRICAL_SURFACE" {
            // CYLINDRICAL_SURFACE('',#axis2,radius)
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 3 {
                if let Some(axis2_id) = Self::parse_entity_ref(parts[1]) {
                    if let Ok(radius) = parts[2].parse::<f64>().map(|r| r * ctx.length_scale) {
                        if let Ok((origin, z_dir, x_dir)) = Self::get_axis2_placement(ctx, axis2_id)
                        {
                            let y_dir = z_dir.cross(&x_dir).normalize();
                            let weight = std::f64::consts::FRAC_1_SQRT_2;
                            // 90度円柱パッチ
                            let p0 = origin + x_dir * radius;
                            let p1 = origin + y_dir * radius;
                            let corner = origin + (x_dir + y_dir) * radius;
                            let row0 = vec![
                                ControlPoint3::unweighted(p0),
                                ControlPoint3::unweighted(p0 + z_dir),
                            ];
                            let row1 = vec![
                                ControlPoint3::new(corner, weight),
                                ControlPoint3::new(corner + z_dir, weight),
                            ];
                            let row2 = vec![
                                ControlPoint3::unweighted(p1),
                                ControlPoint3::unweighted(p1 + z_dir),
                            ];
                            if let Ok(nurbs) = NurbsSurface3::new(
                                2,
                                1,
                                vec![row0, row1, row2],
                                KnotVector::clamped_uniform(3, 2),
                                KnotVector::clamped_uniform(2, 1),
                            ) {
                                let geom = FaceGeometry::Nurbs(nurbs);
                                ctx.surfaces.insert(id, geom.clone());
                                return Ok(geom);
                            }
                        }
                    }
                }
            }
        }

        if raw.name == "CONICAL_SURFACE" {
            // CONICAL_SURFACE('',#axis2,radius,semi_angle)
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 4 {
                if let Some(axis2_id) = Self::parse_entity_ref(parts[1]) {
                    let radius = parts[2].parse::<f64>().unwrap_or(1.0) * ctx.length_scale;
                    let semi_angle = parts[3].parse::<f64>().unwrap_or(0.0);
                    if let Ok((origin, z_dir, x_dir)) = Self::get_axis2_placement(ctx, axis2_id) {
                        let y_dir = z_dir.cross(&x_dir).normalize();
                        let weight = std::f64::consts::FRAC_1_SQRT_2;
                        let r_top = radius + 1.0 * semi_angle.tan();
                        let p0_b = origin + x_dir * radius;
                        let p1_b = origin + y_dir * radius;
                        let c_b = origin + (x_dir + y_dir) * radius;
                        let p0_t = origin + z_dir + x_dir * r_top;
                        let p1_t = origin + z_dir + y_dir * r_top;
                        let c_t = origin + z_dir + (x_dir + y_dir) * r_top;
                        let row0 = vec![
                            ControlPoint3::unweighted(p0_b),
                            ControlPoint3::unweighted(p0_t),
                        ];
                        let row1 = vec![
                            ControlPoint3::new(c_b, weight),
                            ControlPoint3::new(c_t, weight),
                        ];
                        let row2 = vec![
                            ControlPoint3::unweighted(p1_b),
                            ControlPoint3::unweighted(p1_t),
                        ];
                        if let Ok(nurbs) = NurbsSurface3::new(
                            2,
                            1,
                            vec![row0, row1, row2],
                            KnotVector::clamped_uniform(3, 2),
                            KnotVector::clamped_uniform(2, 1),
                        ) {
                            let geom = FaceGeometry::Nurbs(nurbs);
                            ctx.surfaces.insert(id, geom.clone());
                            return Ok(geom);
                        }
                    }
                }
            }
        }

        if raw.name == "SPHERICAL_SURFACE" {
            // SPHERICAL_SURFACE('',#axis2,radius)
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 3 {
                if let Some(axis2_id) = Self::parse_entity_ref(parts[1]) {
                    if let Ok(radius) = parts[2].parse::<f64>().map(|r| r * ctx.length_scale) {
                        if let Ok((origin, z_dir, x_dir)) = Self::get_axis2_placement(ctx, axis2_id)
                        {
                            let y_dir = z_dir.cross(&x_dir).normalize();
                            let weight = std::f64::consts::FRAC_1_SQRT_2;
                            let p0 = origin + x_dir * radius;
                            let p1 = origin + y_dir * radius;
                            let p_top = origin + z_dir * radius;
                            let c_xy = origin + (x_dir + y_dir) * radius;
                            let row0 = vec![
                                ControlPoint3::unweighted(p0),
                                ControlPoint3::unweighted(p_top),
                            ];
                            let row1 = vec![
                                ControlPoint3::new(c_xy, weight),
                                ControlPoint3::new(origin + z_dir * radius, weight),
                            ];
                            let row2 = vec![
                                ControlPoint3::unweighted(p1),
                                ControlPoint3::unweighted(p_top),
                            ];
                            if let Ok(nurbs) = NurbsSurface3::new(
                                2,
                                1,
                                vec![row0, row1, row2],
                                KnotVector::clamped_uniform(3, 2),
                                KnotVector::clamped_uniform(2, 1),
                            ) {
                                let geom = FaceGeometry::Nurbs(nurbs);
                                ctx.surfaces.insert(id, geom.clone());
                                return Ok(geom);
                            }
                        }
                    }
                }
            }
        }

        if raw.name == "TOROIDAL_SURFACE" {
            // TOROIDAL_SURFACE('',#axis2,major_radius,minor_radius)
            let parts = Self::split_top_level_args(&raw.args);
            if parts.len() >= 4 {
                if let Some(axis2_id) = Self::parse_entity_ref(parts[1]) {
                    let major_r = parts[2].parse::<f64>().unwrap_or(10.0);
                    let minor_r = parts[3].parse::<f64>().unwrap_or(2.0);
                    if let Ok((origin, z_dir, x_dir)) = Self::get_axis2_placement(ctx, axis2_id) {
                        let y_dir = z_dir.cross(&x_dir).normalize();
                        let w = std::f64::consts::FRAC_1_SQRT_2;

                        // 90度 x 90度の有理2次 x 2次 トーラスパッチ
                        let mut grid = Vec::with_capacity(3);
                        for i in 0..3 {
                            let mut row = Vec::with_capacity(3);
                            let (dir_u, w_u) = match i {
                                0 => (x_dir, 1.0),
                                1 => ((x_dir + y_dir), w),
                                _ => (y_dir, 1.0),
                            };
                            let c_u = origin + dir_u * major_r;

                            for j in 0..3 {
                                let (offset_v, w_v) = match j {
                                    0 => (dir_u * minor_r, 1.0),
                                    1 => ((dir_u + z_dir) * minor_r, w),
                                    _ => (z_dir * minor_r, 1.0),
                                };
                                let pt = c_u + offset_v;
                                row.push(ControlPoint3::new(pt, w_u * w_v));
                            }
                            grid.push(row);
                        }

                        if let Ok(nurbs) = NurbsSurface3::new(
                            2,
                            2,
                            grid,
                            KnotVector::clamped_uniform(3, 2),
                            KnotVector::clamped_uniform(3, 2),
                        ) {
                            let geom = FaceGeometry::Nurbs(nurbs);
                            ctx.surfaces.insert(id, geom.clone());
                            return Ok(geom);
                        }
                    }
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

        // ここは既定の平面（原点を通る Y-Z 平面）を返していた。読めなかった
        // 曲面を、まったく別の平面に差し替えて返すということである。
        //
        // 実測では、その先の p-curve 検証が「境界が面から 4.0e1 離れている」と
        // 言って必ず落とすので、**誤答にはならず、クリーンなエラーになって
        // いた**。ただしそのエラーは「p-curve が退化している」としか言わない
        // ので、読めなかった理由——対応していない曲面型に当たったこと——が
        // 呼び出し側に伝わらない。原因の分からないエラーは、追いかける人に
        // 幾何の疑いを持たせる。ここで名指しする。
        Err(format!(
            "Unsupported surface entity {} (#{}). Zenith reads PLANE, \
             CYLINDRICAL_SURFACE, CONICAL_SURFACE, SPHERICAL_SURFACE, \
             TOROIDAL_SURFACE, SURFACE_OF_LINEAR_EXTRUSION, SURFACE_OF_REVOLUTION \n             and (rational) B_SPLINE_SURFACE_WITH_KNOTS",
            raw.name, id
        ))
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
        Solid::try_simple(outer_shell, &Self::import_tolerance(ctx)).map_err(|err| err.to_string())
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

        Solid::try_new(outer_shell, inner_shells, &Self::import_tolerance(ctx))
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

            // 2. Bounds の取得
            //
            // FACE_OUTER_BOUND は FACE_BOUND の subtype であって必須ではない。
            // OpenCASCADE をはじめ多くの書き手はすべての境界を FACE_BOUND として
            // 出すので、印が付いていなければ幾何から外周を決める必要がある。
            // これを必須としていたため、他カーネルの STEP が一切読めなかった。
            let mut unmarked_wires: Vec<Wire> = Vec::new();
            // 辺を持たない境界からは形が取れないので、頂点だけは別に控えておく。
            // 曲面をどう張るかを決めるのに、この一点でも位置の手掛かりになる。
            let mut vertex_loop_points: Vec<Point3> = Vec::new();
            for b_id in Self::parse_ref_list(parts[1]) {
                let bound_raw = ctx
                    .raw_entities
                    .get(&b_id)
                    .ok_or("Bound not found")?
                    .clone();
                let b_parts = Self::split_top_level_args(&bound_raw.args);
                if b_parts.len() >= 2 {
                    let loop_id = Self::parse_entity_ref(b_parts[1]).ok_or("loop ref")?;
                    if let Some(point) = Self::vertex_loop_point(ctx, loop_id) {
                        vertex_loop_points.push(point);
                    }
                    let mut wire = Self::get_wire(ctx, loop_id)?;
                    // FACE_BOUND の向きフラグ。.F. のとき、ループは書かれている
                    // のと逆向きに面を囲む。これを無視すると、辺が隣り合う面から
                    // 同じ向きに2度使われ、シェル検証が対で見つけられなくなる。
                    // **2つの旗を並べて出します**（4-280）。面の向き
                    // （`ADVANCED_FACE` の `same_sense`）と、境界の向き
                    // （`FACE_BOUND` の旗）は**別のもの**で、取り込みは後者だけを
                    // ワイヤに適用します。前者は `Face::orientation` に入ります。
                    if std::env::var_os("ZENITH_ORIENT_WHY").is_some() {
                        eprintln!(
                            "ORIENTWHY 取り込み: 面 #{face_id} の {} 旗 {}（面の same_sense {}）",
                            bound_raw.name,
                            b_parts.get(2).map(|p| p.trim()).unwrap_or("?"),
                            if same_sense { ".T." } else { ".F." }
                        );
                    }
                    if b_parts.len() >= 3 && b_parts[2].trim() == ".F." {
                        wire = Wire::new(
                            wire.edges
                                .iter()
                                .rev()
                                .map(|oriented| {
                                    OrientedEdge::new(
                                        oriented.edge.clone(),
                                        oriented.orientation.reversed(),
                                    )
                                })
                                .collect(),
                        );
                    }
                    if bound_raw.name == "FACE_OUTER_BOUND" {
                        outer_wire = Some(wire);
                    } else {
                        unmarked_wires.push(wire);
                    }
                }
            }

            match outer_wire {
                // 印付きの外周があるなら、残りはすべて穴。
                Some(_) => inner_wires.extend(unmarked_wires),
                None => {
                    if unmarked_wires.is_empty() {
                        return Err(format!("No bounds at all for face #{}", face_id));
                    }
                    // 外周は他のすべてを囲むループ。境界の広がりで選ぶ。
                    let outer_index = unmarked_wires
                        .iter()
                        .enumerate()
                        .max_by(|(_, left), (_, right)| {
                            wire_extent(left).total_cmp(&wire_extent(right))
                        })
                        .map(|(index, _)| index)
                        .unwrap_or(0);
                    let chosen = unmarked_wires.remove(outer_index);
                    outer_wire = Some(chosen);
                    inner_wires.extend(unmarked_wires);
                }
            }

            let outer =
                outer_wire.ok_or_else(|| format!("No outer bound for face #{}", face_id))?;

            // 曲面は境界が決まってから組む。STEP の解析曲面（円柱・円錐など）は
            // 無限に伸びた形で書かれ、どこを使うかは面の境界だけが決めるので、
            // 境界を見ずに固定サイズのパッチを作ると境界が曲面から外れる。
            let mut boundary_points = outer.sample_points(24);
            for wire in &inner_wires {
                boundary_points.extend(wire.sample_points(12));
            }
            boundary_points.extend(vertex_loop_points);
            let geom = Self::get_surface_for_boundary(ctx, surface_id, &boundary_points)?;

            let orientation = if same_sense {
                Orientation::Forward
            } else {
                Orientation::Reversed
            };
            // **旗と、実際の巻き方を並べます**（4-280）。
            //
            // 規約はこうです——境界は**面の法線**まわりに反時計回り。面の法線は
            // `same_sense` が `.F.` なら曲面の法線の逆。したがって**曲面の法線
            // まわりでは、`.T.` は反時計回り、`.F.` は時計回り**のはずです。
            // 破れている面をここで名指しします。
            if std::env::var_os("ZENITH_ORIENT_WHY").is_some() {
                if let FaceGeometry::Plane(plane) = &geom {
                    let points = outer.sample_points(24);
                    let mut normal = zenith_math::Vec3::zeros();
                    for index in 0..points.len() {
                        let a = points[index];
                        let b = points[(index + 1) % points.len()];
                        normal += a.coords.cross(&b.coords);
                    }
                    if normal.norm() > 0.0 {
                        let agreement = normal.normalize().dot(&plane.normal.normalize());
                        let expected_positive = same_sense;
                        let holds = (agreement > 0.0) == expected_positive;
                        eprintln!(
                            "ORIENTWHY 面 #{face_id}: same_sense {}、巻きと曲面の法線の内積 {agreement:+.4} → {}",
                            if same_sense { ".T." } else { ".F." },
                            if holds { "規約どおり" } else { "**規約が破れています**" }
                        );
                    }
                }
            }
            // **面ごとに、境界の点が曲面からどれだけ離れているかを出す口**
            // （`ZENITH_STEP_WHY=1`）。読めなかったファイルで、悪いのが曲線
            // なのか面なのかを分けたあと、**どの曲面の実体か**まで名指しする
            // ためです（4-228）。
            if std::env::var_os("ZENITH_STEP_WHY").is_some() {
                let kind = ctx
                    .raw_entities
                    .get(&surface_id)
                    .map(|raw| raw.name.clone())
                    .unwrap_or_else(|| "?".to_string());
                let probe = Face::new(
                    geom.clone(),
                    outer.clone(),
                    inner_wires.clone(),
                    orientation,
                    1e-6,
                );
                let report = probe.validate_boundary_on_surface(&Self::import_tolerance(ctx), 8);
                if report.max_distance > 1e-6 {
                    eprintln!(
                        "STEPWHY ADVANCED_FACE #{face_id} 曲面 #{surface_id} {kind} 境界の点が曲面から最大 {:.6e}（{} / {} 点が外れ）",
                        report.max_distance,
                        report.off_surface_point_count,
                        report.sampled_point_count
                    );
                }
            }

            // **その面がどこまで正しいかを、測って持たせます**（4-266）。
            //
            // これまでは 1e-6 の決め打ちでした。ところが実務のファイルは、
            // **自分の申告より粗い**ことがあります——実測（4-265）:
            // `screw.step` は不確かさ 1e-6 と宣言しながら境界が 2.946e-4
            // ずれ（**294 倍**）、`linkrods.step` は 2e-5 宣言で 4.145e-4
            // （**20 倍**）でした。
            //
            // **全体の公差を緩めるのではなく、その面に書いておきます。**
            // 下流の検証は `tol.linear.max(face.tolerance)` を使うので
            // （`validate_boundary_on_surface`）、**ビルダーの出力は 1e-6 の
            // まま**で、読んだ面だけがその粗さを持ち歩きます。
            //
            // **上限は `import_tolerance` の天井（1e-3）です。** それを超える
            // 面は、公差の話ではなく**別の曲面**なので、これまでどおり断ります
            // （4-263 が名指しします）。
            let measured = {
                let probe = Face::new(
                    geom.clone(),
                    outer.clone(),
                    inner_wires.clone(),
                    orientation,
                    1e-6,
                );
                probe
                    .validate_boundary_on_surface(&Self::import_tolerance(ctx), 8)
                    .max_distance
            };
            let carried = if measured.is_finite() {
                measured.max(1e-6).min(Self::import_ceiling())
            } else {
                1e-6
            };
            if std::env::var_os("ZENITH_STEP_WHY").is_some() && carried > 1e-6 {
                eprintln!(
                    "STEPWHY ADVANCED_FACE #{face_id} は自分の粗さ {carried:.6e} を持ち歩きます（実測 {measured:.6e}）"
                );
            }
            let face = Face::new(geom, outer, inner_wires, orientation, carried);
            ctx.faces.insert(face_id, face.clone());
            return Ok(face);
        }

        Err(format!("Invalid ADVANCED_FACE #{}", face_id))
    }
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
    use super::{
        angular_span, cone_patch_for_boundary, sphere_patch_for_boundary, torus_patch_for_boundary,
        ImportContext, RawEntity, StepImporter,
    };
    use zenith_geom::NurbsSurface3;
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

    #[test]
    fn imports_cylindrical_and_conical_surfaces_as_exact_nurbs() {
        let mut ctx = ImportContext::new();
        ctx.raw_entities.insert(1, point_entity(0.0, 0.0, 0.0));
        ctx.raw_entities.insert(2, direction_entity(0.0, 0.0, 1.0));
        ctx.raw_entities.insert(3, direction_entity(1.0, 0.0, 0.0));
        ctx.raw_entities.insert(
            10,
            RawEntity {
                name: "AXIS2_PLACEMENT_3D".to_string(),
                args: "'',#1,#2,#3".to_string(),
            },
        );
        ctx.raw_entities.insert(
            20,
            RawEntity {
                name: "CYLINDRICAL_SURFACE".to_string(),
                args: "'',#10,15.0".to_string(),
            },
        );
        ctx.raw_entities.insert(
            21,
            RawEntity {
                name: "CONICAL_SURFACE".to_string(),
                args: "'',#10,12.0,0.5".to_string(),
            },
        );

        ctx.raw_entities.insert(
            22,
            RawEntity {
                name: "TOROIDAL_SURFACE".to_string(),
                args: "'',#10,20.0,5.0".to_string(),
            },
        );

        let cyl_geom = StepImporter::get_surface(&mut ctx, 20).expect("cylindrical surface import");
        match cyl_geom {
            FaceGeometry::Nurbs(s) => {
                assert_eq!(s.degree_u, 2);
                assert_eq!(s.degree_v, 1);
            }
            _ => panic!("Expected Nurbs geometry for cylindrical surface"),
        }

        let cone_geom = StepImporter::get_surface(&mut ctx, 21).expect("conical surface import");
        match cone_geom {
            FaceGeometry::Nurbs(s) => {
                assert_eq!(s.degree_u, 2);
                assert_eq!(s.degree_v, 1);
            }
            _ => panic!("Expected Nurbs geometry for conical surface"),
        }

        let torus_geom = StepImporter::get_surface(&mut ctx, 22).expect("toroidal surface import");
        match torus_geom {
            FaceGeometry::Nurbs(s) => {
                assert_eq!(s.degree_u, 2);
                assert_eq!(s.degree_v, 2);
            }
            _ => panic!("Expected Nurbs geometry for toroidal surface"),
        }
    }

    fn direction_entity(x: f64, y: f64, z: f64) -> RawEntity {
        RawEntity {
            name: "DIRECTION".to_string(),
            args: format!("'',({:.6},{:.6},{:.6})", x, y, z),
        }
    }

    /// Samples a patch on a grid and reports the worst deviation from the
    /// analytic surface the patch is meant to be.
    fn worst_deviation(surface: &NurbsSurface3, distance: impl Fn(Point3) -> f64) -> f64 {
        let mut worst: f64 = 0.0;
        for i in 0..=16 {
            for j in 0..=16 {
                let point = surface.evaluate(i as f64 / 16.0, j as f64 / 16.0);
                worst = worst.max(distance(point).abs());
            }
        }
        worst
    }

    #[test]
    fn conical_patch_matches_the_cone_it_was_sized_from() {
        // 半径10、半角0.291456794478 rad (r=4 at z=20) の円錐。境界は上下の円。
        let semi_angle: f64 = 0.291456794478;
        let slope = semi_angle.tan();
        let mut boundary = Vec::new();
        for step in 0..24 {
            let angle = std::f64::consts::PI * 2.0 * step as f64 / 24.0;
            for z in [0.0, 20.0] {
                let radius = 10.0 + z * slope;
                boundary.push(Point3::new(radius * angle.cos(), radius * angle.sin(), z));
            }
        }

        let patch = cone_patch_for_boundary(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            10.0,
            semi_angle,
            &boundary,
        )
        .expect("cone patch");

        let deviation = worst_deviation(&patch, |point| {
            let expected = 10.0 + point.z * slope;
            (point.x * point.x + point.y * point.y).sqrt() - expected
        });
        assert!(deviation < 1e-9, "cone deviation {deviation}");
    }

    #[test]
    fn conical_patch_closes_on_the_apex() {
        let semi_angle = (10.0f64 / 20.0).atan();
        let mut boundary = Vec::new();
        for step in 0..24 {
            let angle = std::f64::consts::PI * 2.0 * step as f64 / 24.0;
            boundary.push(Point3::new(10.0 * angle.cos(), 10.0 * angle.sin(), 0.0));
        }
        boundary.push(Point3::new(0.0, 0.0, -20.0));

        let patch = cone_patch_for_boundary(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            10.0,
            semi_angle,
            &boundary,
        )
        .expect("cone patch through the apex");

        let apex = patch.evaluate(0.5, 0.0);
        assert!(
            (apex - Point3::new(0.0, 0.0, -20.0)).norm() < 1e-9,
            "apex landed at {apex:?}"
        );
    }

    #[test]
    fn spherical_patch_matches_the_sphere_it_was_sized_from() {
        // 北半球。境界は赤道の円だけ。
        let mut boundary = Vec::new();
        for step in 0..24 {
            let angle = std::f64::consts::PI * 2.0 * step as f64 / 24.0;
            boundary.push(Point3::new(10.0 * angle.cos(), 10.0 * angle.sin(), 0.0));
        }
        boundary.push(Point3::new(0.0, 0.0, 10.0));

        let patch = sphere_patch_for_boundary(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            10.0,
            &boundary,
        )
        .expect("sphere patch");

        let deviation = worst_deviation(&patch, |point| point.coords.norm() - 10.0);
        assert!(deviation < 1e-9, "sphere deviation {deviation}");
    }

    #[test]
    fn toroidal_patch_matches_the_torus_it_was_sized_from() {
        // 90度のトーラス区分。境界は継ぎ目の主円弧と両端の副円。
        let (major, minor) = (12.0f64, 4.0f64);
        let point_on_torus = |major_angle: f64, minor_angle: f64| {
            let distance = major + minor * minor_angle.cos();
            Point3::new(
                distance * major_angle.cos(),
                distance * major_angle.sin(),
                minor * minor_angle.sin(),
            )
        };

        let mut boundary = Vec::new();
        for step in 0..24 {
            let along = std::f64::consts::FRAC_PI_2 * step as f64 / 24.0;
            boundary.push(point_on_torus(along, 0.0));
        }
        for step in 0..24 {
            let around = std::f64::consts::PI * 2.0 * step as f64 / 24.0;
            boundary.push(point_on_torus(0.0, around));
            boundary.push(point_on_torus(std::f64::consts::FRAC_PI_2, around));
        }

        let patch = torus_patch_for_boundary(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            major,
            minor,
            &boundary,
        )
        .expect("torus patch");

        let deviation = worst_deviation(&patch, |point| {
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            ((radial - major).powi(2) + point.z * point.z).sqrt() - minor
        });
        assert!(deviation < 1e-9, "torus deviation {deviation}");
    }

    #[test]
    fn analytic_patch_refuses_a_boundary_that_is_not_on_the_surface() {
        let boundary = vec![
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 5.0),
            Point3::new(-7.0, 0.0, 10.0), // 半径が合わない
        ];
        assert!(sphere_patch_for_boundary(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            10.0,
            &boundary,
        )
        .is_none());
    }

    #[test]
    fn a_gap_no_wider_than_the_sampling_is_not_a_gap() {
        // 15度おきの標本は15度の隙間を空ける。それを穴と読むと面が閉じない。
        let mut even = (0..24)
            .map(|step| std::f64::consts::PI * 2.0 * step as f64 / 24.0)
            .collect::<Vec<_>>();
        let (_, sweep) = angular_span(&mut even);
        assert!(
            (sweep - std::f64::consts::PI * 2.0).abs() < 1e-12,
            "even sampling reported sweep {sweep}"
        );

        // 本物の隙間は標本間隔から明確に離れている。
        let mut quarter = (0..24)
            .map(|step| std::f64::consts::FRAC_PI_2 * step as f64 / 24.0)
            .collect::<Vec<_>>();
        let (start, sweep) = angular_span(&mut quarter);
        assert!((sweep - std::f64::consts::FRAC_PI_2 * 23.0 / 24.0).abs() < 1e-12);
        assert!(start.rem_euclid(std::f64::consts::PI * 2.0) < 1e-12);
    }
}

/// Diagonal of a wire's bounding box, as a cheap stand-in for how much of the
/// face it encloses.
///
/// The outer bound is the one containing the others, and on a well-formed face
/// that is also the one with the largest extent. Comparing extents avoids
/// needing the surface's parameterisation, which is not always available at
/// this point in the read.
fn wire_extent(wire: &Wire) -> f64 {
    let points = wire.sample_points(8);
    if points.is_empty() {
        return 0.0;
    }

    let mut min_pt = points[0];
    let mut max_pt = points[0];
    for point in &points {
        min_pt.x = min_pt.x.min(point.x);
        min_pt.y = min_pt.y.min(point.y);
        min_pt.z = min_pt.z.min(point.z);
        max_pt.x = max_pt.x.max(point.x);
        max_pt.y = max_pt.y.max(point.y);
        max_pt.z = max_pt.z.max(point.z);
    }

    (max_pt - min_pt).norm()
}

/// Builds the arc of a circle that runs from `p_start` to `p_end`, taking the
/// circle's own frame as the authority.
///
/// Coincident endpoints mean the full circle, which is how every writer states
/// the rim of a cylinder; inferring the sweep from the endpoints alone reads
/// that as a zero-length arc.
fn arc_from_angles(
    center: Point3,
    normal: Vec3,
    ref_dir: Vec3,
    radius: f64,
    p_start: Point3,
    p_end: Point3,
) -> Result<NurbsCurve3, String> {
    let axis = normal
        .try_normalize(1e-12)
        .ok_or_else(|| "Circle axis is degenerate".to_string())?;
    let x_axis = {
        let projected = ref_dir - axis * ref_dir.dot(&axis);
        projected
            .try_normalize(1e-12)
            .ok_or_else(|| "Circle reference direction is parallel to its axis".to_string())?
    };
    let y_axis = axis.cross(&x_axis);

    let angle_of = |point: Point3| {
        let offset = point - center;
        offset.dot(&y_axis).atan2(offset.dot(&x_axis))
    };

    let start_angle = angle_of(p_start);
    let mut sweep = angle_of(p_end) - start_angle;
    let full_turn = std::f64::consts::PI * 2.0;
    while sweep <= 1e-9 {
        sweep += full_turn;
    }
    // 端点が一致していれば完全円。上のループで 2*pi になっている。
    if (p_end - p_start).norm() <= 1e-9 {
        sweep = full_turn;
    }

    rational_arc(center, x_axis, y_axis, radius, start_angle, sweep)
}

/// A rational quadratic arc of any sweep, as a clamped multi-span NURBS.
///
/// Each span covers at most a quarter turn, which is the widest a single
/// rational quadratic segment represents exactly.
fn rational_arc(
    center: Point3,
    x_axis: Vec3,
    y_axis: Vec3,
    radius: f64,
    start_angle: f64,
    sweep: f64,
) -> Result<NurbsCurve3, String> {
    rational_elliptic_arc(center, x_axis, y_axis, radius, radius, start_angle, sweep)
}

/// 楕円弧を有理2次の NURBS として厳密に組む。
///
/// 楕円は単位円の像である（`X` 方向に `a`、`Y` 方向に `b` 伸ばしたもの）。
/// 有理 NURBS はアフィン変換のもとで不変——制御点を写して重みをそのまま
/// 使えば同じ曲線になる——ので、円の構成の半径を軸ごとに置き換えるだけで、
/// 近似ではなく**厳密な**楕円が得られる。
fn rational_elliptic_arc(
    center: Point3,
    x_axis: Vec3,
    y_axis: Vec3,
    radius_x: f64,
    radius_y: f64,
    start_angle: f64,
    sweep: f64,
) -> Result<NurbsCurve3, String> {
    let radius = radius_x;
    let scale_y = radius_y / radius_x;
    let y_axis = y_axis * scale_y;
    // ちょうど四半円のとき、除算の丸めで商が 1 をわずかに超えて 2 区間に
    // 割れてしまう。許容差を引いてから切り上げる。
    let span_count = (((sweep / std::f64::consts::FRAC_PI_2) - 1e-9).ceil() as usize).max(1);
    let span_angle = sweep / span_count as f64;
    let weight = (span_angle * 0.5).cos();

    let point_at =
        |angle: f64| center + x_axis * (radius * angle.cos()) + y_axis * (radius * angle.sin());
    // 接線の交点。半径 / cos(half) の距離に、区間の中央方向へ置く。
    let shoulder_at = |angle: f64| {
        let middle = angle + span_angle * 0.5;
        center
            + x_axis * (radius / weight * middle.cos())
            + y_axis * (radius / weight * middle.sin())
    };

    let mut control_points = Vec::with_capacity(span_count * 2 + 1);
    control_points.push(ControlPoint3::unweighted(point_at(start_angle)));
    for span in 0..span_count {
        let angle = start_angle + span_angle * span as f64;
        control_points.push(ControlPoint3::new(shoulder_at(angle), weight));
        control_points.push(ControlPoint3::unweighted(point_at(angle + span_angle)));
    }

    let mut knots = vec![0.0, 0.0, 0.0];
    for span in 1..span_count {
        let value = span as f64 / span_count as f64;
        knots.push(value);
        knots.push(value);
    }
    knots.extend([1.0, 1.0, 1.0]);

    NurbsCurve3::new(2, control_points, KnotVector::new(knots))
}

/// The angular range a set of boundary points occupies around an axis.
///
/// Returned as `(start, sweep)` in radians. The range in use is whatever lies
/// outside the widest gap between neighbouring angles: a boundary that goes all
/// the way round leaves no gap and reports a full turn.
///
/// The widest gap is judged against the next widest rather than against a
/// spacing guessed from the sample count. Sampling runs per edge, so how far
/// apart samples land in this direction depends on which way each edge runs,
/// and the guess was wrong often enough to cut a full turn short: a torus's two
/// cap circles are sampled every 15 degrees, and a 15 degree step between
/// samples read as a 15 degree hole in the face. A real gap stands well clear
/// of the ordinary spacing; a sampling step does not.
fn angular_span(angles: &mut Vec<f64>) -> (f64, f64) {
    let full_turn = std::f64::consts::PI * 2.0;
    if angles.len() < 2 {
        return (0.0, full_turn);
    }

    angles.sort_by(f64::total_cmp);
    let mut gaps: Vec<(f64, f64)> = Vec::with_capacity(angles.len());
    gaps.push((
        angles[0] + full_turn - angles[angles.len() - 1],
        angles[angles.len() - 1],
    ));
    for window in angles.windows(2) {
        gaps.push((window[1] - window[0], window[0]));
    }
    gaps.sort_by(|left, right| right.0.total_cmp(&left.0));

    let (widest_gap, gap_start) = gaps[0];
    let next_widest = gaps.get(1).map(|entry| entry.0).unwrap_or(0.0);
    if widest_gap <= next_widest * 3.0 {
        return (0.0, full_turn);
    }

    let sweep = full_turn - widest_gap;
    if sweep <= 1e-9 {
        // 角度が一点に潰れている。閉じた曲面の継ぎ目なので一周とみなす。
        return (0.0, full_turn);
    }
    (gap_start + widest_gap, sweep)
}

/// Knots for a quadratic arc built from `span_count` equal spans.
fn quadratic_arc_knots(span_count: usize) -> KnotVector {
    let mut knots = vec![0.0, 0.0, 0.0];
    for span in 1..span_count {
        let value = span as f64 / span_count as f64;
        knots.push(value);
        knots.push(value);
    }
    knots.extend([1.0, 1.0, 1.0]);
    KnotVector::new(knots)
}

/// How many 90-degree-or-less spans a sweep needs.
fn arc_span_count(sweep: f64) -> usize {
    (((sweep / std::f64::consts::FRAC_PI_2) - 1e-9).ceil() as usize).max(1)
}

/// A rational quadratic arc, given as `(radial, axial, weight)` triples in the
/// plane a surface's radial and axial directions span, with the knots that go
/// with it. Angle zero points along the radial direction.
///
/// This is the profile half of a surface of revolution: sphere and torus
/// patches differ only in which circle this traces.
fn arc_profile(
    centre_radial: f64,
    centre_axial: f64,
    radius: f64,
    start_angle: f64,
    sweep: f64,
) -> (Vec<(f64, f64, f64)>, KnotVector) {
    let span_count = arc_span_count(sweep);
    let span_angle = sweep / span_count as f64;
    let weight = (span_angle * 0.5).cos();

    let on_arc = |angle: f64, scale: f64| {
        (
            centre_radial + radius * scale * angle.cos(),
            centre_axial + radius * scale * angle.sin(),
        )
    };

    let mut profile = Vec::with_capacity(span_count * 2 + 1);
    let (r, a) = on_arc(start_angle, 1.0);
    profile.push((r, a, 1.0));
    for span in 0..span_count {
        let angle = start_angle + span_angle * span as f64;
        let (r, a) = on_arc(angle + span_angle * 0.5, 1.0 / weight);
        profile.push((r, a, weight));
        let (r, a) = on_arc(angle + span_angle, 1.0);
        profile.push((r, a, 1.0));
    }

    (profile, quadratic_arc_knots(span_count))
}

/// Sweeps a profile around an axis to give an exact rational surface.
///
/// The profile is `(radial, axial, weight)` in the axis frame and becomes the v
/// direction; the swept angle becomes u. Shoulder rows sit where the tangents
/// of neighbouring spans meet, at `radius / cos(half span)` from the axis,
/// which is what makes the sweep a circle rather than an approximation of one.
fn revolve_profile(
    origin: Point3,
    axis: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    profile: &[(f64, f64, f64)],
    profile_knots: KnotVector,
    profile_degree: usize,
    start_angle: f64,
    sweep: f64,
) -> Option<NurbsSurface3> {
    if profile.len() < 2 {
        return None;
    }
    let span_count = arc_span_count(sweep);
    let span_angle = sweep / span_count as f64;
    let weight = (span_angle * 0.5).cos();

    let row_at = |angle: f64, radial_scale: f64, row_weight: f64| {
        profile
            .iter()
            .map(|&(radial, axial, profile_weight)| {
                let point = origin
                    + axis * axial
                    + x_axis * (radial * radial_scale * angle.cos())
                    + y_axis * (radial * radial_scale * angle.sin());
                ControlPoint3::new(point, row_weight * profile_weight)
            })
            .collect::<Vec<_>>()
    };

    let mut rows = Vec::with_capacity(span_count * 2 + 1);
    rows.push(row_at(start_angle, 1.0, 1.0));
    for span in 0..span_count {
        let angle = start_angle + span_angle * span as f64;
        rows.push(row_at(angle + span_angle * 0.5, 1.0 / weight, weight));
        rows.push(row_at(angle + span_angle, 1.0, 1.0));
    }

    NurbsSurface3::new(
        2,
        profile_degree,
        rows,
        quadratic_arc_knots(span_count),
        profile_knots,
    )
    .ok()
}

/// Splits a point into its distance along an axis and its offset from it.
fn axis_frame_coords(point: Point3, origin: Point3, axis: Vec3) -> (f64, Vec3) {
    let offset = point - origin;
    let axial = offset.dot(&axis);
    (axial, offset - axis * axial)
}

/// The axis frame of an analytic surface: axis, and two directions across it.
fn revolution_frame(z_dir: Vec3, x_dir: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
    let axis = z_dir.try_normalize(1e-12)?;
    let x_axis = (x_dir - axis * x_dir.dot(&axis)).try_normalize(1e-12)?;
    Some((axis, x_axis, axis.cross(&x_axis)))
}

/// A cylindrical patch covering the angular and axial span a face occupies.
///
/// Returns `None` when the boundary does not sit on the cylinder, so a
/// mismatch is refused rather than approximated.
fn cylinder_patch_for_boundary(
    origin: Point3,
    z_dir: Vec3,
    x_dir: Vec3,
    radius: f64,
    boundary_points: &[Point3],
) -> Option<NurbsSurface3> {
    let (axis, x_axis, y_axis) = revolution_frame(z_dir, x_dir)?;

    let mut axial_min = f64::INFINITY;
    let mut axial_max = f64::NEG_INFINITY;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());

    for point in boundary_points {
        let (axial, radial) = axis_frame_coords(*point, origin, axis);
        if (radial.norm() - radius).abs() > radius.abs().max(1.0) * 1e-3 {
            return None;
        }
        axial_min = axial_min.min(axial);
        axial_max = axial_max.max(axial);
        angles.push(radial.dot(&y_axis).atan2(radial.dot(&x_axis)));
    }

    if !axial_min.is_finite() || !axial_max.is_finite() {
        return None;
    }
    if (axial_max - axial_min).abs() <= 1e-12 {
        return None;
    }

    let (start_angle, sweep) = angular_span(&mut angles);
    let profile = [(radius, axial_min, 1.0), (radius, axial_max, 1.0)];
    revolve_profile(
        origin,
        axis,
        x_axis,
        y_axis,
        &profile,
        KnotVector::clamped_uniform(2, 1),
        1,
        start_angle,
        sweep,
    )
}

/// A conical patch covering the angular and axial span a face occupies.
///
/// A cone is ruled, so the profile is still a straight segment; only the radius
/// now depends on how far along the axis the row sits. A face that runs to the
/// apex gives a zero radius at that end, which is a degenerate row rather than
/// a failure.
/// 境界の点が広がっている大きさ。判定の尺度に使います。
fn boundary_extent_of(points: &[Point3]) -> f64 {
    let mut worst = 0.0f64;
    for (index, a) in points.iter().enumerate() {
        for b in points.iter().skip(index + 1) {
            worst = worst.max((a - b).norm());
        }
    }
    worst
}

fn cone_patch_for_boundary(
    origin: Point3,
    z_dir: Vec3,
    x_dir: Vec3,
    radius: f64,
    semi_angle: f64,
    boundary_points: &[Point3],
) -> Option<NurbsSurface3> {
    let (axis, x_axis, y_axis) = revolution_frame(z_dir, x_dir)?;
    let slope = semi_angle.tan();
    if !slope.is_finite() {
        return None;
    }
    // 頂点では半径が 0 になるので、判定の尺度は基準半径から取る。
    let scale = radius.abs().max(1.0);

    // **頂点の向こう側の面**（4-261）。
    //
    // STEP の円錐は無限で、`半径 + 軸方向 × 勾配` が**負になる側**も面です
    // （頂点を越えた反対の葉）。そこにある面を素直に測ると「半径 7.5 なのに
    // あるべき値が −7.5」となり、**差 15 で断って**いました。実測:
    // `screw.step` の `CONICAL_SURFACE` 2枚がこれで、決め打ちの 90 度パッチに
    // 落ちて境界が 13〜24 外れていました（4-260）。
    //
    // **裏返して同じ道に乗せます。** 半径と勾配の符号を反転し、角度を π
    // ずらせば、あとの組み立ては何も変わりません。**境界の点が全部向こう側の
    // ときだけ**裏返します——跨いでいる面は頂点を含むので、これまでどおり
    // 断ります。
    let flip = !boundary_points.is_empty()
        && boundary_points.iter().all(|point| {
            let (axial, _) = axis_frame_coords(*point, origin, axis);
            radius + axial * slope < -scale * 1e-6
        });
    let (radius, slope, angle_offset) = if flip {
        (-radius, -slope, std::f64::consts::PI)
    } else {
        (radius, slope, 0.0)
    };

    let mut axial_min = f64::INFINITY;
    let mut axial_max = f64::NEG_INFINITY;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());

    for point in boundary_points {
        let (axial, radial) = axis_frame_coords(*point, origin, axis);
        let expected = radius + axial * slope;
        if (radial.norm() - expected).abs() > scale * 1e-3 {
            // **境界の点が円錐の上に乗っていません**（`ZENITH_STEP_WHY=1`。4-261）。
            //
            // トーラス側には同じ口があるのに、**円錐だけ理由を1行も出して
            // いませんでした**（4-260 の診断の穴）。「決め打ちのパッチに
            // 落ちた」までは出るのに、**なぜ**が出ないと次に測るところが
            // 決まりません。
            if std::env::var_os("ZENITH_STEP_WHY").is_some() {
                eprintln!(
                    "STEPWHY   円錐（基準半径 {radius:.6}、勾配 {slope:.6}）の上に乗らない点: 軸方向 {axial:.6} での半径 {:.6}（あるべき値 {expected:.6}、差 {:.6e}、許す幅 {:.6e}）",
                    radial.norm(),
                    (radial.norm() - expected).abs(),
                    scale * 1e-3
                );
            }
            return None;
        }
        axial_min = axial_min.min(axial);
        axial_max = axial_max.max(axial);
        // 頂点上の点には向きが無い。角度の被覆には数えない。
        if radial.norm() > scale * 1e-6 {
            angles.push(radial.dot(&y_axis).atan2(radial.dot(&x_axis)) - angle_offset);
        }
    }

    if !axial_min.is_finite() || !axial_max.is_finite() {
        return None;
    }
    if (axial_max - axial_min).abs() <= 1e-12 {
        return None;
    }

    let radius_min = radius + axial_min * slope;
    let radius_max = radius + axial_max * slope;
    if radius_min < -scale * 1e-6 || radius_max < -scale * 1e-6 {
        return None;
    }

    let (start_angle, sweep) = angular_span(&mut angles);
    let profile = [
        (radius_min.max(0.0), axial_min, 1.0),
        (radius_max.max(0.0), axial_max, 1.0),
    ];
    revolve_profile(
        origin,
        axis,
        x_axis,
        y_axis,
        &profile,
        KnotVector::clamped_uniform(2, 1),
        1,
        start_angle + angle_offset,
        sweep,
    )
}

/// A spherical patch covering the longitude and latitude a face occupies.
///
/// Latitude comes from the extent of the boundary rather than from a gap: a
/// sphere is not closed in that direction, so the patch has to reach whichever
/// parallels the boundary touches, and reaching past them is harmless.
fn sphere_patch_for_boundary(
    origin: Point3,
    z_dir: Vec3,
    x_dir: Vec3,
    radius: f64,
    boundary_points: &[Point3],
) -> Option<NurbsSurface3> {
    let (axis, x_axis, y_axis) = revolution_frame(z_dir, x_dir)?;
    let scale = radius.abs().max(1.0);
    let half_pi = std::f64::consts::FRAC_PI_2;

    let mut latitude_min = f64::INFINITY;
    let mut latitude_max = f64::NEG_INFINITY;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());

    for point in boundary_points {
        if ((*point - origin).norm() - radius).abs() > scale * 1e-3 {
            return None;
        }
        let (axial, radial) = axis_frame_coords(*point, origin, axis);
        let latitude = axial.atan2(radial.norm());
        latitude_min = latitude_min.min(latitude);
        latitude_max = latitude_max.max(latitude);
        if radial.norm() > scale * 1e-6 {
            angles.push(radial.dot(&y_axis).atan2(radial.dot(&x_axis)));
        }
    }

    if !latitude_min.is_finite() || !latitude_max.is_finite() {
        return None;
    }
    // 境界が緯度方向に広がりを持たないなら、極から極までが使われている。
    if latitude_max - latitude_min <= 1e-9 {
        latitude_min = -half_pi;
        latitude_max = half_pi;
    }
    let latitude_min = latitude_min.max(-half_pi);
    let latitude_max = latitude_max.min(half_pi);

    let (start_angle, sweep) = angular_span(&mut angles);
    let (profile, profile_knots) =
        arc_profile(0.0, 0.0, radius, latitude_min, latitude_max - latitude_min);
    revolve_profile(
        origin,
        axis,
        x_axis,
        y_axis,
        &profile,
        profile_knots,
        2,
        start_angle,
        sweep,
    )
}

/// A toroidal patch covering the major and minor angles a face occupies.
///
/// Both directions are closed, so both are read the same way as a cylinder's
/// angle: the range in use is what lies outside the widest gap.
fn torus_patch_for_boundary(
    origin: Point3,
    z_dir: Vec3,
    x_dir: Vec3,
    major_radius: f64,
    minor_radius: f64,
    boundary_points: &[Point3],
) -> Option<NurbsSurface3> {
    let (axis, x_axis, y_axis) = revolution_frame(z_dir, x_dir)?;
    let scale = minor_radius.abs().max(1.0);

    // **紡錘トーラスの、軸をまたいだ側**（4-264）。
    //
    // 管が芯より大きいトーラス（`minor > major`）は自分と交わり、断面の円が
    // **軸をまたぎます**。またいだ先の点は、符号付きの半径が**負**です。
    // ところが3D の点から測れるのは**符号なしの距離**なので、素直に
    // `|半径| - 芯` で測ると別の値になります。
    //
    // 実測（`screw.step` のねじ山。芯 8.25 / 管 54.873719）:
    //
    // | 頂点 | 半径 | 軸方向 | 符号なしで測ると | **符号付きで測ると** |
    // | :--- | ---: | ---: | ---: | ---: |
    // | #427 | 10.00 | -51.750 | 51.779581 | **54.873719** |
    // | #337 | 1.25 | -54.045 | 54.496560 | **54.873719** |
    //
    // **符号付きなら、管の半径にぴたり一致します。** 円錐の「頂点の向こう側」
    // （4-261）と同じ形の見落としでした。
    //
    // **またいだ側に全部あるときだけ裏返します。** 芯の符号を反転して角度を
    // π ずらせば、あとの組み立ては何も変わりません。**またいでいる面**は
    // 軸の上の点を含むので、これまでどおり断ります。
    let flip = !boundary_points.is_empty()
        && boundary_points.iter().all(|point| {
            let (axial, radial) = axis_frame_coords(*point, origin, axis);
            let straight = ((radial.norm() - major_radius).powi(2) + axial * axial).sqrt();
            let crossed = ((radial.norm() + major_radius).powi(2) + axial * axial).sqrt();
            (crossed - minor_radius).abs() < (straight - minor_radius).abs()
        });
    let (major_radius, angle_offset) = if flip {
        (-major_radius, std::f64::consts::PI)
    } else {
        (major_radius, 0.0)
    };

    let mut major_angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    let mut minor_angles: Vec<f64> = Vec::with_capacity(boundary_points.len());

    for point in boundary_points {
        let (axial, radial) = axis_frame_coords(*point, origin, axis);
        // **半径は符号なしのままです。** 裏返しは芯の符号だけで表せます
        // ——`|r| - (-芯) = |r| + 芯` が、符号付きで見た `|(-|r|) - 芯|` と
        // 同じになるからです。両方を反転すると打ち消し合って元に戻ります
        // （一度そう書いて、数字が1桁も動きませんでした）。
        let radial_distance = radial.norm();
        // 芯円までの距離が副半径に一致していなければ、この面はトーラス上に無い。
        let from_core = ((radial_distance - major_radius).powi(2) + axial * axial).sqrt();
        if (from_core - minor_radius).abs() > scale * 1e-3 {
            // **境界の点がトーラスの上に乗っていません**（`ZENITH_STEP_WHY=1`）。
            // どれだけ外れているかを出します——公差の話なのか、そもそも別の
            // 曲面なのかは、この数字で分かれます（4-228）。
            if std::env::var_os("ZENITH_STEP_WHY").is_some() {
                eprintln!(
                    "STEPWHY   トーラス（芯 {major_radius:.6}、管 {minor_radius:.6}）の上に乗らない点: 芯からの距離 {from_core:.6}（差 {:.6e}、許す幅 {:.6e}）",
                    (from_core - minor_radius).abs(),
                    scale * 1e-3
                );
            }
            return None;
        }
        if radial.norm() > major_radius.abs().max(1.0) * 1e-6 {
            major_angles.push(radial.dot(&y_axis).atan2(radial.dot(&x_axis)) - angle_offset);
        }
        minor_angles.push(axial.atan2(radial_distance - major_radius));
    }

    if minor_angles.is_empty() {
        return None;
    }

    let (start_major, major_sweep) = angular_span(&mut major_angles);
    let (start_minor, minor_sweep) = angular_span(&mut minor_angles);

    let (profile, profile_knots) =
        arc_profile(major_radius, 0.0, minor_radius, start_minor, minor_sweep);
    let patch = revolve_profile(
        origin,
        axis,
        x_axis,
        y_axis,
        &profile,
        profile_knots,
        2,
        start_major + angle_offset,
        major_sweep,
    )?;

    // **張ったパッチの法線が、トーラスの外向きと同じ側か**を確かめます（4-282）。
    //
    // 面の向き（`same_sense`）は「曲面の法線に対して」決まるので、**こちらが
    // 張ったパッチの法線が内向きだと、境界の巻きの規約がひっくり返ります**。
    // 実測（`screw.step`）: `TOROIDAL_SURFACE` から復元した **3枚とも**
    // 裏返っていて、`p-curve loop is inconsistent with face orientation` の
    // 3件はこれでした（4-281）。
    //
    // 直し方は**測ってから**です。真ん中で両方を出して、逆なら v を逆に回します。
    let ((u0, u1), (v0, v1)) = patch.param_range();
    let (um, vm) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    let Some(patch_normal) = patch.normal(um, vm) else {
        return Some(patch);
    };
    let middle = patch.evaluate(um, vm);
    // トーラスの外向き法線: 芯の円の上のいちばん近い点から外へ。
    let from_axis = middle - origin;
    let axial = from_axis.dot(&axis);
    let radial = from_axis - axis * axial;
    if radial.norm() <= f64::EPSILON {
        return Some(patch);
    }
    let core = origin + radial.normalize() * major_radius;
    let outward = middle - core;
    if outward.norm() <= f64::EPSILON {
        return Some(patch);
    }
    let alignment = patch_normal.normalize().dot(&outward.normalize());
    if std::env::var_os("ZENITH_ORIENT_WHY").is_some() {
        eprintln!(
            "ORIENTWHY   トーラスの復元: 法線と外向きの内積 {alignment:+.4}（{}）",
            if alignment >= 0.0 { "そのまま" } else { "**裏返っていたので張り直します**" }
        );
    }
    if alignment >= 0.0 {
        return Some(patch);
    }

    // 裏返っていたので、**小円のほうを逆に回して**張り直します。
    let (flipped_profile, flipped_knots) = arc_profile(
        major_radius,
        0.0,
        minor_radius,
        start_minor + minor_sweep,
        -minor_sweep,
    );
    revolve_profile(
        origin,
        axis,
        x_axis,
        y_axis,
        &flipped_profile,
        flipped_knots,
        2,
        start_major + angle_offset,
        major_sweep,
    )
}
