use std::collections::HashMap;
use std::fs::File;
use std::io::{Error, ErrorKind, Write};
use std::path::Path;
use zenith_geom::{NurbsCurve2, NurbsCurve3, NurbsSurface3};
use zenith_math::Point3;
use zenith_topo::{
    Face, FaceGeometry, Shape, Shell, Solid, Wire,
};

/// ISO 10303-21 (STEP AP214 / AP203 / AP242) 完全共有マニホールド B-Rep エクスポーター
pub struct StepExporter;

struct StepContext {
    id_counter: u64,
    lines: Vec<String>,
    vertex_map: HashMap<u64, u64>, // Vertex ID -> STEP Vertex ID
    edge_map: HashMap<u64, u64>,   // Edge ID -> STEP EdgeCurve ID
    /// 稜ごとの p-curve。`(その面の曲面の STEP id, その面での 2D 曲線)`。
    ///
    /// 稜は2枚の面に共有されるので、ここには最大2件入る。`SURFACE_CURVE` は
    /// 3D 曲線とこの一覧を並べて持つ。
    edge_pcurves: HashMap<u64, Vec<(u64, NurbsCurve2)>>,
    /// 面ごとに先に書いておいた曲面の STEP id。
    face_surface: HashMap<u64, u64>,
    /// 2D パラメータ空間の表現コンテキスト。1つだけ作って使い回す。
    parametric_context: Option<u64>,
}

impl StepContext {
    fn new() -> Self {
        Self {
            id_counter: 1,
            lines: Vec::new(),
            vertex_map: HashMap::new(),
            edge_map: HashMap::new(),
            edge_pcurves: HashMap::new(),
            face_surface: HashMap::new(),
            parametric_context: None,
        }
    }

    /// 2D パラメータ空間の表現コンテキスト。`DEFINITIONAL_REPRESENTATION` が要る。
    fn parametric_context(&mut self) -> u64 {
        if let Some(id) = self.parametric_context {
            return id;
        }
        let id = self.add_entity(
            "( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D Space','') )",
        );
        self.parametric_context = Some(id);
        id
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
        let content =
            Self::export_solids_to_string_checked(std::slice::from_ref(solid), product_name)
                .map_err(|err| Error::new(ErrorKind::InvalidInput, err))?;
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut file = File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }

    /// 複数Solidを1つのSTEPファイルとして出力
    pub fn export_solids_to_file<P: AsRef<Path>>(
        solids: &[Solid],
        path: P,
        product_name: &str,
    ) -> std::io::Result<()> {
        let content = Self::export_solids_to_string_checked(solids, product_name)
            .map_err(|err| Error::new(ErrorKind::InvalidInput, err))?;
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut file = File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }

    /// Shape 内の Solid 群を1つのSTEPファイルとして出力
    pub fn export_shape_to_file<P: AsRef<Path>>(
        shape: &Shape,
        path: P,
        product_name: &str,
    ) -> std::io::Result<()> {
        let content = Self::export_shape_to_string(shape, product_name)
            .map_err(|err| Error::new(ErrorKind::InvalidInput, err))?;
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut file = File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }

    /// SolidをSTEP形式の文字列として生成
    pub fn export_solid_to_string(solid: &Solid, product_name: &str) -> String {
        Self::export_solids_to_string(std::slice::from_ref(solid), product_name)
    }

    /// 複数SolidをSTEP形式の文字列として生成
    ///
    /// 書けない面が1枚でもあれば、**その面を飛ばした不完全な `CLOSED_SHELL` を
    /// 返さない**。返り値が `String` で失敗を表せないので、STEP として読めない
    /// 注釈だけの文字列を返す。理由を受け取りたい呼び出し側は
    /// [`Self::export_solids_to_string_checked`] を使うこと。ファイルに書く口は
    /// すべて `Result` を返すので、そちらでは失敗が握り潰されない。
    pub fn export_solids_to_string(solids: &[Solid], product_name: &str) -> String {
        Self::export_solids_to_string_checked(solids, product_name)
            .unwrap_or_else(|error| format!("/* Zenith STEP export failed: {error} */\n"))
    }

    /// 複数Solidを STEP 形式の文字列にする。面が1枚でも書けなければ理由を返す。
    pub fn export_solids_to_string_checked(
        solids: &[Solid],
        product_name: &str,
    ) -> Result<String, String> {
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
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#{},'distance_accuracy_value','confusion accuracy')",
            length_unit_id
        ));


        // 複数の型を1つの実体にまとめた「複合エンティティ実体」は、ISO 10303-21
        // では外側の丸括弧が必須である（`#N = ( A(..) B(..) );`）。上の単位定義は
        // 括弧付きで書けていたが、ここだけ抜けていた。OpenCASCADE のパーサは
        // 寛容なので読めてしまい、FreeCAD の突き合わせでは表に出ない。厳格な
        // パーサを持つ他社 CAD では拒否され得る。
        let geom_context_id = ctx.add_entity(&format!(
            "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{},#{},#{})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )",
            uncertainty_id, length_unit_id, plane_angle_unit_id, solid_angle_unit_id
        ));

        // 原点と座標軸
        let origin_pt_id = Self::write_point(&mut ctx, Point3::new(0.0, 0.0, 0.0));
        let z_axis_id = ctx.add_entity("DIRECTION('',(0.0,0.0,1.0))");
        let x_axis_id = ctx.add_entity("DIRECTION('',(1.0,0.0,0.0))");
        let world_axis_id = ctx.add_entity(&format!(
            "AXIS2_PLACEMENT_3D('',#{},#{},#{})",
            origin_pt_id, z_axis_id, x_axis_id
        ));

        // 3. 面の曲面を先に書き、稜ごとの p-curve を集める。
        Self::plan_surfaces_and_pcurves(&mut ctx, solids);

        // 4. Shell内の各Faceのエンティティ生成（トポロジー共有）
        let mut manifold_solid_ids = Vec::with_capacity(solids.len());
        for (index, solid) in solids.iter().enumerate() {
            manifold_solid_ids.push(Self::write_solid_brep(&mut ctx, solid, product_name, index)?);
        }
        let representation_items = manifold_solid_ids
            .iter()
            .map(|id| format!("#{id}"))
            .chain([format!("#{world_axis_id}")])
            .collect::<Vec<_>>()
            .join(",");

        let shape_rep_id = ctx.add_entity(&format!(
            "ADVANCED_BREP_SHAPE_REPRESENTATION('{}',({}),#{})",
            product_name, representation_items, geom_context_id
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
        Ok(out)
    }

    /// Shape 内の Solid 群をSTEP形式の文字列として生成
    pub fn export_shape_to_string(shape: &Shape, product_name: &str) -> Result<String, String> {
        let solids = shape.clone().into_solids();
        if solids.is_empty() {
            return Err("STEP export requires at least one Solid in the Shape".to_string());
        }
        Self::export_solids_to_string_checked(&solids, product_name)
    }

    fn write_solid_brep(
        ctx: &mut StepContext,
        solid: &Solid,
        product_name: &str,
        index: usize,
    ) -> Result<u64, String> {
        let brep_name = if index == 0 {
            product_name.to_string()
        } else {
            format!("{product_name}_{index}")
        };
        let outer_shell_id = Self::write_closed_shell(ctx, &solid.outer_shell)?;
        if solid.inner_shells.is_empty() {
            Ok(ctx.add_entity(&format!(
                "MANIFOLD_SOLID_BREP('{}',#{})",
                brep_name, outer_shell_id
            )))
        } else {
            let mut oriented_void_ids = Vec::with_capacity(solid.inner_shells.len());
            for inner_shell in &solid.inner_shells {
                let shell_id = Self::write_closed_shell(ctx, inner_shell)?;
                let oriented =
                    ctx.add_entity(&format!("ORIENTED_CLOSED_SHELL('',*,#{},.F.)", shell_id));
                oriented_void_ids.push(format!("#{oriented}"));
            }
            Ok(ctx.add_entity(&format!(
                "BREP_WITH_VOIDS('{}',#{},({}))",
                brep_name,
                outer_shell_id,
                oriented_void_ids.join(",")
            )))
        }
    }

    fn write_point(ctx: &mut StepContext, p: Point3) -> u64 {
        let p_str = format!("CARTESIAN_POINT('',({:.12},{:.12},{:.12}))", p.x, p.y, p.z);
        ctx.add_entity(&p_str)
    }

    /// 面の曲面を先に書き、稜ごとの p-curve を集める。
    ///
    /// `SURFACE_CURVE` は3D曲線と、その稜を使うすべての面での 2D 曲線を並べて
    /// 持つ。稜は面より先に書かれるので、**面の曲面の id が先に要る**。だから
    /// 曲面だけを先に一巡して書いておく。
    ///
    /// p-curve を書くのは、トリムされた B-spline 面を受け取る側が 2D 境界を
    /// 自分で求め直さずに済むからである。書いた 2D 曲線が 3D の辺とずれて
    /// いれば受け側は割れるが、ここで書くのは `Face::pcurves`——
    /// `pcurve_fidelity_probe` が全標本で 3e-12 以内を見張っているもの——なので、
    /// 3D と 2D の2つの経路が食い違う心配は、その見張りに預けてある。
    fn plan_surfaces_and_pcurves(ctx: &mut StepContext, solids: &[Solid]) {

        for solid in solids {
            for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
                for face in &shell.faces {
                    let Some(surface_id) = Self::write_surface_of(ctx, face) else {
                        continue;
                    };
                    ctx.face_surface.insert(face.id, surface_id);

                    // **面が持っている p-curve だけを書く。導出したものは
                    // 書かない。**
                    //
                    // 導出は「その辺を面に射影し直す」ことなので、答えは射影の
                    // 精度で決まる。実測すると、そこが効きます。他カーネルから
                    // 読んだ球を書き戻したときの体積の相対誤差:
                    //
                    // | 検体 | p-curve なし | 導出して書く | 保持ぶんだけ書く |
                    // | :--- | ---: | ---: | ---: |
                    // | sphere | 1.19e-13 | 1.71e-10 | 1.19e-13 |
                    // | sphere_capped | 1.74e-12 | 5.43e-10 | 1.74e-12 |
                    //
                    // 受け手は書いてある 2D 境界を使うので、こちらが導出で
                    // 作った粗いほうを渡すと、**受け手が自分で求め直したほうが
                    // 良かった**という結果になります。自前ビルダーの面は
                    // 検証済みの p-curve を持っている（`pcurve_fidelity_probe`
                    // が全標本で 3e-12 以内を見張る）ので、そちらは書きます。
                    let Some(pcurves) = &face.pcurves else {
                        continue;
                    };
                    for pcurve_loop in std::iter::once(&pcurves.outer_loop)
                        .chain(pcurves.inner_loops.iter())
                    {
                        for segment in &pcurve_loop.segments {
                            ctx.edge_pcurves
                                .entry(segment.edge_id)
                                .or_default()
                                .push((surface_id, segment.curve.clone()));
                        }
                    }
                }
            }
        }
    }

    /// 面の支持曲面だけを書く。書けない面は `None`。
    fn write_surface_of(ctx: &mut StepContext, face: &Face) -> Option<u64> {
        match &face.geometry {
            FaceGeometry::Plane(plane) => {
                let loc_id = Self::write_point(ctx, plane.origin);
                let n = plane.normal.normalize();
                let u = plane.u_axis.normalize();
                let z_dir_id = ctx.add_entity(&format!(
                    "DIRECTION('',({:.12},{:.12},{:.12}))",
                    n.x, n.y, n.z
                ));
                let x_dir_id = ctx.add_entity(&format!(
                    "DIRECTION('',({:.12},{:.12},{:.12}))",
                    u.x, u.y, u.z
                ));
                let axis2_id = ctx.add_entity(&format!(
                    "AXIS2_PLACEMENT_3D('',#{},#{},#{})",
                    loc_id, z_dir_id, x_dir_id
                ));
                Some(ctx.add_entity(&format!("PLANE('',#{})", axis2_id)))
            }
            FaceGeometry::Nurbs(nurbs) => Some(Self::write_nurbs_surface(ctx, nurbs)),
            FaceGeometry::Coons(coons) => Self::write_sampled_surface(ctx, coons).ok(),
            FaceGeometry::Gordon(gordon) => Self::write_sampled_surface(ctx, gordon).ok(),
            FaceGeometry::Triangular(patch) => Self::write_sampled_surface(ctx, patch).ok(),
        }
    }

    /// パラメータ空間の 2D 曲線。
    fn write_nurbs_curve_2d(ctx: &mut StepContext, curve: &NurbsCurve2) -> u64 {
        let mut is_rational = false;
        let mut pt_ids = Vec::with_capacity(curve.control_points.len());
        let mut weights = Vec::with_capacity(curve.control_points.len());
        for cp in &curve.control_points {
            if (cp.weight - 1.0).abs() > 1e-6 {
                is_rational = true;
            }
            let id = ctx.add_entity(&format!(
                "CARTESIAN_POINT('',({:.12},{:.12}))",
                cp.point.x, cp.point.y
            ));
            pt_ids.push(format!("#{id}"));
            weights.push(format!("{:.12}", cp.weight));
        }

        let (mults, knots) = Self::compress_knots(&curve.knots.knots);
        let mult_str = mults
            .iter()
            .map(|m| m.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let knot_str = knots
            .iter()
            .map(|k| format!("{k:.12}"))
            .collect::<Vec<_>>()
            .join(",");
        let points = pt_ids.join(",");

        if is_rational {
            ctx.add_entity(&format!(
                "( BOUNDED_CURVE() B_SPLINE_CURVE({},({}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM('') )",
                curve.degree,
                points,
                mult_str,
                knot_str,
                weights.join(",")
            ))
        } else {
            ctx.add_entity(&format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{},({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                curve.degree, points, mult_str, knot_str
            ))
        }
    }

    /// 稜の 2D 境界を、その稜を使う面ごとに `PCURVE` として書く。
    fn write_pcurves_for_edge(ctx: &mut StepContext, edge_id: u64) -> Vec<u64> {
        let Some(entries) = ctx.edge_pcurves.get(&edge_id).cloned() else {
            return Vec::new();
        };
        let mut out = Vec::with_capacity(entries.len());
        for (surface_id, curve) in entries {
            let curve_id = Self::write_nurbs_curve_2d(ctx, &curve);
            let context_id = ctx.parametric_context();
            let definition = ctx.add_entity(&format!(
                "DEFINITIONAL_REPRESENTATION('',(#{curve_id}),#{context_id})"
            ));
            out.push(ctx.add_entity(&format!("PCURVE('',#{surface_id},#{definition})")));
        }
        out
    }

    fn write_closed_shell(ctx: &mut StepContext, shell: &Shell) -> Result<u64, String> {
        let mut advanced_face_ids = Vec::with_capacity(shell.faces.len());
        for face in &shell.faces {
            // 1枚でも書けなければ止める。書けなかった面を飛ばして残りで
            // `CLOSED_SHELL` を組むと、閉じていないものを閉じていると称して
            // 書き出すことになる。
            advanced_face_ids.push(Self::write_face(ctx, face)?);
        }

        let face_list_str = advanced_face_ids
            .iter()
            .map(|id| format!("#{}", id))
            .collect::<Vec<_>>()
            .join(",");

        Ok(ctx.add_entity(&format!("CLOSED_SHELL('',({}))", face_list_str)))
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

        // 2D 境界を持っているなら、3D 曲線と並べて `SURFACE_CURVE` にする。
        // 受け側は好きなほうを使える。`.PCURVE_S1.` は「面のパラメータ側を
        // 主とみなす」という宣言で、書き手が両方を持っているときの慣行。
        let pcurve_ids = Self::write_pcurves_for_edge(ctx, edge.id);
        let curve_id = if pcurve_ids.is_empty() {
            curve_id
        } else {
            let list = pcurve_ids
                .iter()
                .map(|id| format!("#{id}"))
                .collect::<Vec<_>>()
                .join(",");
            ctx.add_entity(&format!(
                "SURFACE_CURVE('',#{curve_id},({list}),.PCURVE_S1.)"
            ))
        };

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
                "DIRECTION('',({:.12},{:.12},{:.12}))",
                dir.x, dir.y, dir.z
            ));
            let vec_id = ctx.add_entity(&format!("VECTOR('',#{},1.0)", dir_id));
            return ctx.add_entity(&format!("LINE('',#{},#{})", p_id, vec_id));
        }

        // 一般のNURBS曲線 / 有理2次円弧（B_SPLINE_CURVE_WITH_KNOTS / RATIONAL_B_SPLINE_CURVE）
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
            weights.push(format!("{:.12}", cp.weight));
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
                .map(|k| format!("{:.12}", k))
                .collect::<Vec<_>>()
                .join(",")
        );

        if is_rational {
            ctx.add_entity(&format!(
                "( BOUNDED_CURVE() B_SPLINE_CURVE({},{},.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({},{},.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE({}) REPRESENTATION_ITEM('') )",
                nurbs.degree, pts_str, mult_str, knot_str, weights_str
            ))
        } else {
            ctx.add_entity(&format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{},{},.UNSPECIFIED.,.F.,.F.,{},{},.UNSPECIFIED.)",
                nurbs.degree, pts_str, mult_str, knot_str
            ))
        }
    }

    /// Writes a wire as an EDGE_LOOP of shared ORIENTED_EDGEs.
    ///
    /// 稜が 2D 境界を持っていれば、その 3D 曲線は `SURFACE_CURVE` に包まれ、
    /// 面ごとの `PCURVE` が並ぶ（`get_or_create_edge_curve`）。
    ///
    /// ここには以前「p-curve は出さない。OCC 自身も出さないし、無くても
    /// 往復する」と書いてありました。往復するのは本当ですが、それは
    /// OpenCASCADE が 2D 境界を自分で求め直せるからで、求め直さない読み手には
    /// 効きません。いまは出しています。
    fn write_edge_loop(ctx: &mut StepContext, wire: &Wire) -> u64 {
        let mut oriented_edge_ids = Vec::with_capacity(wire.edges.len());

        for oe in wire.edges.iter() {
            let edge_curve_id = Self::get_or_create_edge_curve(ctx, &oe.edge);
            let same_sense = if oe.orientation.is_forward() {
                ".T."
            } else {
                ".F."
            };
            let oriented_id = ctx.add_entity(&format!(
                "ORIENTED_EDGE('',*,*,#{},{})",
                edge_curve_id, same_sense
            ));
            oriented_edge_ids.push(format!("#{}", oriented_id));
        }

        let edge_list_str = oriented_edge_ids.join(",");
        ctx.add_entity(&format!("EDGE_LOOP('',({}))", edge_list_str))
    }


    fn write_face(ctx: &mut StepContext, face: &Face) -> Result<u64, String> {
        // 曲面は `plan_surfaces_and_pcurves` が先に書いている。稜が
        // `SURFACE_CURVE` から曲面を指すので、面より先に必要だった。
        //
        // 書けない面はここでエラーになる。以前は `write_face` が `None` を返し、
        // `write_closed_shell` がそれを黙って捨てていたので、**面が1枚も欠けて
        // いないふりをした `CLOSED_SHELL`** が書き出されていた。実測では
        // `ThickenBuilder` が Coons パッチで作る6面の立体が STEP では4面に
        // なっていた（`step_face_parity_test`）。
        let surface_id = *ctx.face_surface.get(&face.id).ok_or_else(|| {
            format!(
                "face {} has a surface this exporter cannot write ({})",
                face.id,
                match &face.geometry {
                    FaceGeometry::Plane(_) => "plane",
                    FaceGeometry::Nurbs(_) => "nurbs",
                    FaceGeometry::Coons(_) => "coons",
                    FaceGeometry::Gordon(_) => "gordon",
                    FaceGeometry::Triangular(_) => "triangular",
                }
            )
        })?;

        // B-Rep EDGE_LOOP トポロジーの構築
        let mut bound_ids = Vec::new();

        // 1. 外側境界 (FACE_OUTER_BOUND)
        let outer_loop_id = Self::write_edge_loop(ctx, &face.outer_wire);
        let outer_bound_id =
            ctx.add_entity(&format!("FACE_OUTER_BOUND('',#{},.T.)", outer_loop_id));

        bound_ids.push(format!("#{}", outer_bound_id));

        // 2. 内側穴境界群 (FACE_BOUND)
        for inner_wire in face.inner_wires.iter() {
            let inner_loop_id = Self::write_edge_loop(ctx, inner_wire);
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

        Ok(adv_face_id)
    }

    /// 制御点の格子を持たない曲面を、標本した NURBS として書く。
    ///
    /// 標本の細かさは、面の境界がどのくらい細かく刻まれているかとは独立に
    /// 決めてよい。トリムは境界ワイヤ（3D の辺）が担っており、ここで書くのは
    /// その辺が乗る土台だけだからである。
    fn write_sampled_surface(
        ctx: &mut StepContext,
        surface: &dyn zenith_geom::Surface3,
    ) -> Result<u64, String> {
        const SAMPLES: usize = 12;
        let nurbs = NurbsSurface3::approximate_surface(surface, SAMPLES, SAMPLES)?;
        Ok(Self::write_nurbs_surface(ctx, &nurbs))
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
                w_row.push(format!("{:.12}", cp.weight));
                pt_ids.push(format!("#{}", Self::write_point(ctx, cp.point)));
            }
            weights.push(format!("({})", w_row.join(",")));
            row_ids.push(format!("({})", pt_ids.join(",")));
        }
        let grid_str = format!("({})", row_ids.join(","));
        let weights_str = format!("({})", weights.join(","));

        // 閉フラグを実際の制御網から判定する。閉じている曲面を .F. と宣言すると
        // 読み手がシームを別々の境界として扱い、トーラス・球のように自己閉包する
        // 面で体積計算が破綻する。
        let u_closed = Self::control_grid_closed_in_u(nurbs);
        let v_closed = Self::control_grid_closed_in_v(nurbs);
        let u_closed_flag = if u_closed { ".T." } else { ".F." };
        let v_closed_flag = if v_closed { ".T." } else { ".F." };

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
                .map(|k| format!("{:.12}", k))
                .collect::<Vec<_>>()
                .join(",")
        );
        let v_knots_str = format!(
            "({})",
            v_knots
                .iter()
                .map(|k| format!("{:.12}", k))
                .collect::<Vec<_>>()
                .join(",")
        );

        if is_rational {
            ctx.add_entity(&format!(
                "( BOUNDED_SURFACE() B_SPLINE_SURFACE({},{},{},.UNSPECIFIED.,{},{},.F.) B_SPLINE_SURFACE_WITH_KNOTS({},{},{},{},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE({}) REPRESENTATION_ITEM('') SURFACE() )",
                nurbs.degree_u, nurbs.degree_v, grid_str,
                u_closed_flag, v_closed_flag,
                u_mult_str, v_mult_str, u_knots_str, v_knots_str,
                weights_str
            ))
        } else {
            ctx.add_entity(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},{},.UNSPECIFIED.,{},{},.F.,{},{},{},{},.UNSPECIFIED.)",
                nurbs.degree_u, nurbs.degree_v, grid_str,
                u_closed_flag, v_closed_flag,
                u_mult_str, v_mult_str, u_knots_str, v_knots_str
            ))
        }
    }

    /// A control grid whose first and last rows coincide describes a surface
    /// that wraps around in u.
    fn control_grid_closed_in_u(nurbs: &NurbsSurface3) -> bool {
        let rows = &nurbs.control_points;
        if rows.len() < 3 {
            return false;
        }
        let first = &rows[0];
        let last = &rows[rows.len() - 1];
        if first.len() != last.len() {
            return false;
        }
        first
            .iter()
            .zip(last.iter())
            .all(|(a, b)| (a.point - b.point).norm() <= CLOSED_SURFACE_TOLERANCE)
    }

    fn control_grid_closed_in_v(nurbs: &NurbsSurface3) -> bool {
        let rows = &nurbs.control_points;
        if rows.is_empty() || rows[0].len() < 3 {
            return false;
        }
        rows.iter().all(|row| {
            let Some(first) = row.first() else {
                return false;
            };
            let Some(last) = row.last() else {
                return false;
            };
            (first.point - last.point).norm() <= CLOSED_SURFACE_TOLERANCE
        })
    }
}

/// 制御点の一致で「閉じている」と判定する距離。STEP の既定不確かさ 1.E-05 より
/// 一桁厳しくとってある。
const CLOSED_SURFACE_TOLERANCE: f64 = 1e-6;
