use zenith_geom::PlaneSurface3;
use zenith_math::Point3;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// シェル化・肉厚化ソリッドビルダー（脱OCCT型 中空ケーシング・容器生成）
pub struct ShellBuilder;

impl ShellBuilder {
    /// 直方体を指定肉厚 thickness でシェル化（開口面 open_face_index: デフォルト 1: 天面開口）
    /// 外側5面 + 内側5面 + 開口部リム4面 = 全14面の完全マニホールド閉ソリッドを生成
    pub fn make_hollow_box(
        dx: f64,
        dy: f64,
        dz: f64,
        thickness: f64,
        open_face_index: usize,
    ) -> Result<Solid, String> {
        if thickness <= 0.0 || thickness * 2.0 >= dx || thickness * 2.0 >= dy || thickness >= dz {
            return Err(
                "Invalid shell thickness: must be positive and smaller than dimensions".to_string(),
            );
        }

        let t = thickness;
        let mut faces = Vec::new();

        // 補助関数: 平面Faceを4頂点（反時計回り）から生成
        let make_quad_face =
            |p0: Point3, p1: Point3, p2: Point3, p3: Point3| -> Result<Face, String> {
                let u = p1 - p0;
                let v = p3 - p0;
                let plane = PlaneSurface3::new(p0, u, v).ok_or("Failed to create plane")?;

                let v0 = Vertex::from_point(p0);
                let v1 = Vertex::from_point(p1);
                let v2 = Vertex::from_point(p2);
                let v3 = Vertex::from_point(p3);

                let e0 = Edge::line_between(v0.clone(), v1.clone())?;
                let e1 = Edge::line_between(v1.clone(), v2.clone())?;
                let e2 = Edge::line_between(v2.clone(), v3.clone())?;
                let e3 = Edge::line_between(v3.clone(), v0.clone())?;

                let wire = Wire::new(vec![
                    OrientedEdge::forward(e0),
                    OrientedEdge::forward(e1),
                    OrientedEdge::forward(e2),
                    OrientedEdge::forward(e3),
                ]);

                Ok(Face::simple(FaceGeometry::Plane(plane), wire))
            };

        // 天面開口 (open_face_index = 1) の場合
        if open_face_index == 1 {
            // 外側頂点
            let p0 = Point3::new(0.0, 0.0, 0.0);
            let p1 = Point3::new(dx, 0.0, 0.0);
            let p2 = Point3::new(dx, dy, 0.0);
            let p3 = Point3::new(0.0, dy, 0.0);
            let p4 = Point3::new(0.0, 0.0, dz);
            let p5 = Point3::new(dx, 0.0, dz);
            let p6 = Point3::new(dx, dy, dz);
            let p7 = Point3::new(0.0, dy, dz);

            // 内側頂点（内空洞：底面 z=t、上面 z=dz）
            let q0 = Point3::new(t, t, t);
            let q1 = Point3::new(dx - t, t, t);
            let q2 = Point3::new(dx - t, dy - t, t);
            let q3 = Point3::new(t, dy - t, t);
            let q4 = Point3::new(t, t, dz);
            let q5 = Point3::new(dx - t, t, dz);
            let q6 = Point3::new(dx - t, dy - t, dz);
            let q7 = Point3::new(t, dy - t, dz);

            // --- 1. 外側 5面 ---
            // 外側底面 (-Z) : 法線下向き (p0 -> p3 -> p2 -> p1)
            faces.push(make_quad_face(p0, p3, p2, p1)?);
            // 外面前面 (-Y) : (p0 -> p1 -> p5 -> p4)
            faces.push(make_quad_face(p0, p1, p5, p4)?);
            // 外面後面 (+Y) : (p3 -> p7 -> p6 -> p2)
            faces.push(make_quad_face(p3, p7, p6, p2)?);
            // 外面左面 (-X) : (p0 -> p4 -> p7 -> p3)
            faces.push(make_quad_face(p0, p4, p7, p3)?);
            // 外面右面 (+X) : (p1 -> p2 -> p6 -> p5)
            faces.push(make_quad_face(p1, p2, p6, p5)?);

            // --- 2. 内側 5面（内側から見てソリッド外向き = 中空側から見て外法線） ---
            // 内側底面 (+Z) : (q0 -> q1 -> q2 -> q3)
            faces.push(make_quad_face(q0, q1, q2, q3)?);
            // 内面前面 (+Y) : (q0 -> q4 -> q5 -> q1)
            faces.push(make_quad_face(q0, q4, q5, q1)?);
            // 内面後面 (-Y) : (q3 -> q2 -> q6 -> q7)
            faces.push(make_quad_face(q3, q2, q6, q7)?);
            // 内面左面 (+X) : (q0 -> q3 -> q7 -> q4)
            faces.push(make_quad_face(q0, q3, q7, q4)?);
            // 内面右面 (-X) : (q1 -> q5 -> q6 -> q2)
            faces.push(make_quad_face(q1, q5, q6, q2)?);

            // --- 3. 開口部リム（上面の縁） 4面 (+Z法線) ---
            // 前縁: (p4 -> p5 -> q5 -> q4)
            faces.push(make_quad_face(p4, p5, q5, q4)?);
            // 右縁: (p5 -> p6 -> q6 -> q5)
            faces.push(make_quad_face(p5, p6, q6, q5)?);
            // 後縁: (p6 -> p7 -> q7 -> q6)
            faces.push(make_quad_face(p6, p7, q7, q6)?);
            // 左縁: (p7 -> p4 -> q4 -> q7)
            faces.push(make_quad_face(p7, p4, q4, q7)?);
        } else if open_face_index == 0 {
            // 底面開口 (open_face_index = 0)
            let p0 = Point3::new(0.0, 0.0, 0.0);
            let p1 = Point3::new(dx, 0.0, 0.0);
            let p2 = Point3::new(dx, dy, 0.0);
            let p3 = Point3::new(0.0, dy, 0.0);
            let p4 = Point3::new(0.0, 0.0, dz);
            let p5 = Point3::new(dx, 0.0, dz);
            let p6 = Point3::new(dx, dy, dz);
            let p7 = Point3::new(0.0, dy, dz);

            let q0 = Point3::new(t, t, 0.0);
            let q1 = Point3::new(dx - t, t, 0.0);
            let q2 = Point3::new(dx - t, dy - t, 0.0);
            let q3 = Point3::new(t, dy - t, 0.0);
            let q4 = Point3::new(t, t, dz - t);
            let q5 = Point3::new(dx - t, t, dz - t);
            let q6 = Point3::new(dx - t, dy - t, dz - t);
            let q7 = Point3::new(t, dy - t, dz - t);

            // 外側天面 (+Z)
            faces.push(make_quad_face(p4, p5, p6, p7)?);
            // 外壁4面
            faces.push(make_quad_face(p0, p1, p5, p4)?);
            faces.push(make_quad_face(p3, p7, p6, p2)?);
            faces.push(make_quad_face(p0, p4, p7, p3)?);
            faces.push(make_quad_face(p1, p2, p6, p5)?);

            // 内側天面 (-Z)
            faces.push(make_quad_face(q7, q6, q5, q4)?);
            // 内壁4面
            faces.push(make_quad_face(q0, q4, q5, q1)?);
            faces.push(make_quad_face(q3, q2, q6, q7)?);
            faces.push(make_quad_face(q0, q3, q7, q4)?);
            faces.push(make_quad_face(q1, q5, q6, q2)?);

            // 底面リム4面 (-Z)
            faces.push(make_quad_face(p1, p0, q0, q1)?);
            faces.push(make_quad_face(p2, p1, q1, q2)?);
            faces.push(make_quad_face(p3, p2, q2, q3)?);
            faces.push(make_quad_face(p0, p3, q3, q0)?);
        } else {
            return Err("Supported open_face_index: 0 (bottom) or 1 (top)".to_string());
        }

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 直方体の両端面（底面 Z=0 および 天面 Z=dz）を開口した角パイプ中空ソリッドの生成（全16面閉ソリッド）
    pub fn make_through_hollow_box(
        dx: f64,
        dy: f64,
        dz: f64,
        thickness: f64,
    ) -> Result<Solid, String> {
        if thickness <= 0.0 || thickness * 2.0 >= dx || thickness * 2.0 >= dy {
            return Err("Invalid shell thickness for through tube".to_string());
        }
        let t = thickness;
        let mut faces = Vec::with_capacity(16);

        let make_quad_face =
            |p0: Point3, p1: Point3, p2: Point3, p3: Point3| -> Result<Face, String> {
                let u = p1 - p0;
                let v = p3 - p0;
                let plane = PlaneSurface3::new(p0, u, v).ok_or("Failed to create plane")?;

                let v0 = Vertex::from_point(p0);
                let v1 = Vertex::from_point(p1);
                let v2 = Vertex::from_point(p2);
                let v3 = Vertex::from_point(p3);

                let e0 = Edge::line_between(v0.clone(), v1.clone())?;
                let e1 = Edge::line_between(v1.clone(), v2.clone())?;
                let e2 = Edge::line_between(v2.clone(), v3.clone())?;
                let e3 = Edge::line_between(v3.clone(), v0.clone())?;

                let wire = Wire::new(vec![
                    OrientedEdge::forward(e0),
                    OrientedEdge::forward(e1),
                    OrientedEdge::forward(e2),
                    OrientedEdge::forward(e3),
                ]);

                Ok(Face::simple(FaceGeometry::Plane(plane), wire))
            };

        // 外側頂点
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(dx, 0.0, 0.0);
        let p2 = Point3::new(dx, dy, 0.0);
        let p3 = Point3::new(0.0, dy, 0.0);
        let p4 = Point3::new(0.0, 0.0, dz);
        let p5 = Point3::new(dx, 0.0, dz);
        let p6 = Point3::new(dx, dy, dz);
        let p7 = Point3::new(0.0, dy, dz);

        // 内側頂点（Z=0 から Z=dz まで完全貫通）
        let q0 = Point3::new(t, t, 0.0);
        let q1 = Point3::new(dx - t, t, 0.0);
        let q2 = Point3::new(dx - t, dy - t, 0.0);
        let q3 = Point3::new(t, dy - t, 0.0);
        let q4 = Point3::new(t, t, dz);
        let q5 = Point3::new(dx - t, t, dz);
        let q6 = Point3::new(dx - t, dy - t, dz);
        let q7 = Point3::new(t, dy - t, dz);

        // --- 1. 外壁 4面 ---
        faces.push(make_quad_face(p0, p1, p5, p4)?); // 前面 (-Y)
        faces.push(make_quad_face(p3, p7, p6, p2)?); // 後面 (+Y)
        faces.push(make_quad_face(p0, p4, p7, p3)?); // 左面 (-X)
        faces.push(make_quad_face(p1, p2, p6, p5)?); // 右面 (+X)

        // --- 2. 内壁 4面 ---
        faces.push(make_quad_face(q0, q4, q5, q1)?); // 内面前面 (+Y)
        faces.push(make_quad_face(q3, q2, q6, q7)?); // 内面後面 (-Y)
        faces.push(make_quad_face(q0, q3, q7, q4)?); // 内面左面 (+X)
        faces.push(make_quad_face(q1, q5, q6, q2)?); // 内面右面 (-X)

        // --- 3. 天面リム 4面 (+Z) ---
        faces.push(make_quad_face(p4, p5, q5, q4)?);
        faces.push(make_quad_face(p5, p6, q6, q5)?);
        faces.push(make_quad_face(p6, p7, q7, q6)?);
        faces.push(make_quad_face(p7, p4, q4, q7)?);

        // --- 4. 底面リム 4面 (-Z) ---
        faces.push(make_quad_face(p1, p0, q0, q1)?);
        faces.push(make_quad_face(p2, p1, q1, q2)?);
        faces.push(make_quad_face(p3, p2, q2, q3)?);
        faces.push(make_quad_face(p0, p3, q3, q0)?);

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }
}

