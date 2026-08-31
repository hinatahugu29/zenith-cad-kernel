//! 補強リブ（Rib Feature）モデリングアルゴリズム
//!
//! 機械設計におけるブラケット、ハウジング、鋳物・樹脂成形品の剛性向上のための
//! 三角リブ・平板リブ補強構造を正確なB-Rep多様体ソリッドとして構築します。

use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance, Transform3, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 補強リブ（Rib）ビルダー
pub struct RibBuilder;

impl RibBuilder {
    /// 独立した三角柱リブソリッド（Triangular Prism Rib）を構築
    /// 直角三角形断面（底辺: length, 高さ: height）を厚み thickness で押し出した5面構成ソリッド
    pub fn make_triangular_rib(
        length: f64,
        height: f64,
        thickness: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if length <= 1e-6 || height <= 1e-6 || thickness <= 1e-6 {
            return Err("Rib dimensions must be strictly positive".to_string());
        }

        let half_t = thickness * 0.5;

        // 手前三角形 (y = -half_t, CCW: (0,0,0) -> (length, 0, 0) -> (0, 0, height))
        let p_front = [
            Point3::new(0.0, -half_t, 0.0),
            Point3::new(length, -half_t, 0.0),
            Point3::new(0.0, -half_t, height),
        ];

        // 奥三角形 (y = half_t, CCW: (0,0,0) -> (0, 0, height) -> (length, 0, 0))
        let p_back = [
            Point3::new(0.0, half_t, 0.0),
            Point3::new(length, half_t, 0.0),
            Point3::new(0.0, half_t, height),
        ];

        let vf: Vec<Vertex> = p_front.iter().map(|&p| Vertex::from_point(p)).collect();
        let vb: Vec<Vertex> = p_back.iter().map(|&p| Vertex::from_point(p)).collect();

        // 前面三角形エッジ
        let ef01 = Edge::line_between(vf[0].clone(), vf[1].clone())?;
        let ef12 = Edge::line_between(vf[1].clone(), vf[2].clone())?;
        let ef20 = Edge::line_between(vf[2].clone(), vf[0].clone())?;

        // 背面三角形エッジ
        let eb01 = Edge::line_between(vb[0].clone(), vb[1].clone())?;
        let eb12 = Edge::line_between(vb[1].clone(), vb[2].clone())?;
        let eb20 = Edge::line_between(vb[2].clone(), vb[0].clone())?;

        // 3本のY軸平行エッジ
        let ey0 = Edge::line_between(vf[0].clone(), vb[0].clone())?;
        let ey1 = Edge::line_between(vf[1].clone(), vb[1].clone())?;
        let ey2 = Edge::line_between(vf[2].clone(), vb[2].clone())?;

        let mut faces = Vec::with_capacity(5);

        // 1. 前面 (y = -half_t, 法線 -Y, CCW: vf0 -> vf1 -> vf2)
        let pl_front = PlaneSurface3::new(
            p_front[0],
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .ok_or("plane front")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_front),
            Wire::new(vec![
                OrientedEdge::forward(ef01.clone()),
                OrientedEdge::forward(ef12.clone()),
                OrientedEdge::forward(ef20.clone()),
            ]),
        ));

        // 2. 背面 (y = half_t, 法線 +Y, CCW: vb0 -> vb2 -> vb1 -> vb0)
        let pl_back = PlaneSurface3::new(
            p_back[0],
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .ok_or("plane back")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_back),
            Wire::new(vec![
                OrientedEdge::reversed(eb20.clone()),
                OrientedEdge::reversed(eb12.clone()),
                OrientedEdge::reversed(eb01.clone()),
            ]),
        ));

        // 3. 底面 (z = 0, 法線 -Z, CCW: vf0 -> vb0 -> vb1 -> vf1)
        let pl_bot = PlaneSurface3::new(
            p_front[0],
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bot")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_bot),
            Wire::new(vec![
                OrientedEdge::forward(ey0.clone()),
                OrientedEdge::forward(eb01.clone()),
                OrientedEdge::reversed(ey1.clone()),
                OrientedEdge::reversed(ef01.clone()),
            ]),
        ));

        // 4. 背面垂直面 (x = 0, 法線 -X, CCW: vf0 -> vf2 -> vb2 -> vb0)
        let pl_vert = PlaneSurface3::new(
            p_front[0],
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane vert")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_vert),
            Wire::new(vec![
                OrientedEdge::reversed(ef20.clone()),
                OrientedEdge::forward(ey2.clone()),
                OrientedEdge::forward(eb20.clone()),
                OrientedEdge::reversed(ey0.clone()),
            ]),
        ));

        // 5. 斜面 (Hypotenuse Sloped Face, CCW: vf1 -> vb1 -> vb2 -> vf2)
        let u_hyp = Vec3::new(0.0, 1.0, 0.0);
        let n_hyp = Vec3::new(height, 0.0, length).normalize();
        let v_hyp = n_hyp.cross(&u_hyp).normalize();
        let pl_hyp = PlaneSurface3::new(p_front[1], u_hyp, v_hyp).ok_or("plane hyp")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(pl_hyp),
            Wire::new(vec![
                OrientedEdge::forward(ey1.clone()),
                OrientedEdge::forward(eb12.clone()),
                OrientedEdge::reversed(ey2.clone()),
                OrientedEdge::reversed(ef12.clone()),
            ]),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// L字型ブラケットに中央補強リブ（Reinforced Ribbed Bracket）を持つ完全閉B-Repソリッドを構築
    pub fn make_ribbed_bracket(
        base_dx: f64,
        base_dy: f64,
        base_dz: f64,
        wall_height: f64,
        wall_thickness: f64,
        rib_thickness: f64,
        rib_length: f64,
        rib_height: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if wall_thickness >= base_dx || rib_thickness >= base_dy {
            return Err("Wall/Rib thickness must be smaller than base width".to_string());
        }
        if rib_length > base_dx - wall_thickness || rib_height > wall_height {
            return Err("Rib dimensions cannot exceed bracket inner span".to_string());
        }

        // 1. 底板 (Base Plate: base_dx x base_dy x base_dz)
        let base_plate = crate::primitive::PrimitiveBuilder::make_box(base_dx, base_dy, base_dz)?;

        // 2. 垂直背板 (Vertical Wall: wall_thickness x base_dy x wall_height, z=base_dzの上に配置)
        let wall_box =
            crate::primitive::PrimitiveBuilder::make_box(wall_thickness, base_dy, wall_height)?;
        let wall_transform = Transform3::from_translation(Vec3::new(0.0, 0.0, base_dz));
        let positioned_wall =
            crate::brep_transform::BrepTransform::transform_solid(&wall_box, &wall_transform)?;

        // L字フレームの結合
        let l_frame = crate::boolean::BooleanEngine::boolean_solids_exact(
            &base_plate,
            &positioned_wall,
            crate::boolean::BooleanOpType::Union,
            tol,
        )?;

        // 3. 三角リブ (Triangular Rib)
        let rib = Self::make_triangular_rib(rib_length, rib_height, rib_thickness, tol)?;
        // リブを背板前面（x=wall_thickness）、底板天面（z=base_dz）、Y軸中央（y=base_dy/2）に配置
        let rib_transform =
            Transform3::from_translation(Vec3::new(wall_thickness, base_dy * 0.5, base_dz));
        let positioned_rib =
            crate::brep_transform::BrepTransform::transform_solid(&rib, &rib_transform)?;

        // リブをL字フレームに結合
        crate::boolean::BooleanEngine::boolean_solids_exact(
            &l_frame,
            &positioned_rib,
            crate::boolean::BooleanOpType::Union,
            tol,
        )
    }
}
