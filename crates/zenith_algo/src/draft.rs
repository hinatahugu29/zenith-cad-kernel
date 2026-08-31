//! 金型抜き勾配（Draft Angle）モデリングアルゴリズム
//!
//! 射出成形や鋳造・鍛造の金型設計において必須となる、
//! パーティング面および引抜方向（Pull Direction）に対する抜き勾配（テーパー角）
//! を持つソリッドおよびキャビティを正確なB-Rep多様体として構築します。

use zenith_geom::PlaneSurface3;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 抜き勾配（Draft Angle）ビルダー
pub struct DraftBuilder;

impl DraftBuilder {
    /// 抜き勾配（draft_angle_rad）を持つテーパー角錐台ブロック（Drafted Box）を構築
    pub fn make_drafted_block(
        dx: f64,
        dy: f64,
        dz: f64,
        draft_angle_rad: f64,
        _tol: &Tolerance,
    ) -> Result<Solid, String> {
        if draft_angle_rad < 0.0 {
            return Err("Draft angle must not be negative".to_string());
        }
        if draft_angle_rad >= std::f64::consts::FRAC_PI_4 {
            return Err("Draft angle must be smaller than 45 degrees".to_string());
        }

        let tan_a = draft_angle_rad.tan();
        let exp_x = dz * tan_a;
        let exp_y = dz * tan_a;

        // 底面 (z=0) 4頂点 (CCW)
        let pb = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(dx, 0.0, 0.0),
            Point3::new(dx, dy, 0.0),
            Point3::new(0.0, dy, 0.0),
        ];

        // 天面 (z=dz) 4頂点 (CCW, 外側にテーパー拡大)
        let pt = [
            Point3::new(-exp_x, -exp_y, dz),
            Point3::new(dx + exp_x, -exp_y, dz),
            Point3::new(dx + exp_x, dy + exp_y, dz),
            Point3::new(-exp_x, dy + exp_y, dz),
        ];

        let vb: Vec<Vertex> = pb.iter().map(|&p| Vertex::from_point(p)).collect();
        let vt: Vec<Vertex> = pt.iter().map(|&p| Vertex::from_point(p)).collect();

        // 底面エッジ（4本）
        let eb01 = Edge::line_between(vb[0].clone(), vb[1].clone())?;
        let eb12 = Edge::line_between(vb[1].clone(), vb[2].clone())?;
        let eb23 = Edge::line_between(vb[2].clone(), vb[3].clone())?;
        let eb30 = Edge::line_between(vb[3].clone(), vb[0].clone())?;

        // 天面エッジ（4本）
        let et01 = Edge::line_between(vt[0].clone(), vt[1].clone())?;
        let et12 = Edge::line_between(vt[1].clone(), vt[2].clone())?;
        let et23 = Edge::line_between(vt[2].clone(), vt[3].clone())?;
        let et30 = Edge::line_between(vt[3].clone(), vt[0].clone())?;

        // 垂直柱エッジ（4本）
        let ev0 = Edge::line_between(vb[0].clone(), vt[0].clone())?;
        let ev1 = Edge::line_between(vb[1].clone(), vt[1].clone())?;
        let ev2 = Edge::line_between(vb[2].clone(), vt[2].clone())?;
        let ev3 = Edge::line_between(vb[3].clone(), vt[3].clone())?;

        let mut faces = Vec::with_capacity(6);

        // 1. 前面 (Front: vb0 -> vb1 -> vt1 -> vt0)
        let n_front = Vec3::new(0.0, -1.0, -tan_a).normalize();
        let u_front = Vec3::new(1.0, 0.0, 0.0);
        let v_front = n_front.cross(&u_front).normalize();
        let p_front = PlaneSurface3::new(pb[0], u_front, v_front).ok_or("plane front")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_front),
            Wire::new(vec![
                OrientedEdge::forward(eb01.clone()),
                OrientedEdge::forward(ev1.clone()),
                OrientedEdge::reversed(et01.clone()),
                OrientedEdge::reversed(ev0.clone()),
            ]),
        ));

        // 2. 右面 (Right: vb1 -> vb2 -> vt2 -> vt1)
        let n_right = Vec3::new(1.0, 0.0, -tan_a).normalize();
        let u_right = Vec3::new(0.0, 1.0, 0.0);
        let v_right = n_right.cross(&u_right).normalize();
        let p_right = PlaneSurface3::new(pb[1], u_right, v_right).ok_or("plane right")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_right),
            Wire::new(vec![
                OrientedEdge::forward(eb12.clone()),
                OrientedEdge::forward(ev2.clone()),
                OrientedEdge::reversed(et12.clone()),
                OrientedEdge::reversed(ev1.clone()),
            ]),
        ));

        // 3. 背面 (Back: vb2 -> vb3 -> vt3 -> vt2)
        let n_back = Vec3::new(0.0, 1.0, -tan_a).normalize();
        let u_back = Vec3::new(-1.0, 0.0, 0.0);
        let v_back = n_back.cross(&u_back).normalize();
        let p_back = PlaneSurface3::new(pb[2], u_back, v_back).ok_or("plane back")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_back),
            Wire::new(vec![
                OrientedEdge::forward(eb23.clone()),
                OrientedEdge::forward(ev3.clone()),
                OrientedEdge::reversed(et23.clone()),
                OrientedEdge::reversed(ev2.clone()),
            ]),
        ));

        // 4. 左面 (Left: vb3 -> vb0 -> vt0 -> vt3)
        let n_left = Vec3::new(-1.0, 0.0, -tan_a).normalize();
        let u_left = Vec3::new(0.0, -1.0, 0.0);
        let v_left = n_left.cross(&u_left).normalize();
        let p_left = PlaneSurface3::new(pb[3], u_left, v_left).ok_or("plane left")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_left),
            Wire::new(vec![
                OrientedEdge::forward(eb30.clone()),
                OrientedEdge::forward(ev0.clone()),
                OrientedEdge::reversed(et30.clone()),
                OrientedEdge::reversed(ev3.clone()),
            ]),
        ));

        // 5. 底面 (Bottom: -Z, CCW: vb3 -> vb2 -> vb1 -> vb0)
        let p_bot = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .ok_or("plane bottom")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_bot),
            Wire::new(vec![
                OrientedEdge::reversed(eb23),
                OrientedEdge::reversed(eb12),
                OrientedEdge::reversed(eb01),
                OrientedEdge::reversed(eb30),
            ]),
        ));

        // 6. 天面 (Top: +Z, CCW: vt0 -> vt1 -> vt2 -> vt3)
        let p_top = PlaneSurface3::new(
            Point3::new(0.0, 0.0, dz),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .ok_or("plane top")?;
        faces.push(Face::simple(
            FaceGeometry::Plane(p_top),
            Wire::new(vec![
                OrientedEdge::forward(et01),
                OrientedEdge::forward(et12),
                OrientedEdge::forward(et23),
                OrientedEdge::forward(et30),
            ]),
        ));

        let shell = Shell::closed(faces);
        crate::validated_solid(shell)
    }

    /// 直方体ブロック内に抜き勾配付きキャビティ凹みを持つ金型キャビティブロックを構築
    pub fn make_drafted_cavity_block(
        dx: f64,
        dy: f64,
        dz: f64,
        cavity_dx: f64,
        cavity_dy: f64,
        cavity_depth: f64,
        draft_angle_rad: f64,
        tol: &Tolerance,
    ) -> Result<Solid, String> {
        if cavity_dx >= dx || cavity_dy >= dy || cavity_depth >= dz {
            return Err("Cavity dimensions must be strictly smaller than outer block".to_string());
        }

        // 外側直方体
        let outer_box = crate::primitive::PrimitiveBuilder::make_box(dx, dy, dz)?;

        // 抜き勾配付きキャビティツールソリッド
        // キャビティ開口部は天面 z=dz に配置され、深さ cavity_depth に向かって抜き勾配で狭まる
        let tan_a = draft_angle_rad.tan();
        let bot_c_dx = cavity_dx - 2.0 * cavity_depth * tan_a;
        let bot_c_dy = cavity_dy - 2.0 * cavity_depth * tan_a;
        if bot_c_dx <= 1e-3 || bot_c_dy <= 1e-3 {
            return Err("Cavity bottom width becomes too small with given draft angle".to_string());
        }

        let cavity_solid = Self::make_drafted_block(
            bot_c_dx,
            bot_c_dy,
            cavity_depth + 1.0, // 突き抜け用に少し高くする
            draft_angle_rad,
            tol,
        )?;

        // キャビティを天面中央に配置
        let cx = (dx - cavity_dx) * 0.5 + (cavity_dx - bot_c_dx) * 0.5;
        let cy = (dy - cavity_dy) * 0.5 + (cavity_dy - bot_c_dy) * 0.5;
        let cz = dz - cavity_depth;

        let transform = zenith_math::Transform3::from_translation(Vec3::new(cx, cy, cz));
        let positioned_cavity =
            crate::brep_transform::BrepTransform::transform_solid(&cavity_solid, &transform)?;

        crate::boolean::BooleanEngine::boolean_solids_exact(
            &outer_box,
            &positioned_cavity,
            crate::boolean::BooleanOpType::Difference,
            tol,
        )
    }
}
