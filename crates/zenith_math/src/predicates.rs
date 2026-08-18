use crate::point::{Point2, Point3};

/// ロバスト幾何述語（幾何学的判定における符号誤り防止）
pub struct RobustPredicates;

impl RobustPredicates {
    /// 2次元の厳密向き判定 (Jonathan Shewchuk の適応精度 Orient2D)
    /// - > 0: a -> b -> c が反時計回り (CCW / 左折)
    /// - < 0: a -> b -> c が時計回り (CW / 右折)
    /// - = 0: a, b, c が一直線 (Collinear)
    pub fn orient2d(a: Point2, b: Point2, c: Point2) -> f64 {
        let pa = robust::Coord { x: a.x, y: a.y };
        let pb = robust::Coord { x: b.x, y: b.y };
        let pc = robust::Coord { x: c.x, y: c.y };
        robust::orient2d(pa, pb, pc)
    }

    /// 3次元の厳密向き判定 (Jonathan Shewchuk の適応精度 Orient3D)
    /// 点 d が平面 (a, b, c) のどちら側（表/裏）にあるかを判定
    /// - > 0: a, b, c を反時計回りに見たとき、d は手前側（正の側）
    /// - < 0: d は奥側（負の側）
    /// - = 0: d は平面 (a, b, c) 上にある（Coplanar）
    pub fn orient3d(a: Point3, b: Point3, c: Point3, d: Point3) -> f64 {
        let pa = robust::Coord3D {
            x: a.x,
            y: a.y,
            z: a.z,
        };
        let pb = robust::Coord3D {
            x: b.x,
            y: b.y,
            z: b.z,
        };
        let pc = robust::Coord3D {
            x: c.x,
            y: c.y,
            z: c.z,
        };
        let pd = robust::Coord3D {
            x: d.x,
            y: d.y,
            z: d.z,
        };
        robust::orient3d(pa, pb, pc, pd)
    }

    /// 3次元三角形 (a, b, c) と半直線 (ray_origin, ray_dir) の交差判定 (Möller–Trumbore アルゴリズム)
    pub fn ray_triangle_intersect(
        ray_origin: Point3,
        ray_dir: crate::vector::Vec3,
        a: Point3,
        b: Point3,
        c: Point3,
    ) -> Option<f64> {
        let edge1 = b - a;
        let edge2 = c - a;
        let h = ray_dir.cross(&edge2);
        let det = edge1.dot(&h);

        if det.abs() < 1e-12 {
            return None; // 半直線と三角形が平行
        }

        let inv_det = 1.0 / det;
        let s = ray_origin - a;
        let u = s.dot(&h) * inv_det;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }

        let q = s.cross(&edge1);
        let v = ray_dir.dot(&q) * inv_det;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }

        let t = edge2.dot(&q) * inv_det;
        if t > 1e-10 {
            Some(t)
        } else {
            None
        }
    }
}
