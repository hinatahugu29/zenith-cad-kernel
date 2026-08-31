//! 解析的曲面交差（Analytic Surface-Surface Intersection）
//!
//! 平面・球面・円柱面などの基本代数曲面同士の幾何交差を代数的（閉形式）に直接解き、
//! 数値マーチング反復なし（O(1) 時間）かつ完全誤差ゼロで交差曲線（直線・円・楕円）を返します。

use crate::curve::{Circle3, Ellipse3, Line3};
use crate::surface::PlaneSurface3;
use std::f64::consts::PI;
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};

/// 解析的曲面交差の交線結果
#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticIntersectionResult {
    /// 交差なし（空集合）
    Empty,
    /// 1本の直線
    Line(Line3),
    /// 2本の平行直線
    TwoLines(Line3, Line3),
    /// 1つの真円
    Circle(Circle3),
    /// 1つの楕円
    Ellipse(Ellipse3),
    /// 2つの真円（球と円柱の同軸交差など）
    TwoCircles(Circle3, Circle3),
}

/// 解析的曲面交差ソルバー
pub struct AnalyticIntersection;

impl AnalyticIntersection {
    /// 平面と平相の代数交差（直線）
    pub fn intersect_plane_plane(
        p1: &PlaneSurface3,
        p2: &PlaneSurface3,
        tol: &Tolerance,
    ) -> Option<Line3> {
        let n1 = p1.normal.try_normalize_safe(1e-12)?;
        let n2 = p2.normal.try_normalize_safe(1e-12)?;
        let dir = n1.cross(&n2);
        let dir_norm = dir.norm();

        if dir_norm < tol.angular.max(1e-9) {
            // 平行または同一平面
            return None;
        }

        let dir_unit = (dir / dir_norm).try_normalize_safe(1e-12)?;

        // 2平面の連立方程式から交線上の最近傍点 P0 を代数的に解く
        // P0 = c1 * n1 + c2 * n2
        // [ 1        n1.n2 ] [ c1 ] = [ n1 . P1 ]
        // [ n1.n2    1     ] [ c2 ]   [ n2 . P2 ]
        let dot = n1.dot(&n2);
        let det = 1.0 - dot * dot;
        if det.abs() < 1e-12 {
            return None;
        }

        let d1 = n1.dot(&p1.origin.coords);
        let d2 = n2.dot(&p2.origin.coords);

        let c1 = (d1 - dot * d2) / det;
        let c2 = (d2 - dot * d1) / det;

        let p0 = Point3::from(n1 * c1 + n2 * c2);

        Some(Line3::new(p0, p0 + dir_unit))
    }

    /// 平面と球面の代数交差（円または点接または空）
    pub fn intersect_plane_sphere(
        plane: &PlaneSurface3,
        center: Point3,
        radius: f64,
        tol: &Tolerance,
    ) -> AnalyticIntersectionResult {
        if radius <= 0.0 {
            return AnalyticIntersectionResult::Empty;
        }

        let n = match plane.normal.try_normalize_safe(1e-12) {
            Some(vec) => vec,
            None => return AnalyticIntersectionResult::Empty,
        };

        // 球中心から平面への符号付き距離
        let dist = (center - plane.origin).dot(&n);
        let abs_dist = dist.abs();

        if abs_dist > radius + tol.linear {
            return AnalyticIntersectionResult::Empty;
        }

        let circle_center = center - n * dist;
        let r_sq = (radius * radius - abs_dist * abs_dist).max(0.0);
        let circle_r = r_sq.sqrt();

        if circle_r < tol.linear {
            // 点接（半径ゼロ）
            AnalyticIntersectionResult::Empty
        } else if let Some(circle) = Circle3::new(circle_center, circle_r, n, 0.0, 2.0 * PI) {
            AnalyticIntersectionResult::Circle(circle)
        } else {
            AnalyticIntersectionResult::Empty
        }
    }

    /// 平面と無限円柱の代数交差（円・楕円・平行2直線・接線）
    pub fn intersect_plane_cylinder(
        plane: &PlaneSurface3,
        axis_origin: Point3,
        axis_dir: Vec3,
        radius: f64,
        tol: &Tolerance,
    ) -> AnalyticIntersectionResult {
        if radius <= 0.0 {
            return AnalyticIntersectionResult::Empty;
        }

        let n = match plane.normal.try_normalize_safe(1e-12) {
            Some(vec) => vec,
            None => return AnalyticIntersectionResult::Empty,
        };
        let a = match axis_dir.try_normalize_safe(1e-12) {
            Some(vec) => vec,
            None => return AnalyticIntersectionResult::Empty,
        };

        let cos_theta = n.dot(&a);
        let abs_cos = cos_theta.abs();

        // 1. 軸と平面法線が平行（円柱の横断・垂直断面） => 真円
        if (1.0 - abs_cos).abs() < 1e-9 {
            // 軸と平面の交点
            let t = (plane.origin - axis_origin).dot(&n) / cos_theta;
            let center = axis_origin + a * t;
            if let Some(circle) = Circle3::new(center, radius, a, 0.0, 2.0 * PI) {
                return AnalyticIntersectionResult::Circle(circle);
            } else {
                return AnalyticIntersectionResult::Empty;
            }
        }

        // 2. 軸と平面法線が直交（軸平行断面） => 2直線または1接線
        if abs_cos < 1e-9 {
            let d = (axis_origin - plane.origin).dot(&n);
            let abs_d = d.abs();

            if abs_d > radius + tol.linear {
                return AnalyticIntersectionResult::Empty;
            }

            let p_center = axis_origin - n * d;
            let tangent_dir = n.cross(&a).try_normalize_safe(1e-12).unwrap_or(a);

            if (abs_d - radius).abs() <= tol.linear {
                // 1接線
                return AnalyticIntersectionResult::Line(Line3::new(p_center, p_center + a));
            } else {
                // 2本の平行直線
                let half_w = (radius * radius - abs_d * abs_d).max(0.0).sqrt();
                let p1 = p_center + tangent_dir * half_w;
                let p2 = p_center - tangent_dir * half_w;
                return AnalyticIntersectionResult::TwoLines(
                    Line3::new(p1, p1 + a),
                    Line3::new(p2, p2 + a),
                );
            }
        }

        // 3. 斜め交差 => 楕円
        // 軸と平面の交点を楕円中心とする
        let t = (plane.origin - axis_origin).dot(&n) / cos_theta;
        let center = axis_origin + a * t;

        // 長軸方向: 平面法線 n と軸 a が張る平面内で、n に直交するベクトル
        let proj = a - n * cos_theta;
        let major_dir = match proj.try_normalize_safe(1e-12) {
            Some(vec) => vec,
            None => return AnalyticIntersectionResult::Empty,
        };

        // 短軸半径 b = radius
        // 長軸半径 a_rad = radius / |cos_theta| (幾何学的に断面の傾き角 alpha = arccos(n . a))
        let major_radius = radius / abs_cos;
        let minor_radius = radius;

        if let Some(ellipse) = Ellipse3::new(
            center,
            major_radius,
            minor_radius,
            n,
            major_dir,
            0.0,
            2.0 * PI,
        ) {
            AnalyticIntersectionResult::Ellipse(ellipse)
        } else {
            AnalyticIntersectionResult::Empty
        }
    }

    /// 同軸な球と円柱の交差（1つまたは2つの水平円）
    pub fn intersect_sphere_cylinder_coaxial(
        sphere_center: Point3,
        sphere_radius: f64,
        cyl_axis_pt: Point3,
        cyl_axis_dir: Vec3,
        cyl_radius: f64,
        tol: &Tolerance,
    ) -> AnalyticIntersectionResult {
        if sphere_radius <= 0.0 || cyl_radius <= 0.0 {
            return AnalyticIntersectionResult::Empty;
        }

        let a = match cyl_axis_dir.try_normalize_safe(1e-12) {
            Some(vec) => vec,
            None => return AnalyticIntersectionResult::Empty,
        };

        // 球中心が円柱軸上にあるか確認
        let offset = sphere_center - cyl_axis_pt;
        let radial_offset = (offset - a * offset.dot(&a)).norm();
        if radial_offset > tol.linear {
            // 同軸ではない
            return AnalyticIntersectionResult::Empty;
        }

        if cyl_radius > sphere_radius + tol.linear {
            return AnalyticIntersectionResult::Empty;
        }

        let h_sq = (sphere_radius * sphere_radius - cyl_radius * cyl_radius).max(0.0);
        let h = h_sq.sqrt();

        if h < tol.linear {
            // 赤道で接する単一円
            if let Some(c) = Circle3::new(sphere_center, cyl_radius, a, 0.0, 2.0 * PI) {
                AnalyticIntersectionResult::Circle(c)
            } else {
                AnalyticIntersectionResult::Empty
            }
        } else {
            // 上下に2つの円
            let c1_center = sphere_center + a * h;
            let c2_center = sphere_center - a * h;
            let c1 = Circle3::new(c1_center, cyl_radius, a, 0.0, 2.0 * PI);
            let c2 = Circle3::new(c2_center, cyl_radius, a, 0.0, 2.0 * PI);
            match (c1, c2) {
                (Some(c1), Some(c2)) => AnalyticIntersectionResult::TwoCircles(c1, c2),
                _ => AnalyticIntersectionResult::Empty,
            }
        }
    }
}
