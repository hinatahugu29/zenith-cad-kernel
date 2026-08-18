use crate::nurbs_curve_2d::NurbsCurve2;
use crate::nurbs_surface::NurbsSurface3;
use crate::surface::Surface3;
use serde::{Deserialize, Serialize};
use zenith_math::{Point2, Point3, Vec3};

/// UVパラメータ空間内の2D閉じたトリムループ（Trim Loop）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimLoop2D {
    pub curves: Vec<NurbsCurve2>,
    /// サンプリングされたポリライン頂点列（高速インサイド判定用）
    cached_polygon: Vec<Point2>,
}

impl TrimLoop2D {
    pub fn new(curves: Vec<NurbsCurve2>) -> Self {
        let mut cached_polygon = Vec::new();
        for c in &curves {
            let pts = c.sample_points(16);
            if !pts.is_empty() {
                cached_polygon.extend_from_slice(&pts[..pts.len() - 1]);
            }
        }
        Self {
            curves,
            cached_polygon,
        }
    }

    /// 点 (u, v) がこのトリムループの内部にあるか（Ray Casting法による奇偶判定）
    pub fn contains_point(&self, uv: Point2) -> bool {
        let pts = &self.cached_polygon;
        let n = pts.len();
        if n < 3 {
            return false;
        }

        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let pi = pts[i];
            let pj = pts[j];

            if ((pi.y > uv.y) != (pj.y > uv.y))
                && (uv.x < (pj.x - pi.x) * (uv.y - pi.y) / (pj.y - pi.y) + pi.x)
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }
}

/// トリム曲面（Trimmed Surface）: 基底曲面 + 外側トリムループ + 穴トリムループ群
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimmedSurface3 {
    pub surface: NurbsSurface3,
    pub outer_loop: Option<TrimLoop2D>,
    pub inner_loops: Vec<TrimLoop2D>,
}

impl TrimmedSurface3 {
    pub fn new(
        surface: NurbsSurface3,
        outer_loop: Option<TrimLoop2D>,
        inner_loops: Vec<TrimLoop2D>,
    ) -> Self {
        Self {
            surface,
            outer_loop,
            inner_loops,
        }
    }

    /// パラメータ点 (u, v) が有効なトリム領域内（有効サーフェス領域）にあるか
    pub fn is_uv_valid(&self, u: f64, v: f64) -> bool {
        let uv = Point2::new(u, v);

        // 外側ループがある場合、外側ループの内側でなければ無効
        if let Some(ref outer) = self.outer_loop {
            if !outer.contains_point(uv) {
                return false;
            }
        }

        // 内側ループ（穴）の内側にある場合は無効（穴の中）
        for inner in &self.inner_loops {
            if inner.contains_point(uv) {
                return false;
            }
        }

        true
    }
}

impl Surface3 for TrimmedSurface3 {
    fn param_range(&self) -> ((f64, f64), (f64, f64)) {
        self.surface.param_range()
    }

    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.surface.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vec3> {
        self.surface.normal(u, v)
    }
}
