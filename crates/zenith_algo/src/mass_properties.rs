use zenith_math::{Point3, Vec3};
use zenith_tess::TriangleMesh;

/// 幾何特性・物性値（体積、表面積、重心、慣性モーメント）
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    /// 表面積 (mm^2)
    pub surface_area: f64,
    /// 体積 (mm^3)
    pub volume: f64,
    /// 重心座標 (mm)
    pub center_of_mass: Point3,
    /// 慣性モーメント主成分 (Ixx, Iyy, Izz) (mm^5 または密度1時の単位)
    pub inertia_diagonal: Vec3,
}

/// ガウスの発散定理に基づく高精度物性値計算エンジン
pub struct MassCalculator;

impl MassCalculator {
    /// テッセレーションメッシュから厳密な幾何特性・物性値を計算（発散定理・四面体積分）
    pub fn compute_from_mesh(mesh: &TriangleMesh) -> MassProperties {
        let mut total_area = 0.0;
        let mut total_vol = 0.0;
        let mut cx_sum = 0.0;
        let mut cy_sum = 0.0;
        let mut cz_sum = 0.0;

        let mut ixx = 0.0;
        let mut iyy = 0.0;
        let mut izz = 0.0;

        for tri in &mesh.indices {
            let p0 = mesh.positions[tri[0] as usize];
            let p1 = mesh.positions[tri[1] as usize];
            let p2 = mesh.positions[tri[2] as usize];

            // 1. 表面積
            let cross = (p1 - p0).cross(&(p2 - p0));
            let area = 0.5 * cross.norm();
            total_area += area;

            // 2. 符号付き体積 (Signed Volume of Tetrahedron with origin)
            let det = p0.x * (p1.y * p2.z - p1.z * p2.y) - p0.y * (p1.x * p2.z - p1.z * p2.x)
                + p0.z * (p1.x * p2.y - p1.y * p2.x);
            let vol = det / 6.0;
            total_vol += vol;

            // 3. 重心寄与
            cx_sum += (p0.x + p1.x + p2.x) * vol * 0.25;
            cy_sum += (p0.y + p1.y + p2.y) * vol * 0.25;
            cz_sum += (p0.z + p1.z + p2.z) * vol * 0.25;

            // 4. 慣性モーメント寄与 (各四面体の2次モーメント)
            let x2 =
                p0.x * p0.x + p1.x * p1.x + p2.x * p2.x + p0.x * p1.x + p1.x * p2.x + p2.x * p0.x;
            let y2 =
                p0.y * p0.y + p1.y * p1.y + p2.y * p2.y + p0.y * p1.y + p1.y * p2.y + p2.y * p0.y;
            let z2 =
                p0.z * p0.z + p1.z * p1.z + p2.z * p2.z + p0.z * p1.z + p1.z * p2.z + p2.z * p0.z;

            ixx += vol * (y2 + z2) / 10.0;
            iyy += vol * (x2 + z2) / 10.0;
            izz += vol * (x2 + y2) / 10.0;
        }

        let total_vol_abs = total_vol.abs();
        let (cm_x, cm_y, cm_z) = if total_vol_abs > 1e-12 {
            (cx_sum / total_vol, cy_sum / total_vol, cz_sum / total_vol)
        } else {
            (0.0, 0.0, 0.0)
        };

        MassProperties {
            surface_area: total_area,
            volume: total_vol_abs,
            center_of_mass: Point3::new(cm_x, cm_y, cm_z),
            inertia_diagonal: Vec3::new(ixx.abs(), iyy.abs(), izz.abs()),
        }
    }
}
