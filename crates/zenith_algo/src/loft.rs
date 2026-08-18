use zenith_geom::{KnotVector, NurbsCurve3, NurbsSurface3};
use zenith_math::Tolerance;

/// ロフト（Loft / スキニング）モデリングアルゴリズム
/// 複数の断面プロファイル曲線群から滑らかなNURBS曲面を生成
pub struct LoftBuilder;

impl LoftBuilder {
    /// 2本以上のプロファイル曲線からロフト曲面を生成
    pub fn loft_curves(
        profiles: &[NurbsCurve3],
        degree_v: usize,
        _tol: &Tolerance,
    ) -> Result<NurbsSurface3, String> {
        let m = profiles.len();
        if m < 2 {
            return Err("Loft requires at least 2 profile curves".to_string());
        }

        let num_u = profiles[0].control_points.len();
        let degree_u = profiles[0].degree;

        // 全プロファイルの制御点数と次数の整合性チェック
        for (idx, prof) in profiles.iter().enumerate() {
            if prof.control_points.len() != num_u || prof.degree != degree_u {
                return Err(format!(
                    "Profile {} does not match control points count ({}) or degree ({})",
                    idx, num_u, degree_u
                ));
            }
        }

        let effective_degree_v = degree_v.min(m - 1).max(1);

        // 制御点グリッドの構築 [row_u][col_v]
        let mut ctrl_pts_grid = vec![Vec::with_capacity(m); num_u];

        for (i, row) in ctrl_pts_grid.iter_mut().enumerate().take(num_u) {
            for profile in profiles.iter().take(m) {
                row.push(profile.control_points[i]);
            }
        }

        let knots_u = profiles[0].knots.clone();
        let knots_v = KnotVector::clamped_uniform(m, effective_degree_v);

        NurbsSurface3::new(
            degree_u,
            effective_degree_v,
            ctrl_pts_grid,
            knots_u,
            knots_v,
        )
    }
}
