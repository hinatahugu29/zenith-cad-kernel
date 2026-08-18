use zenith_geom::{CoonsPatch3, NurbsCurve3};
use zenith_math::Tolerance;
use zenith_topo::{Edge, Face, OrientedEdge, Vertex, Wire};

/// カーブパッチ（Curve Patch）ビルダー
/// Plasticityのように4本のカーブ境界から自動で整合性の取れたB-Rep Faceを構築
pub struct CurvePatchBuilder;

impl CurvePatchBuilder {
    /// 4本の3次元NURBS曲線からパッチFaceを構築
    /// `c0`: 下辺 (v=0), `c1`: 上辺 (v=1), `d0`: 左辺 (u=0), `d1`: 右辺 (u=1)
    pub fn build_from_4_curves(
        c0: NurbsCurve3,
        c1: NurbsCurve3,
        d0: NurbsCurve3,
        d1: NurbsCurve3,
        tol: &Tolerance,
    ) -> Result<Face, String> {
        let (u0_min, u0_max) = c0.param_range();
        let (u1_min, u1_max) = c1.param_range();

        let p00 = c0.evaluate(u0_min);
        let p10 = c0.evaluate(u0_max);
        let p01 = c1.evaluate(u1_min);
        let p11 = c1.evaluate(u1_max);

        // 幾何パッチ（Coons）の生成
        let coons = CoonsPatch3::new(c0.clone(), c1.clone(), d0.clone(), d1.clone(), tol)?;

        // B-Rep トポロジーの構築
        let v00 = Vertex::new(p00, tol.linear);
        let v10 = Vertex::new(p10, tol.linear);
        let v01 = Vertex::new(p01, tol.linear);
        let v11 = Vertex::new(p11, tol.linear);

        let e_bottom = Edge::new(c0, v00.clone(), v10.clone(), tol.linear);
        let e_top = Edge::new(c1, v01.clone(), v11.clone(), tol.linear);
        let e_left = Edge::new(d0, v00.clone(), v01.clone(), tol.linear);
        let e_right = Edge::new(d1, v10.clone(), v11.clone(), tol.linear);

        // 外側境界ワイヤのループ構築: bottom -> right -> top(rev) -> left(rev)
        let wire = Wire::new(vec![
            OrientedEdge::forward(e_bottom),
            OrientedEdge::forward(e_right),
            OrientedEdge::reversed(e_top),
            OrientedEdge::reversed(e_left),
        ]);

        if !wire.is_closed(tol) {
            return Err("Constructed wire is not topologically closed".to_string());
        }

        Ok(Face::from_coons_patch(coons, wire))
    }
}
