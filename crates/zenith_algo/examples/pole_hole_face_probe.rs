//! **4-842 を直すときの、いちばん短い検体。** ブーリアンを回さずに、
//! 極を囲む帯状の穴を1枚だけ持つ面を組み、`face_uv_triangulation` と
//! `face_parameter_area` が何を返すかを直接見る（10秒で回ります）。
//!
//! 期待する直りかた: 穴が「回る向きに1周ぶんの帯」のとき、
//! **穴ではなく緯度での切り落とし（キャップ）**として引くこと。
//! 正しい uv 面積は「帯より下だけ」＝ 1 - (1 - v_band) ではなく、
//! 3D 面積で 1188.773（球R10 の全面積 1256.637 から窓 67.864 を引いた値）。
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;

fn main() {
    let tol = Tolerance::default();
    // 北極に窓を開けた和を1回だけ作り、球の面（穴1つ）を取り出す。
    let periodic = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(-4.0, -4.0, 5.0),
    );
    let result =
        BooleanEngine::boolean_solids_exact(&periodic, &tool, BooleanOpType::Union, &tol).unwrap();
    let face = result
        .outer_shell
        .faces
        .iter()
        .find(|f| !f.inner_wires.is_empty())
        .expect("穴を持つ球の面");

    let params = TessellationParams::default();
    let domain = zenith_tess::face_uv_triangulation(face, &params);
    println!(
        "穴1つの球の面: 三角形 {} 枚、uv面積の検算 {:?}",
        domain.triangles.len(),
        zenith_tess::face_parameter_area(face)
    );
    let (area, volume) = MassCalculator::compute_face_integral(face, &params);
    println!("  いまの面積 {area:.6}（正しくは 1188.773）体積寄与 {volume:.6}");
    // **どちら側が材料か**（4-842 の直しが要る最後の情報）。帯より上（v が
    // 大きい側）に極があり、そこが**落とされる**側なら、引く量は
    // 「帯から v_max までのキャップ」です。
    if let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry {
        for v in [0.70f64, 0.8057, 0.8642, 0.95, 1.0] {
            let point = surface.evaluate(0.25, v);
            println!("  v={v:.4} → z {:.4}（箱は z 5〜13 を覆う）", point.z);
        }
    }
    if let Ok(p) = face.pcurves(&tol) {
        for (which, l) in std::iter::once(("外周", &p.outer_loop))
            .chain(p.inner_loops.iter().map(|l| ("穴", l)))
        {
            let mut lo = (f64::MAX, f64::MAX);
            let mut hi = (f64::MIN, f64::MIN);
            for seg in &l.segments {
                for c in &seg.curve.control_points {
                    lo = (lo.0.min(c.point.x), lo.1.min(c.point.y));
                    hi = (hi.0.max(c.point.x), hi.1.max(c.point.y));
                }
            }
            println!(
                "  {which} 区間{} uv ({:.4},{:.4})-({:.4},{:.4}){}",
                l.segments.len(),
                lo.0,
                lo.1,
                hi.0,
                hi.1,
                if hi.0 - lo.0 >= 1.0 - 1e-6 { "  ← u が1周ぶん（帯）" } else { "" }
            );
        }
    }
}
