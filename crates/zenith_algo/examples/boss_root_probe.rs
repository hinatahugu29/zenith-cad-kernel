//! 直方体のボスの根元（凹んだ 270 度の直線稜）を1本ずつ丸め、返った立体が
//! 閉じているか、足された体積が閉形式 `(1 - π/4) r² L` と合うかを見る（4-938）。
//!
//! ```bash
//! ZENITH_CONCAVE_LINE_BLEND=1 cargo run --release -p zenith_algo --example boss_root_probe
//! ```

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, EdgeBlender, MassCalculator, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
}

fn main() {
    let tolerance = Tolerance::default();
    let base = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).expect("base");
    let boss = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(12.0, 10.0, 12.0).expect("boss"),
        Vec3::new(14.0, 15.0, 20.0),
    );
    let solid =
        BooleanEngine::boolean_solids_exact_simplified(&base, &boss, BooleanOpType::Union, &tolerance)
            .expect("union");
    let v0 = volume(&solid);
    let mut seen = Vec::new();
    for face in &solid.outer_shell.faces {
        for oriented in &face.outer_wire.edges {
            let edge = &oriented.edge;
            let (a, b) = (edge.start_vertex.point, edge.end_vertex.point);
            let root = (a.z - 20.0).abs() < 1e-9
                && (b.z - 20.0).abs() < 1e-9
                && [a.x, b.x].iter().all(|x| (14.0..=26.0).contains(x))
                && [a.y, b.y].iter().all(|y| (15.0..=25.0).contains(y));
            if !root || seen.contains(&edge.id) {
                continue;
            }
            seen.push(edge.id);
            let length = (b - a).norm();
            let r = 1.0;
            for (kind, expected) in [
                ("fillet", (1.0 - std::f64::consts::FRAC_PI_4) * r * r * length),
                ("chamfer", 0.5 * r * r * length),
            ] {
                let result = if kind == "fillet" {
                    EdgeBlender::fillet_edge(&solid, edge.id, r)
                } else {
                    EdgeBlender::chamfer_edge(&solid, edge.id, r)
                };
                match result {
                    Ok(out) => {
                        let closed = out.outer_shell.validate_closed(&tolerance).is_valid();
                        let added = volume(&out) - v0;
                        println!(
                            "edge {} L={length} {kind}: 閉={closed} 足した体積={added:.9} 予測={expected:.9} 相対差={:.2e}",
                            edge.id,
                            (added - expected).abs() / expected
                        );
                    }
                    Err(e) => println!("edge {} L={length} {kind}: 断り {e}", edge.id),
                }
            }
        }
    }
}
