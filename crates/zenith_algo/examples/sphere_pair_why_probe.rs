//! 球の周期1面化（ROADMAP柱③）がブーリアンで断られる理由を、
//! `ZENITH_PAIR_WHY=1`で広域の絞り込み（候補にするかの判定）まで
//! 遡って見るための検体（HANDOVER 4-823）。
//!
//! 使い方: `ZENITH_PAIR_WHY=1 cargo run -p zenith_algo --example sphere_pair_why_probe`
//!
//! 立方体6面中3面が「囲み箱が交わらない」で候補から落ちることが
//! 見える——`face_boundary_bbox`が面の境界（輪）だけから箱を作るため、
//! 輪が子午線1本（y=0の1平面上）を2回使うだけの球の周期1面では、
//! y方向にほぼ幅0の箱になってしまう。`ZENITH_WIDE_FACE_BBOX=1`を
//! 重ねると、この絞り込み自体は直る（候補が4組→10組に増える）が、
//! ブーリアン自体はまだ別の理由（輪を閉じる側の問題）で断られる。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let periodic = PrimitiveBuilder::make_sphere_periodic(radius).unwrap();
    let box_solid = PrimitiveBuilder::make_box(12.0, 12.0, 12.0).unwrap();
    let box_centered = BrepTransform::translate_solid(&box_solid, Vec3::new(-6.0, -6.0, -6.0));
    let result = BooleanEngine::boolean_solids_exact(&box_centered, &periodic, BooleanOpType::Union, &tol);
    println!("{:?}", result.map(|s| s.outer_shell.faces.len()));
}
