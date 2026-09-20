//! **軸に平行な平面で円錐を切ると、双曲線**——それを厳密に出すこと（4-500）。
//!
//! これまでは辿っていました。**尖った円錐では、それが通りません**——
//! **相対 1e-6 の交線を 2048 点で辿れず**（4-495）、**緩めれば p-curve が
//! 読めず**（4-497）、**もっと緩めれば面積が合わない**（4-499）。
//! **3 枚とも、辿ることから来ていました。**
//!
//! 見るのは 2 つです。
//!
//! * **解析で出ていること**（`analytic` の旗。辿っていたら立ちません）
//! * **本当に双曲線であること**——標本した点が、**平面の上**にあり、
//!   **軸からの距離がその高さの円錐の半径と一致**すること。
//!   **円錐の半径は高さの 1 次式**なので、**閉じた式で測れます**。

use zenith_algo::{BrepIntersectionBuilder, BrepTransform, FaceIntersectionKind, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

const RADIUS: f64 = 5.0;

/// その高さでの円錐の半径（閉じた式）。
fn radius_at(height: f64, z: f64) -> f64 {
    RADIUS * (1.0 - z / height)
}

fn check_cone_of_height(height: f64) {
    let tol = Tolerance::default();
    let cone = PrimitiveBuilder::make_cone(RADIUS, 0.0, height).expect("円錐");
    // 箱は `aspect_sweep_probe` と同じ置き方——**切り口の平面 x=2 は軸に平行**。
    let box_solid = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(40.0, 40.0, height * 4.0).expect("箱"),
        Vec3::new(2.0, -20.0, -height),
    );

    let candidates = BrepIntersectionBuilder::collect_face_pair_candidates(
        &cone.outer_shell.faces,
        &box_solid.outer_shell.faces,
        &tol,
    );

    let mut curved = 0usize;
    for candidate in &candidates {
        let edges: Vec<_> = match &candidate.kind {
            FaceIntersectionKind::Curve { edge } => vec![edge.clone()],
            FaceIntersectionKind::Curves { edges } => edges.clone(),
            _ => continue,
        };
        for edge in edges {
            // 底面の円板が作る**まっすぐな弦**は、双曲線ではありません。
            // 見たいのは、側面が作る曲がった弧のほうです。
            let middle = edge.evaluate(0.5);
            let straight = (edge.start_vertex.point.z - middle.z).abs() < 1e-9
                && (edge.end_vertex.point.z - middle.z).abs() < 1e-9;
            if straight {
                continue;
            }
            curved += 1;
            assert!(
                candidate.analytic,
                "高さ {height} の側面の交線が、辿りで出ています（解析で出るはずです）"
            );
            for step in 0..=32 {
                let point = edge.evaluate(step as f64 / 32.0);
                assert!(
                    (point.x - 2.0).abs() < 1e-9,
                    "高さ {height}: 点が平面 x=2 から {} 離れています",
                    (point.x - 2.0).abs()
                );
                let from_axis = (point.x * point.x + point.y * point.y).sqrt();
                let closed = radius_at(height, point.z);
                assert!(
                    (from_axis - closed).abs() < 1e-9 * height.max(1.0),
                    "高さ {height}: z={} で軸から {from_axis}、閉じた式は {closed}",
                    point.z
                );
            }
        }
    }
    assert!(
        curved >= 2,
        "高さ {height}: 側面の交線が {curved} 本しかありません（2 本以上あるはずです）"
    );
}

#[test]
fn a_plane_parallel_to_a_cone_axis_cuts_an_exact_hyperbola() {
    // **低い円錐から尖った円錐まで。** 尖ったほうが、辿りでは出せません。
    for height in [10.0_f64, 50.0, 200.0, 1000.0] {
        check_cone_of_height(height);
    }
}
