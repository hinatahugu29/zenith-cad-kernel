//! 傾けた切り手で切れること。
//!
//! **軸に平行な切り手は特別な場合です。** 面の組がすべて座標面に乗るので、
//! 平面同士の交線も等パラメータ線も易しくなります。27 度傾けると、そこが
//! 全部一般の場合になり、30配置で拒否 0 だったものが 180演算中 27件の拒否に
//! なりました（4-61）。
//!
//! ここで押さえるのは、そこから直した3つが効いたままであることです。
//!
//! - 交線が隣のパッチへ続かず輪が閉じない（4-62）
//! - 鎖が1本でも当たると残りが捨てられる（4-63）
//! - クリップの内外判定が折れ線で、切れる位置が境界からずれる（4-64）
//!
//! 期待値は OpenCASCADE に同じ配置を計算させたものです
//! （`occ_cut_reference.py <検体> "tilted drill" --box ...`）。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn volume(solids: &[Solid]) -> f64 {
    solids
        .iter()
        .map(|solid| MassCalculator::compute_from_brep(solid, &params()).volume)
        .sum()
}

fn fixture(name: &str) -> Solid {
    let path = std::path::PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/occ_reference_"
    ));
    let path = std::path::PathBuf::from(format!("{}{name}.step", path.display()));
    zenith_io::StepImporter::import_solids_from_file(&path)
        .expect("the fixture must be readable")
        .into_iter()
        .next()
        .expect("one solid")
}

/// 境界箱の中心まわりに 27 度傾ける。`foreign_boolean_probe` の
/// `ZENITH_TILTED=1` と同じ変換です。
fn tilt(solid: &Solid, centre: Vec3) -> Solid {
    let transform = Transform3::from_translation(centre)
        .compose(&Transform3::from_axis_angle(
            &Vec3::new(1.0, 1.0, 1.0),
            27f64.to_radians(),
        ))
        .compose(&Transform3::from_translation(-centre));
    BrepTransform::transform_solid(solid, &transform).expect("the tilt must apply")
}

/// 境界箱 (-10,-10,0)-(10,10,20) に対する `centre drill` を傾けたもの。
fn tilted_drill() -> Solid {
    let upright = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(3.6, 60.0).expect("drill"),
        Vec3::new(0.0, 0.0, -20.0),
    );
    tilt(&upright, Vec3::new(0.0, 0.0, 10.0))
}

fn run(subject: &str, op: BooleanOpType) -> Vec<Solid> {
    let tol = Tolerance::default();
    BooleanEngine::boolean_solids_exact_result(&fixture(subject), &tilted_drill(), op, &tol)
        .unwrap_or_else(|err| {
            panic!(
                "{subject} / tilted drill / {op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
}

/// 全周円錐。側面が4分の1に割れていて、交線は4枚をまたぐ閉曲線になります。
/// **交線が隣のパッチへ続かないと、ここで輪が閉じません**（4-62）。
mod cone_full {
    use super::*;

    const VOLUME: f64 = 2094.395102;
    /// OpenCASCADE（`occ_cut_reference.py cone_full "tilted drill" --box -10 -10 0 10 10 20`）。
    const OCC_DIFFERENCE: f64 = 1453.770752;

    #[test]
    fn the_tilted_drill_goes_through() {
        let got = volume(&run("cone_full", BooleanOpType::Difference));
        assert!(
            (got - VOLUME).abs() > 1e-6,
            "the difference came back as the untouched cone ({got})"
        );
        // 切り口が曲面どうしの交線で囲まれるので、桁いっぱいでは合いません。
        let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
        assert!(
            relative <= 1e-4,
            "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
        );
    }

    #[test]
    fn the_two_halves_add_back_up() {
        let difference = volume(&run("cone_full", BooleanOpType::Difference));
        let intersection = volume(&run("cone_full", BooleanOpType::Intersection));
        assert!(
            intersection > VOLUME * 1e-3,
            "the intersection came out empty ({intersection})"
        );
        let relative = (difference + intersection - VOLUME).abs() / VOLUME;
        assert!(
            relative <= 1e-7,
            "V(A-B) + V(A^B) = {} against V(A) = {VOLUME} (relative {relative:.3e})",
            difference + intersection
        );
    }
}

/// 切頭円錐。上面が円で、**切り口の端がその円に乗らなければなりません**。
/// クリップが折れ線で切ると 4.6e-4 内側に落ち、分割が断ります（4-64）。
mod cone {
    use super::*;

    const VOLUME: f64 = 3267.256360;
    /// OpenCASCADE（`occ_cut_reference.py cone "tilted drill" --box -10 -10 0 10 10 20`）。
    const OCC_DIFFERENCE: f64 = 2456.083364;

    #[test]
    fn the_tilted_drill_goes_through() {
        let got = volume(&run("cone", BooleanOpType::Difference));
        assert!(
            (got - VOLUME).abs() > 1e-6,
            "the difference came back as the untouched cone ({got})"
        );
        let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
        assert!(
            relative <= 1e-5,
            "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
        );
    }

    #[test]
    fn the_two_halves_add_back_up() {
        let difference = volume(&run("cone", BooleanOpType::Difference));
        let intersection = volume(&run("cone", BooleanOpType::Intersection));
        assert!(
            intersection > VOLUME * 1e-3,
            "the intersection came out empty ({intersection})"
        );
        let relative = (difference + intersection - VOLUME).abs() / VOLUME;
        assert!(
            relative <= 1e-7,
            "V(A-B) + V(A^B) = {} against V(A) = {VOLUME} (relative {relative:.3e})",
            difference + intersection
        );
    }
}

/// 全周トーラス。管の底の継ぎ目で四半パッチが2枚出会い、**同じ2点を結ぶ
/// 別々の弧**が交線として出ます（中点は 3.839 離れています）。稜を端点だけで
/// 見分けていたころは、閉じた殻なのに非多様体と報告されて断られていました
/// （4-65）。
mod torus {
    use super::*;

    const VOLUME: f64 = 3789.928090;
    /// OpenCASCADE（`occ_cut_reference.py torus "tilted slab" --box -16 -16 -4 16 16 4`）。
    const OCC_DIFFERENCE: f64 = 1937.013599;

    /// 境界箱 (-16,-16,-4)-(16,16,4) に対する `half slab` を傾けたもの。
    fn tilted_slab() -> Solid {
        let upright = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(19.2, 64.0, 16.0).expect("slab"),
            Vec3::new(-19.52, -32.0, -8.0),
        );
        tilt(&upright, Vec3::new(0.0, 0.0, 0.0))
    }

    fn cut(op: BooleanOpType) -> Vec<Solid> {
        let tol = Tolerance::default();
        BooleanEngine::boolean_solids_exact_result(&fixture("torus"), &tilted_slab(), op, &tol)
            .unwrap_or_else(|err| {
                panic!(
                    "torus / tilted slab / {op:?} refused: {}",
                    err.chars().take(140).collect::<String>()
                )
            })
            .solids
    }

    #[test]
    fn the_tilted_slab_takes_a_piece() {
        let got = volume(&cut(BooleanOpType::Difference));
        assert!(
            (got - VOLUME).abs() > 1e-6,
            "the difference came back as the untouched torus ({got})"
        );
        let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
        assert!(
            relative <= 1e-4,
            "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
        );
    }

    #[test]
    fn the_two_halves_add_back_up() {
        let difference = volume(&cut(BooleanOpType::Difference));
        let intersection = volume(&cut(BooleanOpType::Intersection));
        assert!(
            intersection > VOLUME * 1e-3,
            "the intersection came out empty ({intersection})"
        );
        let relative = (difference + intersection - VOLUME).abs() / VOLUME;
        assert!(
            relative <= 1e-7,
            "V(A-B) + V(A^B) = {} against V(A) = {VOLUME} (relative {relative:.3e})",
            difference + intersection
        );
    }
}

/// 輪（円環のキャップを持つ）。**穴のある面を傾いた切り込みで割ります。**
/// 面積の判定が多角形で粗かった件、鎖の継ぎ目に境界を要求していた件、
/// 「穴を横切る」を狭く取りすぎていた件——3つとも、ここで断っていました
/// （4-66）。
mod revolved_ring {
    use super::*;

    const VOLUME: f64 = 1583.362697;
    /// OpenCASCADE（`occ_cut_reference.py revolved_ring "tilted slab" --box -10 -10 0 10 10 6`）。
    const OCC_DIFFERENCE: f64 = 808.697682;

    /// 境界箱 (-10,-10,0)-(10,10,6) に対する `half slab` を傾けたもの。
    fn tilted_slab() -> Solid {
        let upright = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_box(12.0, 40.0, 12.0).expect("slab"),
            Vec3::new(-12.2, -20.0, -3.0),
        );
        tilt(&upright, Vec3::new(0.0, 0.0, 3.0))
    }

    fn cut(op: BooleanOpType) -> Vec<Solid> {
        let tol = Tolerance::default();
        BooleanEngine::boolean_solids_exact_result(
            &fixture("revolved_ring"),
            &tilted_slab(),
            op,
            &tol,
        )
        .unwrap_or_else(|err| {
            panic!(
                "revolved_ring / tilted slab / {op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
    }

    #[test]
    fn the_tilted_slab_takes_a_piece() {
        let got = volume(&cut(BooleanOpType::Difference));
        assert!(
            (got - VOLUME).abs() > 1e-6,
            "the difference came back as the untouched ring ({got})"
        );
        let relative = (got - OCC_DIFFERENCE).abs() / OCC_DIFFERENCE;
        assert!(
            relative <= 1e-6,
            "difference {got} against OpenCASCADE's {OCC_DIFFERENCE} (relative {relative:.3e})"
        );
    }

    /// **穴が残っていること。** 円環を割ったあとも、内側の壁は結果の一部です。
    #[test]
    fn the_bore_survives() {
        let got = volume(&cut(BooleanOpType::Difference));
        // 穴を失えば、その体積ぶん増えます（π·16·6 のうち切り残るぶん）。
        let solid_disc = std::f64::consts::PI * 100.0 * 6.0;
        assert!(
            got < solid_disc * 0.6,
            "the bore was lost: {got} against the solid disc {solid_disc}"
        );
    }

    #[test]
    fn the_two_halves_add_back_up() {
        let difference = volume(&cut(BooleanOpType::Difference));
        let intersection = volume(&cut(BooleanOpType::Intersection));
        assert!(
            intersection > VOLUME * 1e-3,
            "the intersection came out empty ({intersection})"
        );
        let relative = (difference + intersection - VOLUME).abs() / VOLUME;
        assert!(
            relative <= 1e-7,
            "V(A-B) + V(A^B) = {} against V(A) = {VOLUME} (relative {relative:.3e})",
            difference + intersection
        );
    }

    /// 境界箱 (-10,-10,0)-(10,10,6) に対する `centre drill` を傾けたもの。
    ///
    /// **切り込みが穴の縁から出て穴の縁へ戻ります。** 答えの片方は
    /// 「外周を外側の輪、[穴の弧＋切り込み] を内側の輪」に持つ**穴のある面**
    /// で、そこを組み立てられるようになったのが 4-67 です。
    fn tilted_drill_for_ring() -> Solid {
        let upright = BrepTransform::translate_solid(
            &PrimitiveBuilder::make_cylinder(3.6, 18.0).expect("drill"),
            Vec3::new(0.0, 0.0, -6.0),
        );
        tilt(&upright, Vec3::new(0.0, 0.0, 3.0))
    }

    fn drill(op: BooleanOpType) -> Vec<Solid> {
        let tol = Tolerance::default();
        BooleanEngine::boolean_solids_exact_result(
            &fixture("revolved_ring"),
            &tilted_drill_for_ring(),
            op,
            &tol,
        )
        .unwrap_or_else(|err| {
            panic!(
                "revolved_ring / tilted drill / {op:?} refused: {}",
                err.chars().take(140).collect::<String>()
            )
        })
        .solids
    }

    /// OpenCASCADE（`occ_cut_reference.py revolved_ring "tilted drill" --box -10 -10 0 10 10 6`）。
    const OCC_DRILL_DIFFERENCE: f64 = 1567.804501;

    #[test]
    fn the_tilted_drill_bites_the_bore() {
        let got = volume(&drill(BooleanOpType::Difference));
        assert!(
            (got - VOLUME).abs() > 1e-6,
            "the difference came back as the untouched ring ({got})"
        );
        let relative = (got - OCC_DRILL_DIFFERENCE).abs() / OCC_DRILL_DIFFERENCE;
        assert!(
            relative <= 1e-6,
            "difference {got} against OpenCASCADE's {OCC_DRILL_DIFFERENCE} (relative {relative:.3e})"
        );
    }

    #[test]
    fn the_drilled_halves_add_back_up() {
        let difference = volume(&drill(BooleanOpType::Difference));
        let intersection = volume(&drill(BooleanOpType::Intersection));
        assert!(
            intersection > 1.0,
            "the intersection came out empty ({intersection})"
        );
        let relative = (difference + intersection - VOLUME).abs() / VOLUME;
        assert!(
            relative <= 1e-7,
            "V(A-B) + V(A^B) = {} against V(A) = {VOLUME} (relative {relative:.3e})",
            difference + intersection
        );
    }
}
