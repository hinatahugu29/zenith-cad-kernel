//! 断面が、平面の置き方とテッセレーションの細かさによらず取れるか。
//!
//! 断面はメッシュと平面の交線から作る。頂点がちょうど平面の上に乗ったときの
//! 扱いを場合分けで決めているので、答えが**三角形の並び方に依存**しうる。
//! 依存していれば、分割数を変えたり平面を格子行にぴったり合わせたりしただけで
//! 輪郭が閉じなくなる。
//!
//! ここでは、格子行にちょうど乗る高さと、わざと外した高さの両方を、複数の
//! 分割数で切る。断面積の解析解が分かるものは値も突き合わせる。

use std::f64::consts::PI;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, HoleBuilder, PrimitiveBuilder, SectionSlicer,
};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::TessellationParams;
use zenith_topo::Solid;

struct Subject {
    name: &'static str,
    solid: Solid,
    /// (z, 期待する断面積)
    planes: Vec<(f64, Option<f64>)>,
}

fn main() {
    let tol = Tolerance::default();
    let densities = [4usize, 6, 8, 11, 12, 16, 20, 24, 32];

    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let bore = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
        Vec3::new(20.0, 20.0, -10.0),
    );
    let bored =
        BooleanEngine::boolean_solids_exact(&block, &bore, BooleanOpType::Difference, &tol).unwrap();

    let mut subjects = vec![
        Subject {
            name: "box 40x40x20",
            solid: block.clone(),
            // 10.0 は多くの分割数で格子行にぴったり乗る。7.3 は乗らない。
            planes: vec![(10.0, Some(1600.0)), (7.3, Some(1600.0))],
        },
        Subject {
            name: "cylinder r10 h40",
            solid: PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap(),
            planes: vec![(20.0, Some(PI * 100.0)), (13.7, Some(PI * 100.0))],
        },
        Subject {
            name: "boolean bored block",
            solid: bored,
            planes: vec![
                (10.0, Some(1600.0 - PI * 36.0)),
                (6.4, Some(1600.0 - PI * 36.0)),
            ],
        },
        Subject {
            name: "drilled box (builder)",
            solid: HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap(),
            planes: vec![
                (10.0, Some(1600.0 - PI * 64.0)),
                (4.9, Some(1600.0 - PI * 64.0)),
            ],
        },
        Subject {
            name: "sphere r12",
            solid: PrimitiveBuilder::make_sphere(12.0).unwrap(),
            planes: vec![(0.0, Some(PI * 144.0)), (3.1, Some(PI * (144.0 - 9.61)))],
        },
    ];
    subjects.push(Subject {
        name: "torus R12 r4",
        solid: PrimitiveBuilder::make_torus(12.0, 4.0).unwrap(),
        planes: vec![(0.0, None), (1.7, None)],
    });

    print!("{:<24}{:>7}", "subject", "z");
    for density in densities {
        print!("{:>9}", density);
    }
    println!();
    println!("{}", "-".repeat(31 + 9 * densities.len()));

    let mut failures = 0;
    let mut wrong_area = 0;

    for subject in &subjects {
        for (z, expected) in &subject.planes {
            print!("{:<24}{:>7.1}", subject.name, z);
            for density in densities {
                let params = TessellationParams {
                    u_divisions: density,
                    v_divisions: density,
                };
                let outcome = SectionSlicer::slice_solid_with_tessellation(
                    &subject.solid,
                    Point3::new(0.0, 0.0, *z),
                    Vec3::new(0.0, 0.0, 1.0),
                    &tol,
                    &params,
                );
                match outcome {
                    Ok(result) => match expected {
                        Some(want) => {
                            let error = (result.total_area - want).abs() / want.abs();
                            // 粗い刻みでの誤差は近似の精度であって、置き方への
                            // 依存ではない。12分割以上で解析解に乗ることを求める
                            // （収束そのものは `section_slice_test` が見ている）。
                            if error > 1e-6 {
                                print!("{:>9}", format!("{error:.0e}"));
                                if density >= 12 {
                                    wrong_area += 1;
                                }
                            } else {
                                print!("{:>9}", "ok");
                            }
                        }
                        None => print!("{:>9}", "ok"),
                    },
                    Err(_) => {
                        print!("{:>9}", "FAILED");
                        failures += 1;
                    }
                }
            }
            println!();
        }
    }

    println!("{}", "-".repeat(31 + 9 * densities.len()));
    println!("sections that could not be closed                    : {failures}");
    println!("sections off the closed form at 12 divisions or finer: {wrong_area}");
    println!();
    println!("The heights are chosen so that some land exactly on a tessellation row and");
    println!("some do not. Numbers in the table are the relative area error where it is");
    println!("above 1e-6; at 4 to 8 divisions that is the approximation, not the placement.");
    if failures > 0 || wrong_area > 0 {
        std::process::exit(1);
    }
    println!("every section closed at every tessellation and placement tried");
}
