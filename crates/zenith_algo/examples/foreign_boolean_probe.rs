//! 他カーネルが書いた立体を読んで、それにブーリアンを掛ける。
//!
//! # なぜこれが要るか
//!
//! 既存のプローブは2本の経路を見ています。`closure_probe` は「自分が作った
//! 立体を、自分のブーリアンと STEP 往復に渡す」。`foreign_reexport` は
//! 「他カーネルの立体を読んで、書き戻す」。**読んだ立体をブーリアンに渡す
//! 経路だけが測られていません。**
//!
//! 実務ではそこが最初に来ます。客先のファイルを開いて、穴をあける。
//! 自作の立体と違い、読んだ立体の曲面は他人の媒介変数・他人のノット列・
//! 他人のトリム境界を持っています。ビルダーが作る整った曲面で通ったことは、
//! ここで通る保証になりません。
//!
//! # どう測るか — 解析解を使わない
//!
//! 読んだ立体の体積の閉じた式は、検体によっては書けます。しかし切った後の
//! 体積は書けません。切り方に合わせて式を立てると、**式のほうを間違えた
//! ときに気づけません**（4-33 で実際にやりました）。
//!
//! そこで、形に依らず成り立つ**恒等式**で見ます。任意の A, B について:
//!
//! ```text
//!   V(A - B) + V(A ∩ B) = V(A)              … 分割の恒等式
//!   V(A ∪ B) = V(A) + V(B) - V(A ∩ B)       … 包除の恒等式
//! ```
//!
//! この2本は、A が何であっても、B をどこに置いても成り立ちます。3つの演算が
//! **揃って**間違わない限り破れるので、1つの演算の誤りは必ず出ます。
//! しかも A の体積の真値を知らなくても使えます。
//!
//! ここで測っている体積はすべて同じテッセレーション設定で求めており、
//! 恒等式の残差には求積の誤差も乗ります。相対 1e-6 は求積側の実力
//! （4-19 の側面の残差）で決めた線で、これを割るのは「揃って間違えた」
//! ときだけです。

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;

use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, MassCalculator, PrimitiveBuilder, Regularizer,
};
use zenith_io::StepImporter;
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams, TriangleMesh};
use zenith_topo::{FaceGeometry, Solid};

/// 求積の刻み。恒等式の両辺で同じものを使う。
fn params() -> TessellationParams {
    TessellationParams {
        u_divisions: 64,
        v_divisions: 64,
    }
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_from_brep(solid, &params()).volume
}

fn total_volume(solids: &[Solid]) -> f64 {
    solids.iter().map(volume).sum()
}

fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for vertex in &mesh.positions {
        low.x = low.x.min(vertex.x);
        low.y = low.y.min(vertex.y);
        low.z = low.z.min(vertex.z);
        high.x = high.x.max(vertex.x);
        high.y = high.y.max(vertex.y);
        high.z = high.z.max(vertex.z);
    }
    (low, high)
}

/// 検体の境界箱から、切り手の置き方を決める。
///
/// 検体ごとに寸法が違うので、絶対座標では書けない。境界箱に対する比で置く。
struct Placement {
    name: &'static str,
    build: fn(&Point3, &Point3) -> Result<Solid, String>,
}

/// 境界箱の x 側の半分を覆う箱。端をわずかにずらして、面が重なる配置を避ける。
fn half_slab(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let solid = PrimitiveBuilder::make_box(size.x * 0.6, size.y * 2.0, size.z * 2.0)?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            low.x - size.x * 0.11,
            low.y - size.y * 0.5,
            low.z - size.z * 0.5,
        ),
    ))
}

/// 検体の中心を z 方向に貫く丸穴。半径は境界箱の 18%。
fn centre_drill(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let radius = size.x.min(size.y) * 0.18;
    let height = size.z * 3.0;
    let solid = PrimitiveBuilder::make_cylinder(radius, height)?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            (low.x + high.x) * 0.5,
            (low.y + high.y) * 0.5,
            low.z - size.z,
        ),
    ))
}

/// 境界箱の角を落とす箱。頂点まわりの面の集まりを触る。
fn corner_block(low: &Point3, high: &Point3) -> Result<Solid, String> {
    let size = Vec3::new(high.x - low.x, high.y - low.y, high.z - low.z);
    let solid = PrimitiveBuilder::make_box(size.x * 0.45, size.y * 0.45, size.z * 0.45)?;
    Ok(BrepTransform::translate_solid(
        &solid,
        Vec3::new(
            high.x - size.x * 0.30,
            high.y - size.y * 0.30,
            high.z - size.z * 0.30,
        ),
    ))
}

fn placements() -> Vec<Placement> {
    vec![
        Placement {
            name: "half slab",
            build: half_slab,
        },
        Placement {
            name: "centre drill",
            build: centre_drill,
        },
        Placement {
            name: "corner block",
            build: corner_block,
        },
    ]
}

/// 読んだ立体を、自前のビルダーが作るのと同じ持ち方に整えてから渡す。
///
/// 2つのことをします。どちらも形は変えません。
///
/// 1. 制御点が1平面に乗っている NURBS 面を、平面として持ち直す（`as_plane`）。
///    これは近似ではなく凸包の性質で決まります。
/// 2. 全周1枚の面と全周1本の辺を刻む（`Regularizer`）。
///
/// 1 が先で、順序が要ります。平面の p-curve は射影で厳密に出せるので、
/// 平面として持ち直した面は「p-curve を失うと積分が変わる面」ではなくなり、
/// 正規化が上下の円を刻めるようになります。NURBS のままだと守られたままで、
/// **正規化は何もしません**（実測: 割った面 0、残した 1）。
///
/// `ZENITH_FOREIGN_CONDITION=1` で有効になります。既定は素のままです。
fn condition(solid: &Solid, tol: &Tolerance) -> Solid {
    let mut out = solid.clone();
    for face in &mut out.outer_shell.faces {
        if let FaceGeometry::Nurbs(surface) = &face.geometry {
            if let Some(plane) = surface.as_plane() {
                face.geometry = FaceGeometry::Plane(plane);
                face.pcurves = face.derive_plane_pcurves().ok();
            }
        }
    }
    let (regularized, _) = Regularizer::regularize_solid(&out, tol);
    regularized
}

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
        .join(format!("occ_reference_{name}.step"))
}

/// 演算1つを走らせて、体積か、断られた理由を返す。
enum Outcome {
    Volume(f64),
    Refused(String),
    Panicked,
}

fn run(a: &Solid, b: &Solid, op: BooleanOpType, tol: &Tolerance) -> Outcome {
    match catch_unwind(AssertUnwindSafe(|| {
        BooleanEngine::boolean_solids_exact_result_unverified(a, b, op, tol)
    })) {
        Err(_) => Outcome::Panicked,
        Ok(Err(err)) => Outcome::Refused(err.split(';').next().unwrap_or(&err).to_string()),
        Ok(Ok(result)) => {
            let closed = result
                .solids
                .iter()
                .all(|s| s.outer_shell.validate_closed(tol).is_valid());
            if !closed {
                return Outcome::Refused("returned an open shell".to_string());
            }
            Outcome::Volume(total_volume(&result.solids))
        }
    }
}

fn main() {
    let tol = Tolerance::default();
    let subjects = [
        "cone",
        "cone_full",
        "cylinder_nurbs",
        "elliptic_prism",
        "extruded_spline",
        "revolved_ring",
        "sphere",
        "sphere_capped",
        "torus",
        "torus_segment",
    ];

    println!(
        "{:<18} {:<14} {:>14} {:>14} {:>14} {:>11} {:>11}  {}",
        "subject", "cutter", "V(A-B)", "V(A^B)", "V(AuB)", "split", "incl-excl", "verdict"
    );
    println!("{}", "-".repeat(126));

    // ok / refused / WRONG / PANIC
    let mut tally = [0usize; 4];
    let mut worst: f64 = 0.0;

    for name in subjects {
        let solids = match StepImporter::import_solids_from_file(&fixture(name)) {
            Ok(solids) if !solids.is_empty() => solids,
            Ok(_) => {
                println!("{name:<18} could not be read: no solids in the file");
                tally[1] += 1;
                continue;
            }
            Err(err) => {
                println!(
                    "{name:<18} could not be read: {}",
                    err.chars().take(70).collect::<String>()
                );
                tally[1] += 1;
                continue;
            }
        };
        let conditioned;
        let a = if std::env::var_os("ZENITH_FOREIGN_CONDITION").is_some() {
            conditioned = condition(&solids[0], &tol);
            &conditioned
        } else {
            &solids[0]
        };
        let volume_a = volume(a);
        let (low, high) = mesh_bounds(&tessellate_solid(a, &params()));

        for placement in placements() {
            let b = match (placement.build)(&low, &high) {
                Ok(b) => b,
                Err(err) => {
                    println!(
                        "{name:<18} {:<14} the cutter could not be built: {err}",
                        placement.name
                    );
                    tally[1] += 1;
                    continue;
                }
            };
            let volume_b = volume(&b);

            let difference = run(a, &b, BooleanOpType::Difference, &tol);
            let intersection = run(a, &b, BooleanOpType::Intersection, &tol);
            let union = run(a, &b, BooleanOpType::Union, &tol);

            // 断られたものが1つでもあれば恒等式は組めない。断ること自体は
            // 欠陥ではないので、理由を出して次へ。
            let (v_diff, v_inter, v_union) = match (&difference, &intersection, &union) {
                (Outcome::Volume(d), Outcome::Volume(i), Outcome::Volume(u)) => (*d, *i, *u),
                _ => {
                    for (op_name, outcome) in [
                        ("difference", &difference),
                        ("intersection", &intersection),
                        ("union", &union),
                    ] {
                        match outcome {
                            Outcome::Panicked => {
                                println!("{name:<18} {:<14} {op_name:<12} PANIC", placement.name);
                                tally[3] += 1;
                            }
                            Outcome::Refused(err) => {
                                println!(
                                    "{name:<18} {:<14} {op_name:<12} refused: {}",
                                    placement.name,
                                    err.chars().take(58).collect::<String>()
                                );
                                tally[1] += 1;
                            }
                            Outcome::Volume(..) => {}
                        }
                    }
                    continue;
                }
            };

            let split = (v_diff + v_inter - volume_a).abs() / volume_a;
            let incl_excl = (v_union - (volume_a + volume_b - v_inter)).abs() / volume_a;
            let miss = split.max(incl_excl);
            worst = worst.max(miss);

            let verdict = if miss <= 1e-6 { "ok" } else { "WRONG" };
            if miss <= 1e-6 {
                tally[0] += 1;
            } else {
                tally[2] += 1;
            }

            println!(
                "{name:<18} {:<14} {v_diff:>14.4} {v_inter:>14.4} {v_union:>14.4} {split:>11.2e} {incl_excl:>11.2e}  {verdict}",
                placement.name
            );
        }
    }

    println!("{}", "-".repeat(126));
    println!(
        "ok {}   refused {}   WRONG {}   PANIC {}   worst identity residual {:.2e}",
        tally[0], tally[1], tally[2], tally[3], worst
    );
    println!();
    println!("split     = |V(A-B) + V(A^B) - V(A)| / V(A)");
    println!("incl-excl = |V(AuB) - (V(A) + V(B) - V(A^B))| / V(A)");
    println!();
    println!("Both hold whatever A is and wherever B sits, so neither needs a");
    println!("closed form for the cut shape. Refusing is not a defect; WRONG is.");
}
