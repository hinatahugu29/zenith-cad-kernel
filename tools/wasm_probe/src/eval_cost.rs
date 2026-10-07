//! **曲面を 1 回評価する値段**（4-696）。
//!
//! **H8 の差は 4,538 万回評価します**（`ZENITH_H8_WORK=1` の実測）。
//! **1 回がいくらか**が分かれば、**240〜380 秒のどれだけが評価か**が
//! 言えます。
//!
//! **検体の面をそのまま使います**——**合成した曲面では、次数も
//! 制御点の数も違ってしまう**ので。

fn main() {
    const STEP: &str = include_str!("../../../reference/OCCT/data/step/linkrods.step");
    let Ok(solids) = zenith_io::StepImporter::import_solids_from_str(STEP) else {
        println!("読めません");
        return;
    };
    let Some(solid) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };
    println!("面 {} 枚", solid.outer_shell.faces.len());
    let mut nurbs = 0usize;
    let mut planes = 0usize;
    let mut total_ns = 0.0f64;
    let mut total_calls = 0u64;
    for (index, face) in solid.outer_shell.faces.iter().enumerate() {
        let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry else {
            planes += 1;
            continue;
        };
        nurbs += 1;
        let ((u0, u1), (v0, v1)) = surface.param_range();
        const N: u32 = 20_000;
        let t = std::time::Instant::now();
        let mut sink = 0.0f64;
        for k in 0..N {
            let a = k as f64 / N as f64;
            let u = u0 + (u1 - u0) * a;
            let v = v0 + (v1 - v0) * (1.0 - a);
            sink += surface.evaluate(u, v).x;
        }
        let ns = t.elapsed().as_secs_f64() / N as f64 * 1e9;
        total_ns += ns * N as f64;
        total_calls += N as u64;
        if index < 4 || ns > 400.0 {
            println!("  面{index:<3} 1 回 {ns:>7.0} ns  （sink={sink:.3e}）");
        }
    }
    let mean = total_ns / total_calls as f64;
    println!("NURBS の面 {nurbs} 枚 / 平面 {planes} 枚");
    println!("**平均 {mean:.0} ns/回**");
    println!("4,538 万回なら {:.1} 秒", 45_379_164.0 * mean / 1e9);
}
