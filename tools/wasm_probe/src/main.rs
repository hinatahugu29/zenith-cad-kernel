//! **同じ呼び出しを、ネイティブでも走らせます**（4-691）。
//! **wasm と同じ答えが返るか**を見るためだけの口です。
fn main() {
    let v = zenith_wasm_probe::zenith_wasm_embedded_step();
    println!("  埋め込んだ STEP: 立体 {} 個 / 面 {} 枚", v / 1000, v % 1000);
    for n in [8u32, 16, 24] {
        let t = std::time::Instant::now();
        let tris = zenith_wasm_probe::zenith_wasm_box_difference(n);
        println!("  刻み {n}: 三角形 {tris} 枚  {:.1} ms", t.elapsed().as_secs_f64() * 1000.0);
    }
}
