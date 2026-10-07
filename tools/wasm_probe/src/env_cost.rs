//! **`std::env::var` 1 回の値段**（4-694）。
//!
//! **核の中に 259 箇所ある**（4-693）ので、**1 回がいくらか**を知らないと
//! 「面ごとに引いて重いのでは」に答えられません。
//! **立っていないとき（Err）と立っているとき（Ok）**の両方を測ります。
fn main() {
    const N: u32 = 200_000;
    // **立っていない口**——核の中のほとんどがこれです。
    let t = std::time::Instant::now();
    let mut sink = 0u64;
    for _ in 0..N {
        if std::env::var_os("ZENITH_THIS_PORT_DOES_NOT_EXIST").is_some() {
            sink += 1;
        }
    }
    let miss = t.elapsed().as_secs_f64() / N as f64 * 1e9;
    // **立っている口**。
    std::env::set_var("ZENITH_ENV_COST_PROBE", "1");
    let t = std::time::Instant::now();
    for _ in 0..N {
        if std::env::var_os("ZENITH_ENV_COST_PROBE").is_some() {
            sink += 1;
        }
    }
    let hit = t.elapsed().as_secs_f64() / N as f64 * 1e9;
    // **文字列まで取るとき**（`var` は `var_os` より重いはずです）。
    let t = std::time::Instant::now();
    for _ in 0..N {
        if std::env::var("ZENITH_ENV_COST_PROBE").is_ok() {
            sink += 1;
        }
    }
    let parse = t.elapsed().as_secs_f64() / N as f64 * 1e9;
    println!("var_os（立っていない） {miss:.0} ns/回");
    println!("var_os（立っている）   {hit:.0} ns/回");
    println!("var  （立っている）    {parse:.0} ns/回");
    println!("（{N} 回の平均。sink={sink} は最適化で消えないためのもの）");
}
