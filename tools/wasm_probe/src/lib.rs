//! **核が wasm に本当に入るかを測るための、最小の cdylib**（4-691）。
//!
//! `cargo build --example` では測れません——**`wasm32-unknown-unknown` の
//! bin は `main` がどこからも届かないので、核ごと落ちます**（素の
//! `fn main(){}` が 22,374 バイト、核を使う例が 40,310 バイト。
//! 核が入っているはずがない大きさです）。
//! **export から届く形**にして初めて、入るかどうかが分かります。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

/// **箱と箱の差を取って、三角形の枚数を返します。**
/// **0 を返したら、差が返らなかったということ**です。
#[no_mangle]
pub extern "C" fn zenith_wasm_box_difference(divisions: u32) -> u32 {
    let Ok(a) = PrimitiveBuilder::make_box(10.0, 10.0, 10.0) else {
        return 0;
    };
    let Ok(inner) = PrimitiveBuilder::make_box(4.0, 4.0, 12.0) else {
        return 0;
    };
    let b = BrepTransform::translate_solid(&inner, Vec3::new(3.0, 3.0, -1.0));
    let tol = Tolerance::default();
    let result = match BooleanEngine::boolean_solids_exact_result(
        &a,
        &b,
        BooleanOpType::Difference,
        &tol,
    ) {
        Ok(result) => result,
        Err(_) => return 0,
    };
    let Some(solid) = result.solids.first() else {
        return 0;
    };
    let n = divisions.max(2) as usize;
    let mesh = tessellate_solid(
        solid,
        &TessellationParams {
            u_divisions: n,
            v_divisions: n,
        },
    );
    mesh.indices.len() as u32
}

/// **STEP の文字列を読んで、立体の数を返します**（`zenith_io` も引き込むため）。
#[no_mangle]
pub extern "C" fn zenith_wasm_read_step_count(ptr: *const u8, len: usize) -> u32 {
    if ptr.is_null() {
        return 0;
    }
    let bytes = unsafe { std::slice::from_raw_parts(ptr, len) };
    let Ok(text) = std::str::from_utf8(bytes) else {
        return 0;
    };
    match zenith_io::StepImporter::import_solids_from_str(text) {
        Ok(solids) => solids.len() as u32,
        Err(_) => 0,
    }
}

/// **埋め込んだ STEP を読んで、立体の数と面の数を返します**（4-691）。
/// **`zenith_io` を wasm の中で本当に通すため**で、**ファイルは読みません**
/// （`wasm32-unknown-unknown` に file system はありません）。
/// 返り値: `立体の数 * 1000 + いちばん面の多い立体の面の数`
#[no_mangle]
pub extern "C" fn zenith_wasm_embedded_step() -> u32 {
    const STEP: &str = include_str!(
        "../../../crates/zenith_algo/tests/fixtures/occ_reference_cylinder.step"
    );
    match zenith_io::StepImporter::import_solids_from_str(STEP) {
        Ok(solids) => {
            let faces = solids
                .iter()
                .map(|s| s.outer_shell.faces.len())
                .max()
                .unwrap_or(0);
            (solids.len() as u32) * 1000 + faces as u32
        }
        Err(_) => 0,
    }
}
