//! **読んだ立体の体積は、刻みを細かくすると増えるのか**（4-559）。
//!
//! # なぜ要るか
//!
//! 4-558 で積が返り、**OCC と 1.682e-4 ずれている**と書きました。
//! **ところが、こちらが読んだ `A` そのものの体積も、OCC の `V(A)` から
//! 1.4e-4 ずれています**——**ブーリアンを 1 回もかけない段階で**です。
//!
//! **体積は三角形分割から積んでいます**（`compute_volume_from_brep`）。
//! **曲がった面を平らな三角形で覆えば、体積は必ず小さめに出ます。**
//! **なら、刻みを細かくすれば増えるはず**です。**増えるなら、
//! ずれているのは立体ではなく、測り方**。
//!
//! 使い方: `read_volume_convergence_probe`
use zenith_algo::MassCalculator;
use zenith_io::StepImporter;
use zenith_tess::TessellationParams;

fn main() {
    // **検体を選べます**（4-667。**既定は linkrods のまま**）——
    // **「たるみの内側」が linkrods 固有かどうかを、他のファイルでも
    // 測れるように**します（4-664）。**OCC の V(A) は linkrods の
    // 数**（4-511）なので、**他の検体では並べません。**
    const DEFAULT: &str = "reference/OCCT/data/step/linkrods.step";
    let owned = std::env::var("ZENITH_SUBJECT").unwrap_or_else(|_| DEFAULT.to_string());
    let sample: &str = &owned;
    let is_linkrods = sample == DEFAULT;
    let Ok(solids) = StepImporter::import_solids_from_file(sample) else {
        println!("{sample} が読めません");
        return;
    };
    let Some(read) = solids.into_iter().max_by_key(|s| s.outer_shell.faces.len()) else {
        return;
    };

    // **OCC の数**（4-511）: `V(A)` = 3.847002。
    const OCC: f64 = 3.847002;
    let base = TessellationParams::default();
    if is_linkrods {
        println!("OCC の V(A) = {OCC:.6}");
    } else {
        println!("**OCC の V(A) は linkrods の数です**——この検体では並べません。");
    }
    println!();
    if is_linkrods {
        println!("分割     体積          OCC との差     相対");
    } else {
        println!("分割     体積");
    }
    for divisions in [8usize, 12, 16, 24, 32, 48, 64, 96] {
        let mut params = base.clone();
        params.u_divisions = divisions;
        params.v_divisions = divisions;
        let volume = MassCalculator::compute_volume_from_brep(&read, &params);
        if is_linkrods {
            println!(
                "{divisions:>4}   {volume:.6}   {:+.3e}   {:+.3e}",
                volume - OCC,
                (volume - OCC) / OCC
            );
        } else {
            // **並べる相手がいない列は、出しません**（4-667）——
            // **出すと、意味のある差に見えます。**
            println!("{divisions:>4}   {volume:.6}");
        }
    }
    println!();
    println!("**増え続けるなら、ずれているのは立体ではなく測り方です。**");

    // **面ごとの面積**（4-560）。**OCC の面積**は
    // `py tools/occ_face_area_reference.py` が出します。**並べれば、
    // どの面の形が違うのかが分かります**——**体積は全部の和**なので、
    // **和だけ見ていても、どこがずれているかは出てきません。**
    if std::env::var_os("ZENITH_FACE_AREAS").is_some() {
        println!();
        println!("面ごとの面積（こちら）:");
        // **刻みを上げられます**（4-679。既定 64 は 4-561 のまま）——
        // **面積が収束しているかを見るため。**
        let n: usize = std::env::var("ZENITH_FACE_AREA_GRID")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(64);
        let fine = TessellationParams {
            u_divisions: n,
            v_divisions: n,
        };
        // **体積への寄与**も出します（4-561）。`compute_face_integral` は
        // **(面積, 体積) を返します**——**体積の内訳は、こちら**です。
        // **面積が合っていても、面が内側に在れば寄与は減ります**（4-560）。
        let mut total_area = 0.0;
        let mut total_volume = 0.0;
        for (index, face) in read.outer_shell.faces.iter().enumerate() {
            let (area, volume) = zenith_algo::MassCalculator::compute_face_integral(face, &fine);
            total_area += area;
            total_volume += volume;
            println!("  面{index:<3} 面積 {area:.9}  寄与 {volume:+.9}");
        }
        println!("  面積の合計 {total_area:.9}、寄与の合計 {total_volume:.9}");
    }

    // **こちらの面の上の点を書き出します**（4-561。`ZENITH_FACE_POINTS=<先>`）。
    //
    // **OCC に「内か外か」を聞くため**です（`tools/occ_face_area_reference.py`）。
    // **面積は合っていて体積だけ小さいなら、面の位置**——**一様に内側なら、
    // こちらの面は OCC の面より内を通っています。**
    if let Ok(target) = std::env::var("ZENITH_FACE_POINTS") {
        use std::io::Write;
        let mut out = String::new();
        // **刻みを上げられます**（4-671。既定 12 は 4-561 のまま）——
        // **面の点が粗いと、いちばん離れている所を踏みません。**
        let n: usize = std::env::var("ZENITH_FACE_POINT_GRID")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(12);
        let grid = TessellationParams {
            u_divisions: n,
            v_divisions: n,
        };
        for (index, face) in read.outer_shell.faces.iter().enumerate() {
            let mesh = zenith_tess::tessellate_face(face, &grid);
            for point in &mesh.positions {
                out.push_str(&format!(
                    "{index} {:.9} {:.9} {:.9}
",
                    point.x, point.y, point.z
                ));
            }
        }
        match std::fs::File::create(&target).and_then(|mut f| f.write_all(out.as_bytes())) {
            Ok(()) => println!("面の上の点を {target} に書きました（{} 行）", out.lines().count()),
            Err(error) => println!("{target} に書けません: {error}"),
        }
    }

    // **こちらの 3D 稜の上の点を書き出します**（4-671。`ZENITH_EDGE_POINTS=<先>`）。
    //
    // **4-668 の宿題**——**離れ ÷ 稜のたるみ が、ある面では 1.0、
    // 別の面では 0.5** でした。**OCC の面が、こちらの稜を通っているか
    // どうか**が分かれば、その違いが決まります:
    //   **稜が OCC の面に乗っている** → 浮いているのはこちらの曲面だけ（1.0）
    //   **稜が たるみの半分だけ離れている** → 両方が折半（0.5）
    //
    // **形は `ZENITH_FACE_POINTS` と同じ**（面番号 x y z）ので、
    // **同じ `tools/occ_face_distance.py` に渡せます。**
    if let Ok(target) = std::env::var("ZENITH_EDGE_POINTS") {
        use std::io::Write;
        const STEPS: usize = 64;
        let mut out = String::new();
        for (index, face) in read.outer_shell.faces.iter().enumerate() {
            let wires = std::iter::once(&face.outer_wire).chain(face.inner_wires.iter());
            for wire in wires {
                for edge in &wire.edges {
                    for step in 0..=STEPS {
                        let point = edge.evaluate_normalized(step as f64 / STEPS as f64);
                        out.push_str(&format!(
                            "{index} {:.9} {:.9} {:.9}
",
                            point.x, point.y, point.z
                        ));
                    }
                }
            }
        }
        match std::fs::File::create(&target).and_then(|mut f| f.write_all(out.as_bytes())) {
            Ok(()) => println!("稜の上の点を {target} に書きました（{} 行）", out.lines().count()),
            Err(error) => println!("{target} に書けません: {error}"),
        }
    }
}
