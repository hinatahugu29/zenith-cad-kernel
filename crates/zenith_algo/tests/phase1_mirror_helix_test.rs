use zenith_algo::{HelixBuilder, MassCalculator, MirrorBuilder, PrimitiveBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

fn make_rect_wire(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> Wire {
    let pts = vec![
        Point3::new(min_x, min_y, 0.0),
        Point3::new(max_x, min_y, 0.0),
        Point3::new(max_x, max_y, 0.0),
        Point3::new(min_x, max_y, 0.0),
    ];
    let vertices: Vec<Vertex> = pts.into_iter().map(Vertex::from_point).collect();
    let mut edges = Vec::with_capacity(4);
    for i in 0..4 {
        let next_i = (i + 1) % 4;
        let edge = Edge::line_between(vertices[i].clone(), vertices[next_i].clone()).unwrap();
        edges.push(OrientedEdge::forward(edge));
    }
    Wire::new(edges)
}

#[test]
fn test_mirror_box_and_cylinder() {
    let tol = Tolerance::default();
    let base_box = PrimitiveBuilder::make_box(10.0, 20.0, 30.0).expect("make box");

    // 1. X=0 平面（法線 (1,0,0)）に対する鏡像反転
    let plane_origin = Point3::new(0.0, 0.0, 0.0);
    let plane_normal = Vec3::new(1.0, 0.0, 0.0);

    let mirrored_box =
        MirrorBuilder::mirror_solid(&base_box, plane_origin, plane_normal, &tol)
            .expect("mirror box");

    assert_eq!(mirrored_box.outer_shell.faces.len(), 6);
    let report = mirrored_box.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "Mirrored box invalid: {:?}", report.errors);

    let mesh_orig = tessellate_solid(&base_box, &TessellationParams::default());
    let mesh_mir = tessellate_solid(&mirrored_box, &TessellationParams::default());
    let mass_orig = MassCalculator::compute_from_mesh(&mesh_orig);
    let mass_mir = MassCalculator::compute_from_mesh(&mesh_mir);
    assert!(
        (mass_orig.volume - mass_mir.volume).abs() < 1e-3,
        "Volume must match after mirror"
    );

    // 2. 円柱の斜め平面ミラー
    let base_cyl = PrimitiveBuilder::make_cylinder(5.0, 20.0).expect("make cyl");
    let diag_normal = Vec3::new(1.0, 1.0, 0.0).normalize();
    let mirrored_cyl =
        MirrorBuilder::mirror_solid(&base_cyl, Point3::new(10.0, 0.0, 0.0), diag_normal, &tol)
            .expect("mirror cyl");
    assert_eq!(mirrored_cyl.outer_shell.faces.len(), 6);
    let r_cyl = mirrored_cyl.outer_shell.validate_closed(&tol);
    assert!(r_cyl.is_valid(), "Mirrored cyl invalid: {:?}", r_cyl.errors);

    // 3. 複合Compound STEPラウンドトリップ
    let compound_shape = MirrorBuilder::mirror_compound(&base_box, plane_origin, plane_normal, &tol)
        .expect("mirror compound");
    let step_path = "test_mirror_compound.step";
    StepExporter::export_shape_to_file(&compound_shape, step_path, "MIRROR_COMPOUND")
        .expect("STEP export failed");
    let imported_shape = StepImporter::import_shape_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);

    match imported_shape {
        zenith_topo::Shape::Compound(solids) => assert_eq!(solids.len(), 2),
        _ => panic!("Expected compound shape"),
    }
}

#[test]
fn test_helix_spring_solid() {
    let tol = Tolerance::default();
    // 2.0 x 2.0 正方形断面
    let profile = make_rect_wire(-1.0, 1.0, -1.0, 1.0);
    let radius = 15.0;
    let pitch = 10.0;
    let turns = 2.0; // 2巻き (全高 20.0)
    let axis_origin = Point3::new(0.0, 0.0, 0.0);
    let axis_dir = Vec3::new(0.0, 0.0, 1.0);

    let helix_solid = HelixBuilder::sweep_wire_along_helix(
        &profile,
        radius,
        pitch,
        turns,
        axis_origin,
        axis_dir,
        32,
        &tol,
    )
    .expect("sweep helix solid");

    // 1. トポロジー閉シェル検証
    let report = helix_solid.outer_shell.validate_closed(&tol);
    assert!(report.is_valid(), "Helix solid invalid: {:?}", report.errors);

    // 2. 解析体積検証（断面積 4.0 * 螺旋弧長）
    let params = TessellationParams {
        u_divisions: 8,
        v_divisions: 8,
    };
    let mesh = tessellate_solid(&helix_solid, &params);
    let mass = MassCalculator::compute_from_mesh(&mesh);
    let helix_length = turns * ((2.0 * std::f64::consts::PI * radius).powi(2) + pitch.powi(2)).sqrt();
    let expected_vol = 4.0 * helix_length;
    let rel_err = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 0.05,
        "Helix volume relative error too large: got {}, expected {}",
        mass.volume,
        expected_vol
    );

    // 3. STEPラウンドトリップ
    let step_path = "test_helix_solid_roundtrip.step";
    StepExporter::export_solid_to_file(&helix_solid, step_path, "HELIX_SOLID")
        .expect("STEP export failed");
    let imported = StepImporter::import_solid_from_file(step_path).expect("STEP import failed");
    let _ = std::fs::remove_file(step_path);
    assert_eq!(imported.outer_shell.faces.len(), helix_solid.outer_shell.faces.len());
}

/// 組んだ螺旋曲線が、**厳密な螺旋**の上にあるか。
///
/// 螺旋は有理曲線では表せない。xy は真円になるが、真の螺旋は z が角度に比例
/// するのに対し、有理2次の角度は媒介変数に比例しない。両者は各区間の
/// t = 0, 1/2, 1 で一致し、その間でずれる。90度刻みだと半径10・ピッチ6で
/// 高さが 3.16e-2 外れていた。刻みは公差から決まるようになっている。
///
/// 角度は標本を順に辿って連続に開く。周回番号を z から推し測ると、ピッチの
/// ぶんだけ飛んだ答えが出て、存在しない欠陥を見ることになる（実際に一度
/// 1.53 という値を見た）。
#[test]
fn test_helix_curve_follows_the_exact_helix_within_the_linear_tolerance() {
    let tol = Tolerance::default();
    for (radius, pitch, turns) in [(10.0, 6.0, 2.0), (30.0, 6.0, 2.0), (10.0, 12.0, 1.5)] {
        let curve = HelixBuilder::build_helix_curve(
            radius,
            pitch,
            turns,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
        )
        .expect("helix curve");

        let (t0, t1) = curve.param_range();
        let samples = 2003;
        let mut unwrapped = 0.0f64;
        let mut previous = 0.0f64;
        let mut worst_radius: f64 = 0.0;
        let mut worst_height: f64 = 0.0;

        for index in 0..=samples {
            let t = t0 + (t1 - t0) * index as f64 / samples as f64;
            let point = curve.evaluate(t);
            let raw = point.y.atan2(point.x);
            if index > 0 {
                let mut step = raw - previous;
                while step > std::f64::consts::PI {
                    step -= std::f64::consts::TAU;
                }
                while step < -std::f64::consts::PI {
                    step += std::f64::consts::TAU;
                }
                unwrapped += step;
            }
            previous = raw;

            let radial = (point.x * point.x + point.y * point.y).sqrt();
            worst_radius = worst_radius.max((radial - radius).abs());
            let height = pitch * unwrapped / std::f64::consts::TAU;
            worst_height = worst_height.max((point.z - height).abs());
        }

        assert!(
            worst_radius < 1e-12,
            "helix R{radius} p{pitch} left its cylinder by {worst_radius:.3e}"
        );
        assert!(
            worst_height < tol.linear,
            "helix R{radius} p{pitch} rises wrongly by {worst_height:.3e}, over {}",
            tol.linear
        );
        assert!(
            (unwrapped - turns * std::f64::consts::TAU).abs() < 1e-12,
            "helix R{radius} p{pitch} swept {unwrapped} rather than {} radians",
            turns * std::f64::consts::TAU
        );
    }
}

/// 螺旋掃引の体積が、閉じた式 `V = A x L` に乗るか。
///
/// 断面の重心が経路の上にあり経路に垂直なら、経路が曲がっていても
/// この式はきっかり成り立つ。螺旋の経路長は閉じた式なので、これは
/// 外の物差しである。
///
/// 以前の経路では、**掃引をいくら細かくしても** 4.278e-5 ずれた値に収束して
/// いた。収束したことは正しさの証拠にならない。だからここは「収束するか」
/// ではなく「**どこへ**収束するか」を見る。
#[test]
fn test_helix_sweep_volume_matches_the_closed_form() {
    let tol = Tolerance::default();
    let radius = 10.0;
    let pitch = 6.0;
    let turns = 2.0;
    let expected = 4.0 * turns * ((std::f64::consts::TAU * radius).powi(2) + pitch * pitch).sqrt();

    let solid = HelixBuilder::sweep_wire_along_helix(
        &make_rect_wire(-1.0, 1.0, -1.0, 1.0),
        radius,
        pitch,
        turns,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        64,
        &tol,
    )
    .expect("helix solid");

    let volume = MassCalculator::compute_from_brep(
        &solid,
        &TessellationParams {
            u_divisions: 96,
            v_divisions: 96,
        },
    )
    .volume;
    let error = (volume - expected).abs() / expected;
    assert!(
        error < 1e-6,
        "helix volume {volume} is {error:.3e} from the closed form {expected}"
    );
}

/// 刻みを細かくすると高さのずれが**3乗**で減ること。
/// ここが 8 前後でなければ、刻みと精度の関係が変わっている。
#[test]
fn test_helix_height_error_falls_with_the_cube_of_the_step_angle() {
    let height_error = |per_turn: usize| -> f64 {
        let (radius, pitch, turns) = (10.0, 6.0, 2.0);
        let curve = HelixBuilder::build_helix_curve_with_segments(
            radius,
            pitch,
            turns,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            per_turn,
        )
        .expect("helix curve");
        let (t0, t1) = curve.param_range();
        let samples = 2003;
        let mut unwrapped = 0.0f64;
        let mut previous = 0.0f64;
        let mut worst: f64 = 0.0;
        for index in 0..=samples {
            let t = t0 + (t1 - t0) * index as f64 / samples as f64;
            let point = curve.evaluate(t);
            let raw = point.y.atan2(point.x);
            if index > 0 {
                let mut step = raw - previous;
                while step > std::f64::consts::PI {
                    step -= std::f64::consts::TAU;
                }
                while step < -std::f64::consts::PI {
                    step += std::f64::consts::TAU;
                }
                unwrapped += step;
            }
            previous = raw;
            worst = worst.max((point.z - pitch * unwrapped / std::f64::consts::TAU).abs());
        }
        worst
    };

    let coarse = height_error(16);
    let fine = height_error(32);
    let ratio = coarse / fine;
    assert!(
        (6.0..12.0).contains(&ratio),
        "halving the step should divide the height error by about 8, got {ratio:.2} \
         (coarse {coarse:.3e}, fine {fine:.3e})"
    );
}
