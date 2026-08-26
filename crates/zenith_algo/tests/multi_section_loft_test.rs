use zenith_algo::{LoftBuilder, MassCalculator, ProfileBuilder};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_tess::{tessellate_solid, TessellationParams};

#[test]
fn test_multi_section_loft_duct() {
    let tol = Tolerance::default();

    // 断面 0 (z=0): 真円 (R=20) (4エッジ)
    let w0 = ProfileBuilder::make_circle(
        20.0,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("circle wire");

    // 断面 1 (z=30): 長方形 (w=36, h=24) (4エッジ)
    let w1 = ProfileBuilder::make_rectangle(
        36.0,
        24.0,
        Point3::new(0.0, 0.0, 30.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("rectangle wire");

    // 断面 2 (z=60): 楕円 (a=30, b=15) (4エッジ)
    let w2 = ProfileBuilder::make_ellipse(
        30.0,
        15.0,
        Point3::new(0.0, 0.0, 60.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("ellipse wire");

    let solid = LoftBuilder::loft_solid(&[w0, w1, w2], 2, &tol)
        .expect("loft solid");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Loft solid must be valid closed manifold"
    );

    // 2. 体積が正で妥当な範囲にあること
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    assert!(mass.volume > 0.0, "Volume must be positive, got {}", mass.volume);

    // 3. テッセレーション検証
    let mesh = tessellate_solid(&solid, &params);
    assert!(mesh.num_triangles() > 0, "Mesh must have triangles");

    // 4. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "MultiSectionLoftDuct");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported loft solid must be valid closed"
    );
}
