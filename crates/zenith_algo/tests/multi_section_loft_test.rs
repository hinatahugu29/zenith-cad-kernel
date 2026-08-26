use zenith_algo::{LoftBuilder, MassCalculator, ProfileBuilder, SectionSlicer};
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

    // 2. 体積を、**別の経路**で積み直して突き合わせる
    //
    // 円 → 長方形 → 楕円のロフトには初等的な閉じた式が無い。**だからと
    // いって `volume > 0` で済ませると、断面が1枚抜けていても通る。**
    //
    // 代わりに、同じ立体を高さ方向に切って断面積をシンプソン則で積む。質量
    // 計算は発散定理で面を積むので、面の向きや欠落はこの2つを食い違わせる。
    // **これは閉じた式の代わりではない**——2つの独立な積み方が同じ数を出す
    // ことしか言えない。どちらも同じテッセレーションを踏むので、刻みの誤差は
    // 共通に乗る。
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);

    // 刻みは断面ごとに指定する。既定の 128×128 で 121 枚切ると 5 分以上
    // かかり、テストとしては重すぎた（実測 328 秒）。32 分割・40 枚で
    // 残差は同じ桁に収まる。
    let steps = 40usize; // 偶数
    let height = 60.0_f64;
    let slice_tess = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let area_at = |z: f64| {
        // 端の面の上では断面が定まらない（`SectionSlicer` はそこを名指しで
        // 断る）ので、キャップから離して測る。離した分は台形の端点として
        // 効くだけで、シンプソン則の主要項は変わらない。
        let clamped = z.clamp(1e-3, height - 1e-3);
        SectionSlicer::slice_solid_with_tessellation(
            &solid,
            Point3::new(0.0, 0.0, clamped),
            Vec3::new(0.0, 0.0, 1.0),
            &tol,
            &slice_tess,
        )
        .expect("section")
        .total_area
    };
    let step = height / steps as f64;
    let mut integral = area_at(0.0) + area_at(height);
    for index in 1..steps {
        let weight = if index % 2 == 1 { 4.0 } else { 2.0 };
        integral += weight * area_at(index as f64 * step);
    }
    let by_sections = integral * step / 3.0;

    let error = (mass.volume - by_sections).abs() / by_sections;
    assert!(
        error < 5e-3,
        "the divergence integral says {} and the stack of sections says {by_sections} (relative {error:.3e})",
        mass.volume
    );

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
