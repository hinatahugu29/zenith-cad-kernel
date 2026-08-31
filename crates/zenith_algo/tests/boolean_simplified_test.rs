use zenith_algo::{
    BooleanEngine, BooleanOpType, BrepTransform, FaceMerger, HoleBuilder, PrimitiveBuilder,
};
use zenith_math::{Tolerance, Vec3};

#[test]
fn test_boolean_solids_exact_simplified_reduces_faces_and_edges() {
    let tol = Tolerance::default();
    let block = PrimitiveBuilder::make_box(40.0, 40.0, 20.0).unwrap();
    let corner = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(20.0, 20.0, 20.0).unwrap(),
        Vec3::new(20.0, 20.0, 0.0),
    );

    // 通常のブーリアン（面片分割のまま）: 14面
    let raw_l =
        BooleanEngine::boolean_solids_exact(&block, &corner, BooleanOpType::Difference, &tol)
            .expect("raw difference");
    assert_eq!(raw_l.outer_shell.faces.len(), 14);

    // 簡約化ブーリアン（自動FaceMerger統合）: 8面
    let simplified_l = BooleanEngine::boolean_solids_exact_simplified(
        &block,
        &corner,
        BooleanOpType::Difference,
        &tol,
    )
    .expect("simplified difference");

    assert_eq!(simplified_l.outer_shell.faces.len(), 8);
    assert!(simplified_l.outer_shell.validate_closed(&tol).is_valid());
}

#[test]
fn test_drilled_box_face_merger_simplification() {
    let tol = Tolerance::default();
    let drilled = HoleBuilder::make_drilled_box(40.0, 40.0, 20.0, 8.0).unwrap();
    assert_eq!(drilled.outer_shell.faces.len(), 16);

    let (simplified, report) = FaceMerger::simplify_solid(&drilled, &tol).expect("simplify");
    assert_eq!(simplified.outer_shell.faces.len(), 10); // 上面1 + 下面1 + 外側側面4 + 穴内側4曲面
    assert_eq!(report.merged_groups, 2); // 上面グループと下面グループ
    assert!(simplified.outer_shell.validate_closed(&tol).is_valid());
}
