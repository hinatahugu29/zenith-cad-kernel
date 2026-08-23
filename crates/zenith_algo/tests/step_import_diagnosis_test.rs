//! 読めなかったとき、読めなかった理由が分かるか。
//!
//! インポーターは、知らない曲面に当たると**原点を通る既定の平面**を、知らない
//! 曲線に当たると**端点を結ぶ直線**を返していた。
//!
//! 幸い、置き換わった平面は下流の p-curve 検証が必ず落とすので、誤答ではなく
//! エラーになっていた。ただしそのエラーは
//!
//!     Solid validation failed with 138 outer-shell errors
//!     (Face 0 planar p-curve outer loop is degenerate; ...
//!      boundary point on outer loop is off surface by 4.000000e1; ...)
//!
//! としか言わない。読めなかった理由——対応していないエンティティに当たった
//! こと——がどこにも出てこないので、追いかける人は幾何を疑うことになる。
//! （実際、監査中に一度そう疑いました。）
//!
//! 曲線側はもっと悪く、楕円弧を弦に置き換えても平面の上には載るので、
//! p-curve 検証を素通りしうる。閉じた円は両端が同じ点になって退化として
//! 捕まるが、それは捕まる側の運が良いだけである。
//!
//! いまはどちらも、エンティティ名を名指ししてエラーを返す。
//!
//! 検体は OpenCASCADE 7.8 が書いたものを `include_str!` で読む。`target/` の
//! 中を参照して「無ければスキップ」にすると、検査が黙って消える日が来る。

use zenith_io::StepImporter;

/// `CYLINDRICAL_SURFACE` と `CIRCLE` の両方を持つ検体。型名を差し替えるだけで、
/// 曲面側と曲線側のどちらの経路も試せる。
const SUBJECT: &str = include_str!("fixtures/occ_reference_revolved_ring.step");

#[test]
fn test_the_subject_imports_before_it_is_mutated() {
    assert!(
        StepImporter::import_solid_from_str(SUBJECT).is_ok(),
        "the fixture must import as it stands, otherwise the mutations below prove nothing"
    );
}

#[test]
fn test_an_unsupported_surface_is_named_in_the_error() {
    // 引数はそのままに、型名だけインポーターが知らないものへ差し替える。
    let mutated = SUBJECT.replace("CYLINDRICAL_SURFACE", "SURFACE_OF_REVOLUTION");
    assert_ne!(
        SUBJECT, mutated,
        "the fixture must contain CYLINDRICAL_SURFACE"
    );

    let error = StepImporter::import_solid_from_str(&mutated)
        .expect_err("an unreadable surface must not import");
    assert!(
        error.contains("SURFACE_OF_REVOLUTION"),
        "the error must name the entity it could not read, got: {error}"
    );
    assert!(
        error.contains("Unsupported surface entity"),
        "the error must say what kind of problem this is, got: {error}"
    );
}

#[test]
fn test_an_unsupported_curve_is_named_in_the_error() {
    let mutated = SUBJECT.replace("CIRCLE(", "OFFSET_CURVE_3D(");
    assert_ne!(SUBJECT, mutated, "the fixture must contain CIRCLE");

    let error = StepImporter::import_solid_from_str(&mutated)
        .expect_err("an unreadable curve must not import");
    assert!(
        error.contains("OFFSET_CURVE_3D"),
        "the error must name the entity it could not read, got: {error}"
    );
    assert!(
        error.contains("Unsupported curve entity"),
        "the error must say what kind of problem this is, got: {error}"
    );
}

/// 対応した型を「知らない型」に差し替えても読めてしまうなら、その対応は
/// 効いていない。ELLIPSE と SURFACE_OF_LINEAR_EXTRUSION が実際に使われて
/// いることを、逆側から押さえておく。
#[test]
fn test_the_newly_supported_entities_are_actually_taken() {
    const PRISM: &str = include_str!("fixtures/occ_reference_elliptic_prism.step");
    assert!(
        PRISM.contains("ELLIPSE") && PRISM.contains("SURFACE_OF_LINEAR_EXTRUSION"),
        "the fixture must exercise both entities"
    );
    assert!(
        StepImporter::import_solid_from_str(PRISM).is_ok(),
        "an elliptic prism must import"
    );

    for entity in ["ELLIPSE", "SURFACE_OF_LINEAR_EXTRUSION"] {
        let mutated = PRISM.replace(entity, "SOMETHING_WE_DO_NOT_READ");
        let error = StepImporter::import_solid_from_str(&mutated).expect_err(
            "renaming a supported entity must break the import; \
             if it still reads, the handler for it is not on the path",
        );
        assert!(
            error.contains("SOMETHING_WE_DO_NOT_READ"),
            "renaming {entity} must be reported by name, got: {error}"
        );
    }
}
