use zenith_algo::{FastenerBuilder, MassCalculator};
use zenith_io::{StepExporter, StepImporter};
use zenith_math::Tolerance;
use zenith_tess::TessellationParams;

#[test]
fn test_hex_prism() {
    let tol = Tolerance::default();
    let s = 30.0; // 二面幅
    let h = 20.0;

    let solid = FastenerBuilder::make_hex_prism(s, h, &tol).expect("hex prism");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Hex prism must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let expected_vol = (3.0_f64.sqrt() * 0.5) * s * s * h;
    let params = TessellationParams {
        u_divisions: 24,
        v_divisions: 24,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-12,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "HexPrism");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported hex prism must be valid closed"
    );
}

#[test]
fn test_hex_nut_blank() {
    let tol = Tolerance::default();
    let s = 30.0;
    let h = 15.0;
    let r_hole = 7.5; // M16ボルト用下穴相当

    let solid = FastenerBuilder::make_hex_nut_blank(s, h, r_hole, &tol).expect("hex nut blank");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Hex nut must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let hex_vol = (3.0_f64.sqrt() * 0.5) * s * s * h;
    let hole_vol = std::f64::consts::PI * r_hole * r_hole * h;
    let expected_vol = hex_vol - hole_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-6,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "HexNutBlank");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported hex nut must be valid closed"
    );
}

#[test]
fn test_socket_head_cap_screw() {
    let tol = Tolerance::default();
    let shank_r = 4.0; // M8
    let shank_l = 30.0;
    let head_r = 6.5;
    let head_h = 8.0;
    let socket_s = 6.0;
    let socket_d = 4.0;

    let solid = FastenerBuilder::make_socket_head_cap_screw(
        shank_r,
        shank_l,
        head_r,
        head_h,
        socket_s,
        socket_d,
        &tol,
    )
    .expect("socket head cap screw");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Cap screw must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let shank_vol = pi * shank_r * shank_r * shank_l;
    let head_vol = pi * head_r * head_r * head_h;
    let socket_vol = (3.0_f64.sqrt() * 0.5) * socket_s * socket_s * socket_d;
    let expected_vol = shank_vol + head_vol - socket_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-6,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "SocketHeadCapScrew");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported cap screw must be valid closed"
    );
}

#[test]
fn test_plain_washer() {
    let tol = Tolerance::default();
    let inner_r = 4.25; // M8用平座金 (内径8.5mm相当)
    let outer_r = 8.0;  // 外径16mm相当
    let thickness = 1.6;

    let solid = FastenerBuilder::make_plain_washer(inner_r, outer_r, thickness, &tol)
        .expect("plain washer");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Washer must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let expected_vol = pi * (outer_r * outer_r - inner_r * inner_r) * thickness;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-6,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "PlainWasher");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported washer must be valid closed"
    );
}

#[test]
fn test_flanged_hex_bolt() {
    let tol = Tolerance::default();
    let shank_r = 4.0; // M8
    let shank_l = 25.0;
    let flange_r = 8.5;
    let flange_h = 2.0;
    let hex_s = 12.0;
    let hex_h = 6.0;

    let solid = FastenerBuilder::make_flanged_hex_bolt(
        shank_r,
        shank_l,
        flange_r,
        flange_h,
        hex_s,
        hex_h,
        &tol,
    )
    .expect("flanged hex bolt");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Flanged bolt must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let shank_vol = pi * shank_r * shank_r * shank_l;
    let flange_vol = pi * flange_r * flange_r * flange_h;
    let hex_vol = (3.0_f64.sqrt() * 0.5) * hex_s * hex_s * hex_h;
    let expected_vol = shank_vol + flange_vol + hex_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-4,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "FlangedHexBolt");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported flanged bolt must be valid closed"
    );
}

#[test]
fn test_spring_washer() {
    let tol = Tolerance::default();
    let inner_r = 4.25; // M8
    let outer_r = 7.4;
    let t = 2.0;
    let free_h = 3.5;
    let gap_deg = 20.0;

    let solid = FastenerBuilder::make_spring_washer(
        inner_r,
        outer_r,
        t,
        free_h,
        gap_deg,
        &tol,
    )
    .expect("spring washer");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Spring washer must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let turns = (360.0 - gap_deg) / 360.0;
    let pitch = (free_h - t) / turns;
    let mean_r = (inner_r + outer_r) * 0.5;
    let width = outer_r - inner_r;
    let helix_len = turns * ((2.0 * std::f64::consts::PI * mean_r).powi(2) + pitch.powi(2)).sqrt();
    let expected_vol = width * t * helix_len;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 0.15,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "SpringWasher");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported spring washer must be valid closed"
    );
}

#[test]
fn test_retaining_ring() {
    let tol = Tolerance::default();
    let inner_r = 4.8; // M10 軸用
    let outer_r = 6.2;
    let t = 1.0;
    let gap_deg = 45.0;

    let solid = FastenerBuilder::make_retaining_ring(
        inner_r,
        outer_r,
        t,
        gap_deg,
        &tol,
    )
    .expect("retaining ring");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Retaining ring must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let sweep_fraction = (360.0 - gap_deg) / 360.0;
    let expected_vol = std::f64::consts::PI * (outer_r * outer_r - inner_r * inner_r) * t * sweep_fraction;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-4,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "RetainingRing");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported retaining ring must be valid closed"
    );
}

#[test]
fn test_countersunk_socket_screw() {
    let tol = Tolerance::default();
    let shank_r = 4.0; // M8
    let shank_l = 20.0;
    let head_r = 8.0;
    let head_h = 4.4;
    let socket_s = 5.0;
    let socket_d = 2.8;

    let solid = FastenerBuilder::make_countersunk_socket_screw(
        shank_r,
        shank_l,
        head_r,
        head_h,
        socket_s,
        socket_d,
        &tol,
    )
    .expect("countersunk socket screw");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Countersunk screw must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let shank_vol = pi * shank_r * shank_r * shank_l;
    let head_vol = (pi / 3.0) * head_h * (head_r * head_r + head_r * shank_r + shank_r * shank_r);
    let socket_vol = (3.0_f64.sqrt() * 0.5) * socket_s * socket_s * socket_d;
    let expected_vol = shank_vol + head_vol - socket_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-4,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "CountersunkSocketScrew");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported countersunk screw must be valid closed"
    );
}

#[test]
fn test_weld_neck_flange() {
    let tol = Tolerance::default();
    let flange_r = 25.0;
    let flange_t = 10.0;
    let hub_r = 15.0;
    let hub_h = 15.0;
    let pipe_r = 8.0;
    let pcd_r = 19.0;
    let bolt_r = 3.0;
    let num_bolts = 4;

    let solid = FastenerBuilder::make_weld_neck_flange(
        flange_r,
        flange_t,
        hub_r,
        hub_h,
        pipe_r,
        pcd_r,
        bolt_r,
        num_bolts,
        &tol,
    )
    .expect("weld neck flange");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Weld neck flange must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let blank_vol = pi * flange_r * flange_r * flange_t + pi * hub_r * hub_r * hub_h;
    let pipe_vol = pi * pipe_r * pipe_r * (flange_t + hub_h);
    let bolts_vol = num_bolts as f64 * (pi * bolt_r * bolt_r * flange_t);
    let expected_vol = blank_vol - pipe_vol - bolts_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-4,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "WeldNeckFlange");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported weld neck flange must be valid closed"
    );
}

#[test]
fn test_taper_pipe_plug() {
    let tol = Tolerance::default();
    let r_small = 6.0; // PT 1/4
    let r_large = 6.6;
    let h = 10.0;
    let socket_s = 6.0;
    let socket_d = 5.0;

    let solid = FastenerBuilder::make_taper_pipe_plug(
        r_small,
        r_large,
        h,
        socket_s,
        socket_d,
        &tol,
    )
    .expect("taper pipe plug");

    // 1. B-Rep 閉多様体検証
    assert!(
        solid.outer_shell.validate_closed(&tol).is_valid(),
        "Taper pipe plug must be valid closed manifold"
    );

    // 2. 閉形式体積一致検証
    let pi = std::f64::consts::PI;
    let cone_vol = (pi / 3.0) * h * (r_small * r_small + r_small * r_large + r_large * r_large);
    let socket_vol = (3.0_f64.sqrt() * 0.5) * socket_s * socket_s * socket_d;
    let expected_vol = cone_vol - socket_vol;

    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 32,
    };
    let mass = MassCalculator::compute_from_brep(&solid, &params);
    let vol_diff = (mass.volume - expected_vol).abs() / expected_vol;
    assert!(
        vol_diff < 1e-4,
        "Volume mismatch: computed={}, expected={}, diff={vol_diff}",
        mass.volume,
        expected_vol
    );

    // 3. STEP 往復検証
    let step_str = StepExporter::export_solid_to_string(&solid, "TaperPipePlug");
    let reimported = StepImporter::import_solid_from_str(&step_str).expect("import STEP");
    assert!(
        reimported.outer_shell.validate_closed(&tol).is_valid(),
        "Reimported taper pipe plug must be valid closed"
    );
}
