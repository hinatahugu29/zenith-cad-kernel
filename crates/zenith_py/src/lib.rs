//! Zenith CAD Python Bindings (PyO3)

pub mod direct_edit;
pub mod io;
pub mod mesh;
pub mod modeling;
pub mod payload;
pub mod primitives;

pub use mesh::PyMesh;
use pyo3::prelude::*;

/// Zenith CAD Pythonモジュール
#[pymodule]
fn zenith_cad(_py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMesh>()?;

    // Primitives
    m.add_function(wrap_pyfunction!(primitives::make_box, m)?)?;
    m.add_function(wrap_pyfunction!(primitives::make_cylinder, m)?)?;
    m.add_function(wrap_pyfunction!(primitives::make_sphere, m)?)?;
    m.add_function(wrap_pyfunction!(primitives::make_cone, m)?)?;
    m.add_function(wrap_pyfunction!(primitives::make_torus, m)?)?;
    m.add_function(wrap_pyfunction!(primitives::make_regular_prism, m)?)?;
    m.add_function(wrap_pyfunction!(primitives::make_curve_patch, m)?)?;

    // Modeling & Features
    m.add_function(wrap_pyfunction!(modeling::make_filleted_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_chamfered_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_drilled_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_counterbore_hole_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_hex_nut, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_hex_bolt, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_stepped_shaft, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_shaft_with_keyway, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_hollow_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_through_hollow_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_hollow_extrusion, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_draft_extrusion, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_sweep_pipe, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_sweep_wire, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_helix_solid, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_round_wire_spring, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_revolve, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_revolve_solid, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_partial_revolve_solid, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_loft, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_loft_solid, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_guided_loft_solid, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_mirror_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_mirror_compound_casing, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_polyline_pipe, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_polyline_sweep, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_open_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::slice_box_by_plane, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::compute_box_mass_properties, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::check_boxes_interference, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::check_exact_boxes_interference, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::compute_boxes_min_distance, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_countersink_hole_box, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_circular_flange, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_shaft_with_annular_groove, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::export_box_section_dxf, m)?)?;

    m.add_function(wrap_pyfunction!(modeling::make_boolean, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_exact_box_boolean, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_exact_drill_boolean, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::thicken_surface_patch, m)?)?;
    m.add_function(wrap_pyfunction!(modeling::make_spur_gear, m)?)?;




    // Direct Modeling
    m.add_function(wrap_pyfunction!(direct_edit::fillet_box_single_edge, m)?)?;
    m.add_function(wrap_pyfunction!(direct_edit::chamfer_box_single_edge, m)?)?;
    m.add_function(wrap_pyfunction!(direct_edit::push_pull_box, m)?)?;
    m.add_function(wrap_pyfunction!(direct_edit::taper_box, m)?)?;
    m.add_function(wrap_pyfunction!(direct_edit::cap_planar_wire, m)?)?;
    m.add_function(wrap_pyfunction!(direct_edit::cap_dome_wire, m)?)?;

    // IO Exchange
    m.add_function(wrap_pyfunction!(io::import_step_file, m)?)?;

    // Shader & Solver Payloads
    m.add_function(wrap_pyfunction!(payload::get_box_shader_payload, m)?)?;
    m.add_function(wrap_pyfunction!(payload::get_primitive_shader_payload, m)?)?;
    m.add_function(wrap_pyfunction!(payload::solve_2d_sketch, m)?)?;

    Ok(())
}
