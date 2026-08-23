//! ブーリアン1回ぶんの仕事の内訳を出す。
//!
//! 45ケースの走査は数分かかるので、内訳を見るだけなら重い1ケースで足りる。
//! 数え上げは決定的なので、1回走らせれば同じ値が出る。

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};

fn main() {
    let tol = Tolerance::default();
    let upright = PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap();
    // boolean_envelope の "cylinder x cylinder (perpendicular cross)" と同じ配置。
    let rotation =
        zenith_math::Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_2);
    let along_x = BrepTransform::transform_solid(
        &PrimitiveBuilder::make_cylinder(6.0, 40.0).unwrap(),
        &rotation,
    )
    .unwrap();
    let lying = BrepTransform::translate_solid(&along_x, Vec3::new(-20.0, 0.0, 20.0));

    for (name, op) in [
        ("union", BooleanOpType::Union),
        ("difference", BooleanOpType::Difference),
        ("intersection", BooleanOpType::Intersection),
    ] {
        let before = zenith_geom::work_counter::snapshot();
        let outcome =
            BooleanEngine::boolean_solids_exact_result_unverified(&upright, &lying, op, &tol);
        let work = zenith_geom::work_counter::snapshot().since(&before);

        let coarse_cost = work.point_surface_coarse_searches * 353;
        let marching_cost = work.marching_newton_iterations * 2;
        println!("=== crossed cylinders, {name} ({})", if outcome.is_ok() { "ok" } else { "ERROR" });
        println!("  surface evaluations          {:>12}", work.surface_evaluations);
        println!(
            "    of which coarse searches   {:>12}  ({} searches x 353)",
            coarse_cost, work.point_surface_coarse_searches
        );
        println!(
            "    of which marching Newton   {:>12}  ({} iterations x 2)",
            marching_cost, work.marching_newton_iterations
        );
        println!(
            "    unaccounted for            {:>12}",
            work.surface_evaluations as i64 - coarse_cost as i64 - marching_cost as i64
        );
        println!(
            "  projections {} total, {} of them coarse ({:.0}%)",
            work.point_surface_projections,
            work.point_surface_coarse_searches,
            100.0 * work.point_surface_coarse_searches as f64
                / work.point_surface_projections.max(1) as f64
        );
        println!(
            "  projection Newton {} iterations, {} damping trials ({:.1} trials per iteration)",
            work.projection_newton_iterations,
            work.projection_damping_trials,
            work.projection_damping_trials as f64
                / work.projection_newton_iterations.max(1) as f64
        );
        println!(
            "    those cost about {} evaluations ({} iterations + {} trials)",
            work.projection_newton_iterations + work.projection_damping_trials,
            work.projection_newton_iterations,
            work.projection_damping_trials
        );
        println!(
            "  trim boundaries carried {} points in total, worst single loop {}",
            work.uv_boundary_points, work.uv_worst_boundary
        );
        println!(
            "  {} uv triangulations producing {} triangles ({:.0} each)",
            work.uv_triangulations,
            work.uv_triangles,
            work.uv_triangles as f64 / work.uv_triangulations.max(1) as f64
        );
        println!(
            "  {} face integrals, {} seed searches, {} whole-solid tessellations",
            work.face_integrals, work.seed_searches, work.solid_tessellations
        );
        println!();
    }
}
