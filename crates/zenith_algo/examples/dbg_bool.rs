use std::panic;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Vec3};
use zenith_topo::Solid;

fn probe(name: &str, a: &Solid, b: &Solid, op: BooleanOpType) {
    let tol = Tolerance::default();
    let r = panic::catch_unwind(panic::AssertUnwindSafe(|| {
        BooleanEngine::boolean_solids_exact_result(a, b, op, &tol)
    }));
    match r {
        Ok(Ok(res)) => {
            let all_valid = res.solids.iter().all(|s| s.is_topologically_valid(&tol));
            let params = zenith_tess::TessellationParams { u_divisions: 8, v_divisions: 8 };
            let v: f64 = res.solids.iter().map(|s| zenith_algo::MassCalculator::compute_from_brep(s, &params).volume).sum();
            println!("{name:38} ok solids={} all_valid={all_valid} V={v:.3}", res.len());
        }
        Ok(Err(e)) => println!("{name:38} rejected: {}", e.chars().take(44).collect::<String>()),
        Err(_) => println!("{name:38} *** PANIC ***"),
    }
}

fn main() {
    panic::set_hook(Box::new(|_| {}));
    let cube = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).unwrap();
    let shifted = BrepTransform::translate_solid(&cube, Vec3::new(5.0, 0.0, 0.0));
    let touching = BrepTransform::translate_solid(&cube, Vec3::new(10.0, 0.0, 0.0));
    let edge_touch = BrepTransform::translate_solid(&cube, Vec3::new(10.0, 10.0, 0.0));
    let corner_touch = BrepTransform::translate_solid(&cube, Vec3::new(10.0, 10.0, 10.0));
    let sliver = BrepTransform::translate_solid(&cube, Vec3::new(10.0 - 1e-9, 0.0, 0.0));
    let far = BrepTransform::translate_solid(&cube, Vec3::new(100.0, 0.0, 0.0));
    let inner = BrepTransform::translate_solid(&PrimitiveBuilder::make_box(2.0,2.0,2.0).unwrap(), Vec3::new(4.0,4.0,4.0));

    for (name, b) in [
        ("self", &cube), ("half overlap", &shifted), ("face touching", &touching),
        ("edge touching", &edge_touch), ("corner touching", &corner_touch),
        ("sliver overlap", &sliver), ("disjoint", &far), ("contained", &inner),
    ] {
        for (opname, op) in [("union", BooleanOpType::Union), ("inter", BooleanOpType::Intersection), ("diff", BooleanOpType::Difference)] {
            probe(&format!("{name} / {opname}"), &cube, b, op);
        }
    }
    let _ = panic::take_hook();
    println!("done");
}
