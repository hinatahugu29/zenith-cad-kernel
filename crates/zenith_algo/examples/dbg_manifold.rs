use std::collections::BTreeMap;
use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Shell, Solid};

/// 各頂点まわりの面が1つの扇（サイクル）を成すかを調べる
fn non_manifold_vertices(shell: &Shell, tol: &Tolerance) -> usize {
    // 頂点位置ごとに (辺A, 辺B) の隣接を集める
    let mut vertex_points: Vec<Point3> = Vec::new();
    let mut junctions: Vec<Vec<(u64, u64)>> = Vec::new();

    let key_of = |point: Point3, points: &mut Vec<Point3>, js: &mut Vec<Vec<(u64, u64)>>| -> usize {
        if let Some(i) = points.iter().position(|p| (p - point).norm() <= tol.linear * 10.0) {
            return i;
        }
        points.push(point);
        js.push(Vec::new());
        points.len() - 1
    };

    for face in &shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            let n = wire.edges.len();
            for i in 0..n {
                let current = &wire.edges[i];
                let next = &wire.edges[(i + 1) % n];
                let point = current.end_vertex().point;
                let k = key_of(point, &mut vertex_points, &mut junctions);
                junctions[k].push((current.edge.id, next.edge.id));
            }
        }
    }

    let mut bad = 0;
    for pairs in &junctions {
        let mut degree: BTreeMap<u64, usize> = BTreeMap::new();
        let mut adjacency: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        for (a, b) in pairs {
            *degree.entry(*a).or_insert(0) += 1;
            *degree.entry(*b).or_insert(0) += 1;
            adjacency.entry(*a).or_default().push(*b);
            adjacency.entry(*b).or_default().push(*a);
        }
        if degree.values().any(|d| *d != 2) {
            bad += 1;
            continue;
        }
        // 連結性: 1つのサイクルでなければ非多様体
        let start = *degree.keys().next().unwrap();
        let mut seen = vec![start];
        let mut stack = vec![start];
        while let Some(node) = stack.pop() {
            for next in adjacency.get(&node).cloned().unwrap_or_default() {
                if !seen.contains(&next) {
                    seen.push(next);
                    stack.push(next);
                }
            }
        }
        if seen.len() != degree.len() {
            bad += 1;
        }
    }
    bad
}

fn report(name: &str, solid: &Solid) {
    let tol = Tolerance::default();
    println!("{name:32} non_manifold_vertices={}", non_manifold_vertices(&solid.outer_shell, &tol));
}

fn main() {
    let tol = Tolerance::default();
    report("box", &PrimitiveBuilder::make_box(10.0, 20.0, 30.0).unwrap());
    report("cylinder", &PrimitiveBuilder::make_cylinder(10.0, 30.0).unwrap());
    report("sphere", &PrimitiveBuilder::make_sphere(15.0).unwrap());
    report("cone", &PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).unwrap());
    report("frustum", &PrimitiveBuilder::make_cone(10.0, 4.0, 20.0).unwrap());
    report("torus", &PrimitiveBuilder::make_torus(20.0, 5.0).unwrap());
    report("drilled", &zenith_algo::HoleBuilder::make_drilled_box(30.0,30.0,15.0,5.0).unwrap());
    report("hollow", &zenith_algo::ShellBuilder::make_hollow_box(30.0,40.0,25.0,2.5,1).unwrap());
    report("filleted", &zenith_algo::FilletBuilder::fillet_box_z_edges(20.0,30.0,40.0,4.0,&tol).unwrap());
    report("chamfered", &zenith_algo::ChamferBuilder::chamfer_box_z_edges(20.0,30.0,40.0,3.0,&tol).unwrap());

    let cube = PrimitiveBuilder::make_box(10.0,10.0,10.0).unwrap();
    let corner = BrepTransform::translate_solid(&cube, Vec3::new(10.0,10.0,10.0));
    let r = BooleanEngine::boolean_solids_exact_result(&cube, &corner, BooleanOpType::Union, &tol).unwrap();
    report("corner-touching union", &r.solids[0]);
    let face = BrepTransform::translate_solid(&cube, Vec3::new(10.0,0.0,0.0));
    let r2 = BooleanEngine::boolean_solids_exact_result(&cube, &face, BooleanOpType::Union, &tol).unwrap();
    report("face-touching union", &r2.solids[0]);
}
