//! 継ぎ目をまたぐ窓を、**継ぎ目で2本に割った鎖**で順に切る（4-841 の次の一手）。
//!
//! 窓は uv で u=-0.074..+0.074・v=0.364..0.636——継ぎ目をまたぐので1つの
//! 領域として表せない。しかし**継ぎ目の2点で切ると2本の開いた鎖**になり、
//! どちらも境界（継ぎ目）から境界へ渡る。1本ずつ当てて、片の面積と uv を見る。
use zenith_algo::{
    BrepIntersectionBuilder, BrepTransform, FaceSplitter, MassCalculator, PrimitiveBuilder,
};
use zenith_math::Tolerance;
use zenith_math::Vec3;
use zenith_tess::TessellationParams;
use zenith_topo::Face;

fn report(label: &str, face: &Face, tol: &Tolerance) {
    let (area, _) = MassCalculator::compute_face_integral(face, &TessellationParams::default());
    let uv = face
        .pcurves(tol)
        .ok()
        .map(|p| {
            let (mut lo, mut hi) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
            for seg in &p.outer_loop.segments {
                for c in &seg.curve.control_points {
                    lo = (lo.0.min(c.point.x), lo.1.min(c.point.y));
                    hi = (hi.0.max(c.point.x), hi.1.max(c.point.y));
                }
            }
            let mut points: Vec<zenith_math::Point2> = Vec::new();
            for (index, seg) in p.outer_loop.segments.iter().enumerate() {
                let sampled = seg.curve.sample_points(16);
                points.extend(sampled.into_iter().skip(usize::from(index > 0)));
            }
            let mut signed = 0.0;
            for i in 0..points.len() {
                let (a, b) = (points[i], points[(i + 1) % points.len()]);
                signed += a.x * b.y - b.x * a.y;
            }
            signed *= 0.5;
            format!("u {:.4}..{:.4} v {:.4}..{:.4} uv符号つき面積 {signed:+.6}", lo.0, hi.0, lo.1, hi.1)
        })
        .unwrap_or_else(|| "p曲線が作れない".to_string());
    println!(
        "  {label}: 外周 {} 稜、内輪 {} 本、面積 {area:.6}、{uv}",
        face.outer_wire.edges.len(),
        face.inner_wires.len()
    );
    if let Ok(pcurves) = face.pcurves(tol) {
        for (index, seg) in pcurves.outer_loop.segments.iter().enumerate() {
            let (mut lo, mut hi) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
            for c in &seg.curve.control_points {
                lo = (lo.0.min(c.point.x), lo.1.min(c.point.y));
                hi = (hi.0.max(c.point.x), hi.1.max(c.point.y));
            }
            let (t0, t1) = seg.curve.param_range();
            let a = seg.curve.evaluate(t0);
            let b = seg.curve.evaluate(t1);
            println!(
                "      p曲線{index}: ({:.4},{:.4})->({:.4},{:.4}) 箱 u {:.4}..{:.4} v {:.4}..{:.4}",
                a.x, a.y, b.x, b.y, lo.0, hi.0, lo.1, hi.1
            );
        }
    }
    for oriented in &face.outer_wire.edges {
        let start = oriented.start_vertex().point;
        let end = oriented.end_vertex().point;
        let mid = oriented.evaluate_normalized(0.5);
        if mid.x > 8.0 && mid.y.abs() < 4.01 && mid.z.abs() < 4.01 && (mid.y.abs() > 1.0 || mid.z.abs() > 1.0) {
            println!(
                "      交線の稜 ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3}) 中点 ({:.3},{:.3},{:.3})",
                start.x, start.y, start.z, end.x, end.y, end.z, mid.x, mid.y, mid.z
            );
        }
    }
}

fn main() {
    let tol = Tolerance::default();
    let sphere = PrimitiveBuilder::make_sphere_periodic(10.0).unwrap();
    let tool = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(8.0, 8.0, 8.0).unwrap(),
        Vec3::new(5.0, -4.0, -4.0),
    );
    let faces_a = BrepIntersectionBuilder::all_faces_of(&sphere);
    let faces_b = BrepIntersectionBuilder::all_faces_of(&tool);
    let candidates =
        BrepIntersectionBuilder::collect_intersection_edge_candidates(&faces_a, &faces_b, &tol);

    let mut plus = Vec::new();
    let mut minus = Vec::new();
    for candidate in &candidates {
        if candidate.face_a_index != 0 {
            continue;
        }
        let (t0, t1) = candidate.edge.curve.param_range();
        let mid = candidate.edge.curve.evaluate((t0 + t1) * 0.5);
        if mid.y > 0.0 {
            plus.push(candidate.edge.clone());
        } else {
            minus.push(candidate.edge.clone());
        }
    }
    report("元の面", &faces_a[0], &tol);

    let (first, r1) = FaceSplitter::split_by_chain(&faces_a[0], &plus, &tol).unwrap();
    println!("y>0 を当てた: 片 {} 枚、面積残差 {:.3e}", first.len(), r1.area_residual);
    for (index, face) in first.iter().enumerate() {
        report(&format!("1段目 片{index}"), face, &tol);
    }

    // いちばん広い片（補集合）にだけ2本目を当てる。
    let widest = first
        .iter()
        .enumerate()
        .max_by(|a, b| {
            let area = |f: &Face| MassCalculator::compute_face_integral(f, &TessellationParams::default()).0;
            area(a.1).partial_cmp(&area(b.1)).unwrap()
        })
        .map(|(i, _)| i)
        .unwrap();
    println!("2本目は 1段目 片{widest} にだけ当てる");
    match FaceSplitter::split_by_chain(&first[widest], &minus, &tol) {
        Ok((second, r2)) => {
            println!("y<0 を当てた: 片 {} 枚、面積残差 {:.3e}", second.len(), r2.area_residual);
            for (index, face) in second.iter().enumerate() {
                report(&format!("2段目 片{index}"), face, &tol);
            }
        }
        Err(reason) => println!("y<0: 断り {reason}"),
    }
}
