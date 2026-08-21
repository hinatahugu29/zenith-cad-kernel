//! 同じ平面に乗っている隣り合う面を、1枚に併合する。
//!
//! ## なぜ要るのか
//!
//! 穴あけやブーリアンの面片組み立ては、平面を扇形や短冊に割ってから
//! 組み直します。割った跡はそのまま残るので、出来上がりの面数・稜数が
//! 実形状の2倍近くになります。
//!
//! | 立体 | 併合前 | 実形状 |
//! | :--- | --: | --: |
//! | `make_drilled_box(40, 40, 20, 8)` | 15面 | 7面（上下面2＋側面4＋円筒1） |
//! | 直方体 − 隅の直方体（L 字角柱） | 14面 | 8面 |
//!
//! B-Rep としては妥当ですが、実形状に無い稜と面が選択肢に並び、STEP の
//! エンティティ数が倍になり、稜を選ぶ演算に無関係な候補が混じります。
//!
//! ## やること
//!
//! 支持平面が公差内で一致し、稜を共有して繋がっている面を1つの塊にまとめ、
//! **塊の内側で2回使われている稜を落として**残りを境界ループとして組み直します。
//! 内側に空いたループ（穴の口など）は内側ワイヤになります。
//!
//! ## やらないこと
//!
//! - 曲面は併合しません（同じ制御網に戻せるとは限らないため）
//! - 併合の結果、外側ループが2つ以上できる塊は**そのままにします**。
//!   1枚の面にすると領域が繋がっていないことになるためです。
//! - 稜そのものの併合（一直線に並んだ2本を1本にする）は次の段で、
//!   [`merge_collinear_edges`] が行います。

use std::collections::{BTreeMap, BTreeSet};

use zenith_geom::NurbsCurve3;
use zenith_math::{Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{
    Edge, Face, FaceGeometry, Orientation, OrientedEdge, Shell, Solid, Vertex, Wire,
};

/// 併合で何が起きたか
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MergeReport {
    pub faces_before: usize,
    pub faces_after: usize,
    pub edges_before: usize,
    pub edges_after: usize,
    /// 実際に2枚以上を1枚にまとめた塊の数
    pub merged_groups: usize,
    /// NURBS のまま持たれていた平面を、平面として持ち直した枚数
    pub planarized_faces: usize,
    /// 条件を満たさず手を付けなかった塊と、その理由
    pub skipped: Vec<String>,
}

impl MergeReport {
    pub fn summary(&self) -> String {
        format!(
            "faces {} -> {}, edges {} -> {} ({} planarized, {} group(s) merged, {} skipped)",
            self.faces_before,
            self.faces_after,
            self.edges_before,
            self.edges_after,
            self.planarized_faces,
            self.merged_groups,
            self.skipped.len()
        )
    }
}

/// 同一平面の隣接面を併合する
pub struct FaceMerger;

impl FaceMerger {
    /// ソリッド全体を整理する。
    ///
    /// ① 平面なのに NURBS として持っている面を平面に直し
    /// ② 同一平面の隣接面を1枚に併合し
    /// ③ 一直線に並んだ稜を1本に繋ぐ
    pub fn simplify_solid(solid: &Solid, tol: &Tolerance) -> Result<(Solid, MergeReport), String> {
        let faces_before = solid.outer_shell.faces.len();
        let (flat, planarized) = Self::planarize(solid, tol)?;
        let (merged, mut report) = Self::merge_coplanar(&flat, tol)?;
        let (cleaned, edges_after) = merge_collinear_edges(&merged, tol)?;
        report.faces_before = faces_before;
        report.planarized_faces = planarized;
        report.edges_after = edges_after;
        Ok((cleaned, report))
    }

    /// 制御点が公差内で同一平面に乗っている NURBS 面を、平面として持ち直す。
    ///
    /// 有理 NURBS の像は制御点の凸包に入るので、制御点が1枚の平面に乗って
    /// いれば曲面もその平面に乗ります。**近似ではありません。**
    ///
    /// 平面を NURBS のまま持っていると、
    ///
    /// - 平面しか受け付けない演算（面の併合、稜のフィレット・面取り）が
    ///   一切掛からない
    /// - 質量積分が線積分の閉じた経路ではなく求積に落ちる
    /// - STEP に `PLANE` ではなく `B_SPLINE_SURFACE` が出る
    ///
    /// という実害があります。`HoleBuilder::make_drilled_box` は 16 面すべてを
    /// NURBS で持っており、**1本もフィレットを掛けられない**状態でした。
    pub fn planarize(solid: &Solid, tol: &Tolerance) -> Result<(Solid, usize), String> {
        let mut converted = 0;
        let convert = |shell: &Shell, converted: &mut usize| -> Shell {
            let faces = shell
                .faces
                .iter()
                .map(|face| match plane_behind_nurbs(face, tol) {
                    Some(plane) => {
                        *converted += 1;
                        Face::new(
                            FaceGeometry::Plane(plane),
                            face.outer_wire.clone(),
                            face.inner_wires.clone(),
                            Orientation::Forward,
                            face.tolerance,
                        )
                    }
                    None => face.clone(),
                })
                .collect();
            Shell::new(faces, shell.is_closed)
        };

        let outer = convert(&solid.outer_shell, &mut converted);
        let inners: Vec<Shell> = solid
            .inner_shells
            .iter()
            .map(|shell| convert(shell, &mut converted))
            .collect();

        let solid = Solid::try_new(outer, inners, tol).map_err(|err| err.to_string())?;
        Ok((solid, converted))
    }

    /// 同一平面の隣接面だけを併合する（稜はそのまま）
    pub fn merge_coplanar(solid: &Solid, tol: &Tolerance) -> Result<(Solid, MergeReport), String> {
        let (outer, mut report) = Self::merge_shell(&solid.outer_shell, tol);
        let mut inners = Vec::with_capacity(solid.inner_shells.len());
        for shell in &solid.inner_shells {
            let (merged, inner_report) = Self::merge_shell(shell, tol);
            report.faces_before += inner_report.faces_before;
            report.faces_after += inner_report.faces_after;
            report.edges_before += inner_report.edges_before;
            report.edges_after += inner_report.edges_after;
            report.merged_groups += inner_report.merged_groups;
            report.skipped.extend(inner_report.skipped);
            inners.push(merged);
        }

        let solid = Solid::try_new(outer, inners, tol).map_err(|err| err.to_string())?;
        Ok((solid, report))
    }

    fn merge_shell(shell: &Shell, tol: &Tolerance) -> (Shell, MergeReport) {
        let faces = &shell.faces;
        let mut report = MergeReport {
            faces_before: faces.len(),
            edges_before: distinct_edge_count(faces),
            ..Default::default()
        };

        // 1. 稜を共有し、支持平面が一致する面どうしを1つの塊にする
        let mut groups = DisjointSet::new(faces.len());
        let mut users: BTreeMap<u64, Vec<usize>> = BTreeMap::new();
        for (index, face) in faces.iter().enumerate() {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    users.entry(oriented.edge.id).or_default().push(index);
                }
            }
        }
        for members in users.values() {
            if members.len() != 2 || members[0] == members[1] {
                continue;
            }
            if same_plane(&faces[members[0]], &faces[members[1]], tol) {
                groups.union(members[0], members[1]);
            }
        }

        // 2. 塊ごとに組み直す
        let mut by_root: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for index in 0..faces.len() {
            by_root.entry(groups.find(index)).or_default().push(index);
        }

        let mut merged_faces: Vec<Face> = Vec::with_capacity(faces.len());
        for (_, members) in by_root {
            if members.len() == 1 {
                merged_faces.push(faces[members[0]].clone());
                continue;
            }

            match merge_group(faces, &members, tol) {
                Ok(face) => {
                    report.merged_groups += 1;
                    merged_faces.push(face);
                }
                Err(reason) => {
                    report.skipped.push(reason);
                    for index in members {
                        merged_faces.push(faces[index].clone());
                    }
                }
            }
        }

        report.faces_after = merged_faces.len();
        report.edges_after = distinct_edge_count(&merged_faces);
        (Shell::new(merged_faces, shell.is_closed), report)
    }
}

/// 1つの塊を1枚の面に組み直す
fn merge_group(faces: &[Face], members: &[usize], tol: &Tolerance) -> Result<Face, String> {
    // 塊の内側で2回使われている稜は、併合後は内部の線になるので落とす
    let mut counts: BTreeMap<u64, usize> = BTreeMap::new();
    for index in members {
        for wire in std::iter::once(&faces[*index].outer_wire)
            .chain(faces[*index].inner_wires.iter())
        {
            for oriented in &wire.edges {
                *counts.entry(oriented.edge.id).or_insert(0) += 1;
            }
        }
    }
    if counts.values().any(|count| *count > 2) {
        return Err(format!(
            "a group of {} coplanar faces has an edge used more than twice; left alone",
            members.len()
        ));
    }

    let mut boundary: Vec<OrientedEdge> = Vec::new();
    for index in members {
        for wire in std::iter::once(&faces[*index].outer_wire)
            .chain(faces[*index].inner_wires.iter())
        {
            for oriented in &wire.edges {
                if counts[&oriented.edge.id] == 1 {
                    boundary.push(oriented.clone());
                }
            }
        }
    }
    if boundary.is_empty() {
        return Err(format!(
            "a group of {} coplanar faces has no boundary left; left alone",
            members.len()
        ));
    }

    let loops = assemble_loops(boundary, tol)?;

    // 外向き法線から見て反時計回りのループが外側、時計回りが穴
    let template = &faces[members[0]];
    let FaceGeometry::Plane(plane) = &template.geometry else {
        return Err("a coplanar group is not planar after all; left alone".to_string());
    };
    let flip = if template.orientation.is_forward() {
        1.0
    } else {
        -1.0
    };

    let mut outer: Option<Wire> = None;
    let mut inner: Vec<Wire> = Vec::new();
    for wire in loops {
        let area = signed_area(&wire, plane.origin, plane.u_axis, plane.v_axis) * flip;
        if area > 0.0 {
            if outer.is_some() {
                return Err(format!(
                    "a group of {} coplanar faces would need more than one outer loop; left alone",
                    members.len()
                ));
            }
            outer = Some(wire);
        } else {
            inner.push(wire);
        }
    }

    let outer = outer.ok_or_else(|| {
        format!(
            "a group of {} coplanar faces produced no outer loop; left alone",
            members.len()
        )
    })?;

    Ok(Face::new(
        template.geometry.clone(),
        outer,
        inner,
        template.orientation,
        template.tolerance,
    ))
}

/// 向き付きの稜の集まりを、閉じたループに繋ぎ直す
fn assemble_loops(mut edges: Vec<OrientedEdge>, tol: &Tolerance) -> Result<Vec<Wire>, String> {
    let same = |a: Point3, b: Point3| (a - b).norm() <= tol.linear.max(1e-9);
    let mut loops = Vec::new();

    while let Some(first) = edges.pop() {
        let start = first.start_vertex().point;
        let mut chain = vec![first];

        loop {
            let tail = chain.last().unwrap().end_vertex().point;
            if same(tail, start) {
                break;
            }
            let Some(position) = edges
                .iter()
                .position(|candidate| same(candidate.start_vertex().point, tail))
            else {
                return Err("a merged boundary does not close up; left alone".to_string());
            };
            chain.push(edges.remove(position));
        }

        loops.push(Wire::new(chain));
    }

    Ok(loops)
}

/// 平面の (u, v) 座標で測った符号付き面積
fn signed_area(wire: &Wire, origin: Point3, u_axis: Vec3, v_axis: Vec3) -> f64 {
    let points = wire.sample_points(12);
    let mut total = 0.0;
    for index in 0..points.len() {
        let a = points[index] - origin;
        let b = points[(index + 1) % points.len()] - origin;
        let (au, av) = (a.dot(&u_axis), a.dot(&v_axis));
        let (bu, bv) = (b.dot(&u_axis), b.dot(&v_axis));
        total += au * bv - bu * av;
    }
    total * 0.5
}

/// 併合の結果、頂点に2本しか集まらなくなった一直線の稜どうしを1本に繋ぐ。
///
/// 面を1枚にまとめても、割られた跡の稜はそのまま境界に並びます。その継ぎ目の
/// 頂点にはもう2本しか集まっていないので、両側が一直線なら1本にできます。
/// 曲線どうしは繋ぎません（1本の NURBS に戻せるとは限らないため）。
fn merge_collinear_edges(solid: &Solid, tol: &Tolerance) -> Result<(Solid, usize), String> {
    let mut current = solid.clone();

    // 1回の走査では、繋いだ結果さらに繋げる継ぎ目が出ることがある
    for _ in 0..16 {
        let Some(next) = merge_one_round(&current, tol)? else {
            break;
        };
        current = next;
    }

    let count = distinct_edge_count(&current.outer_shell.faces);
    Ok((current, count))
}

fn merge_one_round(solid: &Solid, tol: &Tolerance) -> Result<Option<Solid>, String> {
    let faces = &solid.outer_shell.faces;

    // 頂点ごとに、そこに端点を持つ稜と、その稜を使っている面
    let mut at_vertex: BTreeMap<(i64, i64, i64), BTreeSet<u64>> = BTreeMap::new();
    let mut edge_faces: BTreeMap<u64, BTreeSet<usize>> = BTreeMap::new();
    let mut edge_by_id: BTreeMap<u64, Edge> = BTreeMap::new();
    let quantize = |point: Point3| {
        let scale = 1.0 / tol.linear.max(1e-9);
        (
            (point.x * scale).round() as i64,
            (point.y * scale).round() as i64,
            (point.z * scale).round() as i64,
        )
    };

    for (index, face) in faces.iter().enumerate() {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                let edge = &oriented.edge;
                edge_by_id.insert(edge.id, edge.clone());
                edge_faces.entry(edge.id).or_default().insert(index);
                at_vertex
                    .entry(quantize(edge.start_vertex.point))
                    .or_default()
                    .insert(edge.id);
                at_vertex
                    .entry(quantize(edge.end_vertex.point))
                    .or_default()
                    .insert(edge.id);
            }
        }
    }

    // 繋げる継ぎ目を1つ探す
    for ids in at_vertex.values() {
        if ids.len() != 2 {
            continue;
        }
        let mut iter = ids.iter();
        let (first, second) = (*iter.next().unwrap(), *iter.next().unwrap());
        if edge_faces[&first] != edge_faces[&second] {
            continue;
        }
        let (a, b) = (&edge_by_id[&first], &edge_by_id[&second]);
        let Some(joined) = join_if_collinear(a, b, tol) else {
            continue;
        };

        return Ok(Some(replace_edges(solid, first, second, joined, tol)?));
    }

    Ok(None)
}

/// 端点を共有し、一直線に並んでいる2本の線分を1本にする
fn join_if_collinear(a: &Edge, b: &Edge, tol: &Tolerance) -> Option<Edge> {
    let a_start = a.start_vertex.point;
    let a_end = a.end_vertex.point;
    let b_start = b.start_vertex.point;
    let b_end = b.end_vertex.point;

    if !is_straight(&a.curve, a_start, a_end, tol) || !is_straight(&b.curve, b_start, b_end, tol) {
        return None;
    }

    let close = |p: Point3, q: Point3| (p - q).norm() <= tol.linear.max(1e-9);
    let (outer_a, shared, outer_b) = if close(a_end, b_start) {
        (a_start, a_end, b_end)
    } else if close(a_end, b_end) {
        (a_start, a_end, b_start)
    } else if close(a_start, b_start) {
        (a_end, a_start, b_end)
    } else if close(a_start, b_end) {
        (a_end, a_start, b_start)
    } else {
        return None;
    };

    let first = (shared - outer_a).try_normalize_safe(1e-12)?;
    let second = (outer_b - shared).try_normalize_safe(1e-12)?;
    if first.dot(&second) < 1.0 - 1e-9 {
        return None;
    }

    let start = Vertex::from_point(outer_a);
    let end = Vertex::from_point(outer_b);
    let curve = NurbsCurve3::bspline_from_points(1, vec![outer_a, outer_b]).ok()?;
    Some(Edge::new(curve, start, end, a.tolerance.max(b.tolerance)))
}

fn is_straight(curve: &NurbsCurve3, start: Point3, end: Point3, tol: &Tolerance) -> bool {
    let span = end - start;
    let length = span.norm();
    if length <= 1e-12 {
        return false;
    }
    let direction = span / length;
    for step in 1..8 {
        let point = curve.evaluate(step as f64 / 8.0);
        let offset = point - start;
        let along = offset.dot(&direction).clamp(0.0, length);
        if (offset - direction * along).norm() > tol.linear.max(1e-9) {
            return false;
        }
    }
    true
}

/// 2本の稜を1本に差し替えたソリッドを作る
fn replace_edges(
    solid: &Solid,
    first: u64,
    second: u64,
    joined: Edge,
    tol: &Tolerance,
) -> Result<Solid, String> {
    let rebuild = |wire: &Wire| -> Wire {
        let mut edges: Vec<OrientedEdge> = Vec::with_capacity(wire.edges.len());
        for oriented in &wire.edges {
            if oriented.edge.id != first && oriented.edge.id != second {
                edges.push(oriented.clone());
                continue;
            }
            // 2本のうち先に出てきた方の位置に、繋いだ1本を入れる
            let already = edges.iter().any(|existing| existing.edge.id == joined.id);
            if already {
                continue;
            }
            // 進む向きは、元の稜がどちらへ進んでいたかに合わせる
            let travel_start = oriented.start_vertex().point;
            let orientation = if (travel_start - joined.start_vertex.point).norm()
                <= (travel_start - joined.end_vertex.point).norm()
            {
                Orientation::Forward
            } else {
                Orientation::Reversed
            };
            edges.push(OrientedEdge::new(joined.clone(), orientation));
        }
        Wire::new(edges)
    };

    let mut faces = Vec::with_capacity(solid.outer_shell.faces.len());
    for face in &solid.outer_shell.faces {
        faces.push(Face::new(
            face.geometry.clone(),
            rebuild(&face.outer_wire),
            face.inner_wires.iter().map(&rebuild).collect(),
            face.orientation,
            face.tolerance,
        ));
    }

    Solid::try_new(
        Shell::new(faces, solid.outer_shell.is_closed),
        solid.inner_shells.clone(),
        tol,
    )
    .map_err(|err| err.to_string())
}

fn same_plane(a: &Face, b: &Face, tol: &Tolerance) -> bool {
    let (FaceGeometry::Plane(pa), FaceGeometry::Plane(pb)) = (&a.geometry, &b.geometry) else {
        return false;
    };
    let na = if a.orientation.is_forward() {
        pa.normal
    } else {
        -pa.normal
    };
    let nb = if b.orientation.is_forward() {
        pb.normal
    } else {
        -pb.normal
    };
    if na.dot(&nb) < 1.0 - tol.angular {
        return false;
    }
    (pb.origin - pa.origin).dot(&na).abs() <= tol.linear
}

fn distinct_edge_count(faces: &[Face]) -> usize {
    let mut ids = BTreeSet::new();
    for face in faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                ids.insert(oriented.edge.id);
            }
        }
    }
    ids.len()
}

/// 面を塊にまとめるための素朴な素集合
struct DisjointSet {
    parent: Vec<usize>,
}

impl DisjointSet {
    fn new(size: usize) -> Self {
        Self {
            parent: (0..size).collect(),
        }
    }

    fn find(&mut self, index: usize) -> usize {
        if self.parent[index] != index {
            let root = self.find(self.parent[index]);
            self.parent[index] = root;
        }
        self.parent[index]
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[rb] = ra;
        }
    }
}

/// この面の支持曲面が実は平面なら、その平面を返す。
///
/// 判定は制御点だけで行う。有理 NURBS の像は（重みが正なら）制御点の凸包に
/// 入るので、制御点が1枚の平面に乗っていれば曲面もその平面に乗る。
fn plane_behind_nurbs(face: &Face, tol: &Tolerance) -> Option<zenith_geom::PlaneSurface3> {
    let FaceGeometry::Nurbs(surface) = &face.geometry else {
        return None;
    };

    let points: Vec<Point3> = surface
        .control_points
        .iter()
        .flat_map(|row| row.iter())
        .map(|control| control.point)
        .collect();
    if points.len() < 3 {
        return None;
    }
    if surface
        .control_points
        .iter()
        .flat_map(|row| row.iter())
        .any(|control| control.weight <= 0.0)
    {
        return None;
    }

    // 曲面自身の法線を基準にする。制御点から取ると、退化した行があるときに
    // 向きが取れないことがある。
    let normal = surface.normal(0.5, 0.5)?;
    let origin = points[0];
    let extent = points
        .iter()
        .map(|point| (*point - origin).norm())
        .fold(0.0_f64, f64::max)
        .max(1.0);
    for point in &points {
        if (*point - origin).dot(&normal).abs() > tol.linear * extent {
            return None;
        }
    }

    // 面の外向きが変わらないように、平面の法線を曲面の法線に合わせる
    let outward = if face.orientation.is_forward() {
        normal
    } else {
        -normal
    };
    let seed = if outward.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u_axis = outward.cross(&seed).try_normalize_safe(1e-12)?;
    let v_axis = outward.cross(&u_axis).try_normalize_safe(1e-12)?;
    // normal = u x v になるよう v の符号を決める
    let plane = zenith_geom::PlaneSurface3::new(origin, u_axis, v_axis)?;
    if plane.normal.dot(&outward) > 0.0 {
        Some(plane)
    } else {
        zenith_geom::PlaneSurface3::new(origin, v_axis, u_axis)
    }
}
