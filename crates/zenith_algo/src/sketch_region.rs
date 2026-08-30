//! **スケッチから閉領域を取り出し、3D の輪にする**（HANDOVER 9-H の第1段）。
//!
//! # ここが何を埋めるのか
//!
//! 押し出しの受け皿は既にあります（[`crate::ExtrudeBuilder::extrude_wire`]）。
//! スケッチソルバーもあります（[`crate::SketchSolver`]）。**繋がっていな
//! かったのは、その間だけ**です——「解けた点と線」から「閉じた輪」を
//! 取り出し、作業平面に載せて 3D の [`Wire`] にするところ。
//!
//! # 何を作らないか
//!
//! **拘束の編集の仕方・選択・履歴は、ここでは扱いません。** 9-G が
//! 「スケッチは『何を編集させるか』で形が決まる。カーネル側だけで先に
//! 作ると、画面の話が始まってから作り直しになりがち」と書いた通りです。
//!
//! **線引きの基準は「閉じた式で測れるか」です。** ここにあるのは全部
//! 測れる側です——輪の面積は多角形の公式で出ますし、押し出した体積は
//! `面積 × 高さ` と突き合わせられます。

use crate::SketchSolver;
use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
use zenith_math::{Point2, Point3, Tolerance, Vec3, Vec3Ext};
use zenith_topo::{Edge, OrientedEdge, Vertex, Wire};

/// 輪の1区間が円弧であるときの、その弧。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopArc {
    pub center: Point2,
    /// 始点から終点へ、反時計回りに回るか。
    pub counterclockwise: bool,
}

/// スケッチの上で閉じた輪。
///
/// **点の並びと、区間ごとの弧**で持ちます。`arcs[i]` は
/// `points[i]` から `points[i+1]` への区間で、`None` なら直線です。
#[derive(Debug, Clone, PartialEq)]
pub struct SketchLoop {
    /// 輪をなす点。**最後の点と最初の点は繋がっています**（重複させません）。
    pub points: Vec<Point2>,
    /// 区間ごとの弧。長さは `points` と同じです。
    pub arcs: Vec<Option<LoopArc>>,
}

impl SketchLoop {
    /// 直線だけの輪。
    pub fn from_points(points: Vec<Point2>) -> Self {
        let arcs = vec![None; points.len()];
        Self { points, arcs }
    }

    /// 区間 `index` の、始点・終点・弧。
    fn span(&self, index: usize) -> (Point2, Point2, Option<LoopArc>) {
        let count = self.points.len();
        (
            self.points[index],
            self.points[(index + 1) % count],
            self.arcs[index],
        )
    }

    /// **符号つき面積**。反時計回りなら正です。
    ///
    /// 多角形の公式（靴紐）に、**円弧の切片ぶんを足します**。
    ///
    /// ```text
    /// 切片の面積 = (r² / 2) (θ − sin θ)      θ は符号つきの回り角
    /// ```
    ///
    /// **どちらも閉じた式です。** 刻んで足していません。
    pub fn signed_area(&self) -> f64 {
        let count = self.points.len();
        if count < 2 {
            return 0.0;
        }
        let mut twice = 0.0;
        for index in 0..count {
            let (here, next, _) = self.span(index);
            twice += here.x * next.y - next.x * here.y;
        }
        let mut area = twice * 0.5;

        for index in 0..count {
            let (here, next, arc) = self.span(index);
            let Some(arc) = arc else { continue };
            let radius = (here - arc.center).norm();
            if !(radius > 0.0) {
                continue;
            }
            let sweep = signed_sweep(arc.center, here, next, arc.counterclockwise);
            area += 0.5 * radius * radius * (sweep - sweep.sin());
        }
        area
    }

    /// 面積の大きさ。向きは見ません。
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    /// **反時計回りに揃えます。** 押し出しの向きは輪の向きで決まるので、
    /// 揃えておかないと外向きの法線が裏返ります。
    pub fn counterclockwise(mut self) -> Self {
        if self.signed_area() < 0.0 {
            let count = self.points.len();
            // 点を逆順にすると、区間の対応もずれます。**弧も一緒に付け替えます。**
            let mut arcs = Vec::with_capacity(count);
            for index in 0..count {
                // 逆順の区間 i は、元の区間 (count - 2 - i) にあたります。
                let source = (count + count - 2 - index) % count;
                arcs.push(self.arcs[source].map(|arc| LoopArc {
                    center: arc.center,
                    counterclockwise: !arc.counterclockwise,
                }));
            }
            self.points.reverse();
            self.arcs = arcs;
        }
        self
    }
}

/// 中心まわりに、始点から終点へ回る角。**符号つき**で、向きに合わせて
/// `0` から `2π` の側へ回します。
fn signed_sweep(center: Point2, from: Point2, to: Point2, counterclockwise: bool) -> f64 {
    let a = from - center;
    let b = to - center;
    let angle = b.y.atan2(b.x) - a.y.atan2(a.x);
    let tau = std::f64::consts::TAU;
    let wrapped = angle.rem_euclid(tau);
    if counterclockwise {
        wrapped
    } else {
        wrapped - tau
    }
}

/// スケッチを置く平面。
///
/// **原点と2本の向きで持ちます。** スケッチの `(u, v)` が
/// `origin + u·x_axis + v·y_axis` に写ります。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorkPlane {
    pub origin: Point3,
    pub x_axis: Vec3,
    pub y_axis: Vec3,
}

impl WorkPlane {
    /// `z = 0` の平面。`u → x`、`v → y`。
    pub fn xy() -> Self {
        Self {
            origin: Point3::origin(),
            x_axis: Vec3::new(1.0, 0.0, 0.0),
            y_axis: Vec3::new(0.0, 1.0, 0.0),
        }
    }

    /// 原点と法線から作る。**`x_axis` は法線に直交する向きから1本選びます。**
    ///
    /// **推測しません**——法線が正規化できるか、選んだ向きが法線と平行で
    /// ないかを測ってから返します。
    pub fn from_normal(origin: Point3, normal: Vec3) -> Option<Self> {
        let normal = normal.try_normalize_safe(1e-12)?;
        // 法線といちばん揃っていない座標軸を種にします。**同じ向きを選ぶと
        // 外積が 0 になります。**
        let helper = if normal.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let x_axis = normal.cross(&helper).try_normalize_safe(1e-12)?;
        let y_axis = normal.cross(&x_axis);
        Some(Self {
            origin,
            x_axis,
            y_axis,
        })
    }

    /// 平面の法線。`x_axis × y_axis`。
    pub fn normal(&self) -> Vec3 {
        self.x_axis.cross(&self.y_axis)
    }

    /// スケッチの `(u, v)` を 3D へ。
    pub fn at(&self, uv: Point2) -> Point3 {
        self.origin + self.x_axis * uv.x + self.y_axis * uv.y
    }
}

/// スケッチから**閉じた輪**を取り出す。
///
/// # 取り出し方
///
/// 線分を「点と点を結ぶ辺」として見て、**次の辺を端点の一致で辿ります**。
/// 出発点へ戻れたら1つの輪です。
///
/// **端点の一致は座標で見ます。** ソルバーは点に番号を振っていますが、
/// 拘束で重ねた2点（`Coincident`）は番号が違うまま同じ場所に来ます。
/// 番号で辿ると、そこで輪が切れます。
///
/// # 分岐は断ります
///
/// 1つの点に3本以上の線分が集まっていたら、**どちらへ進むかを決められ
/// ません**。推測せずに `None` を返します。**もっともらしい輪を返して
/// はいけません**——このリポジトリの決まりです。
pub fn extract_loops(solver: &SketchSolver, tol: &Tolerance) -> Option<Vec<SketchLoop>> {
    let at = |id: crate::sketch_solver::PointId| -> Option<Point2> {
        solver.get_point(id).map(|p| Point2::new(p[0], p[1]))
    };

    // 端点を座標で束ねます。
    let mut nodes: Vec<Point2> = Vec::new();
    let node_of = |point: Point2, nodes: &mut Vec<Point2>| -> usize {
        for (index, existing) in nodes.iter().enumerate() {
            if (existing - point).norm() <= tol.linear {
                return index;
            }
        }
        nodes.push(point);
        nodes.len() - 1
    };

    // 区間は「両端の節点」と「弧（直線なら `None`）」で持ちます。
    let mut segments: Vec<(usize, usize, Option<LoopArc>)> = Vec::new();
    for line in &solver.lines {
        let (Some(a), Some(b)) = (at(line.p1), at(line.p2)) else {
            return None;
        };
        let (ia, ib) = (node_of(a, &mut nodes), node_of(b, &mut nodes));
        if ia == ib {
            // 長さ 0 の線分。輪にはなりません。
            continue;
        }
        segments.push((ia, ib, None));
    }
    for arc in &solver.arcs {
        let (Some(center), Some(start), Some(end)) =
            (at(arc.center), at(arc.start), at(arc.end))
        else {
            return None;
        };
        // **半径が合っているかを測ります。** 合っていなければ、それは弧では
        // ありません——推測せずに断ります。
        let radius = (start - center).norm();
        if !(radius > tol.linear) {
            return None;
        }
        if ((end - center).norm() - radius).abs() > tol.linear * radius.max(1.0) {
            return None;
        }
        let (ia, ib) = (node_of(start, &mut nodes), node_of(end, &mut nodes));
        if ia == ib {
            // 始点と終点が同じ。**まるごとの円は、輪の1区間にはできません**
            // ——半周ずつに割ってください。
            return None;
        }
        segments.push((
            ia,
            ib,
            Some(LoopArc {
                center,
                counterclockwise: arc.counterclockwise,
            }),
        ));
    }
    if segments.is_empty() {
        return Some(Vec::new());
    }

    // 点ごとに、繋がっている辺の番号。
    let mut around: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (index, (a, b, _)) in segments.iter().enumerate() {
        around[*a].push(index);
        around[*b].push(index);
    }
    // **分岐と行き止まりは断ります。**
    for edges in &around {
        if edges.len() != 2 {
            return None;
        }
    }

    let mut used = vec![false; segments.len()];
    let mut loops: Vec<SketchLoop> = Vec::new();
    for start in 0..segments.len() {
        if used[start] {
            continue;
        }
        // **`points[i]` は区間 `i` の始点、`arcs[i]` はその区間の弧**です。
        // 辿り始めた節点へ戻ったら1周です。
        let mut points: Vec<Point2> = Vec::new();
        let mut arcs: Vec<Option<LoopArc>> = Vec::new();
        let (first_node, _, _) = segments[start];
        let mut node = first_node;
        let mut edge = start;
        loop {
            used[edge] = true;
            points.push(nodes[node]);
            // **辿る向きと、弧の向きを揃えます。** 弧は始点から終点へ回る
            // 向きで持っているので、逆から入ったら回り方も裏返します。
            let (a, b, arc) = segments[edge];
            arcs.push(arc.map(|arc| LoopArc {
                center: arc.center,
                counterclockwise: if a == node {
                    arc.counterclockwise
                } else {
                    !arc.counterclockwise
                },
            }));
            node = if a == node { b } else { a };
            if node == first_node {
                break;
            }
            let Some(next) = around[node].iter().copied().find(|candidate| !used[*candidate])
            else {
                // 戻れませんでした。**輪になっていません。**
                return None;
            };
            edge = next;
        }
        // 弧があれば2点でも輪になります（半円と弦、など）。
        let has_arc = arcs.iter().any(|arc| arc.is_some());
        if points.len() < if has_arc { 2 } else { 3 } {
            return None;
        }
        loops.push(SketchLoop { points, arcs });
    }
    Some(loops)
}

/// 輪を作業平面に載せて、3D の [`Wire`] にする。
///
/// **区間は1次の NURBS（直線）です。** 円弧を入れるときはここに分岐を足します。
pub fn loop_to_wire(sketch_loop: &SketchLoop, plane: &WorkPlane, tol: &Tolerance) -> Option<Wire> {
    let count = sketch_loop.points.len();
    if count < 3 {
        return None;
    }
    let mut edges = Vec::with_capacity(count);
    for index in 0..count {
        let from_uv = sketch_loop.points[index];
        let to_uv = sketch_loop.points[(index + 1) % count];
        let from = plane.at(from_uv);
        let to = plane.at(to_uv);
        if (to - from).norm() <= tol.linear {
            return None;
        }
        match sketch_loop.arcs[index] {
            // **弧は四半ずつに割ります。**
            //
            // 有理2次で張れるのは半周未満です（半周ちょうどで重みが 0 に
            // なります）。長円の端は**ちょうど半周**なので、そのままでは
            // 張れません。四半に割れば厳密に張れます——**近似ではありません**。
            // カーネルの他の場所（円柱・球）も四半で持っています。
            Some(arc) => {
                for (piece_from, piece_to) in split_arc(&arc, from_uv, to_uv) {
                    let curve = arc_curve(&arc, piece_from, piece_to, plane)?;
                    let (a, b) = (plane.at(piece_from), plane.at(piece_to));
                    if (b - a).norm() <= tol.linear {
                        return None;
                    }
                    edges.push(OrientedEdge::forward(Edge::new(
                        curve,
                        Vertex::new(a, tol.linear),
                        Vertex::new(b, tol.linear),
                        tol.linear,
                    )));
                }
            }
            None => {
                let curve = NurbsCurve3::bspline_from_points(1, vec![from, to]).ok()?;
                edges.push(OrientedEdge::forward(Edge::new(
                    curve,
                    Vertex::new(from, tol.linear),
                    Vertex::new(to, tol.linear),
                    tol.linear,
                )));
            }
        }
    }
    Some(Wire::new(edges))
}

/// 弧を、**有理2次で張れる大きさ**に割る。
///
/// 返すのは区切りの点の対です。四半（`π/2`）を上限にします——カーネルの
/// 他の場所（円柱・球）も四半で持っているので、揃えます。
fn split_arc(arc: &LoopArc, from: Point2, to: Point2) -> Vec<(Point2, Point2)> {
    let radius = (from - arc.center).norm();
    let sweep = signed_sweep(arc.center, from, to, arc.counterclockwise);
    let quarter = std::f64::consts::FRAC_PI_2;
    let pieces = (sweep.abs() / quarter).ceil().max(1.0) as usize;
    if pieces <= 1 {
        return vec![(from, to)];
    }
    let start_angle = (from.y - arc.center.y).atan2(from.x - arc.center.x);
    let step = sweep / pieces as f64;
    let at = |index: usize| -> Point2 {
        if index == 0 {
            from
        } else if index == pieces {
            to
        } else {
            let angle = start_angle + step * index as f64;
            Point2::new(
                arc.center.x + radius * angle.cos(),
                arc.center.y + radius * angle.sin(),
            )
        }
    };
    (0..pieces).map(|index| (at(index), at(index + 1))).collect()
}

/// 円弧を**有理2次で厳密に**張る。
///
/// 半周以上は重みが 0 以下になるので張れません。**割ってください**——
/// 近似で誤魔化すより、断るほうが正しい形です。
fn arc_curve(
    arc: &LoopArc,
    from: Point2,
    to: Point2,
    plane: &WorkPlane,
) -> Option<NurbsCurve3> {
    let radius = (from - arc.center).norm();
    if !(radius > 0.0) {
        return None;
    }
    let sweep = signed_sweep(arc.center, from, to, arc.counterclockwise);
    let half = sweep * 0.5;
    let weight = half.cos();
    if weight <= 1e-9 {
        // 半周以上。**近似しません。**
        return None;
    }
    // 中間の制御点は、弦の中点から外へ `radius / cos(half)` の所です。
    let start_angle = (from.y - arc.center.y).atan2(from.x - arc.center.x);
    let middle_angle = start_angle + half;
    let middle = Point2::new(
        arc.center.x + radius / weight * middle_angle.cos(),
        arc.center.y + radius / weight * middle_angle.sin(),
    );
    NurbsCurve3::new(
        2,
        vec![
            ControlPoint3::unweighted(plane.at(from)),
            ControlPoint3::new(plane.at(middle), weight),
            ControlPoint3::unweighted(plane.at(to)),
        ],
        KnotVector::clamped_uniform(3, 2),
    )
    .ok()
}

/// スケッチを押し出して立体にする。**外周1つだけの場合**です。
///
/// 穴つきは [`crate::ExtrudeBuilder::extrude_face_with_holes`] が受け取る
/// ので、輪が複数のときはそちらへ渡してください。ここではまだ**どの輪が
/// 穴かを決めていません**（入れ子の判定は書いていません）。
pub fn extrude_sketch(
    solver: &SketchSolver,
    plane: &WorkPlane,
    height: f64,
    tol: &Tolerance,
) -> Result<zenith_topo::Solid, String> {
    let loops = extract_loops(solver, tol).ok_or_else(|| {
        "スケッチから閉じた輪を取り出せません（分岐・行き止まり・開いた鎖）".to_string()
    })?;
    if loops.len() != 1 {
        return Err(format!(
            "外周が1つの場合だけ扱えます（取り出した輪は {} 本）",
            loops.len()
        ));
    }
    if !(height.abs() > tol.linear) {
        return Err("高さが 0 です".to_string());
    }
    let outline = loops.into_iter().next().expect("1本").counterclockwise();
    let wire = loop_to_wire(&outline, plane, tol)
        .ok_or_else(|| "輪を 3D の輪にできません".to_string())?;
    crate::ExtrudeBuilder::extrude_wire(&wire, plane.normal() * height, tol)
}
