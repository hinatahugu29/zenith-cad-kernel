//! 全周を1つのエンティティで持つ面と辺を、刻んだ形に組み直す。
//!
//! # なぜ要るか
//!
//! 他カーネルの STEP を読むと、円柱の側面は「全周を1枚で巻いた有理パッチ」に、
//! その縁は「始点と終点が同じ1本の閉じた円」になる。自前の計算はそれで厳密に
//! 通るが、その形のまま書き出すと OpenCASCADE の積分が外れる。実測（半径10、
//! 高さ40 の円柱）:
//!
//! | 書き方 | OCC が測る側面積 | 解析解 | 相対 |
//! | :--- | ---: | ---: | ---: |
//! | 四半周パッチ4枚（自前ビルダー） | 628.318530712 × 4 | 2513.274123 | 1e-11 |
//! | 全周1枚（他カーネル読み込みのまま） | 2517.673136 | 2513.274123 | +1.75e-3 |
//!
//! 平面のキャップも同じで、全円1本で縁取ると 314.101402（解析解 314.159265、
//! -1.8e-4）、四半弧4本なら 314.159265356 になる。**面でも辺でも、全周を1つで
//! 書くと落ちる。** 我々の幾何は動いていないので、これは書き方の問題である。
//!
//! # 何をするか
//!
//! 1. 閉じた辺（始点と終点が同じ点）を、内部ノットで開いた辺に刻む。
//!    シェル中でその辺を使っている全てのワイヤを同時に置き換える。
//! 2. 巻き付いている面を、そのノットで複数のパッチに割る。境界は 1 で刻んだ
//!    辺から選び直す。
//!
//! # 何をしないか
//!
//! 面の分割は、境界が**等パラメータ線でできている**場合しか行わない。回転面を
//! 読んだときはそうなっているが、そうでない面は**割らずにそのまま返す**。
//! 対応範囲外を近似で通すよりも、元のまま返すほうが良い。割れたかどうかは
//! [`RegularizeReport`] が件数で報告する。

use std::collections::HashMap;

use crate::mass_properties::MassCalculator;
use zenith_geom::{NurbsCurve3, NurbsSurface3};
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

/// 正規化で何が起きたかの内訳。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RegularizeReport {
    /// 刻んだ閉じた辺の本数。
    pub closed_edges_split: usize,
    /// 刻んで生まれた辺の本数。
    pub edges_created: usize,
    /// 割った巻き付き面の枚数。
    pub wrapped_faces_split: usize,
    /// 巻き付いているが境界が等パラメータ線でなく、割らずに残した面の枚数。
    pub wrapped_faces_left_alone: usize,
    /// 出来上がった面の総数。
    pub face_count: usize,
    /// 保持している p-curve を失うと積分が変わるため、触らずに残した面の枚数。
    pub faces_held_by_pcurves: usize,
    /// 割らずに残した面ごとの理由。診断のために残す。
    pub left_alone_reasons: Vec<String>,
}

impl RegularizeReport {
    /// 巻き付いた面も閉じた辺も残っていないか。
    pub fn is_fully_regular(&self) -> bool {
        self.wrapped_faces_left_alone == 0
    }
}

/// ソリッドを、全周を1つで持つ面・辺が無い形に組み直す。
pub struct Regularizer;

impl Regularizer {
    /// 外殻と内殻のそれぞれを正規化する。
    pub fn regularize_solid(solid: &Solid, tol: &Tolerance) -> (Solid, RegularizeReport) {
        let mut report = RegularizeReport::default();
        let (outer, outer_report) = Self::regularize_shell(&solid.outer_shell, tol);
        report.absorb(&outer_report);

        let mut inner = Vec::with_capacity(solid.inner_shells.len());
        for shell in &solid.inner_shells {
            let (regular, shell_report) = Self::regularize_shell(shell, tol);
            report.absorb(&shell_report);
            inner.push(regular);
        }

        (Solid::new(outer, inner), report)
    }

    /// シェル1枚を正規化する。辺を先に刻んでから面を割る。順序が要る:
    /// 面の分割は、刻んだ後の辺から自分の境界を選ぶ。
    pub fn regularize_shell(shell: &Shell, tol: &Tolerance) -> (Shell, RegularizeReport) {
        let mut report = RegularizeReport::default();

        // 保持している p-curve が積分を担っている面は、組み直すと答えが変わる。
        // どの面がそうかは推測せず、p-curve を外して積分し直して測る。
        let protected: Vec<bool> = shell
            .faces
            .iter()
            .map(|face| !Self::survives_losing_its_pcurves(face))
            .collect();
        report.faces_held_by_pcurves = protected.iter().filter(|held| **held).count();
        for (index, held) in protected.iter().enumerate() {
            if *held {
                report.left_alone_reasons.push(format!(
                    "face {index} is held together by the p-curves the file stated"
                ));
            }
        }

        let faces = Self::split_closed_edges(&shell.faces, &protected, tol, &mut report);

        let mut out = Vec::with_capacity(faces.len());
        // 新しく作る辺は、シェル全体で一つの棚から配る。パッチごとに作ると
        // 隣り合うパッチが同じ子午線を別の辺として持ち、辺の対が壊れる。
        let mut shared = SharedEdges::default();
        for (index, face) in faces.iter().enumerate() {
            if protected.get(index).copied().unwrap_or(false) {
                out.push(face.clone());
                continue;
            }
            match Self::split_wrapped_face(face, tol, &mut shared) {
                FaceSplit::Split(pieces) => {
                    report.wrapped_faces_split += 1;
                    out.extend(pieces);
                }
                FaceSplit::NotWrapped => out.push(face.clone()),
                FaceSplit::LeftAlone(reason) => {
                    report.wrapped_faces_left_alone += 1;
                    report.left_alone_reasons.push(reason);
                    out.push(face.clone());
                }
            }
        }

        report.face_count = out.len();
        (Shell::new(out, shell.is_closed), report)
    }

    /// 閉じた辺を開いた辺に刻み、使っている全てのワイヤを差し替える。
    ///
    /// 差し替えはシェル全体で一度に行う。片側の面だけ刻むと、隣の面が古い辺を
    /// 使い続けて辺の対が壊れる。
    fn split_closed_edges(
        faces: &[Face],
        protected: &[bool],
        tol: &Tolerance,
        report: &mut RegularizeReport,
    ) -> Vec<Face> {
        // 触らないと決めた面が使っている辺は、刻んではいけない。片側だけ刻むと
        // 辺の対が壊れる。
        let mut off_limits: std::collections::HashSet<u64> = std::collections::HashSet::new();
        for (index, face) in faces.iter().enumerate() {
            if !protected.get(index).copied().unwrap_or(false) {
                continue;
            }
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    off_limits.insert(oriented.edge.id);
                }
            }
        }

        let mut replacement: HashMap<u64, Vec<Edge>> = HashMap::new();

        for face in faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let edge = &oriented.edge;
                    if replacement.contains_key(&edge.id)
                        || off_limits.contains(&edge.id)
                        || !Self::edge_is_closed(edge, tol)
                    {
                        continue;
                    }
                    if let Some(pieces) = Self::chop_closed_edge(edge, tol) {
                        report.closed_edges_split += 1;
                        report.edges_created += pieces.len();
                        replacement.insert(edge.id, pieces);
                    }
                }
            }
        }

        if replacement.is_empty() {
            return faces.to_vec();
        }

        faces
            .iter()
            .map(|face| {
                let mut rebuilt = face.clone();
                rebuilt.outer_wire = Self::apply_replacement(&face.outer_wire, &replacement);
                rebuilt.inner_wires = face
                    .inner_wires
                    .iter()
                    .map(|wire| Self::apply_replacement(wire, &replacement))
                    .collect();
                // 辺が入れ替わったので、辺ごとに紐づいていた p-curve は当たらない。
                // 残しておくと古い辺 id を指したままになるので落とす。
                rebuilt.pcurves = None;
                rebuilt
            })
            .collect()
    }

    fn apply_replacement(wire: &Wire, replacement: &HashMap<u64, Vec<Edge>>) -> Wire {
        let mut edges = Vec::with_capacity(wire.edges.len());
        for oriented in &wire.edges {
            match replacement.get(&oriented.edge.id) {
                None => edges.push(oriented.clone()),
                Some(pieces) => {
                    if oriented.orientation.is_forward() {
                        for piece in pieces {
                            edges.push(OrientedEdge::forward(piece.clone()));
                        }
                    } else {
                        // 逆向きに使われているなら、並びも逆になる。
                        for piece in pieces.iter().rev() {
                            edges.push(OrientedEdge::reversed(piece.clone()));
                        }
                    }
                }
            }
        }
        Wire::new(edges)
    }

    /// 保持している p-curve を落としても、この面の積分が変わらないか。
    ///
    /// 解析曲面から読んだ面では境界から導出し直しても同じ値になるが、他カーネル
    /// が書いたトリム B-spline では、面の外まで伸びた曲面の上にどう境界が乗るかを
    /// ファイルの p-curve が決めている。実測で、`occ_reference_cylinder_nurbs`
    /// はこれを落とすだけで体積が 9.1e-2 動く（12566.26 -> 13710.91）。
    /// 変わる面は組み直さない。
    fn survives_losing_its_pcurves(face: &Face) -> bool {
        if face.pcurves.is_none() {
            return true;
        }
        let params = TessellationParams::default();
        let (area_before, volume_before) = MassCalculator::compute_face_integral(face, &params);

        let mut stripped = face.clone();
        stripped.pcurves = None;
        let (area_after, volume_after) = MassCalculator::compute_face_integral(&stripped, &params);

        let scale = area_before.abs().max(volume_before.abs()).max(1.0);
        (area_after - area_before).abs() <= scale * 1e-9
            && (volume_after - volume_before).abs() <= scale * 1e-9
    }

    fn edge_is_closed(edge: &Edge, tol: &Tolerance) -> bool {
        (edge.start_vertex.point - edge.end_vertex.point).norm() <= tol.linear.max(edge.tolerance)
    }

    /// 閉じた辺を、内部ノットの位置で開いた辺の列に刻む。
    ///
    /// 全周の円は内部ノットが 1/4, 1/2, 3/4 にあるので四半弧4本になる。内部
    /// ノットが無い曲線は3等分する（2つに割ると両端が同じ点のままの辺が残る）。
    fn chop_closed_edge(edge: &Edge, tol: &Tolerance) -> Option<Vec<Edge>> {
        let cuts = Self::interior_cut_parameters(&edge.curve);
        if cuts.is_empty() {
            return None;
        }

        let mut pieces = Vec::with_capacity(cuts.len() + 1);
        let mut rest = edge.curve.clone();
        for cut in &cuts {
            let (left, right) = rest.split_at(*cut)?;
            pieces.push(left);
            rest = right;
        }
        pieces.push(rest);

        // 割った点をそのまま頂点にする。曲線の端点を使うので、隣り合う辺の
        // 端点は同じ値になり、辺の対が保たれる。
        let mut vertices: Vec<Vertex> = Vec::with_capacity(pieces.len());
        for piece in &pieces {
            let (t0, _) = piece.param_range();
            vertices.push(Vertex::new(piece.evaluate(t0), edge.tolerance));
        }

        let mut edges = Vec::with_capacity(pieces.len());
        for (index, piece) in pieces.iter().enumerate() {
            let start = vertices[index].clone();
            let end = vertices[(index + 1) % vertices.len()].clone();
            // 刻んだ端が本当に隣の始点かを測る。合わなければ刻まない。
            let (_, t1) = piece.param_range();
            if (piece.evaluate(t1) - end.point).norm() > tol.linear.max(edge.tolerance) {
                return None;
            }
            edges.push(Edge::new(piece.clone(), start, end, edge.tolerance));
        }

        Some(edges)
    }

    /// 曲線の内部ノット（重複を除いた値）。無ければ3等分の位置。
    fn interior_cut_parameters(curve: &NurbsCurve3) -> Vec<f64> {
        let (t0, t1) = curve.param_range();
        let span = (t1 - t0).abs().max(1.0);
        let mut cuts: Vec<f64> = Vec::new();
        for knot in &curve.knots.knots {
            if *knot <= t0 + span * 1e-12 || *knot >= t1 - span * 1e-12 {
                continue;
            }
            if cuts.iter().all(|c| (c - knot).abs() > span * 1e-9) {
                cuts.push(*knot);
            }
        }
        if cuts.is_empty() {
            cuts.push(t0 + (t1 - t0) / 3.0);
            cuts.push(t0 + (t1 - t0) * 2.0 / 3.0);
        }
        cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        cuts
    }

    /// 巻き付いている面を、等パラメータ線の格子で割る。
    fn split_wrapped_face(face: &Face, tol: &Tolerance, shared: &mut SharedEdges) -> FaceSplit {
        let surface = match &face.geometry {
            FaceGeometry::Nurbs(nurbs) => nurbs,
            _ => return FaceSplit::NotWrapped,
        };

        let wraps_u = Self::grid_closes_in_u(surface, tol);
        let wraps_v = Self::grid_closes_in_v(surface, tol);
        if !wraps_u && !wraps_v {
            return FaceSplit::NotWrapped;
        }

        // 巻いていると分かった面は、巻いていない側のノットでも刻む。球を u
        // だけで割ると、4枚とも境界が極2点だけの「くさび」になり、どの面も
        // 同じ頂点集合を持ってしまう。自前の球ビルダーが 4 x 2 に割っているのは
        // 同じ理由である。
        let ((u0, u1), (v0, v1)) = surface.param_range();
        let u_cuts = Self::interior_knots(&surface.knots_u.knots, u0, u1);
        let v_cuts = Self::interior_knots(&surface.knots_v.knots, v0, v1);
        let _ = wraps_v;
        if u_cuts.is_empty() && v_cuts.is_empty() {
            return FaceSplit::LeftAlone("wraps but has no interior knot to cut at".to_string());
        }

        let patches = match Self::grid_of_patches(surface, &u_cuts, &v_cuts) {
            Some(patches) => patches,
            None => return FaceSplit::LeftAlone("the surface would not split at its knots".to_string()),
        };

        // 面が持っていた辺を、パッチごとに割り当て直す。どのパッチにも入らない
        // 辺が1本でも残るなら、この面の境界は等パラメータ線でできていない。
        let boundary: Vec<OrientedEdge> = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .flat_map(|wire| wire.edges.iter().cloned())
            .collect();

        let mut pieces = Vec::with_capacity(patches.len());
        let mut used = vec![false; boundary.len()];
        for (index, patch) in patches.iter().enumerate() {
            match Self::face_from_patch(face, patch, &boundary, &mut used, tol, shared) {
                Some(built) => pieces.push(built),
                None => {
                    return FaceSplit::LeftAlone(format!(
                        "patch {index} of {} could not be closed into a loop",
                        patches.len()
                    ))
                }
            }
        }
        let orphans = used.iter().filter(|flag| !**flag).count();
        if orphans > 0 {
            return FaceSplit::LeftAlone(format!(
                "{orphans} of {} boundary edges lie on no patch border",
                used.len()
            ));
        }

        FaceSplit::Split(pieces)
    }

    /// 曲面を u, v の切り位置で格子状に割る。
    fn grid_of_patches(
        surface: &NurbsSurface3,
        u_cuts: &[f64],
        v_cuts: &[f64],
    ) -> Option<Vec<PatchCell>> {
        let mut columns: Vec<NurbsSurface3> = Vec::new();
        let mut rest = surface.clone();
        for cut in u_cuts {
            let (left, right) = rest.split_u(*cut)?;
            columns.push(left);
            rest = right;
        }
        columns.push(rest);

        let mut cells = Vec::new();
        for column in columns {
            let mut rest = column;
            for cut in v_cuts {
                let (bottom, top) = rest.split_v(*cut)?;
                cells.push(PatchCell::new(bottom));
                rest = top;
            }
            cells.push(PatchCell::new(rest));
        }
        Some(cells)
    }

    /// 1枚のパッチから面を組む。
    ///
    /// 辺は「繋がる順に拾う」のではなく、UV の反時計回り（下・右・上・左）に
    /// **決め打ちで**並べる。繋がる順に拾うと、始めた辺しだいで巻きが逆になり、
    /// 面の半分が裏返る。向きは面積分の符号に直結するので、ここは選択の余地を
    /// 残さない。
    fn face_from_patch(
        face: &Face,
        patch: &PatchCell,
        boundary: &[OrientedEdge],
        used: &mut [bool],
        tol: &Tolerance,
        shared: &mut SharedEdges,
    ) -> Option<Face> {
        let mut wire_edges: Vec<OrientedEdge> = Vec::with_capacity(4);

        for border in patch.directed_borders(tol) {
            let (t0, t1) = border.param_range();
            let border_start = border.evaluate(t0);
            let border_end = border.evaluate(t1);

            // 元の面が持っていた辺で、この境界に重なるものがあればそれを使う。
            // 新しく作ると、隣の面が使っている辺と別物になる。
            let mut matched = None;
            for (index, oriented) in boundary.iter().enumerate() {
                if used[index] {
                    continue;
                }
                let candidate = Edge::new(
                    border.clone(),
                    Vertex::new(border_start, tol.linear),
                    Vertex::new(border_end, tol.linear),
                    tol.linear,
                );
                if Self::same_edge_geometry(&oriented.edge, &candidate, tol) {
                    used[index] = true;
                    matched = Some(oriented.edge.clone());
                    break;
                }
            }

            let edge = match matched {
                Some(edge) => edge,
                None => shared.intern(
                    Edge::new(
                        border.clone(),
                        Vertex::new(border_start, tol.linear),
                        Vertex::new(border_end, tol.linear),
                        tol.linear,
                    ),
                    tol,
                ),
            };

            // 辺そのものの向きが境界の向きと同じかを、始点で測って決める。
            let forward = (edge.start_vertex.point - border_start).norm()
                <= (edge.end_vertex.point - border_start).norm();
            wire_edges.push(if forward {
                OrientedEdge::forward(edge)
            } else {
                OrientedEdge::reversed(edge)
            });
        }

        if wire_edges.len() < 3 {
            return None;
        }

        // `Face` を組み立てるときは `Face::new` を通す。構造体リテラルで
        // 作ると `id` を自分で書くことになり、ここでは 0 を書いていた。
        // 割った面がすべて同じ id を名乗るので、id を鍵にして面を覚える
        // 仕組みを後から足すと、静かに取り違える。
        Some(Face::new(
            FaceGeometry::Nurbs(patch.surface.clone()),
            Wire::new(wire_edges),
            Vec::new(),
            face.orientation,
            face.tolerance,
        ))
    }

    fn same_edge_geometry(a: &Edge, b: &Edge, tol: &Tolerance) -> bool {
        let sample = |edge: &Edge, count: usize| -> Vec<Point3> {
            let (t0, t1) = edge.curve.param_range();
            (0..=count)
                .map(|i| edge.curve.evaluate(t0 + (t1 - t0) * i as f64 / count as f64))
                .collect()
        };
        let left = sample(a, 6);
        let right = sample(b, 6);
        let forward = left
            .iter()
            .zip(right.iter())
            .all(|(p, q)| (*p - *q).norm() <= tol.linear);
        let backward = left
            .iter()
            .zip(right.iter().rev())
            .all(|(p, q)| (*p - *q).norm() <= tol.linear);
        forward || backward
    }

    fn interior_knots(knots: &[f64], t0: f64, t1: f64) -> Vec<f64> {
        let span = (t1 - t0).abs().max(1.0);
        let mut cuts: Vec<f64> = Vec::new();
        for knot in knots {
            if *knot <= t0 + span * 1e-12 || *knot >= t1 - span * 1e-12 {
                continue;
            }
            if cuts.iter().all(|c: &f64| (c - knot).abs() > span * 1e-9) {
                cuts.push(*knot);
            }
        }
        cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        cuts
    }

    /// 制御格子の最初の行と最後の行が重なっているか（u 方向に巻いている）。
    pub fn grid_closes_in_u(surface: &NurbsSurface3, tol: &Tolerance) -> bool {
        let rows = &surface.control_points;
        let first = &rows[0];
        let last = &rows[rows.len() - 1];
        rows.len() > 2
            && first
                .iter()
                .zip(last.iter())
                .all(|(a, b)| (a.point - b.point).norm() <= tol.linear)
    }

    /// 制御格子の最初の列と最後の列が重なっているか（v 方向に巻いている）。
    pub fn grid_closes_in_v(surface: &NurbsSurface3, tol: &Tolerance) -> bool {
        let cols = surface.control_points[0].len();
        cols > 2
            && surface
                .control_points
                .iter()
                .all(|row| (row[0].point - row[cols - 1].point).norm() <= tol.linear)
    }
}

impl RegularizeReport {
    pub(crate) fn absorb(&mut self, other: &RegularizeReport) {
        self.closed_edges_split += other.closed_edges_split;
        self.edges_created += other.edges_created;
        self.wrapped_faces_split += other.wrapped_faces_split;
        self.wrapped_faces_left_alone += other.wrapped_faces_left_alone;
        self.faces_held_by_pcurves += other.faces_held_by_pcurves;
        self.face_count += other.face_count;
        self.left_alone_reasons
            .extend(other.left_alone_reasons.iter().cloned());
    }
}

/// 他カーネルに渡す STEP の書き出し口。
///
/// [`zenith_io::StepExporter`] をそのまま呼ぶと、読み込んだ形のまま——全周1枚の
/// パッチ、全周1本の辺のまま——書き出す。それを OpenCASCADE に読ませると積分が
/// 外れる（円柱の側面で +1.75e-3、球の体積で +9.3e-3）。ここを通すと、書く前に
/// [`Regularizer`] が刻む。
///
/// 自前ビルダーの出力は元から刻まれているので、通しても何も起きない（実測で
/// 分割 0 件、体積の移動 0）。**書き出しの既定の口はこちらである。**
///
/// この口が `zenith_io` ではなくここにあるのは、依存の向きのためである。刻んで
/// よいかの判定には面の積分が要り、それは `zenith_algo` にある。`zenith_io` は
/// `zenith_algo` に依存できない。
pub struct StepInterop;

impl StepInterop {
    /// 正規化してから STEP 文字列にする。何が起きたかも返す。
    pub fn export_solid_to_string(
        solid: &Solid,
        product_name: &str,
        tol: &Tolerance,
    ) -> (String, RegularizeReport) {
        let (regular, report) = Regularizer::regularize_solid(solid, tol);
        (
            zenith_io::StepExporter::export_solid_to_string(&regular, product_name),
            report,
        )
    }

    /// 正規化してから STEP ファイルに書く。
    ///
    /// ファイルに書く口は、面が1枚でも書けなければ**書かずにエラーを返す**。
    /// 文字列を返す口は `String` で失敗を表せないので、そちらとは扱いが違う。
    pub fn export_solid_to_file<P: AsRef<std::path::Path>>(
        solid: &Solid,
        path: P,
        product_name: &str,
        tol: &Tolerance,
    ) -> std::io::Result<RegularizeReport> {
        let (regular, report) = Regularizer::regularize_solid(solid, tol);
        let content = zenith_io::StepExporter::export_solids_to_string_checked(
            std::slice::from_ref(&regular),
            product_name,
        )
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidInput, error))?;
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, content)?;
        Ok(report)
    }

    /// 複数の Solid を、それぞれ正規化してから1つの STEP に書く。
    pub fn export_solids_to_file<P: AsRef<std::path::Path>>(
        solids: &[Solid],
        path: P,
        product_name: &str,
        tol: &Tolerance,
    ) -> std::io::Result<RegularizeReport> {
        let mut report = RegularizeReport::default();
        let mut regular = Vec::with_capacity(solids.len());
        for solid in solids {
            let (piece, piece_report) = Regularizer::regularize_solid(solid, tol);
            report.absorb(&piece_report);
            regular.push(piece);
        }
        let content =
            zenith_io::StepExporter::export_solids_to_string_checked(&regular, product_name)
                .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidInput, error))?;
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, content)?;
        Ok(report)
    }
}

/// 新しく作った辺の棚。同じ形の辺は一度だけ作り、隣り合うパッチに配る。
#[derive(Default)]
struct SharedEdges {
    edges: Vec<Edge>,
}

impl SharedEdges {
    /// 同じ形の辺が既にあればそれを、無ければ受け取った辺を登録して返す。
    fn intern(&mut self, edge: Edge, tol: &Tolerance) -> Edge {
        if let Some(existing) = self
            .edges
            .iter()
            .find(|candidate| Regularizer::same_edge_geometry(candidate, &edge, tol))
        {
            return existing.clone();
        }
        self.edges.push(edge.clone());
        edge
    }
}

enum FaceSplit {
    Split(Vec<Face>),
    NotWrapped,
    LeftAlone(String),
}

/// 割ったあとの1枚のパッチと、その4辺。
struct PatchCell {
    surface: NurbsSurface3,
}

impl PatchCell {
    fn new(surface: NurbsSurface3) -> Self {
        Self { surface }
    }

    /// パッチの4辺を、UV 反時計回りに向きを揃えた曲線として返す。
    ///
    /// 1点に潰れている辺は落とす（球の極がこれに当たる。そこは辺ではなく頂点
    /// になるので、パッチの境界は3本になる）。
    fn directed_borders(&self, tol: &Tolerance) -> Vec<NurbsCurve3> {
        let ((u0, u1), (v0, v1)) = self.surface.param_range();
        // `iso_curve_v(v)` は v を固定して u に走る曲線、`iso_curve_u(u)` は
        // その逆。下 (v=v0, u 増) 、右 (u=u1, v 増)、上 (v=v1, u 減)、
        // 左 (u=u0, v 減) の順に辿ると UV 反時計回りになる。
        let candidates = [
            self.surface.iso_curve_v(v0),
            self.surface.iso_curve_u(u1),
            self.surface.iso_curve_v(v1).map(|c| c.reversed()),
            self.surface.iso_curve_u(u0).map(|c| c.reversed()),
        ];
        let mut borders = Vec::with_capacity(4);
        for candidate in candidates.into_iter().flatten() {
            let (t0, t1) = candidate.param_range();
            let start = candidate.evaluate(t0);
            let end = candidate.evaluate(t1);
            let mid = candidate.evaluate((t0 + t1) * 0.5);
            if (start - end).norm() <= tol.linear && (start - mid).norm() <= tol.linear {
                continue; // 退化した辺（極）
            }
            borders.push(candidate);
        }
        borders
    }
}

#[cfg(test)]
mod tests {
    use super::{RegularizeReport, Regularizer};
    use crate::mass_properties::MassCalculator;
    use crate::primitive::PrimitiveBuilder;
    use crate::revolve::RevolveBuilder;
    use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3};
    use zenith_math::{Point3, Tolerance, Vec3};
    use zenith_tess::TessellationParams;
    use zenith_topo::{Face, Shell, Solid, Wire};

    /// 半径 `r` の球を、全周に巻いた1枚の面だけで作る。境界は無い（極は退化
    /// した点なので辺にならない）。他カーネルの STEP を読むとこの形になる。
    fn one_face_sphere(r: f64) -> Solid {
        let tol = Tolerance::default();
        let weight = std::f64::consts::FRAC_1_SQRT_2;
        let profile = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(Point3::new(0.0, 0.0, r)),
                ControlPoint3::new(Point3::new(r, 0.0, r), weight),
                ControlPoint3::unweighted(Point3::new(r, 0.0, 0.0)),
                ControlPoint3::new(Point3::new(r, 0.0, -r), weight),
                ControlPoint3::unweighted(Point3::new(0.0, 0.0, -r)),
            ],
            KnotVector::new(vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]),
        )
        .unwrap();
        let surface = RevolveBuilder::revolve_curve(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::PI * 2.0,
            &tol,
        )
        .unwrap();
        let face = Face::from_nurbs_surface(surface, Wire::new(Vec::new()));
        Solid::new(Shell::new(vec![face], true), Vec::new())
    }

    fn volume(solid: &Solid) -> f64 {
        MassCalculator::compute_from_brep(solid, &TessellationParams::default()).volume
    }

    #[test]
    fn a_wrapped_sphere_becomes_eight_patches_without_moving() {
        let tol = Tolerance::default();
        let solid = one_face_sphere(10.0);
        let before = volume(&solid);
        assert_eq!(solid.outer_shell.faces.len(), 1);

        let (regular, report) = Regularizer::regularize_solid(&solid, &tol);

        // 自前の球ビルダーと同じ 4 x 2 の分割になる。
        assert_eq!(regular.outer_shell.faces.len(), 8);
        assert_eq!(report.wrapped_faces_split, 1);
        assert_eq!(report.wrapped_faces_left_alone, 0);

        let after = volume(&regular);
        let moved = (after - before).abs() / before.abs();
        assert!(moved < 1e-8, "regularizing moved the sphere by {moved:.3e}");

        // 解析解とも比べる。組み直しは形を変えていない。
        let truth = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
        assert!((after - truth).abs() / truth < 1e-8);

        let shell = regular.outer_shell.validate_closed(&tol);
        assert!(
            shell.errors.is_empty(),
            "regularized shell is not closed: {:?}",
            shell.errors
        );
    }

    #[test]
    fn nothing_wraps_or_closes_once_the_shell_is_regular() {
        let tol = Tolerance::default();
        let (regular, _) = Regularizer::regularize_solid(&one_face_sphere(6.0), &tol);

        for face in &regular.outer_shell.faces {
            if let zenith_topo::FaceGeometry::Nurbs(surface) = &face.geometry {
                assert!(
                    !Regularizer::grid_closes_in_u(surface, &tol),
                    "a patch still wraps in u"
                );
                assert!(
                    !Regularizer::grid_closes_in_v(surface, &tol),
                    "a patch still wraps in v"
                );
            }
            for oriented in &face.outer_wire.edges {
                let edge = &oriented.edge;
                let gap = (edge.start_vertex.point - edge.end_vertex.point).norm();
                assert!(gap > tol.linear, "a closed edge survived, gap {gap:.3e}");
            }
        }
    }

    /// 自前のビルダーは元から刻んで作るので、通しても何も起きてはならない。
    /// 「変わらない」ことの確認で、ここが動いたら正規化が余計なことをしている。
    #[test]
    fn a_solid_that_is_already_regular_comes_back_untouched() {
        let tol = Tolerance::default();
        let cases: Vec<(&str, Solid)> = vec![
            ("sphere", PrimitiveBuilder::make_sphere(10.0).unwrap()),
            ("cylinder", PrimitiveBuilder::make_cylinder(10.0, 40.0).unwrap()),
            ("box", PrimitiveBuilder::make_box(20.0, 30.0, 40.0).unwrap()),
        ];

        for (name, solid) in cases {
            let before = volume(&solid);
            let face_count = solid.outer_shell.faces.len();
            let (regular, report) = Regularizer::regularize_solid(&solid, &tol);

            assert_eq!(
                regular.outer_shell.faces.len(),
                face_count,
                "{name} changed face count"
            );
            assert_eq!(report.wrapped_faces_split, 0, "{name} was split");
            assert_eq!(report.closed_edges_split, 0, "{name} had an edge chopped");
            assert_eq!(
                volume(&regular),
                before,
                "{name} volume moved on a no-op pass"
            );
        }
    }

    /// 割って出来た面は、それぞれ別の `id` を名乗ること。
    ///
    /// 以前は構造体リテラルで組み立てていて `id: 0` を書いており、割った面が
    /// 全部同じ id を名乗っていました。面を id で覚える仕組み——ブーリアンの
    /// 交線を面の組ごとに記憶する、など——を足した途端に取り違えます。
    /// 使われていない間は誰も気づかない類の罠なので、測っておきます。
    #[test]
    fn every_face_that_comes_out_of_a_split_has_its_own_id() {
        let tol = Tolerance::default();
        let (regular, report) = Regularizer::regularize_solid(&one_face_sphere(10.0), &tol);
        assert!(report.wrapped_faces_split > 0, "nothing was split");

        let mut ids: Vec<u64> = regular
            .outer_shell
            .faces
            .iter()
            .map(|face| face.id)
            .collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(
            ids.len(),
            count,
            "{count} faces share only {} ids between them",
            ids.len()
        );
        assert!(!ids.contains(&0), "a face came out with id 0");
    }

    #[test]
    fn the_report_adds_up_across_shells() {
        let mut total = RegularizeReport::default();
        let piece = RegularizeReport {
            closed_edges_split: 2,
            edges_created: 8,
            wrapped_faces_split: 1,
            wrapped_faces_left_alone: 0,
            faces_held_by_pcurves: 0,
            face_count: 6,
            left_alone_reasons: Vec::new(),
        };
        total.absorb(&piece);
        total.absorb(&piece);
        assert_eq!(total.closed_edges_split, 4);
        assert_eq!(total.face_count, 12);
        assert!(total.is_fully_regular());
    }
}
