//! 接触は、それ自体では位相を作らない（HANDOVER 3-1 の規約）。
//!
//! 2つの立体が交わらずに触れているだけの場所で、**答えが多様体になるか**を
//! 測ります。予想ではなく、交線のまわりの材料そのものに当てます。
//!
//! ## 何を測るか
//!
//! 交線の上に点を取り、その稜に**垂直な平面**の中で、半径 r の円周を考えます。
//! 円周の上で
//!
//! 1. **A の内側の角度の区間**を求める
//! 2. **B の内側の角度の区間**を求める
//! 3. 演算（和・差・積）を**区間どうしで**取る
//!
//! 残った区間が**いくつに分かれているか**を数えます。
//!
//! - 1つ（または 0）— そこで材料は繋がっている。**多様体**
//! - 2つ以上 — そこで材料は**線でしか触れていない**。答えは非多様体
//!
//! 横断している普通の交線では、A も B も局所的には半空間なので、和・差・積の
//! どれを取っても区間は1つです。**2つ以上になるのは接触のときだけ**で、この
//! 判定に「接触かどうか」の事前分類は要りません。
//!
//! ## なぜ円周に標本を撒いて数えないのか（実測 2026/08/24）
//!
//! 最初はそう書きました。円周に64点撒いて、内側の点が何本の弧に分かれるかを
//! 数える。**測ったら、接触している3件すべてで「弧は1つ」と答えました。**
//!
//! 理由は形のほうにありました。半径 6 の円柱が箱の壁に接している配置で、
//! 差の材料が残るのは
//!
//! ```text
//! 0 < cos(theta) <= r / (2R)
//! ```
//!
//! の帯だけです。r = 0.2、R = 6 なら幅は **0.95 度**。64点の刻みは 5.6 度なので、
//! **またいで通り過ぎます**。半径を小さくしても帯は比例して細くなるので、
//! 刻みを増やす以外に逃げ道がありません。
//!
//! いまは A と B の区間を別々に求めてから**区間どうしで演算します**。細い帯は
//! 「幅 90 度の区間」から「幅 89.04 度の区間」を引いた差として**そのまま出て
//! きます**。撒いた点で細部を捉える必要がありません。
//!
//! ## なぜ断るのか
//!
//! `Solid` は多様体 B-Rep です。線でしか繋がっていない立体を「もっともらしい
//! 立体」にして返すのは誤答で、このカーネルがいちばん避けてきた失敗です
//! （HANDOVER 第2章）。**断るのは実装の不足ではなく、型が表現できないものを
//! 表現したふりをしない、という設計判断です。**
//!
//! ## 値を1つ選ばない
//!
//! 半径は2つ振ります（稜の長さの 1e-2 と 1e-3）。**両方で 2 以上**のときだけ
//! 非多様体と言います。1つの値だけで測って通ったことにするのは、この
//! リポジトリで実際にやらかした失敗です（HANDOVER 4-34）。
//!
//! `ZENITH_CONTACT_WHY=1` を付けると、測った区間と数が1行ずつ出ます。

use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{Edge, Solid};

use crate::boolean::BooleanOpType;
use crate::boolean_validation::exact_inside;

/// 円周を最初に見る刻みの数。**細部はここでは捉えません**（上の説明）。
/// 立体1つぶんの区間の切り替わりを挟むだけの粗さがあれば足ります。
const RING_SAMPLES: usize = 48;

/// 切り替わりの角度を挟み込む回数。48刻みの1区画を 2^-40 まで詰めます。
const BISECTION_STEPS: usize = 40;

/// 決まらなかった標本を取り直す回数。
const JITTER_ATTEMPTS: usize = 4;

/// これ以下の隙間しかない区間どうしは、同じ塊として数えます。
const ANGULAR_MERGE: f64 = 1e-9;

/// 交線の上で、材料が線でしか触れていない場所。
#[derive(Debug, Clone, PartialEq)]
pub struct ContactPinch {
    /// 測った点（交線の上）。
    pub point: Point3,
    /// そこでの交線の向き。
    pub tangent: Vec3,
    /// 材料が分かれている数（2以上）。
    pub components: usize,
    /// その数が出た半径のうち、大きいほう。
    pub radius: f64,
}

impl ContactPinch {
    /// 断るときの言い分。**場所と数を名指しします。**
    pub fn describe(&self) -> String {
        format!(
            "the true result is non-manifold at ({:.6} {:.6} {:.6}): the material there is only in contact, falling into {} pieces around the intersection (measured on rings of radius {:.3e} and {:.3e}). A manifold B-Rep solid cannot hold this, so it is refused rather than returned as a plausible-looking solid",
            self.point.x,
            self.point.y,
            self.point.z,
            self.components,
            self.radius,
            self.radius / 10.0
        )
    }
}

/// 交線の候補を順に測り、非多様体になる場所を最初に1つ返す。
///
/// 交線が1本も無いか、どこも多様体なら `None`。
pub fn find_result_pinch(
    solid_a: &Solid,
    solid_b: &Solid,
    edges: &[Edge],
    op: BooleanOpType,
    tol: &Tolerance,
) -> Option<ContactPinch> {
    edges
        .iter()
        .find_map(|edge| pinch_along_edge(solid_a, solid_b, edge, op, tol))
}

/// 1本の交線を測る。
pub fn pinch_along_edge(
    solid_a: &Solid,
    solid_b: &Solid,
    edge: &Edge,
    op: BooleanOpType,
    tol: &Tolerance,
) -> Option<ContactPinch> {
    let length = sampled_length(edge);
    // 半径は稜の長さから取ります。公差に埋もれる長さの稜では測りません。
    let radius = length * 1e-2;
    if radius <= tol.linear * 100.0 || !radius.is_finite() {
        return None;
    }

    let explain = std::env::var_os("ZENITH_CONTACT_WHY").is_some();

    // 端点は別の稜や頂点が集まる場所なので、内側の3点で測ります。
    for fraction in [0.25_f64, 0.5, 0.75] {
        let point = edge_point(edge, fraction);
        let Some(tangent) = edge_tangent(edge, fraction) else {
            continue;
        };
        let (u, v) = frame_perpendicular_to(tangent);
        let ring = Ring {
            center: point,
            u,
            v,
            radius,
        };

        // **半径を振ります。** 片方だけで 2 以上でも、それは形ではなく半径の
        // 取り方を測っている可能性があります。
        let coarse = count_components(solid_a, solid_b, &ring, op, tol);
        // 粗いほうが 2 未満なら細かいほうは測りません。**測るのは高くつきます**
        // ——標本1つが立体の全面への射影1回です。
        let fine = match coarse {
            Some(count) if count >= 2 => {
                count_components(solid_a, solid_b, &ring.scaled(0.1), op, tol)
            }
            _ => None,
        };

        if explain {
            let show = |value: Option<usize>| match value {
                Some(count) => count.to_string(),
                None => "undecided".to_string(),
            };
            let fine_shown = match coarse {
                Some(count) if count >= 2 => show(fine),
                // 粗いほうで足りているので測っていません。
                _ => "not measured".to_string(),
            };
            eprintln!(
                "CONTACTWHY ({:.4} {:.4} {:.4}) t={fraction} r={:.4}: pieces coarse {} fine {}",
                point.x,
                point.y,
                point.z,
                radius,
                show(coarse),
                fine_shown
            );
        }

        let (Some(coarse), Some(fine)) = (coarse, fine) else {
            continue;
        };
        if coarse < 2 || fine < 2 {
            continue;
        }

        return Some(ContactPinch {
            point,
            tangent,
            components: coarse.min(fine),
            radius,
        });
    }

    None
}

/// 交線に垂直な平面の中の、測る円周。
#[derive(Debug, Clone, Copy)]
struct Ring {
    center: Point3,
    u: Vec3,
    v: Vec3,
    radius: f64,
}

impl Ring {
    fn point_at(&self, angle: f64) -> Point3 {
        self.center + (self.u * angle.cos() + self.v * angle.sin()) * self.radius
    }

    fn scaled(&self, factor: f64) -> Self {
        Self {
            radius: self.radius * factor,
            ..*self
        }
    }
}

/// 円周の上で、答えの材料がいくつの塊に分かれているか。
fn count_components(
    solid_a: &Solid,
    solid_b: &Solid,
    ring: &Ring,
    op: BooleanOpType,
    tol: &Tolerance,
) -> Option<usize> {
    let arcs_a = inside_arcs(solid_a, ring, tol)?;
    let arcs_b = inside_arcs(solid_b, ring, tol)?;

    let result = match op {
        BooleanOpType::Union => union_arcs(&arcs_a, &arcs_b),
        BooleanOpType::Intersection => intersect_arcs(&arcs_a, &arcs_b),
        BooleanOpType::Difference => subtract_arcs(&arcs_a, &arcs_b),
    };

    Some(circular_component_count(&result))
}

/// 円周のうち、立体の内側にある角度の区間。
///
/// 粗い刻みで内外を見て、**切り替わりを挟んだところだけ**を詰めます。区間の
/// 端は、`exact_inside` が「内でも外でもない」と言う位置——つまり境界そのもの
/// ——に落ちます。
fn inside_arcs(solid: &Solid, ring: &Ring, tol: &Tolerance) -> Option<Vec<(f64, f64)>> {
    let step = std::f64::consts::TAU / RING_SAMPLES as f64;
    let at = |angle: f64| ring.point_at(angle);

    let mut angles = Vec::with_capacity(RING_SAMPLES);
    let mut inside = Vec::with_capacity(RING_SAMPLES);
    for index in 0..RING_SAMPLES {
        // 軸にちょうど乗る角度を避けるため、半目盛ずらして始めます。
        let base = (index as f64 + 0.5) * step;
        let mut decided = None;
        let mut used = base;
        for attempt in 0..JITTER_ATTEMPTS {
            // 面にちょうど乗った標本は決まりません。角度を少し振って
            // 取り直します。
            let angle = base + attempt as f64 * step * 0.11;
            if let Some(value) = exact_inside(at(angle), solid, tol) {
                decided = Some(value);
                used = angle;
                break;
            }
        }
        angles.push(used);
        inside.push(decided?);
    }

    if inside.iter().all(|value| *value) {
        return Some(vec![(0.0, std::f64::consts::TAU)]);
    }
    if inside.iter().all(|value| !*value) {
        return Some(Vec::new());
    }

    // 切り替わりの角度を挟み込む。
    let mut transitions: Vec<(f64, bool)> = Vec::new();
    for index in 0..RING_SAMPLES {
        let next = (index + 1) % RING_SAMPLES;
        if inside[index] == inside[next] {
            continue;
        }
        let mut low = angles[index];
        let mut high = angles[next];
        if high < low {
            high += std::f64::consts::TAU;
        }
        let low_inside = inside[index];
        for _ in 0..BISECTION_STEPS {
            let middle = 0.5 * (low + high);
            match exact_inside(at(middle), solid, tol) {
                Some(value) if value == low_inside => low = middle,
                Some(_) => high = middle,
                // 境界の上。**そこが切り替わりです。**
                None => {
                    low = middle;
                    high = middle;
                    break;
                }
            }
        }
        let crossing = 0.5 * (low + high) % std::f64::consts::TAU;
        // `true` なら「ここから内側が始まる」。
        transitions.push((crossing, !low_inside));
    }

    if transitions.is_empty() {
        return None;
    }
    transitions.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut arcs = Vec::new();
    for (index, (angle, starts_inside)) in transitions.iter().enumerate() {
        if !starts_inside {
            continue;
        }
        let (end, _) = transitions[(index + 1) % transitions.len()];
        let mut finish = end;
        if finish <= *angle {
            finish += std::f64::consts::TAU;
        }
        arcs.push((*angle, finish));
    }

    Some(arcs)
}

/// 区間を 0..TAU の並びに正規化する（またぐものは2つに割る）。
fn normalize_arcs(arcs: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let tau = std::f64::consts::TAU;
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (start, end) in arcs {
        let (mut start, mut end) = (*start, *end);
        if end - start >= tau {
            return vec![(0.0, tau)];
        }
        start = start.rem_euclid(tau);
        end = start + (end - start).rem_euclid(tau);
        if end > tau {
            out.push((start, tau));
            out.push((0.0, end - tau));
        } else {
            out.push((start, end));
        }
    }
    out.retain(|(start, end)| end - start > 0.0);
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    out
}

fn union_arcs(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut all = normalize_arcs(a);
    all.extend(normalize_arcs(b));
    all.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (start, end) in all {
        match merged.last_mut() {
            Some(last) if start <= last.1 + ANGULAR_MERGE => last.1 = last.1.max(end),
            _ => merged.push((start, end)),
        }
    }
    merged
}

fn intersect_arcs(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let left = normalize_arcs(a);
    let right = normalize_arcs(b);
    let mut out = Vec::new();
    for (a_start, a_end) in &left {
        for (b_start, b_end) in &right {
            let start = a_start.max(*b_start);
            let end = a_end.min(*b_end);
            if end > start {
                out.push((start, end));
            }
        }
    }
    out.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    out
}

fn subtract_arcs(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let cut = normalize_arcs(b);
    let mut out = Vec::new();
    for (start, end) in normalize_arcs(a) {
        let mut pieces = vec![(start, end)];
        for (cut_start, cut_end) in &cut {
            let mut next = Vec::new();
            for (piece_start, piece_end) in pieces {
                if *cut_end <= piece_start || *cut_start >= piece_end {
                    next.push((piece_start, piece_end));
                    continue;
                }
                if *cut_start > piece_start {
                    next.push((piece_start, *cut_start));
                }
                if *cut_end < piece_end {
                    next.push((*cut_end, piece_end));
                }
            }
            pieces = next;
        }
        out.extend(pieces);
    }
    out.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    out
}

/// 円周の上で、区間がいくつの塊になっているか（0 と TAU は繋がっている）。
fn circular_component_count(arcs: &[(f64, f64)]) -> usize {
    let tau = std::f64::consts::TAU;
    let merged = union_arcs(arcs, &[]);
    if merged.is_empty() {
        return 0;
    }
    let mut count = merged.len();
    // 0 をまたいで繋がっているものは1つに数えます。
    if count >= 2 {
        let first = merged[0];
        let last = merged[merged.len() - 1];
        if first.0 <= ANGULAR_MERGE && last.1 >= tau - ANGULAR_MERGE {
            count -= 1;
        }
    }
    count
}

/// 稜の上の点を、0.0 = 始点、1.0 = 終点で取る。
///
/// `Edge` は曲線のパラメータでしか評価できないので、ここで正規化します
/// （`OrientedEdge` のほうには同じものがあります）。
fn edge_point(edge: &Edge, fraction: f64) -> Point3 {
    let (start, end) = edge.curve.param_range();
    edge.evaluate(start + fraction.clamp(0.0, 1.0) * (end - start))
}

/// 稜の長さ（折れ線の当てはめ）。
fn sampled_length(edge: &Edge) -> f64 {
    let steps = 16;
    let mut length = 0.0;
    let mut previous = edge_point(edge, 0.0);
    for step in 1..=steps {
        let point = edge_point(edge, step as f64 / steps as f64);
        length += (point - previous).norm();
        previous = point;
    }
    length
}

/// 稜の向き。前後の差で取ります。
fn edge_tangent(edge: &Edge, fraction: f64) -> Option<Vec3> {
    let delta = 1e-4;
    let before = edge_point(edge, (fraction - delta).max(0.0));
    let after = edge_point(edge, (fraction + delta).min(1.0));
    let direction = after - before;
    if direction.norm() <= f64::EPSILON {
        return None;
    }
    Some(direction.normalize())
}

/// 与えた向きに垂直な平面の、正規直交な2軸。
fn frame_perpendicular_to(direction: Vec3) -> (Vec3, Vec3) {
    let helper = if direction.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = direction.cross(&helper).normalize();
    let v = direction.cross(&u).normalize();
    (u, v)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TAU: f64 = std::f64::consts::TAU;

    #[test]
    fn a_frame_is_orthonormal_to_its_direction() {
        for direction in [
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 2.0, 3.0).normalize(),
        ] {
            let (u, v) = frame_perpendicular_to(direction);
            assert!(u.dot(&direction).abs() < 1e-12);
            assert!(v.dot(&direction).abs() < 1e-12);
            assert!(u.dot(&v).abs() < 1e-12);
            assert!((u.norm() - 1.0).abs() < 1e-12);
            assert!((v.norm() - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn counting_treats_the_ring_as_a_ring() {
        assert_eq!(circular_component_count(&[]), 0);
        assert_eq!(circular_component_count(&[(0.0, TAU)]), 1);
        assert_eq!(circular_component_count(&[(0.5, 1.0)]), 1);
        assert_eq!(circular_component_count(&[(0.5, 1.0), (2.0, 3.0)]), 2);
        // 0 をまたいで繋がっているものは1つ。
        assert_eq!(circular_component_count(&[(0.0, 1.0), (5.0, TAU)]), 1);
    }

    /// 円柱が壁に内接している配置の断面。**細い帯が残るのが正しい答え**で、
    /// 円周に点を撒いて数えていたときは、これを跨いで見落としていました。
    #[test]
    fn a_tangent_contact_leaves_two_slivers_in_the_difference() {
        // 壁の内側 = 角度 (-90, 90) 度、接している円の内側 = (-89.04, 89.04) 度。
        let half = std::f64::consts::FRAC_PI_2;
        let wall = vec![(-half, half)];
        let circle_half = half - 0.0166_f64;
        let circle = vec![(-circle_half, circle_half)];

        let difference = subtract_arcs(&wall, &circle);
        assert_eq!(circular_component_count(&difference), 2);

        // 同じ配置でも、和と積は繋がったままです。
        assert_eq!(circular_component_count(&union_arcs(&wall, &circle)), 1);
        assert_eq!(circular_component_count(&intersect_arcs(&wall, &circle)), 1);
    }

    #[test]
    fn a_transversal_crossing_leaves_one_piece_whatever_the_operation() {
        // 半空間どうしが横断している断面: どちらも半円。
        let half = std::f64::consts::FRAC_PI_2;
        let a = vec![(-half, half)];
        let b = vec![(0.0, std::f64::consts::PI)];

        assert_eq!(circular_component_count(&union_arcs(&a, &b)), 1);
        assert_eq!(circular_component_count(&intersect_arcs(&a, &b)), 1);
        assert_eq!(circular_component_count(&subtract_arcs(&a, &b)), 1);
    }
}
