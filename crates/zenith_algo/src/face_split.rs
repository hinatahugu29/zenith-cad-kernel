//! 面を、その上に乗った1本の曲線で2枚に割る。
//!
//! # なぜ別に作るか
//!
//! `brep_intersection` の分割は、切り口が**軸まわりの円**であり、面の境界が
//! 「断面2辺 ＋ 側辺2辺」に読めることを前提にしている。回転面を軸に垂直な
//! 平面で切るときはそうなるが、曲面同士が交わるときの交線はパラメータ線では
//! なく、境界のどこにでも着地する。
//!
//! ここは前提を1つに減らす。
//!
//! > 分割線は面の**上**にあり、両端が面の**境界の上**にある。
//!
//! それだけを測って確かめ、あとは境界の巡回を2本に割って、それぞれを分割線で
//! 閉じる。軸も、断面と側辺の区別も、辺の本数も要らない。
//!
//! # 何を確かめるか
//!
//! 割ったあとに**面積を測って足す**。2枚の和が元の面積に戻らなければ、
//! 領域を取り違えている（重複か取りこぼし）。閉じたワイヤになったこと、
//! p-curve が辺に乗っていることだけでは、そこは分からない。
//!
//! **その面積は、2026/08/25 から「パラメータ面積」です**（4-77）。割る前と
//! 割ったあとで見ているのは同じ曲面の同じパラメータ領域なので、パラメータの
//! 上で足し算が合えば領域は合っている。**3D の面積を積む必要はない**——
//! そちらはトリム境界（実測で 4000 点級）を三角形 3万〜16万枚に割って6点則を
//! 当てるので、ブーリアン1回の仕事の 90% がそこに行っていた（4-75）。
//!
//! もう1つ、**面積を囲まない片が出ていないか**を見る。和が合うだけでは
//! 「割れた」と言えない（片方が 0 でも和は合う）。以前は 3D 側が偶然
//! これを捕まえたり逃したりしていた（4-76）。
//!
//! **3D の面積が要るなら、返った片から呼ぶ側が積む。** 閉じた式と突き合わせる
//! プローブとテストはそうしている。判定に使わないものを検査で使うので、同じ量を
//! 両側から見ることになる。

use zenith_geom::{ExtremumEngine, NurbsCurve3};
use zenith_math::{Point3, Tolerance};
use zenith_tess::TessellationParams;
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Vertex, Wire};

use crate::mass_properties::MassCalculator;

/// 割った結果と、その割り方が正しかったかを測った値。
///
/// # 面積は**パラメータ面積**です（2026/08/25、4-76）
///
/// 以前はここに 3D の面積が入っていて、割る前と割ったあとで積み比べて
/// いました。**その積分がブーリアン1回の仕事の 90% でした**（4-75）。
///
/// 確かめたいのは「領域を取り違えていないか」で、割る前と割ったあとは
/// **同じ曲面の同じパラメータ領域**を見ています。パラメータ面積で足し算が
/// 合えば領域は合っており、そちらは曲面を1回も評価しません。実測でも、
/// 揃うところでは 3〜4 桁鋭く出ます（3D は三角形の細かさの誤差を含むため）。
///
/// **3D の面積が要るなら、返った片から呼ぶ側が積んでください**
/// （`MassCalculator::compute_face_integral`）。判定には要りません。
#[derive(Debug, Clone, PartialEq)]
pub struct FaceSplitReport {
    /// 元の面の**パラメータ面積**（外周 − 穴）。
    pub original_parameter_area: f64,
    /// 出来た各片の**パラメータ面積**。
    pub piece_parameter_areas: Vec<f64>,
    /// 片の面積の和が元からどれだけずれたか（相対）。
    pub area_residual: f64,
    /// 分割線が面から離れていた最大距離。
    pub curve_off_surface: f64,
    /// 分割線の端が境界から離れていた距離。
    pub ends_off_boundary: f64,
}

/// 複数の曲線で割った結果と、その割り方が正しかったかを測った値。
///
/// 面積は [`FaceSplitReport`] と同じく**パラメータ面積**です（4-76）。
#[derive(Debug, Clone, PartialEq)]
pub struct MultiSplitReport {
    /// 元の面の**パラメータ面積**。
    pub original_parameter_area: f64,
    /// 出来た各片の**パラメータ面積**。
    pub piece_parameter_areas: Vec<f64>,
    /// 片の面積の和が元からどれだけずれたか（相対）。
    pub area_residual: f64,
    /// 実際に入った切り込みの本数。
    pub cuts_applied: usize,
    /// どの片にも入らなかった本数。
    pub cuts_refused: usize,
    /// 入らなかった理由。診断のために残す。
    pub refusals: Vec<String>,
}

/// 割り方を面積で検算した結果。
struct ParameterAreaCheck {
    original: f64,
    pieces: Vec<f64>,
    residual: f64,
    /// パラメータ面積で測れたか。`false` なら 3D の面積に落ちています。
    ///
    /// **落ちるのは、p-curve が取れない面があるときだけ**です。球のパッチを
    /// 極を通る弧で割った片がそれでした（3-N-1）。安いほうで測れないから
    /// といって断るのは筋が違います——**検算は高いほうでもできます。**
    parametric: bool,
}

/// 面積を囲まない片は、片ではありません。
///
/// 元に対してこの割合より小さい片が出たら、**割れていない**と見なします。
/// 実測で出てくる潰れた片は 1e-14〜1e-17 の桁で、意味のある片
/// （いちばん小さいもので 0.03 程度）とは大きく離れています。
const NULL_PIECE_FRACTION: f64 = 1e-9;

/// 元の面と片を**パラメータ面積**で突き合わせる。
///
/// 3D の面積を積むのと**同じことを確かめて**、曲面を1回も評価しません
/// （[`zenith_tess::face_parameter_area`]）。割る前と割ったあとで同じ曲面の
/// 同じパラメータ領域を見ているので、領域の取り違えはこちらにも同じように
/// 出ます。
///
/// `ZENITH_AREA_CHECK_WHY=1` を付けると、3D 側の残差と1行ずつ並べて出ます。
///
/// どれかの面で p-curve が取れなければ `None`。**そのときは検算できません**
/// ので、呼ぶ側は割り方を断ってください（黙って通すほうが危ない）。
fn parameter_area_check(face: &Face, pieces: &[Face]) -> ParameterAreaCheck {
    match parameter_areas(face, pieces) {
        Some((original, areas)) => ParameterAreaCheck::new(original, areas, true),
        None => {
            // **測れないなら、高いほうで測ります。** 断りません。
            let params = TessellationParams::default();
            let original = MassCalculator::compute_face_integral(face, &params).0;
            let areas = pieces
                .iter()
                .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
                .collect();
            ParameterAreaCheck::new(original, areas, false)
        }
    }
}

/// 元と片のパラメータ面積。1枚でも取れなければ `None`。
fn parameter_areas(face: &Face, pieces: &[Face]) -> Option<(f64, Vec<f64>)> {
    let original = zenith_tess::face_parameter_area(face)?;
    let mut areas = Vec::with_capacity(pieces.len());
    for piece in pieces {
        areas.push(zenith_tess::face_parameter_area(piece)?);
    }
    Some((original, areas))
}

impl ParameterAreaCheck {
    fn new(original: f64, pieces: Vec<f64>, parametric: bool) -> Self {
        let summed: f64 = pieces.iter().sum();
        let residual = if original.abs() > 1e-12 {
            (summed - original).abs() / original.abs()
        } else {
            (summed - original).abs()
        };
        Self {
            original,
            pieces,
            residual,
            parametric,
        }
    }
}

/// 面積を囲まない片が出ていないか。出ていたら、その添字と大きさを言う。
///
/// **この判定は、以前は偶然に効いていました。** 潰れた片はトリムが読めない
/// ので、3D の面積を積むほうが曲面のパラメータ矩形まるごとに落ち、面積が
/// 元の4倍などに膨らんで残差の関門に引っかかっていました。落ちた先が小さい
/// ときは素通りしていました（実測で両方あります。4-76）。いまは大きさを
/// 直接見ます。
fn null_piece(check: &ParameterAreaCheck) -> Option<(usize, f64)> {
    let scale = check.original.abs();
    check
        .pieces
        .iter()
        .enumerate()
        .find(|(_, area)| area.abs() <= scale * NULL_PIECE_FRACTION)
        .map(|(index, area)| (index, *area))
}

/// いま使っているパラメータ側の判定と、**以前の 3D 側**を並べて出す
/// （`ZENITH_AREA_CHECK_WHY=1`）。
///
/// 入れ替えたあとも残してあります。**3D 側は重いので、既定では走りません。**
/// 判定を疑ったとき、両方を1行ずつ並べて見るための口です（4-76 の実測は
/// これで取りました）。
fn compare_area_checks(label: &str, face: &Face, pieces: &[Face], check: &ParameterAreaCheck) {
    if std::env::var_os("ZENITH_AREA_CHECK_WHY").is_none() {
        return;
    }
    let params = TessellationParams::default();
    let original_3d = MassCalculator::compute_face_integral(face, &params).0;
    let pieces_3d: Vec<f64> = pieces
        .iter()
        .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
        .collect();
    let summed_3d: f64 = pieces_3d.iter().sum();
    let residual_3d = if original_3d.abs() > 1e-12 {
        (summed_3d - original_3d).abs() / original_3d.abs()
    } else {
        (summed_3d - original_3d).abs()
    };

    let null = null_piece(check);
    let verdict = if check.residual > 1e-6 || null.is_some() {
        "REFUSE"
    } else {
        "pass"
    };
    eprintln!(
        "AREACHECK {label:<22} pieces {:>2}  {} {:>10.3e} {:<6} {:<10}  3d {:>10.3e} {}",
        pieces.len(),
        // 落ちていたら、左の数字も 3D です（p-curve が取れない面があった）。
        if check.parametric { "uv" } else { "3d*" },
        check.residual,
        verdict,
        match null {
            Some((index, _)) => format!("null piece {index}"),
            None => String::new(),
        },
        residual_3d,
        if (residual_3d > 1e-6) == (verdict == "REFUSE") {
            "3d agrees"
        } else {
            "**3d differs**"
        }
    );
}

/// 面の上の曲線で面を割る。
pub struct FaceSplitter;

impl FaceSplitter {
    /// `split` で `face` を2枚に割る。
    ///
    /// `split` は面の上に乗り、両端が `face` の外周ワイヤの上になければ
    /// ならない。内周（穴）を持つ面は、まだ扱わない。
    pub fn split_by_curve(
        face: &Face,
        split: &Edge,
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, FaceSplitReport), String> {
        Self::split_by_chain(face, std::slice::from_ref(split), tol)
    }

    /// 面の**内側で閉じたループ**で `face` を2枚に割る。
    ///
    /// 境界から境界へ届く切り込みとは別の形です。球の八分片を、角に置いた箱の
    /// 3面が切ると、交線は3本の弧になり、**3本で球面上の閉じたループ**を
    /// 作ります。どの弧も八分片の境界に着かないので、`split_by_chain` は
    /// 「境界に届かない」と断ります。実測では、断られた結果 A の面は1枚も
    /// 割られず、B 側の平面パッチが持つ弧に相手がいなくなっていました
    /// （`unmatched_edge_probe`）。
    ///
    /// 割り方は素直です。**曲面はそのままで、境界だけを付け替えます。**
    ///
    /// - 内側の片: ループを**外周**にした面。
    /// - 外側の片: 元の外周のまま、ループを**穴**として足した面。
    ///
    /// 面積の和が元に戻ることを測って返します。戻らなければ、ループが本当に
    /// 面の内側にあったのかを疑ってください。
    pub fn split_by_interior_loop(
        face: &Face,
        loop_edges: &[Edge],
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, FaceSplitReport), String> {
        if loop_edges.is_empty() {
            return Err("an interior cut needs at least one curve".to_string());
        }
        if !face.inner_wires.is_empty() {
            return Err("splitting a face that already has holes is not implemented".to_string());
        }

        let ordered = order_closed_loop(loop_edges, tol)?;

        // ループが本当にこの面の上にあるか。構成に使っていない位置で測る。
        let scale = boundary_extent(&face.outer_wire).max(1.0);
        let limit = tol.linear * 10.0 * scale;
        let mut curve_off_surface: f64 = 0.0;
        for piece in &ordered {
            curve_off_surface =
                curve_off_surface.max(Self::distance_to_surface(face, &piece.edge.curve, 23)?);
        }
        if curve_off_surface > limit {
            return Err(format!(
                "the interior loop leaves the face by {curve_off_surface:.3e}, over {limit:.3e}"
            ));
        }

        // **巻き方を、面の向きと突き合わせます。**
        //
        // ループの並びは辿った順でしかなく、面がどちら向きかを見ていません。
        // 逆に巻いた片を返すと、形も面積も合ったまま、**共有する稜を両側が
        // 同じ向きに辿ります**。閉じた多様体ではそれは起こりえないので、
        // 縫合が落ちます。実測: 円錐の角を箱で削る積・和で、縫えない稜 0・
        // 非多様体 0 のまま**同方向の稜が 6 本**出ていました。
        //
        // 正規化の側には 4-46 で同じ検査を入れてあります。こちらに入れて
        // いなかったので、球では偶然合っていて円錐で合いませんでした。
        let ordered = if interior_loop_winds_with(face, &ordered, tol) {
            ordered
        } else {
            ordered
                .iter()
                .rev()
                .map(|oriented| {
                    OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
                })
                .collect()
        };

        let loop_wire = Wire::new(ordered.clone());
        // 内側の片は、ループをそのまま外周にする。
        let inside = Face::new(
            face.geometry.clone(),
            loop_wire.clone(),
            Vec::new(),
            face.orientation,
            face.tolerance,
        );
        // 外側の片は、元の外周のまま、ループを穴として持つ。**穴のワイヤは
        // 外周と逆に巻きます。**
        let hole = Wire::new(
            ordered
                .iter()
                .rev()
                .map(|oriented| {
                    OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
                })
                .collect(),
        );
        let outside = Face::new(
            face.geometry.clone(),
            face.outer_wire.clone(),
            vec![hole],
            face.orientation,
            face.tolerance,
        );

        let pieces = vec![inside, outside];
        let check = Self::checked_areas("split_by_interior_loop", face, &pieces)?;

        Ok((
            pieces,
            FaceSplitReport {
                original_parameter_area: check.original,
                piece_parameter_areas: check.pieces,
                area_residual: check.residual,
                curve_off_surface,
                // 内側のループは境界に着きません。着いていたら、それは
                // 境界から境界への切り込みで、こちらの口ではありません。
                ends_off_boundary: 0.0,
            },
        ))
    }

    /// 端で繋がった何本かの曲線を1本の切り込みとして `face` を2枚に割る。
    ///
    /// 曲面同士の交線は、相手のパッチの境界で細切れになって届く。円柱を円柱で
    /// 貫くと、片方の四半パッチに入る切り込みは2本に分かれ、**どちらも面の
    /// 内側で終わる**。1本ずつ当てると両方とも「境界に着かない」と断られる。
    /// 繋いで初めて、境界から境界へ届く1本の切り込みになる。
    pub fn split_by_chain(
        face: &Face,
        chain: &[Edge],
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, FaceSplitReport), String> {
        if chain.is_empty() {
            return Err("a cut needs at least one curve".to_string());
        }
        if chain.len() == 1 {
            return Self::split_one(face, &chain[0], tol);
        }

        let ordered = order_chain(chain, tol)?;
        Self::split_with_ordered_cut(face, &ordered, tol)
    }

    fn split_one(
        face: &Face,
        split: &Edge,
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, FaceSplitReport), String> {
        let start = split.start_vertex.point;
        let end = split.end_vertex.point;
        let forward = orient_between(split, start, end, tol)
            .ok_or_else(|| "the splitting edge does not run between its own ends".to_string())?;
        Self::split_with_ordered_cut(face, &[forward], tol)
    }

    fn split_with_ordered_cut(
        face: &Face,
        cut: &[OrientedEdge],
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, FaceSplitReport), String> {
        if !face.inner_wires.is_empty() {
            return Err("splitting a face that has holes is not implemented".to_string());
        }
        let edges = &face.outer_wire.edges;
        if edges.len() < 2 {
            return Err("a face boundary needs at least two edges to be split".to_string());
        }

        let scale = boundary_extent(&face.outer_wire).max(1.0);
        let limit = tol.linear * 10.0 * scale;

        // 1. 切り込みが本当にこの面の上にあるか。構成に使っていない位置で測る。
        let mut curve_off_surface: f64 = 0.0;
        for piece in cut {
            curve_off_surface =
                curve_off_surface.max(Self::distance_to_surface(face, &piece.edge.curve, 23)?);
        }
        if curve_off_surface > limit {
            return Err(format!(
                "the splitting curve leaves the face by {curve_off_surface:.3e}, over {limit:.3e}"
            ));
        }

        // 2. 切り込みの両端が境界のどこに乗るか。乗っていなければ割れない。
        let start = oriented_start(&cut[0]);
        let end = oriented_end(&cut[cut.len() - 1]);
        let (from, from_distance) = locate_on_wire(&face.outer_wire, start, tol)
            .ok_or_else(|| "the splitting curve does not start on the boundary".to_string())?;
        let (to, to_distance) = locate_on_wire(&face.outer_wire, end, tol)
            .ok_or_else(|| "the splitting curve does not end on the boundary".to_string())?;
        let ends_off_boundary = from_distance.max(to_distance);
        if ends_off_boundary > limit {
            return Err(format!(
                "the splitting curve ends {ends_off_boundary:.3e} away from the boundary"
            ));
        }

        let count = edges.len() as f64;
        let separation = ((to - from).rem_euclid(count)).min((from - to).rem_euclid(count));
        if separation <= 1e-9 {
            return Err("both ends of the splitting curve land at the same place".to_string());
        }

        // 3. 巡回を2本に割り、それぞれを切り込みで閉じる。
        let forward = walk(edges, from, to, tol)?;
        let backward = walk(edges, to, from, tol)?;

        // **切り込みの端を、境界の着地点にぴったり合わせます**（2026/08/25）。
        //
        // 上の 2 で、端が境界から `limit`（大きさに比例。20 幅の面で 2e-4）
        // 以内にあることは確かめました。ところが組み上げたワイヤの閉性は
        // `tol.linear`（1e-6）で見るので、**着地は認めたのに閉じていないと
        // 断る**、という食い違いが残っていました。
        //
        // 実測（球を平面で切る。3-N-1）: 弧の端が極から **1.669e-5** ずれ、
        // 「piece 0 came out with an open wire」で断られていました。極では
        // パラメータが潰れるので、辿った交線の端はそこへぴったりは着きません。
        //
        // 着地点は境界の上の点で、**真の交線もそこを通ります**。合わせるのは
        // 近似を真値へ寄せる向きです。ずれが `limit` を超えていたら、上の 2 が
        // 既に断っています。
        let start_target = forward
            .first()
            .map(|oriented| oriented.start_vertex().point)
            .ok_or_else(|| "the boundary walk came out empty".to_string())?;
        let end_target = forward
            .last()
            .map(|oriented| oriented.end_vertex().point)
            .ok_or_else(|| "the boundary walk came out empty".to_string())?;
        let cut = &snapped_onto_boundary(cut, start_target, end_target, limit)?;

        let cut_forward: Vec<OrientedEdge> = cut.to_vec();
        let cut_backward: Vec<OrientedEdge> = cut
            .iter()
            .rev()
            .map(|oriented| {
                OrientedEdge::new(oriented.edge.clone(), oriented.orientation.reversed())
            })
            .collect();

        let mut first = forward;
        first.extend(cut_backward);
        let mut second = backward;
        second.extend(cut_forward);

        let pieces: Vec<Face> = [first, second]
            .into_iter()
            .map(|wire_edges| {
                Face::new(
                    face.geometry.clone(),
                    Wire::new(wire_edges),
                    Vec::new(),
                    face.orientation,
                    face.tolerance,
                )
            })
            .collect();

        for (index, piece) in pieces.iter().enumerate() {
            if piece.outer_wire.edges.len() < 2 {
                return Err(format!("piece {index} came out with too few edges"));
            }
            if !piece.outer_wire.is_closed(tol) {
                // どこで開いているかを言う。開いたという事実だけでは、
                // 巡回の割り方と切り込みのどちらが悪いのか分からない。
                let edges = &piece.outer_wire.edges;
                let mut gaps = Vec::new();
                for position in 0..edges.len() {
                    let here = edges[position].end_vertex().point;
                    let next = edges[(position + 1) % edges.len()].start_vertex().point;
                    let gap = (here - next).norm();
                    if gap > tol.linear {
                        gaps.push(format!(
                            "after edge {position} of {}: {:.3e} between ({:.4},{:.4},{:.4}) and ({:.4},{:.4},{:.4})",
                            edges.len(), gap, here.x, here.y, here.z, next.x, next.y, next.z
                        ));
                    }
                }
                return Err(format!(
                    "piece {index} came out with an open wire; {}",
                    gaps.join("; ")
                ));
            }
        }

        // 4. 面積を測って足す。ここが合わなければ領域を取り違えている。
        let check = Self::checked_areas("split_by_chain", face, &pieces)?;

        Ok((
            pieces,
            FaceSplitReport {
                original_parameter_area: check.original,
                piece_parameter_areas: check.pieces,
                area_residual: check.residual,
                curve_off_surface,
                ends_off_boundary,
            },
        ))
    }

    /// 割った結果を**パラメータ面積**で検算する。
    ///
    /// 検算できない（p-curve が取れない）ときと、**面積を囲まない片が出た**
    /// ときは、割れなかったこととして断ります。もっともらしい割り方を返すより、
    /// 割れなかったと言うほうが良い、というのがこのモジュールの方針です。
    fn checked_areas(
        label: &str,
        face: &Face,
        pieces: &[Face],
    ) -> Result<ParameterAreaCheck, String> {
        let check = parameter_area_check(face, pieces);
        compare_area_checks(label, face, pieces, &check);
        if let Some((index, area)) = null_piece(&check) {
            return Err(format!(
                "{label}: piece {index} encloses no area ({area:.3e} of {:.3e}), so this is not a split",
                check.original
            ));
        }
        Ok(check)
    }

    /// 複数の曲線で1枚の面を割る。
    ///
    /// 曲線を1本ずつ当て、そのときどきの片のうち**受け取れるもの**に入れる。
    /// どの片も受け取らない曲線は、理由を添えて数えるだけで捨てる。
    /// もっともらしい割り方を作るより、割れなかったと言うほうが良い。
    ///
    /// 交線どうしが**面の内側で交わる**場合は、まだ扱えない。先に交点で
    /// 互いを刻む段（imprint）が要る。ここは互いに交わらない切り込みだけを
    /// 想定している。
    pub fn split_by_curves(
        face: &Face,
        splits: &[Edge],
        tol: &Tolerance,
    ) -> Result<(Vec<Face>, MultiSplitReport), String> {

        let mut pieces = vec![face.clone()];
        let mut applied = 0usize;
        let mut refusals = Vec::new();

        for split in splits {
            let mut done = false;
            for index in 0..pieces.len() {
                match Self::split_by_curve(&pieces[index], split, tol) {
                    Ok((two, report)) => {
                        if report.area_residual > 1e-6 {
                            refusals.push(format!(
                                "a cut lost {:.3e} of the piece it was applied to",
                                report.area_residual
                            ));
                            continue;
                        }
                        pieces.remove(index);
                        pieces.extend(two);
                        applied += 1;
                        done = true;
                        break;
                    }
                    Err(reason) => {
                        // どの片にも当たらなかったときだけ理由を残す。
                        if index + 1 == pieces.len() {
                            refusals.push(reason);
                        }
                    }
                }
            }
            if !done && refusals.is_empty() {
                refusals.push("a cut landed on no piece".to_string());
            }
        }

        // ここでは潰れた片を断りません。1本も当たらなければ片は元の面1枚の
        // ままで、それは「割れなかった」であって欠陥ではないからです。
        let check = parameter_area_check(face, &pieces);
        compare_area_checks("split_by_curves", face, &pieces, &check);

        Ok((
            pieces,
            MultiSplitReport {
                original_parameter_area: check.original,
                piece_parameter_areas: check.pieces,
                area_residual: check.residual,
                cuts_applied: applied,
                cuts_refused: splits.len() - applied,
                refusals,
            },
        ))
    }

    /// 曲線が面からどれだけ離れているか。標本の数は面の作りと互いに素にする。
    fn distance_to_surface(
        face: &Face,
        curve: &NurbsCurve3,
        samples: usize,
    ) -> Result<f64, String> {
        let (t0, t1) = curve.param_range();
        let mut worst: f64 = 0.0;
        for step in 0..=samples {
            let point = curve.evaluate(t0 + (t1 - t0) * step as f64 / samples as f64);
            let distance = match &face.geometry {
                FaceGeometry::Plane(plane) => {
                    let normal = plane.normal.normalize();
                    (point - plane.origin).dot(&normal).abs()
                }
                FaceGeometry::Nurbs(surface) => {
                    ExtremumEngine::point_to_surface(point, surface, 64, 1e-13)
                        .map_err(|err| format!("could not project onto the face: {err}"))?
                        .distance
                }
                _ => return Err("this face geometry cannot be split yet".to_string()),
            };
            worst = worst.max(distance);
        }
        Ok(worst)
    }
}

fn oriented_start(oriented: &OrientedEdge) -> Point3 {
    if oriented.orientation.is_forward() {
        oriented.edge.start_vertex.point
    } else {
        oriented.edge.end_vertex.point
    }
}

fn oriented_end(oriented: &OrientedEdge) -> Point3 {
    if oriented.orientation.is_forward() {
        oriented.edge.end_vertex.point
    } else {
        oriented.edge.start_vertex.point
    }
}

/// 端で繋がった辺の集まりを、1本の道として並べ替える。
///
/// 端点が1度しか現れない辺が両端になる。輪になっている（すべての端点が2度
/// 現れる）集まりは、境界に着地しないので断る。
/// 内側のループが、面の向きと同じ側を囲んでいるか。
///
/// 媒介変数空間での符号付き面積で見ます。`Forward` の面なら正、`Reversed` の
/// 面なら負が正しい向きです（シェルの検証がそう見ています）。
///
/// p-curve が出せない面では判定できないので、そのときは並べ替えません。
/// 触らないほうが、当てずっぽうで裏返すよりましです。
fn interior_loop_winds_with(face: &Face, ordered: &[OrientedEdge], tol: &Tolerance) -> bool {
    let candidate = Face::new(
        face.geometry.clone(),
        Wire::new(ordered.to_vec()),
        Vec::new(),
        face.orientation,
        face.tolerance,
    );
    let Ok(pcurves) = candidate.pcurves(tol) else {
        return true;
    };

    let mut area = 0.0;
    let mut previous: Option<zenith_math::Point2> = None;
    let mut first: Option<zenith_math::Point2> = None;
    for segment in &pcurves.outer_loop.segments {
        let (t0, t1) = segment.curve.param_range();
        const SAMPLES: usize = 8;
        for step in 0..=SAMPLES {
            let point = segment
                .curve
                .evaluate(t0 + (t1 - t0) * step as f64 / SAMPLES as f64);
            if first.is_none() {
                first = Some(point);
            }
            if let Some(last) = previous {
                area += last.x * point.y - point.x * last.y;
            }
            previous = Some(point);
        }
    }
    if let (Some(last), Some(start)) = (previous, first) {
        area += last.x * start.y - start.x * last.y;
    }
    let area = area * 0.5;
    let oriented = if face.orientation.is_forward() {
        area
    } else {
        -area
    };
    oriented > tol.parametric
}

/// 閉じたループになる稜の並びを、端から端へ辿って揃える。
///
/// `order_chain` は「1度しか出ない端点が2つ」を道の端として使いますが、
/// 閉じたループでは**そういう点はありません**。どの端点もちょうど2回出ます。
/// そこが違うだけで、辿り方は同じです。
fn order_closed_loop(loop_edges: &[Edge], tol: &Tolerance) -> Result<Vec<OrientedEdge>, String> {
    let limit = tol.linear.max(1e-9) * 10.0;
    let same = |a: Point3, b: Point3| (a - b).norm() <= limit;

    // 閉じているなら、どの端点もちょうど2回出ます。1回や3回があれば
    // 閉じたループではありません。
    let mut endpoints: Vec<(Point3, usize)> = Vec::new();
    for edge in loop_edges {
        for point in [edge.start_vertex.point, edge.end_vertex.point] {
            match endpoints.iter_mut().find(|(known, _)| same(*known, point)) {
                Some((_, count)) => *count += 1,
                None => endpoints.push((point, 1)),
            }
        }
    }
    if let Some((point, count)) = endpoints.iter().find(|(_, count)| *count != 2) {
        return Err(format!(
            "the cut is not a closed loop: ({:.4} {:.4} {:.4}) is used {count} time(s), not 2",
            point.x, point.y, point.z
        ));
    }

    let mut used = vec![false; loop_edges.len()];
    let mut ordered: Vec<OrientedEdge> = Vec::with_capacity(loop_edges.len());
    // どこから始めても閉じたループは同じものになります。最初の稜の向きを
    // そのまま採ります。
    used[0] = true;
    ordered.push(OrientedEdge::forward(loop_edges[0].clone()));
    let start = loop_edges[0].start_vertex.point;
    let mut here = loop_edges[0].end_vertex.point;

    while ordered.len() < loop_edges.len() {
        let mut advanced = false;
        for (index, edge) in loop_edges.iter().enumerate() {
            if used[index] {
                continue;
            }
            if same(edge.start_vertex.point, here) {
                used[index] = true;
                here = edge.end_vertex.point;
                ordered.push(OrientedEdge::forward(edge.clone()));
                advanced = true;
                break;
            }
            if same(edge.end_vertex.point, here) {
                used[index] = true;
                here = edge.start_vertex.point;
                ordered.push(OrientedEdge::reversed(edge.clone()));
                advanced = true;
                break;
            }
        }
        if !advanced {
            return Err("the cut does not join up into one loop".to_string());
        }
    }

    if !same(here, start) {
        return Err("the cut does not come back to where it started".to_string());
    }
    Ok(ordered)
}

fn order_chain(chain: &[Edge], tol: &Tolerance) -> Result<Vec<OrientedEdge>, String> {
    let limit = tol.linear.max(1e-9) * 10.0;
    let same = |a: Point3, b: Point3| (a - b).norm() <= limit;

    // 端点の出現回数を数え、1度しか出ない点を道の端とする。
    let mut endpoints: Vec<(Point3, usize)> = Vec::new();
    for edge in chain {
        for point in [edge.start_vertex.point, edge.end_vertex.point] {
            match endpoints.iter_mut().find(|(known, _)| same(*known, point)) {
                Some((_, count)) => *count += 1,
                None => endpoints.push((point, 1)),
            }
        }
    }
    let ends: Vec<Point3> = endpoints
        .iter()
        .filter(|(_, count)| *count == 1)
        .map(|(point, _)| *point)
        .collect();
    if ends.len() != 2 {
        return Err(format!(
            "a cut made of {} curves has {} loose ends, not two",
            chain.len(),
            ends.len()
        ));
    }

    let mut remaining: Vec<Edge> = chain.to_vec();
    let mut ordered: Vec<OrientedEdge> = Vec::with_capacity(chain.len());
    let mut cursor = ends[0];
    while !remaining.is_empty() {
        let found = remaining.iter().position(|edge| {
            same(edge.start_vertex.point, cursor) || same(edge.end_vertex.point, cursor)
        });
        let Some(index) = found else {
            return Err("the curves of a cut do not join end to end".to_string());
        };
        let edge = remaining.remove(index);
        if same(edge.start_vertex.point, cursor) {
            cursor = edge.end_vertex.point;
            ordered.push(OrientedEdge::forward(edge));
        } else {
            cursor = edge.start_vertex.point;
            ordered.push(OrientedEdge::reversed(edge));
        }
    }

    if !same(cursor, ends[1]) {
        return Err("a cut did not reach its own far end".to_string());
    }
    Ok(ordered)
}

/// 外周ワイヤの広がり。公差を形の大きさに合わせるために使う。
fn boundary_extent(wire: &Wire) -> f64 {
    let Some(first) = wire.edges.first() else {
        return 1.0;
    };
    let origin = first.start_vertex().point;
    wire.edges.iter().fold(0.0f64, |worst, oriented| {
        worst
            .max((oriented.start_vertex().point - origin).norm())
            .max((oriented.end_vertex().point - origin).norm())
    })
}

/// 切り込みの両端を、境界の上の着地点にぴったり合わせた写しを返す。
///
/// クランプされた B-spline は、**端の制御点がそのまま端点**です。そこを
/// 差し替えれば、曲線の途中は動かさずに端だけを合わせられます。頂点も
/// 同じ点に置き直します。
///
/// 動かす距離が `limit` を超えていたら、それは着地していないということなので
/// 断ります（呼ぶ側が先に測っていますが、ここでも念のため見ます）。
fn snapped_onto_boundary(
    cut: &[OrientedEdge],
    start_target: Point3,
    end_target: Point3,
    limit: f64,
) -> Result<Vec<OrientedEdge>, String> {
    let mut out = cut.to_vec();
    let last = out.len() - 1;
    move_traversal_end(&mut out[0], true, start_target, limit)?;
    move_traversal_end(&mut out[last], false, end_target, limit)?;
    Ok(out)
}

/// 辿る向きで見た端（始点側 or 終点側）を、指定の点へ動かす。
fn move_traversal_end(
    oriented: &mut OrientedEdge,
    at_traversal_start: bool,
    target: Point3,
    limit: f64,
) -> Result<(), String> {
    // 辿る向きの「始め」は、順向きなら曲線の始点、逆向きなら曲線の終点。
    let move_curve_start = oriented.orientation.is_forward() == at_traversal_start;

    let control_points = &mut oriented.edge.curve.control_points;
    let index = if move_curve_start {
        0
    } else {
        control_points.len() - 1
    };
    let shift = (target - control_points[index].point).norm();
    if shift <= f64::EPSILON {
        return Ok(());
    }
    if shift > limit {
        return Err(format!(
            "the splitting curve would have to move {shift:.3e} to meet the boundary, over {limit:.3e}"
        ));
    }
    control_points[index].point = target;

    // 頂点も置き直す。ここを忘れると、稜の実体と端点が食い違ったまま残る。
    if move_curve_start {
        oriented.edge.start_vertex.point = target;
    } else {
        oriented.edge.end_vertex.point = target;
    }
    Ok(())
}

/// 点が巡回のどこに乗るかを、`辺の番号 + 辺内の割合` で返す。
///
/// 割合は曲線の媒介変数で測る。弧長ではないが、同じ物差しで一貫していれば
/// 巡回を割るには足りる。
fn locate_on_wire(wire: &Wire, point: Point3, tol: &Tolerance) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for (index, oriented) in wire.edges.iter().enumerate() {
        let projection =
            ExtremumEngine::point_to_curve(point, &oriented.edge.curve, 128, 1e-13).ok()?;
        let (t0, t1) = oriented.edge.curve.param_range();
        if (t1 - t0).abs() <= f64::EPSILON {
            continue;
        }
        let raw = ((projection.parameter - t0) / (t1 - t0)).clamp(0.0, 1.0);
        let fraction = if oriented.orientation.is_forward() {
            raw
        } else {
            1.0 - raw
        };
        let distance = projection.distance;
        if best.as_ref().map(|(_, d)| distance < *d).unwrap_or(true) {
            best = Some((index as f64 + fraction, distance));
        }
    }
    let _ = tol;
    best
}

/// 巡回座標 `from` から `to` まで、巡回の向きに辿った辺の並び。
fn walk(
    edges: &[OrientedEdge],
    from: f64,
    to: f64,
    tol: &Tolerance,
) -> Result<Vec<OrientedEdge>, String> {
    let count = edges.len();
    let total = count as f64;
    let mut end = to;
    if end <= from + 1e-12 {
        end += total;
    }

    let mut out = Vec::new();
    let mut cursor = from;
    let mut guard = 0;
    while cursor < end - 1e-12 {
        guard += 1;
        if guard > count * 3 + 4 {
            return Err("walking the boundary did not terminate".to_string());
        }
        let base = cursor.floor();
        let index = (base as usize) % count;
        let next = (base + 1.0).min(end);
        let low = cursor - base;
        let high = next - base;
        if high - low > 1e-12 {
            out.push(sub_edge(&edges[index], low, high, tol)?);
        }
        cursor = next;
    }

    if out.is_empty() {
        return Err("walking the boundary produced nothing".to_string());
    }
    Ok(out)
}

/// 辺の、辿る向きで `low` から `high` までの部分。割合は 0..1。
fn sub_edge(
    oriented: &OrientedEdge,
    low: f64,
    high: f64,
    tol: &Tolerance,
) -> Result<OrientedEdge, String> {
    let whole = low <= 1e-12 && high >= 1.0 - 1e-12;
    if whole {
        return Ok(oriented.clone());
    }

    let (t0, t1) = oriented.edge.curve.param_range();
    let span = t1 - t0;
    // 辿る向きの割合を、曲線そのものの媒介変数に直す。
    let (a, b) = if oriented.orientation.is_forward() {
        (t0 + span * low, t0 + span * high)
    } else {
        (t0 + span * (1.0 - high), t0 + span * (1.0 - low))
    };

    let piece = subcurve(&oriented.edge.curve, a, b)
        .ok_or_else(|| format!("could not take the curve between {a} and {b}"))?;
    let (p0, p1) = piece.param_range();
    let start_point = piece.evaluate(p0);
    let end_point = piece.evaluate(p1);

    // 端が元の頂点と同じなら、その頂点をそのまま使う。新しく作ると、隣の面が
    // 使っている頂点と別物になる。
    let reuse = |point: Point3| -> Vertex {
        for candidate in [&oriented.edge.start_vertex, &oriented.edge.end_vertex] {
            if (candidate.point - point).norm() <= tol.linear {
                return candidate.clone();
            }
        }
        Vertex::new(point, tol.linear)
    };

    let edge = Edge::new(
        piece,
        reuse(start_point),
        reuse(end_point),
        oriented.edge.tolerance,
    );
    Ok(OrientedEdge::new(edge, oriented.orientation))
}

/// 曲線の `a` から `b` までを取り出す。`a < b`。
fn subcurve(curve: &NurbsCurve3, a: f64, b: f64) -> Option<NurbsCurve3> {
    let (t0, t1) = curve.param_range();
    let span = (t1 - t0).abs().max(1.0);
    let mut piece = curve.clone();
    if a > t0 + span * 1e-12 {
        piece = piece.split_at(a)?.1;
    }
    if b < t1 - span * 1e-12 {
        piece = piece.split_at(b)?.0;
    }
    Some(piece)
}

/// `start` から `end` へ向くように辺の向きを決める。
fn orient_between(
    edge: &Edge,
    start: Point3,
    end: Point3,
    tol: &Tolerance,
) -> Option<OrientedEdge> {
    let limit = tol.linear.max(1e-9) * 10.0;
    if (edge.start_vertex.point - start).norm() <= limit
        && (edge.end_vertex.point - end).norm() <= limit
    {
        return Some(OrientedEdge::forward(edge.clone()));
    }
    if (edge.start_vertex.point - end).norm() <= limit
        && (edge.end_vertex.point - start).norm() <= limit
    {
        return Some(OrientedEdge::reversed(edge.clone()));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::FaceSplitter;
    use crate::mass_properties::MassCalculator;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};
    use zenith_tess::TessellationParams;

    /// 片の **3D の面積**。
    ///
    /// 割り方の判定はパラメータ面積でやります（4-76）が、閉じた式と
    /// 突き合わせるのは 3D の面積です。**判定に使わないものを、検査では
    /// 使います**——同じ量を両側から見ることになるので、そのほうが強い。
    fn areas_3d(pieces: &[Face]) -> Vec<f64> {
        let params = TessellationParams::default();
        pieces
            .iter()
            .map(|piece| MassCalculator::compute_face_integral(piece, &params).0)
            .collect()
    }

    use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
    use zenith_math::{Point3, Tolerance, Vec3};
    use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Orientation, Vertex, Wire};

    /// 半径 `r`、高さ `0..h` の円柱側面の四半パッチ。
    fn cylinder_quarter(r: f64, h: f64) -> Face {
        let w = FRAC_1_SQRT_2;
        let grid: Vec<Vec<ControlPoint3>> = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)]
            .iter()
            .map(|(x, y, weight)| {
                vec![
                    ControlPoint3::new(Point3::new(*x, *y, 0.0), *weight),
                    ControlPoint3::new(Point3::new(*x, *y, h), *weight),
                ]
            })
            .collect();
        let surface = NurbsSurface3::new(
            2,
            1,
            grid,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(2, 1),
        )
        .unwrap();
        let arc = |z: f64| {
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(Point3::new(r, 0.0, z)),
                    ControlPoint3::new(Point3::new(r, r, z), w),
                    ControlPoint3::unweighted(Point3::new(0.0, r, z)),
                ],
                KnotVector::clamped_uniform(3, 2),
            )
            .unwrap()
        };
        let bottom_start = Vertex::from_point(Point3::new(r, 0.0, 0.0));
        let bottom_end = Vertex::from_point(Point3::new(0.0, r, 0.0));
        let top_start = Vertex::from_point(Point3::new(r, 0.0, h));
        let top_end = Vertex::from_point(Point3::new(0.0, r, h));
        Face::new(
            FaceGeometry::Nurbs(surface),
            Wire::new(vec![
                OrientedEdge::forward(Edge::new(
                    arc(0.0),
                    bottom_start.clone(),
                    bottom_end.clone(),
                    1e-6,
                )),
                OrientedEdge::forward(
                    Edge::line_between(bottom_end.clone(), top_end.clone()).unwrap(),
                ),
                OrientedEdge::reversed(Edge::new(arc(h), top_start.clone(), top_end.clone(), 1e-6)),
                OrientedEdge::reversed(
                    Edge::line_between(bottom_start.clone(), top_start.clone()).unwrap(),
                ),
            ]),
            Vec::new(),
            Orientation::Forward,
            1e-6,
        )
    }

    /// 傾いた平面が円柱を切ってできる楕円弧。
    ///
    /// 楕円は円のアフィン像なので、円弧の制御点に同じ写像をかければ**厳密に**
    /// 表せる。折れ線で近づけると、確かめたいものが測れなくなる。
    fn tilted_section(r: f64, z0: f64, slope: f64) -> Edge {
        let w = FRAC_1_SQRT_2;
        let lift = |x: f64, y: f64| Point3::new(x, y, z0 + slope * x);
        let curve = NurbsCurve3::new(
            2,
            vec![
                ControlPoint3::unweighted(lift(r, 0.0)),
                ControlPoint3::new(lift(r, r), w),
                ControlPoint3::unweighted(lift(0.0, r)),
            ],
            KnotVector::clamped_uniform(3, 2),
        )
        .unwrap();
        let (t0, t1) = curve.param_range();
        Edge::new(
            curve.clone(),
            Vertex::from_point(curve.evaluate(t0)),
            Vertex::from_point(curve.evaluate(t1)),
            1e-6,
        )
    }

    fn planar_square(side: f64) -> Face {
        let plane = PlaneSurface3::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(side, 0.0, 0.0),
            Point3::new(side, side, 0.0),
            Point3::new(0.0, side, 0.0),
        ];
        let vertices: Vec<Vertex> = corners.into_iter().map(Vertex::from_point).collect();
        Face::new(
            FaceGeometry::Plane(plane),
            Wire::new(
                (0..4)
                    .map(|index| {
                        OrientedEdge::forward(
                            Edge::line_between(
                                vertices[index].clone(),
                                vertices[(index + 1) % 4].clone(),
                            )
                            .unwrap(),
                        )
                    })
                    .collect(),
            ),
            Vec::new(),
            Orientation::Forward,
            1e-6,
        )
    }

    /// パラメータ線でない曲線で四辺形パッチを割れること。
    ///
    /// **これが曲面同士の交差の本当の壁である。** 既存の分割は「切り口は軸
    /// まわりの円」「境界は断面2辺と側辺2辺」を前提にしており、そこを外れると
    /// 割れない。
    #[test]
    fn a_quadrilateral_patch_splits_along_a_curve_that_is_not_a_parameter_line() {
        let tol = Tolerance::default();
        let radius = 10.0;
        // 下側の面積は円柱を開いて積める: r * ∫(z0 + slope r cos t) dt, t = 0..pi/2
        for (z0, slope) in [(20.0, 0.6), (25.0, -0.9), (12.0, 0.4)] {
            let face = cylinder_quarter(radius, 40.0);
            let split = tilted_section(radius, z0, slope);
            let (pieces, report) = FaceSplitter::split_by_curve(&face, &split, &tol)
                .unwrap_or_else(|err| panic!("z0 {z0} slope {slope}: {err}"));

            assert_eq!(pieces.len(), 2);
            assert!(
                report.curve_off_surface < 1e-9,
                "the split curve was not on the face: {:.3e}",
                report.curve_off_surface
            );
            // 面積の和が元に戻ること。閉じたワイヤになっただけでは、領域の
            // 重複や取りこぼしは分からない。
            assert!(
                report.area_residual < 1e-9,
                "z0 {z0} slope {slope}: the pieces do not add up, residual {:.3e}",
                report.area_residual
            );

            let lower = radius * (z0 * FRAC_PI_2 + slope * radius);
            let piece_areas = areas_3d(&pieces);
            let best = piece_areas
                .iter()
                .map(|area| (area - lower).abs() / lower)
                .fold(f64::INFINITY, f64::min);
            assert!(
                best < 1e-6,
                "z0 {z0} slope {slope}: no piece matches the closed form {lower}, \
                 got {piece_areas:?} (closest {best:.3e})"
            );

            for piece in &pieces {
                assert!(piece.outer_wire.is_closed(&tol));
                let pcurves = piece.validate_pcurves(&tol, 37).expect("p-curves");
                assert!(
                    pcurves.is_valid(),
                    "a split piece's p-curves left its edges: {} mismatches",
                    pcurves.mismatch_count
                );
            }
        }
    }

    /// 平面を割るのは厳密でなければならない。曲がっていないので、近似の余地が
    /// どこにも無い。
    #[test]
    fn splitting_a_planar_face_is_exact() {
        let tol = Tolerance::default();

        // 角から角へ: ちょうど半分になる。
        let face = planar_square(10.0);
        let split = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 0.0, 0.0)),
            Vertex::from_point(Point3::new(0.0, 10.0, 0.0)),
        )
        .unwrap();
        let (pieces, report) = FaceSplitter::split_by_curve(&face, &split, &tol).expect("corner cut");
        assert!(report.area_residual < 1e-14);
        for area in &areas_3d(&pieces) {
            assert!(
                (area - 50.0).abs() < 1e-12,
                "half of the square is 50, got {area}"
            );
        }

        // 辺の途中から辺の途中へ: 角を切り落とす三角形と、残りの五角形。
        let face = planar_square(10.0);
        let split = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 4.0, 0.0)),
            Vertex::from_point(Point3::new(3.0, 10.0, 0.0)),
        )
        .unwrap();
        let (pieces, report) = FaceSplitter::split_by_curve(&face, &split, &tol).expect("mid cut");
        assert_eq!(pieces.len(), 2);
        assert!(report.area_residual < 1e-14);
        let triangle = 0.5 * 6.0 * 7.0;
        let piece_areas = areas_3d(&pieces);
        let best = piece_areas
            .iter()
            .map(|area| (area - triangle).abs())
            .fold(f64::INFINITY, f64::min);
        assert!(
            best < 1e-12,
            "the corner piece should be {triangle}, got {piece_areas:?}"
        );
    }

    /// 1枚に切り込みを2本入れて3枚にする。
    ///
    /// 面積の和で見るのは1本のときと同じだが、ここではもう一つ確かめる。
    /// **どの片も潰れていないこと。** 片方が 0 でも和は合うので、和だけでは
    /// 「割れた」と言えない。
    #[test]
    fn two_cuts_make_three_pieces_that_add_back_up() {
        let tol = Tolerance::default();

        // 平面を横に2本で切る。3枚の面積は 30, 40, 30。
        let face = planar_square(10.0);
        let cuts = vec![
            Edge::line_between(
                Vertex::from_point(Point3::new(0.0, 3.0, 0.0)),
                Vertex::from_point(Point3::new(10.0, 3.0, 0.0)),
            )
            .unwrap(),
            Edge::line_between(
                Vertex::from_point(Point3::new(0.0, 7.0, 0.0)),
                Vertex::from_point(Point3::new(10.0, 7.0, 0.0)),
            )
            .unwrap(),
        ];
        let (pieces, report) =
            FaceSplitter::split_by_curves(&face, &cuts, &tol).expect("two cuts on a plane");

        assert_eq!(pieces.len(), 3, "two cuts should make three pieces");
        assert_eq!(report.cuts_applied, 2);
        assert_eq!(report.cuts_refused, 0, "{:?}", report.refusals);
        assert!(report.area_residual < 1e-12);

        let piece_areas = areas_3d(&pieces);
        let mut areas = piece_areas.clone();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for (area, expected) in areas.iter().zip([30.0, 30.0, 40.0]) {
            assert!(
                (area - expected).abs() < 1e-9,
                "pieces should be 30, 30 and 40, got {piece_areas:?}"
            );
        }

        // 曲面でも同じ。互いに交わらない2本なら、1本ずつ当てれば足りる。
        let face = cylinder_quarter(10.0, 40.0);
        let cuts: Vec<Edge> = [(12.0, 0.4), (28.0, -0.5)]
            .into_iter()
            .map(|(z0, slope)| tilted_section(10.0, z0, slope))
            .collect();
        let (pieces, report) =
            FaceSplitter::split_by_curves(&face, &cuts, &tol).expect("two cuts on a cylinder");

        assert_eq!(pieces.len(), 3);
        assert_eq!(report.cuts_refused, 0, "{:?}", report.refusals);
        assert!(
            report.area_residual < 1e-9,
            "the three pieces do not add up: {:.3e}",
            report.area_residual
        );
        let piece_areas = areas_3d(&pieces);
        let total: f64 = piece_areas.iter().sum();
        for area in &piece_areas {
            assert!(
                *area > total * 0.05,
                "a piece came out empty: {piece_areas:?}"
            );
        }
    }

    /// どの片にも当たらない切り込みは、数えて捨てる。
    /// もっともらしい割り方を作るより、割れなかったと言うほうが良い。
    #[test]
    fn a_cut_that_lands_on_no_piece_is_counted_not_forced() {
        let tol = Tolerance::default();
        let face = planar_square(10.0);
        let cuts = vec![
            // 通る切り込み
            Edge::line_between(
                Vertex::from_point(Point3::new(0.0, 5.0, 0.0)),
                Vertex::from_point(Point3::new(10.0, 5.0, 0.0)),
            )
            .unwrap(),
            // 面から離れたところを通る切り込み
            Edge::line_between(
                Vertex::from_point(Point3::new(0.0, 5.0, 30.0)),
                Vertex::from_point(Point3::new(10.0, 5.0, 30.0)),
            )
            .unwrap(),
        ];

        let (pieces, report) =
            FaceSplitter::split_by_curves(&face, &cuts, &tol).expect("one cut lands");
        assert_eq!(pieces.len(), 2, "only the cut that lands should apply");
        assert_eq!(report.cuts_applied, 1);
        assert_eq!(report.cuts_refused, 1);
        assert!(!report.refusals.is_empty(), "a refusal must say why");
        assert!(report.area_residual < 1e-12);
    }

    /// 面の上に無い曲線、境界に届かない曲線は**断らなければならない**。
    /// もっともらしい2枚を返すほうが悪い。
    #[test]
    fn a_curve_that_does_not_lie_on_the_face_is_refused() {
        let tol = Tolerance::default();
        let face = cylinder_quarter(10.0, 40.0);

        // 円柱から離れたところを通る直線
        let off_surface = Edge::line_between(
            Vertex::from_point(Point3::new(10.0, 0.0, 20.0)),
            Vertex::from_point(Point3::new(0.0, 5.0, 20.0)),
        )
        .unwrap();
        assert!(FaceSplitter::split_by_curve(&face, &off_surface, &tol).is_err());

        // 面の上ではあるが、境界に届かない（片端が内部にある）
        let whole = tilted_section(10.0, 20.0, 0.6);
        let (t0, t1) = whole.curve.param_range();
        let half = whole.curve.split_at((t0 + t1) * 0.5).unwrap().0;
        let (m0, m1) = half.param_range();
        let stub = Edge::new(
            half.clone(),
            Vertex::from_point(half.evaluate(m0)),
            Vertex::from_point(half.evaluate(m1)),
            1e-6,
        );
        assert!(
            FaceSplitter::split_by_curve(&face, &stub, &tol).is_err(),
            "a curve that stops inside the face must not produce two pieces"
        );
    }
}
