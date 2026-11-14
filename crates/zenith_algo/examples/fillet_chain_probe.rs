//! **フィレットの上にフィレットを掛けられるか**（ROADMAP 柱⑦の第1段）。
//!
//! CAD の日常操作は「丸めたあと、また別の稜を丸める」の繰り返しです。
//! この探針は、検体を1回（または数回）丸めたあと、**残りの稜を1本ずつ
//! 丸めてみて**、通った件数と断り文を数えます。直す前に、何件が・どの
//! 理由で断られるかを測るためのものです。
//!
//! 通った件は、削れた体積が演算の予測（閉形式）と相対 1e-6 で合うかも見ます。
//! 合わなければ「誤答」と数えます。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example fillet_chain_probe
//! ```

use std::collections::BTreeMap;
use zenith_algo::{BlendKind, EdgeBlender, MassCalculator, PrimitiveBuilder};
use zenith_tess::TessellationParams;
use zenith_topo::{FaceGeometry, Solid};

fn edge_ids(solid: &Solid) -> Vec<u64> {
    let mut ids: Vec<u64> = Vec::new();
    for face in &solid.outer_shell.faces {
        for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
            for oriented in &wire.edges {
                if !ids.contains(&oriented.edge.id) {
                    ids.push(oriented.edge.id);
                }
            }
        }
    }
    ids
}

/// 稜の両側の面の種類（"平面×曲面" など）。フィレット面は NURBS で持つので「曲面」。
fn sides(solid: &Solid, edge_id: u64) -> String {
    let mut kinds: Vec<&str> = Vec::new();
    for face in &solid.outer_shell.faces {
        let uses = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .any(|wire| wire.edges.iter().any(|oriented| oriented.edge.id == edge_id));
        if uses {
            kinds.push(match &face.geometry {
                FaceGeometry::Plane(_) => "平面",
                _ => "曲面",
            });
        }
    }
    kinds.sort_unstable();
    kinds.join("×")
}

/// 点 `point`（面の上にある前提）での面の単位法線。向きは問わない。
/// NURBS は格子で一番近い点を探し、ガウス・ニュートンで詰める。
fn normal_near(face: &zenith_topo::Face, point: zenith_math::Point3) -> Option<zenith_math::Vec3> {
    match &face.geometry {
        FaceGeometry::Plane(plane) => Some(plane.normal.normalize()),
        FaceGeometry::Nurbs(surface) => {
            let ((u0, u1), (v0, v1)) = surface.param_range();
            let mut best = (f64::INFINITY, u0, v0);
            for i in 0..=48 {
                for j in 0..=48 {
                    let u = u0 + (u1 - u0) * i as f64 / 48.0;
                    let v = v0 + (v1 - v0) * j as f64 / 48.0;
                    let d = (surface.evaluate(u, v) - point).norm();
                    if d < best.0 {
                        best = (d, u, v);
                    }
                }
            }
            let (_, mut u, mut v) = best;
            for _ in 0..30 {
                let (p, du, dv) = surface.evaluate_derivatives_1st(u, v);
                let r = point - p;
                let (a, b, c) = (du.dot(&du), du.dot(&dv), dv.dot(&dv));
                let det = a * c - b * b;
                if det.abs() < 1e-300 {
                    break;
                }
                let (g1, g2) = (du.dot(&r), dv.dot(&r));
                u = (u + (c * g1 - b * g2) / det).clamp(u0, u1);
                v = (v + (a * g2 - b * g1) / det).clamp(v0, v1);
            }
            if (surface.evaluate(u, v) - point).norm() > 1e-7 {
                return None;
            }
            surface.normal_or_limit(u, v).map(|n| n.normalize())
        }
        _ => None,
    }
}

/// 両側の面が稜の中点で接している（G1、丸める角が無い）か。
fn is_smooth(solid: &Solid, edge_id: u64) -> bool {
    let mut normals = Vec::new();
    let mut middle = None;
    for face in &solid.outer_shell.faces {
        let found = std::iter::once(&face.outer_wire)
            .chain(face.inner_wires.iter())
            .flat_map(|wire| wire.edges.iter())
            .find(|oriented| oriented.edge.id == edge_id);
        if let Some(oriented) = found {
            let (t0, t1) = oriented.edge.curve.param_range();
            let point = *middle.get_or_insert(oriented.edge.curve.evaluate((t0 + t1) * 0.5));
            match normal_near(face, point) {
                Some(n) => normals.push(n),
                None => return false,
            }
        }
    }
    normals.len() == 2 && normals[0].cross(&normals[1]).norm() < 1e-6
}

fn volume(solid: &Solid) -> f64 {
    MassCalculator::compute_volume_from_brep(
        solid,
        &TessellationParams {
            u_divisions: 64,
            v_divisions: 64,
        },
    )
}

/// 断り文を短くまとめる（数字を落として同じ理由を束ねる）。
fn reason_key(text: &str) -> String {
    let short: String = text
        .chars()
        .map(|c| if c.is_ascii_digit() || c == '.' || c == '-' { '#' } else { c })
        .collect();
    let mut collapsed = String::new();
    for c in short.chars() {
        if c == '#' && collapsed.ends_with('#') {
            continue;
        }
        collapsed.push(c);
    }
    collapsed.chars().take(110).collect()
}

struct Case {
    name: &'static str,
    why: &'static str,
    solid: Result<Solid, String>,
    radius: f64,
}

fn vertical_edges(solid: &Solid) -> Vec<u64> {
    edge_ids(solid)
        .into_iter()
        .filter(|id| {
            solid
                .outer_shell
                .faces
                .iter()
                .flat_map(|face| std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()))
                .flat_map(|wire| wire.edges.iter())
                .find(|oriented| oriented.edge.id == *id)
                .map(|oriented| {
                    let a = oriented.edge.start_vertex.point;
                    let b = oriented.edge.end_vertex.point;
                    (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() > 1e-9
                })
                .unwrap_or(false)
        })
        .collect()
}

fn cases() -> Vec<Case> {
    let base = PrimitiveBuilder::make_box(20.0, 30.0, 40.0);
    let mut out = Vec::new();

    let one = base.clone().and_then(|solid| {
        let first = vertical_edges(&solid)[0];
        EdgeBlender::fillet_edge(&solid, first, 3.0)
    });
    out.push(Case {
        name: "box, 1 vertical edge filleted (r3)",
        why: "縦稜1本を丸めた直方体。残りの稜、特にフィレット面に接する上下の稜を丸める",
        solid: one,
        radius: 2.0,
    });

    let four: Result<Solid, String> = base.clone().and_then(|solid| {
        let edges: Vec<(u64, f64)> = vertical_edges(&solid).into_iter().map(|id| (id, 3.0)).collect();
        EdgeBlender::fillet_edges(&solid, &edges)
    });
    out.push(Case {
        name: "box, 4 vertical edges filleted (r3)",
        why: "縦稜4本を丸めた直方体（角丸の角柱）。上下の縁（直線4本＋円弧4本）を丸める",
        solid: four.clone(),
        radius: 2.0,
    });

    // 3段目: 角丸の角柱の上縁を丸めたあと、残りを丸める。上縁は直線の稜を
    // 1本選べば全周へ伝わる（`ZENITH_PRISM_RIM_BLEND`）。
    let rounded_top = four.clone().and_then(|solid| {
        let top_z = 40.0;
        let id = edge_ids(&solid)
            .into_iter()
            .find(|id| {
                solid
                    .outer_shell
                    .faces
                    .iter()
                    .flat_map(|face| std::iter::once(&face.outer_wire))
                    .flat_map(|wire| wire.edges.iter())
                    .any(|oriented| {
                        oriented.edge.id == *id
                            && (oriented.edge.start_vertex.point.z - top_z).abs() < 1e-9
                            && (oriented.edge.end_vertex.point.z - top_z).abs() < 1e-9
                    })
            })
            .ok_or("no top edge")?;
        EdgeBlender::fillet_edge(&solid, id, 2.0)
    });
    out.push(Case {
        name: "box, 4 vertical edges filleted (r3), rims at r3",
        why: "角丸の角柱の上下の縁を角の半径と同じ r3 で丸める。円弧4本が全部点に潰れ、角は球の面（4-946）",
        solid: four.clone(),
        radius: 3.0,
    });
    let rounded_top_same = four.clone().and_then(|solid| {
        let id = edge_ids(&solid)
            .into_iter()
            .find(|id| {
                solid.outer_shell.faces.iter().flat_map(|face| face.outer_wire.edges.iter()).any(|o| {
                    o.edge.id == *id
                        && (o.edge.start_vertex.point.z - 40.0).abs() < 1e-9
                        && (o.edge.end_vertex.point.z - 40.0).abs() < 1e-9
                })
            })
            .ok_or("no top edge")?;
        EdgeBlender::fillet_edge(&solid, id, 3.0)
    });
    out.push(Case {
        name: "box, 4 vertical (r3) then top rim (r3) filleted",
        why: "上の4つの角が球の面で閉じた角丸の箱。残りの稜を丸める",
        solid: rounded_top_same,
        radius: 1.5,
    });
    out.push(Case {
        name: "box, 4 vertical (r3) then top rim (r2) filleted",
        why: "角丸の角柱の上縁を全周丸めた立体。下の縁や、丸めた面どうしの稜を丸める",
        solid: rounded_top,
        radius: 1.5,
    });

    let chamfered = base.clone().and_then(|solid| {
        let first = vertical_edges(&solid)[0];
        EdgeBlender::chamfer_edge(&solid, first, 3.0)
    });
    out.push(Case {
        name: "box, 1 vertical edge chamfered (c3)",
        why: "縦稜1本を面取りした直方体。面取りは平面なので、上下の稜は「平面×平面」のまま",
        solid: chamfered,
        radius: 2.0,
    });

    out.push(Case {
        name: "stepped shaft (r10 h12 + r7 h10)",
        why: "段付き軸。両端の円い縁（1/4円4本のループ）と、肩の根元（凹稜）",
        solid: zenith_algo::ShaftBuilder::make_stepped_shaft(&[(10.0, 12.0), (7.0, 10.0)]),
        radius: 1.5,
    });

    // 4-937 のあとに足した検体（柱⑦の手順3へ向けて）
    let point_of = |solid: &Solid, id: u64| {
        solid
            .outer_shell
            .faces
            .iter()
            .flat_map(|face| face.outer_wire.edges.iter())
            .find(|oriented| oriented.edge.id == id)
            .map(|oriented| oriented.edge.start_vertex.point)
    };
    let two = base.clone().and_then(|solid| {
        let verticals = vertical_edges(&solid);
        let first = verticals[0];
        let p0 = point_of(&solid, first).ok_or("no point")?;
        let second = verticals
            .iter()
            .copied()
            .find(|id| {
                *id != first
                    && point_of(&solid, *id)
                        .map(|p| (p.x - p0.x).abs() < 1e-9 || (p.y - p0.y).abs() < 1e-9)
                        .unwrap_or(false)
            })
            .ok_or("no adjacent vertical")?;
        EdgeBlender::fillet_edges(&solid, &[(first, 3.0), (second, 3.0)])
    });
    // 4-941 のあと: 2本の縦稜の上の開いた鎖（直線・円弧・直線・円弧・直線）まで丸めた立体
    let two_then_chain = two.clone().and_then(|rounded| {
        let arc = edge_ids(&rounded)
            .into_iter()
            .find(|id| {
                rounded
                    .outer_shell
                    .faces
                    .iter()
                    .flat_map(|face| face.outer_wire.edges.iter())
                    .any(|oriented| {
                        let a = oriented.edge.start_vertex.point;
                        let b = oriented.edge.end_vertex.point;
                        oriented.edge.id == *id
                            && (a.z - 40.0).abs() < 1e-9
                            && (b.z - 40.0).abs() < 1e-9
                            && (b - a).norm() < 5.0
                    })
            })
            .ok_or("no top arc")?;
        EdgeBlender::fillet_edge(&rounded, arc, 2.0)
    });
    out.push(Case {
        name: "box, 2 adjacent verticals (r3) then their top chain (r2) filleted",
        why: "縦稜2本の上の開いた鎖を丸めた立体。鎖の両端の側壁に、縦稜・端の円弧・上の稜の鎖ができる",
        solid: two_then_chain,
        radius: 1.5,
    });
    out.push(Case {
        name: "box, 2 adjacent vertical edges filleted (r3)",
        why: "隣り合う縦稜2本を丸めた直方体。上縁は「直線・円弧・直線・円弧・直線」の開いた鎖",
        solid: two,
        radius: 2.0,
    });

    let one_then_chain = base.clone().and_then(|solid| {
        let first = vertical_edges(&solid)[0];
        let rounded = EdgeBlender::fillet_edge(&solid, first, 3.0)?;
        let arc = edge_ids(&rounded)
            .into_iter()
            .find(|id| {
                rounded
                    .outer_shell
                    .faces
                    .iter()
                    .flat_map(|face| face.outer_wire.edges.iter())
                    .any(|oriented| {
                        let a = oriented.edge.start_vertex.point;
                        let b = oriented.edge.end_vertex.point;
                        oriented.edge.id == *id
                            && (a.z - 40.0).abs() < 1e-9
                            && (b.z - 40.0).abs() < 1e-9
                            && (b - a).norm() < 5.0
                    })
            })
            .ok_or("no top arc")?;
        EdgeBlender::fillet_edge(&rounded, arc, 2.0)
    });
    out.push(Case {
        name: "box, 1 vertical (r3) then its top chain (r2) filleted",
        why: "縦稜1本と、その上の開いた鎖を丸めた立体。鎖の端の角に、丸めた面と素の稜が集まる（頂点ブレンドの入口）",
        solid: one_then_chain,
        radius: 1.5,
    });

    // L字の輪郭（20×30 から x 10..20, y 15..30 を欠いた形）をそのまま押し出す。
    let l_shape = (|| -> Result<Solid, String> {
        let corners = [
            (0.0, 0.0),
            (20.0, 0.0),
            (20.0, 15.0),
            (10.0, 15.0),
            (10.0, 30.0),
            (0.0, 30.0),
        ];
        let vertices: Vec<zenith_topo::Vertex> = corners
            .iter()
            .map(|(x, y)| zenith_topo::Vertex::from_point(zenith_math::Point3::new(*x, *y, 0.0)))
            .collect();
        let mut edges = Vec::new();
        for index in 0..vertices.len() {
            let next = (index + 1) % vertices.len();
            edges.push(zenith_topo::OrientedEdge::forward(zenith_topo::Edge::line_between(
                vertices[index].clone(),
                vertices[next].clone(),
            )?));
        }
        zenith_algo::ExtrudeBuilder::extrude_wire(
            &zenith_topo::Wire::new(edges),
            zenith_math::Vec3::new(0.0, 0.0, 40.0),
            &zenith_math::Tolerance::default(),
        )
    })();
    out.push(Case {
        name: "L-shaped prism (box minus corner notch)",
        why: "L字の角柱。凹の縦稜が1本ある（凹の鎖・凹の角の入口）",
        solid: l_shape,
        radius: 2.0,
    });

    out.push(Case {
        name: "regular hexagonal prism (r10 h25)",
        why: "正六角柱。上下の稜の端は隣の側面（稜と60度）にぶつかる（斜めの端、4-936）",
        solid: PrimitiveBuilder::make_regular_prism(6, 10.0, 25.0),
        radius: 1.5,
    });

    // 正六角柱の縦稜6本を r2 で丸めた立体。上縁は直線6本・60度の円弧6本の閉じたループ。
    let rounded_hexagon = PrimitiveBuilder::make_regular_prism(6, 10.0, 25.0).and_then(|solid| {
        let verticals: Vec<(u64, f64)> = vertical_edges(&solid).into_iter().map(|id| (id, 2.0)).collect();
        EdgeBlender::fillet_edges(&solid, &verticals)
    });
    out.push(Case {
        name: "hexagonal prism, 6 verticals filleted (r2)",
        why: "角丸の正六角柱。上縁のループの円弧は中心角60度",
        solid: rounded_hexagon,
        radius: 1.0,
    });

    // 上の稜1本を丸めた・面取りした直方体。端の側壁で縦稜・断面・上の稜が集まる。
    let top_edge = |solid: &Solid| {
        edge_ids(solid).into_iter().find(|id| {
            solid
                .outer_shell
                .faces
                .iter()
                .flat_map(|face| face.outer_wire.edges.iter())
                .any(|oriented| {
                    let a = oriented.edge.start_vertex.point;
                    let b = oriented.edge.end_vertex.point;
                    oriented.edge.id == *id && (a.z - 40.0).abs() < 1e-9 && (b.z - 40.0).abs() < 1e-9
                })
        })
    };
    out.push(Case {
        name: "box, 1 top edge filleted (r2)",
        why: "上の稜1本を丸めた直方体。両端の縦稜の上端は丸めた面の断面に当たる",
        solid: base.clone().and_then(|solid| {
            let id = top_edge(&solid).ok_or("no top edge")?;
            EdgeBlender::fillet_edge(&solid, id, 2.0)
        }),
        radius: 1.5,
    });
    out.push(Case {
        name: "box, 1 top edge chamfered (c2)",
        why: "上の稜1本を面取りした直方体。両端の縦稜の上端は面取り面の断面に当たる",
        solid: base.clone().and_then(|solid| {
            let id = top_edge(&solid).ok_or("no top edge")?;
            EdgeBlender::chamfer_edge(&solid, id, 2.0)
        }),
        radius: 1.5,
    });
    // L字の凹の縦稜を丸めた立体。上縁は凹の円弧を含む開いた鎖。
    let l_rounded = (|| -> Result<Solid, String> {
        let corners = [(0.0, 0.0), (20.0, 0.0), (20.0, 15.0), (10.0, 15.0), (10.0, 30.0), (0.0, 30.0)];
        let vertices: Vec<zenith_topo::Vertex> = corners
            .iter()
            .map(|(x, y)| zenith_topo::Vertex::from_point(zenith_math::Point3::new(*x, *y, 0.0)))
            .collect();
        let mut edges = Vec::new();
        for index in 0..vertices.len() {
            let next = (index + 1) % vertices.len();
            edges.push(zenith_topo::OrientedEdge::forward(zenith_topo::Edge::line_between(
                vertices[index].clone(),
                vertices[next].clone(),
            )?));
        }
        let solid = zenith_algo::ExtrudeBuilder::extrude_wire(
            &zenith_topo::Wire::new(edges),
            zenith_math::Vec3::new(0.0, 0.0, 40.0),
            &zenith_math::Tolerance::default(),
        )?;
        let concave = vertical_edges(&solid)
            .into_iter()
            .find(|id| {
                solid
                    .outer_shell
                    .faces
                    .iter()
                    .flat_map(|face| face.outer_wire.edges.iter())
                    .any(|oriented| {
                        let a = oriented.edge.start_vertex.point;
                        oriented.edge.id == *id && (a.x - 10.0).abs() < 1e-9 && (a.y - 15.0).abs() < 1e-9
                    })
            })
            .ok_or("no concave vertical")?;
        EdgeBlender::fillet_edge(&solid, concave, 3.0)
    })();
    out.push(Case {
        name: "L-shaped prism, concave vertical filleted (r3)",
        why: "L字の凹の縦稜を丸めた立体。上縁は凹の円弧を含む開いた鎖",
        solid: l_rounded,
        radius: 1.5,
    });

    // 4-942 のあとに足した検体（手順3の断りを探す）
    out.push(Case {
        name: "drilled box (20x30x40, hole r5)",
        why: "貫通穴の直方体。穴の縁は平面×円柱の円い稜",
        solid: zenith_algo::HoleBuilder::make_drilled_box(20.0, 30.0, 40.0, 5.0),
        radius: 1.5,
    });
    out.push(Case {
        name: "regular pentagonal prism (r10 h25)",
        why: "正五角柱。上下の稜の端は隣の側面と108度",
        solid: PrimitiveBuilder::make_regular_prism(5, 10.0, 25.0),
        radius: 1.5,
    });
    out.push(Case {
        name: "drafted block (20x30x40, 5 deg)",
        why: "抜き勾配の付いたブロック。側面が傾いた平面",
        solid: zenith_algo::DraftBuilder::make_drafted_block(
            20.0,
            30.0,
            40.0,
            5f64.to_radians(),
            &zenith_math::Tolerance::default(),
        ),
        radius: 1.5,
    });
    out.push(Case {
        name: "plain washer (ri 5, ro 12, t 3)",
        why: "平座金。内外の円い縁",
        solid: zenith_algo::FastenerBuilder::make_plain_washer(5.0, 12.0, 3.0, &zenith_math::Tolerance::default()),
        radius: 0.8,
    });
    out.push(Case {
        name: "drilled box after merge_coplanar",
        why: "貫通穴の直方体で、継ぎ目で割れた同じ平面の面を先に1枚にまとめた形",
        solid: zenith_algo::HoleBuilder::make_drilled_box(20.0, 30.0, 40.0, 5.0).and_then(|solid| {
            zenith_algo::FaceMerger::merge_coplanar(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
        }),
        radius: 1.5,
    });
    out.push(Case {
        name: "counterbore hole box (40x40x20, r5 / cb r9 d6)",
        why: "座ぐり穴の直方体。段の面と上面に穴の縁の円がある",
        solid: zenith_algo::HoleBuilder::make_counterbore_hole_box(40.0, 40.0, 20.0, 5.0, 9.0, 6.0),
        radius: 1.0,
    });
    out.push(Case {
        name: "box, corner rounded by two fillets (r3)",
        why: "1つの角に集まる3本の稜をまとめて丸めた直方体（頂点ブレンド、手順3）",
        solid: base.clone().and_then(|solid| {
            let first = vertical_edges(&solid)[0];
            let p = point_of(&solid, first).ok_or("no point")?;
            let corner = if (p.z - 0.0).abs() < 1e-9 { p } else { zenith_math::Point3::new(p.x, p.y, 0.0) };
            let at_corner: Vec<(u64, f64)> = edge_ids(&solid)
                .into_iter()
                .filter(|id| {
                    solid
                        .outer_shell
                        .faces
                        .iter()
                        .flat_map(|face| face.outer_wire.edges.iter())
                        .any(|o| {
                            o.edge.id == *id
                                && ((o.edge.start_vertex.point - corner).norm() < 1e-9
                                    || (o.edge.end_vertex.point - corner).norm() < 1e-9)
                        })
                })
                .map(|id| (id, 3.0))
                .collect();
            if at_corner.len() != 3 {
                return Err(format!("corner has {} edges", at_corner.len()));
            }
            // 1本目を丸め、2本目を丸めると鎖で3本目も丸まる（角は球の面、4-946）。
            // fillet_edges にまとめて渡すと、鎖で消えた3本目が見つからず断られる。
            let one = EdgeBlender::fillet_edge(&solid, at_corner[0].0, 3.0)?;
            let second = edge_ids(&one)
                .into_iter()
                .find(|id| *id == at_corner[1].0)
                .ok_or("second corner edge was rebuilt")?;
            EdgeBlender::fillet_edge(&one, second, 3.0)
        }),
        radius: 1.5,
    });

    // ブーリアンで作った立体（4-952）。
    let union = |a: Result<Solid, String>, b: Result<Solid, String>| -> Result<Solid, String> {
        let (a, b) = (a?, b?);
        let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
            &a,
            &b,
            zenith_algo::BooleanOpType::Union,
            &zenith_math::Tolerance::default(),
        )?;
        let solid = result.solids.into_iter().next().ok_or("empty union")?;
        zenith_algo::FaceMerger::simplify_solid(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
    };
    out.push(Case {
        name: "box with a cylindrical boss on top (union, simplified)",
        why: "直方体の上面に円柱のボスを立てた和。ボスの根元は凹の円、上は凸の円",
        solid: union(
            base.clone(),
            PrimitiveBuilder::make_cylinder(5.0, 10.0)
                .map(|c| zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(10.0, 15.0, 40.0))),
        ),
        radius: 1.0,
    });
    out.push(Case {
        name: "T-shaped prism (union of two boxes, merged)",
        why: "2つの箱の和で作った T 字の角柱。凹の縦稜2本と、凹凸の混じる上縁",
        solid: union(
            PrimitiveBuilder::make_box(30.0, 10.0, 20.0),
            PrimitiveBuilder::make_box(10.0, 20.0, 20.0)
                .map(|c| zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(10.0, 10.0, 0.0))),
        ),
        radius: 1.5,
    });

    let difference = |a: Result<Solid, String>, b: Result<Solid, String>| -> Result<Solid, String> {
        let (a, b) = (a?, b?);
        let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
            &a,
            &b,
            zenith_algo::BooleanOpType::Difference,
            &zenith_math::Tolerance::default(),
        )?;
        let solid = result.solids.into_iter().next().ok_or("empty difference")?;
        zenith_algo::FaceMerger::simplify_solid(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
    };
    out.push(Case {
        name: "box with a rectangular pocket (difference, simplified)",
        why: "上面に 10x14 深さ 8 の止まりのポケットを掘った差。凹の縦稜4本・底の凹の稜4本・口の凸の稜4本",
        solid: difference(
            base.clone(),
            PrimitiveBuilder::make_box(10.0, 14.0, 20.0)
                .map(|c| zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(5.0, 8.0, 32.0))),
        ),
        radius: 1.0,
    });
    out.push(Case {
        name: "box with a blind round hole (difference, simplified)",
        why: "上面に r4 深さ 10 の止まり穴を開けた差。口の凸の円と底の凹の円",
        solid: difference(
            base.clone(),
            PrimitiveBuilder::make_cylinder(4.0, 20.0)
                .map(|c| zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(10.0, 15.0, 30.0))),
        ),
        radius: 1.0,
    });

    out.push(Case {
        name: "box with a through slot (difference, simplified)",
        why: "上面に幅 8 深さ 10 の溝を y 方向に貫通させた差。溝の底の凹の稜2本と口の凸の稜2本が端の面まで届く",
        solid: difference(
            base.clone(),
            PrimitiveBuilder::make_box(8.0, 40.0, 20.0)
                .map(|c| zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(6.0, -5.0, 30.0))),
        ),
        radius: 1.0,
    });
    out.push(Case {
        name: "box with a side hole (difference, simplified)",
        why: "側面 x=0 から r4 の丸穴を x 方向に貫通させた差。穴の縁が側面の内側ワイヤになる",
        solid: difference(
            base.clone(),
            PrimitiveBuilder::make_cylinder(4.0, 40.0).and_then(|c| {
                let turn = zenith_math::Transform3::from_axis_angle(
                    &zenith_math::Vec3::new(0.0, 1.0, 0.0),
                    std::f64::consts::FRAC_PI_2,
                );
                let c = zenith_algo::BrepTransform::transform_solid(&c, &turn)?;
                Ok(zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(-10.0, 15.0, 20.0)))
            }),
        ),
        radius: 1.0,
    });

    out.push(Case {
        name: "box with a rounded pocket (pocket verticals r2)",
        why: "四角いポケットの縦稜4本（凹）を r2 で丸めた箱。ポケットの口は凹の円弧を含む内側ワイヤ、底は凸の円弧を含む",
        solid: difference(
            base.clone(),
            PrimitiveBuilder::make_box(10.0, 14.0, 20.0)
                .map(|c| zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(5.0, 8.0, 32.0))),
        )
        .and_then(|mut solid| {
            for (x, y) in [(5.0, 8.0), (15.0, 8.0), (15.0, 22.0), (5.0, 22.0)] {
                let id = solid
                    .outer_shell
                    .faces
                    .iter()
                    .flat_map(|face| face.outer_wire.edges.iter())
                    .find(|o| {
                        let (a, b) = (o.edge.start_vertex.point, o.edge.end_vertex.point);
                        (a.x - x).abs() < 1e-9 && (b.x - x).abs() < 1e-9 && (a.y - y).abs() < 1e-9
                            && (b.y - y).abs() < 1e-9 && (a.z - b.z).abs() > 7.0
                    })
                    .map(|o| o.edge.id)
                    .ok_or("pocket vertical")?;
                solid = EdgeBlender::fillet_edge(&solid, id, 2.0)?;
            }
            Ok(solid)
        }),
        radius: 1.0,
    });

    out.push(Case {
        name: "D-shaped block (box ∩ cylinder r18, simplified)",
        why: "直方体と半径 18 の円柱の積。上下の縁は直線と円弧が角で出会い、側面に円柱面が1枚",
        solid: (|| -> Result<Solid, String> {
            let a = base.clone()?;
            let b = zenith_algo::BrepTransform::translate_solid(
                &PrimitiveBuilder::make_cylinder(18.0, 60.0)?,
                zenith_math::Vec3::new(0.0, 15.0, -10.0),
            );
            let result = zenith_algo::BooleanEngine::boolean_solids_exact_result(
                &a,
                &b,
                zenith_algo::BooleanOpType::Intersection,
                &zenith_math::Tolerance::default(),
            )?;
            let solid = result.solids.into_iter().next().ok_or("empty")?;
            zenith_algo::FaceMerger::simplify_solid(&solid, &zenith_math::Tolerance::default()).map(|(s, _)| s)
        })(),
        radius: 1.0,
    });

    out.push(Case {
        name: "box ∪ box turned 30 deg about z (simplified)",
        why: "直方体の角に、z まわりに 30 度回した箱を同じ高さで重ねた和。斜めに出会う凹の縦稜と、直角でない上縁の角",
        solid: union(
            base.clone(),
            PrimitiveBuilder::make_box(16.0, 10.0, 40.0).and_then(|c| {
                let turn = zenith_math::Transform3::from_axis_angle(
                    &zenith_math::Vec3::new(0.0, 0.0, 1.0),
                    30f64.to_radians(),
                );
                let c = zenith_algo::BrepTransform::transform_solid(&c, &turn)?;
                Ok(zenith_algo::BrepTransform::translate_solid(&c, zenith_math::Vec3::new(14.0, 24.0, 0.0)))
            }),
        ),
        radius: 1.0,
    });

    out.push(Case {
        name: "box (no prior blend)",
        why: "比較用: 何も丸めていない直方体",
        solid: base,
        radius: 2.0,
    });
    out
}

fn main() {
    // `ZENITH_FCP_CHAMFER=1` で面取りを当てる（既定はフィレット）。
    let chamfer = std::env::var("ZENITH_FCP_CHAMFER").map_or(false, |v| !v.is_empty() && v != "0");
    let mut total_ok = 0usize;
    let mut total_refused = 0usize;
    let mut total_wrong = 0usize;
    let mut total_smooth = 0usize;
    for case in cases() {
        println!("== {} ==", case.name);
        println!("   {}", case.why);
        let solid = match case.solid {
            Ok(solid) => solid,
            Err(err) => {
                println!("   下ごしらえで断られた: {err}");
                continue;
            }
        };
        let before = volume(&solid);
        let mut ok = 0usize;
        let mut wrong = 0usize;
        let mut reasons: BTreeMap<(String, String), usize> = BTreeMap::new();
        let mut ok_kinds: BTreeMap<String, usize> = BTreeMap::new();
        let mut worst = 0.0f64;
        let mut smooth = 0usize;
        for id in edge_ids(&solid) {
            let kind = sides(&solid, id);
            // 接している稜（平面とフィレット面の継ぎ目など）には丸める角が無い。
            // 断りとも通ったとも数えず、別に数える。
            if is_smooth(&solid, id) {
                smooth += 1;
                continue;
            }
            let kind_of_blend = if chamfer {
                BlendKind::Chamfer { distance: case.radius }
            } else {
                BlendKind::Fillet { radius: case.radius }
            };
            match EdgeBlender::blend_edge(&solid, id, kind_of_blend) {
                Ok((result, report)) => {
                    let after = volume(&result);
                    let removed = before - after;
                    let predicted = report.predicted_removed_volume;
                    let error = (removed - predicted).abs() / predicted.abs().max(1e-12);
                    // 凹稜（肩の根元など）は材料を足すので、削れ量も予測も負になる。
                    if removed.abs() > 1e-9 && error <= 1e-6 {
                        ok += 1;
                        worst = worst.max(error);
                        *ok_kinds.entry(kind).or_insert(0) += 1;
                    } else {
                        wrong += 1;
                        println!(
                            "   WRONG 稜 {id}（{kind}）: 削れた {removed:.9}、予測 {predicted:.9}（相対差 {error:.3e}）"
                        );
                    }
                }
                Err(err) => {
                    // ZENITH_FCP_WHY=1 なら断った稜の端点と理由を出す
                    if std::env::var("ZENITH_FCP_WHY").is_ok_and(|v| v == "1") {
                        let edge = solid
                            .outer_shell
                            .faces
                            .iter()
                            .flat_map(|face| face.outer_wire.edges.iter())
                            .find(|oriented| oriented.edge.id == id);
                        if let Some(oriented) = edge {
                            let (a, b) = (oriented.edge.start_vertex.point, oriented.edge.end_vertex.point);
                            println!(
                                "   断り 稜 {id}（{kind}） ({:.3},{:.3},{:.3})→({:.3},{:.3},{:.3}): {err}",
                                a.x, a.y, a.z, b.x, b.y, b.z
                            );
                        }
                    }
                    *reasons.entry((kind, reason_key(&err))).or_insert(0) += 1;
                }
            }
        }
        let refused: usize = reasons.values().sum();
        println!(
            "   体積 {before:.6}、稜 {} 本: 通った {ok}（予測体積との相対差の最悪 {worst:.3e}）、断り {refused}、誤答 {wrong}、接していて角の無い稜 {smooth}",
            ok + refused + wrong + smooth
        );
        for (kind, count) in &ok_kinds {
            println!("     通った {count:>3}  [{kind}]");
        }
        for ((kind, reason), count) in &reasons {
            println!("     断り   {count:>3}  [{kind}] {reason}");
        }
        total_ok += ok;
        total_refused += refused;
        total_wrong += wrong;
        total_smooth += smooth;
    }
    println!("合計: 通った {total_ok}、断り {total_refused}、誤答 {total_wrong}、接していて角の無い稜 {total_smooth}");
}
