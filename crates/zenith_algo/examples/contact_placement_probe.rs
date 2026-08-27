//! 接触している配置を、演算ごとに数える。
//!
//! `boolean_envelope` の45ケースのうち、いま通らないのは1件だけです
//! （2026/08/24 に 6件 → 4件 → 8/27 に 1件。4-74）。**その1件は直す対象では
//! ありません**——答えのほうが非多様体なので、名指しして断ります。
//! この文書は長らく「6件はすべて接線配置」と書いて
//! いましたが、測ったら**内訳が違いました**（HANDOVER 3-1）。ここはその
//! 内訳を、いつでも測り直せるようにしたものです。
//!
//! ## 規約（HANDOVER 3-1 で決めたもの）
//!
//! **接触は、それ自体では位相を作りません。** 2つの曲面が交わらずに触れて
//! いるだけの場所には、交線を作らない。そのうえで、
//!
//! - 真の答えが多様体の立体になるなら、**返す**
//! - 真の答えが**本当に非多様体**なら、**名指しして断る**
//!
//! `Solid` は多様体 B-Rep なので、非多様体を「もっともらしい立体」にして
//! 返すのは誤答です。**断るのは実装の不足ではなく、型が表現できないものを
//! 表現したふりをしない、という設計判断です。**
//!
//! ## この表の読み方
//!
//! **「非多様体のはずだ」と決めつけません。返ってきた立体を測ります。**
//! 立体が返ったら、そのメッシュの稜が**ちょうど2枚の三角形に共有されて
//! いるか**を数えます。共有されていない稜が1本でもあれば、それは多様体
//! ではありません。
//!
//! - `REFUSED` — 断られた。**赤にはしません**。断り方は2種類あって、
//!   **答えが本当に非多様体なので断るもの**（そのときは場所を名指しします）と、
//!   **まだ実装していないもの**があります。後者は直したぶんだけ `ok` に
//!   変わりますが、前者は変わりません——変わったら、それは誤答です
//! - `ok` かつ 非多様体の稜 0 — 多様体の立体が返った
//! - `ok` かつ 非多様体の稜 > 0 — **赤**。非多様体を立体として返すのは、
//!   断るより悪い
//!
//! `note` の列は**実測**です。ここには予想を書いていましたが、**3件書いて
//! 3件とも外れました**（4-74）。外れた予想を残さず、測った内容に置き換えて
//! あります。予想を実測の代わりにしない、というのがこのリポジトリの決まり
//! です（5章、4-33）。
//!
//! ```bash
//! cargo run --release -p zenith_algo --example contact_placement_probe
//! ZENITH_SPLIT_WHY=1 cargo run --release -p zenith_algo --example contact_placement_probe
//! ZENITH_CONTACT_FILTER="spun 45" ZENITH_CONTACT_OP=intersection \
//!   cargo run --release -p zenith_algo --example contact_placement_probe
//! ```

use zenith_algo::{BooleanEngine, BooleanOpType, BrepTransform, PrimitiveBuilder};
use zenith_math::{Tolerance, Transform3, Vec3};
use zenith_topo::Solid;

struct Case {
    name: &'static str,
    /// なぜ接触配置なのか。
    why: &'static str,
    a: Solid,
    b: Solid,
    /// union / difference / intersection それぞれについて、**測って分かって
    /// いること**。表示のためだけで、判定の根拠ではない。
    note: [&'static str; 3],
}

/// **B-Rep** の稜のうち、ちょうど2つの面ループに使われていない本数。
///
/// 「非多様体の立体を返した」かどうかは、これで決まります。`Solid` は
/// B-Rep なので、多様体かどうかは**稜が2枚の面に共有されているか**です。
///
/// **最初はメッシュで測っていました。間違いです。** メッシュは派生物で、
/// そこが壊れていても B-Rep は正しいことがあります。実測（4-83）: 球を
/// 45度回して切った結果は、B-Rep の稜がすべてちょうど2回使われている
/// （例外 0）のに、24分割のメッシュには非多様体の稜が 247 本ありました。
/// **測る対象を取り違えると、ブーリアンの責任でないものをブーリアンの
/// 赤として数えてしまいます。**
///
/// 稜は `id` ではなく**位置**で突き合わせます。同じ弧を共有していても、
/// 面ごとに別の `Edge` の実体を持つことがあるからです（4-80）。
fn non_manifold_brep_edges(solid: &Solid) -> usize {
    let quantise = |p: zenith_math::Point3| {
        let q = |v: f64| (v * 1e7).round() as i64;
        (q(p.x), q(p.y), q(p.z))
    };
    let mut uses: std::collections::HashMap<((i64, i64, i64), (i64, i64, i64)), usize> =
        std::collections::HashMap::new();
    for shell in std::iter::once(&solid.outer_shell).chain(solid.inner_shells.iter()) {
        for face in &shell.faces {
            for wire in std::iter::once(&face.outer_wire).chain(face.inner_wires.iter()) {
                for oriented in &wire.edges {
                    let a = quantise(oriented.edge.start_vertex.point);
                    let b = quantise(oriented.edge.end_vertex.point);
                    let key = if a <= b { (a, b) } else { (b, a) };
                    *uses.entry(key).or_insert(0) += 1;
                }
            }
        }
    }
    uses.values().filter(|count| **count != 2).count()
}

/// メッシュの稜のうち、ちょうど2枚の三角形に共有されていない本数。
///
/// **これは報告だけで、赤にはしません。** ブーリアンが返す立体の責任は
/// B-Rep までで、メッシュ化は別の段だからです。ただし**0 でない立体は
/// STL に書けません**ので、数は見えるところに置いておきます（4-83）。
fn non_manifold_mesh_edges(solid: &Solid) -> usize {
    let mesh = zenith_tess::tessellate_solid(
        solid,
        &zenith_tess::TessellationParams {
            u_divisions: 24,
            v_divisions: 24,
        },
    );
    let mut uses: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
    for triangle in &mesh.indices {
        for step in 0..3 {
            let (a, b) = (triangle[step], triangle[(step + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0) += 1;
        }
    }
    let bad = uses
        .iter()
        .filter(|(_, count)| **count != 2)
        .collect::<Vec<_>>();
    if std::env::var_os("ZENITH_TESS_WHY").is_some() {
        for ((a, b), count) in bad.iter().take(32) {
            let pa = mesh.positions[*a as usize];
            let pb = mesh.positions[*b as usize];
            eprintln!(
                "TESSWHY mesh non-manifold edge uses {count}: ({:.9},{:.9},{:.9}) -> ({:.9},{:.9},{:.9}), length {:.3e}",
                pa.x,
                pa.y,
                pa.z,
                pb.x,
                pb.y,
                pb.z,
                (pb - pa).norm()
            );
            for (triangle_index, triangle) in mesh.indices.iter().enumerate() {
                if !triangle.contains(a) || !triangle.contains(b) {
                    continue;
                }
                let third = triangle
                    .iter()
                    .find(|vertex| **vertex != *a && **vertex != *b)
                    .copied()
                    .unwrap_or(*a);
                let pc = mesh.positions[third as usize];
                let area = (pb - pa).cross(&(pc - pa)).norm() * 0.5;
                eprintln!(
                    "TESSWHY   triangle {triangle_index}, third ({:.9},{:.9},{:.9}), area {:.6e}",
                    pc.x, pc.y, pc.z, area
                );
            }
        }
    }
    bad.len()
}

fn shifted(solid: &Solid, x: f64, y: f64, z: f64) -> Solid {
    BrepTransform::translate_solid(solid, Vec3::new(x, y, z))
}

fn cases() -> Vec<Case> {
    let boxa = PrimitiveBuilder::make_box(20.0, 20.0, 20.0).expect("box");
    let cylinder = PrimitiveBuilder::make_cylinder(6.0, 40.0).expect("cylinder");
    let sphere = PrimitiveBuilder::make_sphere(10.0).expect("sphere");

    let mut out = Vec::new();

    // `boolean_envelope` と同じ置き方。B は (10,10,0) ずらしてから
    // **世界原点まわりに** 45 度回す（箱の中心まわりではない。そこを
    // 取り違えると 3演算とも通ってしまい、再現しない）。
    let turn = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4);
    out.push(Case {
        name: "box x box (45deg about Z)",
        why: "B の角 (10,10) が回って (0, 10√2) に来る。頂点が A の面 x=0 の上に乗る",
        a: boxa.clone(),
        b: BrepTransform::transform_solid(&shifted(&boxa, 10.0, 10.0, 0.0), &turn).expect("turn"),
        note: [
            "実測: 材料は割れない。B の稜が乗るだけ",
            "同上。原因は接触の交線ではなく面積 0 の面片だった（4-74）",
            "同上",
        ],
    });

    // 円柱が箱の側面にちょうど接する。
    out.push(Case {
        name: "box x cylinder (tangent)",
        why: "円柱の側面が箱の面に線で接する",
        a: boxa.clone(),
        // `boolean_envelope` と同じ: 半径6を中心 (6, 10) に置くと x=0 面に接する。
        b: shifted(&cylinder, 6.0, 10.0, -10.0),
        note: [
            "実測: 壁は残るので繋がったまま（非多様体という予想は外れ）",
            "実測: 壁の厚みが 0 になり材料が2つに割れる。断るのが正しい",
            "実測: 円柱の箱に入っている部分（空という予想は外れ。体積 2261.9）",
        ],
    });

    // 箱の4面が球に接し、5面目が中心を通る。
    out.push(Case {
        name: "box x sphere (four faces tangent)",
        why: "球が箱の断面に内接し、5面目（x=20）が中心を通る。**その平面は球の極 (20,10,0) と (20,10,20) を両方通る**——交線がそこで点に潰れる",
        a: boxa.clone(),
        // `boolean_envelope` と同じ置き方。**向きが効きます。** 中心を
        // (10,10,20) に置いて赤道で切ると3演算とも通りますが、それは
        // 継ぎ目を踏んでいないだけで、別の配置を測ったことになります。
        b: shifted(&sphere, 20.0, 10.0, 10.0),
        note: [
            "極で交線が点に潰れる（機構B。3-N-1）",
            "同上",
            "同上",
        ],
    });

    // **同じ「4面が接する」配置を、向きだけ変えたもの。**
    // 中心を (10,10,20) に置くと、切る平面 z=20 は赤道の面で、**極を通り
    // ません**。上の配置（x=20 で切る）は極 (20,10,0) と (20,10,20) を
    // 両方通り、交線がそこで点に潰れます。
    // 接している面の数は同じなので、**差が出るならそれは接触のせいでは
    // ありません**。
    out.push(Case {
        name: "box x sphere (cut at the equator)",
        why: "上と同じ4面接触。ただし切る平面（z=20、赤道の面）は極を通らない",
        a: boxa.clone(),
        b: shifted(&sphere, 10.0, 10.0, 20.0),
        note: ["上との違いは向きだけ", "同上", "同上"],
    });

    // **直した配置は、回帰として残します**（4-82）。球を自分の軸まわりに
    // 45 度回して、同じところで切る。継ぎ目は平面から外れますが、**極は
    // 軸の上なので平面に乗ったまま**です。交線はそこで平面に接するので、
    // 交点として着地しようとすると重根になり、位置が 2.36e-5 ずれていました。
    let spin_z = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 45f64.to_radians());
    out.push(Case {
        name: "box x sphere (spun 45 about z)",
        why: "継ぎ目は平面から外れるが、極は軸の上なので平面に乗ったまま。交線はそこで接する",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&sphere, &spin_z).expect("spin"),
            20.0,
            10.0,
            10.0,
        ),
        note: [
            "4-82 で通るようになった",
            "同上",
            "同上。半球 2094.3951 に乗る",
        ],
    });

    // **ここから下は、まだ測っていなかった置き方です。**
    // 極を切断平面から**外す**と、接する点そのものが無くなります。
    let tip = Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 1.0), 35f64.to_radians());
    out.push(Case {
        name: "box x sphere (tipped off axis)",
        why: "軸以外のまわりに回すので、極が切断平面から外れる（接する点が無くなる）",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&sphere, &tip).expect("tip"),
            20.0,
            10.0,
            10.0,
        ),
        note: ["未測定だった置き方", "同上", "同上"],
    });

    // 円柱の継ぎ目を、平面に重ならない向きへ回す。
    let spin_cyl = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 33f64.to_radians());
    out.push(Case {
        name: "box x cylinder (seam turned 33)",
        why: "円柱の継ぎ目を切断平面から外す。接する線はそのまま残る",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&cylinder, &spin_cyl).expect("turn"),
            6.0,
            10.0,
            -10.0,
        ),
        note: [
            "実測: 立体1つ、多様体",
            "実測: 場所を名指しして断る（(0,10,...) で材料が2つに割れる）。**規約どおり**——接する線が残るので、差では壁の厚みが 0 になる",
            "実測: 立体1つ、多様体",
        ],
    });

    // 円錐（頂点 apex あり）を箱で斜め45度に切断
    let cone = PrimitiveBuilder::make_cone(10.0, 0.0, 20.0).expect("cone");
    let tilt_cone = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 45f64.to_radians());
    out.push(Case {
        name: "box x cone (apex tilted 45deg)",
        why: "真の頂点を持つ円錐を45度傾けて箱と交差させる。頂点・母線・底面円弧が平面と斜めに交わる",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&cone, &tilt_cone).expect("tilt cone"),
            10.0,
            10.0,
            -5.0,
        ),
        note: [
            "実測: B-Rep多様体、メッシュ3本非多様体（4-83）",
            "実測: B-Rep多様体、メッシュ多様体（0本）",
            "実測: B-Rep多様体、メッシュ14本非多様体（4-83）",
        ],
    });

    // 直交する2本の円柱（パイプ交差・十字分岐）
    let cyl_x = Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 90f64.to_radians());
    out.push(Case {
        name: "cylinder x cylinder (orthogonal cross)",
        why: "Z軸円柱とX軸円柱を直交させて交差。曲面同士の鞍部交線が生じる",
        a: shifted(&cylinder, 0.0, 0.0, -20.0),
        b: shifted(
            &BrepTransform::transform_solid(&cylinder, &cyl_x).expect("turn x"),
            -20.0,
            0.0,
            0.0,
        ),
        note: [
            "実測: 曲面同士の直交交差。現在は未実装として拒否（規約遵守）",
            "同上",
            "同上",
        ],
    });

    // 球と円柱の交差（偏心）
    out.push(Case {
        name: "sphere x cylinder (eccentric intersection)",
        why: "球と円柱を偏心させて交差。曲面同士の非対称交線",
        a: sphere.clone(),
        b: shifted(&cylinder, 4.0, 0.0, -20.0),
        note: [
            "実測: 球と円柱の偏心交差。現在は未実装として拒否（規約遵守）",
            "同上",
            "同上",
        ],
    });

    // トーラスを傾けて箱で切断
    let torus = PrimitiveBuilder::make_torus(12.0, 4.0).expect("torus");
    let tilt_torus = Transform3::from_axis_angle(&Vec3::new(1.0, 1.0, 0.0), 25f64.to_radians());
    out.push(Case {
        name: "box x torus (inclined 25deg)",
        why: "16パッチのトーラスを25度傾けて箱と交差。複数パッチにまたがる楕円状ループ交線",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&torus, &tilt_torus).expect("tilt torus"),
            10.0,
            10.0,
            10.0,
        ),
        note: [
            "実測: B-Rep多様体、メッシュ168本非多様体（4-83）",
            "実測: B-Rep多様体、メッシュ123本非多様体（4-83）",
            "実測: B-Rep多様体、メッシュ122本非多様体（4-83）",
        ],
    });

    // **ここから下は 3-N-2 が名指しで求めていた置き方です**（2026/08/27）。
    // 「同じ形を置き方だけ変えて測る」が、いまのところいちばん当たる手だと
    // 書いてあるので、そのとおりにしました。

    // 切る側と切られる側を**両方**回す。どちらか片方だけを回した配置は
    // 上にありますが、両方回すと、箱の面ももう軸に平行ではなくなります。
    let spin_box = Transform3::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 19f64.to_radians());
    let tilt_cyl = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 27f64.to_radians());
    out.push(Case {
        name: "box x cylinder (both turned)",
        why: "箱を 19 度、円柱を 27 度回す。切る面も切られる面も軸に平行でなくなる",
        a: BrepTransform::transform_solid(&boxa, &spin_box).expect("spin box"),
        b: shifted(
            &BrepTransform::transform_solid(&cylinder, &tilt_cyl).expect("tilt cylinder"),
            6.0,
            10.0,
            -10.0,
        ),
        note: ["未測定だった置き方", "同上", "同上"],
    });

    // 円錐を、軸のまわりにも回してから傾ける。上の `apex tilted 45` は
    // 傾けるだけなので、**継ぎ目は切断平面に対して同じ位置**にあります。
    let spin_then_tilt = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 45f64.to_radians())
        .compose(&Transform3::from_axis_angle(
            &Vec3::new(0.0, 0.0, 1.0),
            37f64.to_radians(),
        ));
    out.push(Case {
        name: "box x cone (seam spun then tilted)",
        why: "円錐を軸まわりに 37 度回してから 45 度傾ける。継ぎ目が切断平面から外れる",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&cone, &spin_then_tilt).expect("spin then tilt"),
            10.0,
            10.0,
            -5.0,
        ),
        note: ["未測定だった置き方", "同上", "同上"],
    });

    // トーラスを、穴の軸が箱の稜を向くように回す。上の `inclined 25` は
    // 1軸まわりなので、穴の軸は座標平面の中に残っています。
    let torus_two_axes = Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 41f64.to_radians())
        .compose(&Transform3::from_axis_angle(
            &Vec3::new(1.0, 0.0, 0.0),
            33f64.to_radians(),
        ));
    out.push(Case {
        name: "box x torus (two axes)",
        why: "トーラスを2軸で回す。穴の軸が座標平面から外れ、継ぎ目も切断面から外れる",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&torus, &torus_two_axes).expect("turn torus"),
            10.0,
            10.0,
            10.0,
        ),
        note: ["未測定だった置き方", "同上", "同上"],
    });

    // 球を、極が切断平面のどこにも来ない角度で回して、**角**で当てる。
    // 上の球の配置はどれも面で切っています。
    let sphere_corner = Transform3::from_axis_angle(&Vec3::new(1.0, 2.0, 3.0), 51f64.to_radians());
    out.push(Case {
        name: "box x sphere (at a corner)",
        why: "球を箱の角に当てる。3枚の面が同時に切る（面1枚で切る上の配置とは別）",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&sphere, &sphere_corner).expect("turn sphere"),
            20.0,
            20.0,
            20.0,
        ),
        note: ["未測定だった置き方", "同上", "同上"],
    });

    // **ここから下は 2026/08/27 に足した「触っているだけ」の置き方です。**
    //
    // 上の配置はどれも、接触の**まわりで実際に交わって**います。ここでは
    // 接触そのものが答えの形を決める置き方——点で触る、線で触る、面で
    // 触る——を測ります。規約（この文書の冒頭）では、真の答えが非多様体
    // なら**断るのが正しい**ので、`REFUSED` は赤ではありません。**赤に
    // なるのは、非多様体を立体として返したときだけです。**

    // 点で触る。球の最下点が箱の上面にちょうど乗る。
    // 真の union は**1点でだけ繋がった2つの塊**——非多様体です。
    out.push(Case {
        name: "box x sphere (touching at one point)",
        why: "球の最下点 (10,10,20) が箱の上面 z=20 にちょうど乗る。接するのは1点だけ",
        a: boxa.clone(),
        b: shifted(&sphere, 10.0, 10.0, 30.0),
        note: [
            "実測: 立体2つ。接触は位相を作らないので繋がらない",
            "実測: 箱がそのまま返る（立体1つ）",
            "実測: 立体0つ。1点は体積を持たない",
        ],
    });

    // 面で触る。円柱の底の円板が、箱の上面とちょうど同一平面に乗る。
    // 真の union は**円板1枚で繋がった立体**——これは多様体です。
    out.push(Case {
        name: "box x cylinder (cap coplanar with a face)",
        why: "円柱の底面 z=20 が箱の上面 z=20 と同一平面。円板の面積ぶんで触れる",
        a: boxa.clone(),
        b: shifted(&cylinder, 10.0, 10.0, 20.0),
        note: [
            "実測: 立体1つ。円板1枚で繋がる（これは多様体）",
            "実測: 箱がそのまま返る（立体1つ）",
            "実測: 立体0つ。円板は体積を持たない",
        ],
    });

    // 点で触る（頂点）。円錐の真の頂点が、箱の上面にちょうど乗る。
    // 球の点接触と違い、**触っている側が特異点**です。
    out.push(Case {
        name: "box x cone (apex on a face)",
        why: "円錐の頂点 (10,10,20) が箱の上面 z=20 にちょうど乗る。触る側が特異点",
        a: boxa.clone(),
        b: shifted(&cone, 10.0, 10.0, 0.0),
        note: [
            "実測: 箱そのもの（体積 8000）。円錐は内接している",
            "実測: 8000 - 2094.395102",
            "実測: 円錐そのもの（2094.395102 = πr²h/3 に一致）。4-127 までは断られていた",
        ],
    });

    // 線で触る（円1本）。トーラスの上の接平面が、箱の上面と一致する。
    // 触っているのは**半径 12 の円まるごと**です。
    out.push(Case {
        name: "box x torus (top tangent plane)",
        why: "トーラスの上端 z=20 が箱の上面と一致。半径 12 の円まるごとで触れる",
        a: boxa.clone(),
        b: shifted(&torus, 10.0, 10.0, 16.0),
        note: [
            "実測: 未実装として断られる（あぶれた稜 48、非多様体の稜 24）",
            "実測: 場所を名指しして断る（(19.29,17.59,20) で材料が2つに割れる）。**規約どおり**",
            "実測: 未実装として断られる（あぶれた稜 56）",
        ],
    });

    // 線で触る（母線1本）。円錐を、母線が鉛直になる角度だけ回す。
    // 半頂角は atan(10/20) なので、Y まわりに同じ角だけ回すと、母線の1本が
    // 鉛直になり、そこで x が最大 20/√5 になります。その距離だけずらすと、
    // **母線がまるごと箱の面 x=20 の中に乗ります。**
    let half_angle = (10f64 / 20.0).atan();
    let stand_cone = Transform3::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), half_angle);
    let generatrix_x = 20.0 / 5f64.sqrt();
    out.push(Case {
        name: "box x cone (generatrix in a face)",
        why: "円錐を半頂角だけ回すと母線が鉛直になる。その母線が箱の面 x=20 の中にまるごと乗る",
        a: boxa.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&cone, &stand_cone).expect("stand cone"),
            20.0 - generatrix_x,
            10.0,
            5.0,
        ),
        note: [
            "実測: 立体1つ、多様体。恒等式が閉じる",
            "同上",
            "実測: 立体1つ、多様体。4-127 までは断られていた",
        ],
    });

    // **ここから下は 2026/08/27 に足した「曲面どうし」の置き方です**（4-131）。
    //
    // 上の配置は、曲面が触れる相手が**平面**（箱の面）ばかりでした。
    // 4-128 で曲面どうしの交差が通るようになったので、**曲面どうしが
    // 触れている**ほうを測ります。4-130 の「次に測るなら」に書いた3つ
    // （接する／どちらもパッチに割れている／寸法比だけ変える）です。

    // 点で触る（曲面どうし）。球2つを外接させる。
    // 真の union は1点でだけ繋がった2つの塊——非多様体です。
    out.push(Case {
        name: "sphere x sphere (externally tangent)",
        why: "半径 10 の球2つを 20 離す。(10,0,0) の1点だけで触れる",
        a: sphere.clone(),
        b: shifted(&sphere, 20.0, 0.0, 0.0),
        note: [
            "実測: 立体2つ。接触は位相を作らないので繋がらない",
            "実測: A がそのまま返る（立体1つ）",
            "実測: 立体0つ。1点は体積を持たない",
        ],
    });

    // 線で触る（曲面どうし）。平行な円柱2本を、側面がちょうど接するまで寄せる。
    out.push(Case {
        name: "cylinder x cylinder (tangent along a line)",
        why: "半径 6 の円柱2本を 12 離す。母線 x=6, y=0 の1本で触れる",
        a: shifted(&cylinder, 0.0, 0.0, -20.0),
        b: shifted(&cylinder, 12.0, 0.0, -20.0),
        note: [
            "実測: 断られる。**答えが本当に非多様体**（母線1本を4枚の面が共有する）ので断るのが正しい。ただし**断り文が「未実装」になっている**——場所を名指しすべき（4-131。縫合は非多様体の稜 20、あぶれ 0 と、そこまで測れている）",
            "実測: A がそのまま返る（立体1つ）",
            "実測: 立体0つ。線は体積を持たない",
        ],
    });

    // 円で触る（曲面どうし）。球と、同じ半径の同軸円柱。赤道の円で接する。
    // **4-130 のトーラスと同じ「曲線に沿った接触」ですが、相手が平面では
    // なく曲面です。**
    let wide_cylinder = PrimitiveBuilder::make_cylinder(10.0, 40.0).expect("wide cylinder");
    out.push(Case {
        name: "sphere x cylinder (tangent along the equator)",
        why: "半径 10 の球と、同じ半径の同軸円柱。赤道の円まるごとで触れる（曲面どうしの曲線接触）",
        a: sphere.clone(),
        b: shifted(&wide_cylinder, 0.0, 0.0, -20.0),
        note: [
            "実測: 円柱がそのまま返る（球は中に入っている）",
            "実測: 立体0つ（球は円柱に含まれる）",
            "実測: 球がそのまま返る",
        ],
    });

    // **寸法比だけを変える。** 4-128 で分かったのは「等半径だけが落ちて
    // いた」ことでした。ここは**ほとんど等しい**半径で当てます。交線は
    // 閉じた輪1本のはずですが、接点のすぐ近くを通ります。
    let nearly = PrimitiveBuilder::make_cylinder(5.9, 40.0).expect("nearly equal cylinder");
    out.push(Case {
        name: "cylinder x cylinder (crossed, nearly equal radii)",
        why: "直交2円柱の半径を 6 と 5.9 にする。交線は輪1本だが、接点のすぐ近くを通る",
        a: shifted(&cylinder, 0.0, 0.0, -20.0),
        b: shifted(
            &BrepTransform::transform_solid(&nearly, &cyl_x).expect("turn x"),
            -20.0,
            0.0,
            0.0,
        ),
        note: [
            "実測: 3演算とも多様体。**等半径だけが落ちていた**（4-128）ことの裏取り",
            "同上",
            "同上",
        ],
    });

    // **どちらもパッチに割れている曲面どうし。** トーラスの穴の中を、
    // 管に食い込む太さの円柱が通る。
    let bore = PrimitiveBuilder::make_cylinder(9.0, 40.0).expect("bore");
    out.push(Case {
        name: "torus x cylinder (through the tube)",
        why: "トーラス（穴 8、外 16）の中を半径 9 の円柱が通る。管に食い込む。どちらもパッチに割れている",
        a: torus.clone(),
        b: shifted(&bore, 0.0, 0.0, -20.0),
        note: [
            "実測: 3演算とも多様体。**4-131 の前は 74 / 108 / 64 本のメッシュ非多様体**だった（細分が境界の点の 1.7e-7 のところに点を作り、面積 1e-8 の薄片が残っていた）",
            "同上（108 本 → 0）",
            "同上（64 本 → 0）",
        ],
    });

    // **ここから下は 2026/08/27 に足した「まだ1件も無かった組み合わせ」です**
    // （4-132）。
    //
    // ここまでの配置を数えたら、**曲面の組み合わせに穴がありました**——
    // 円錐×円柱、円錐×円錐、トーラス×球、トーラス×トーラスが1件も
    // ありません。実務では面取り穴やテーパのボスで普通に出ます。
    // **「置き方」だけでなく「組み合わせ」も測っていない、が抜けていました。**

    // 円錐と円柱を直交させる。テーパの穴に横穴を開ける形。
    out.push(Case {
        name: "cone x cylinder (crossed)",
        why: "円錐（底 10、高さ 20）に、半径 4 の円柱を直交させて通す。母線と円柱の交線は片方だけが直線",
        a: cone.clone(),
        b: shifted(
            &BrepTransform::transform_solid(
                &PrimitiveBuilder::make_cylinder(4.0, 40.0).expect("pin"),
                &cyl_x,
            )
            .expect("turn x"),
            -20.0,
            0.0,
            6.0,
        ),
        note: [
            "実測: 3演算とも多様体",
            "同上",
            "同上",
        ],
    });

    // 円錐2つを頂点どうしで合わせる。**触っているのは特異点どうし**です。
    let flipped_cone = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 180f64.to_radians());
    out.push(Case {
        name: "cone x cone (apex to apex)",
        why: "円錐2つを頂点どうしで合わせる。触っているのは特異点どうし1点だけ",
        a: cone.clone(),
        b: shifted(
            &BrepTransform::transform_solid(&cone, &flipped_cone).expect("flip"),
            0.0,
            0.0,
            40.0,
        ),
        note: [
            "実測: 立体2つ。接触は位相を作らないので繋がらない",
            "実測: A がそのまま返る（立体1つ）",
            "実測: 立体0つ。1点は体積を持たない",
        ],
    });

    // 円錐と球。球を円錐の中に落として、側面に**円で内接**させる。
    // 半頂角 atan(10/20) の円錐に半径 r の球を軸上で内接させると、
    // 中心の高さは頂点から r/sin(半頂角) 下がったところ。
    let half = (10f64 / 20.0).atan();
    let inscribed_radius = 3.0;
    out.push(Case {
        name: "cone x sphere (inscribed, tangent along a circle)",
        why: "円錐の内側に球を内接させる。側面と円まるごとで触れる（曲面どうしの曲線接触）",
        a: cone.clone(),
        b: shifted(
            &sphere_of(inscribed_radius),
            0.0,
            0.0,
            20.0 - inscribed_radius / half.sin(),
        ),
        note: [
            "実測: 円錐がそのまま返る（球は中に入っている）",
            "実測: **検証ゲートが誤答を捕まえて断る**——体積が A + B（2207.492438 = 2094.395102 + 113.097336）になっている。差なのに球が凸として足されている（4-132。未解決）",
            "実測: 球がそのまま返る",
        ],
    });

    // トーラスと球。球をトーラスの管に食い込ませる。
    out.push(Case {
        name: "torus x sphere (biting the tube)",
        why: "トーラス（芯 12、管 4）の管に、半径 6 の球を食い込ませる。どちらもパッチに割れている",
        a: torus.clone(),
        b: shifted(&sphere_of(6.0), 12.0, 0.0, 0.0),
        note: [
            "実測: 3演算とも多様体",
            "同上",
            "同上",
        ],
    });

    // トーラス2つを、軸を直交させて絡ませる。**いちばんパッチが多い組**です。
    let torus_upright = Transform3::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 90f64.to_radians());
    out.push(Case {
        name: "torus x torus (crossed axes)",
        why: "トーラス2つの軸を直交させ、中心を揃える。芯の円が (±12,0,0) で交わるので、管どうしがそこで食い合う。パッチが 16 x 16 で当たる",
        a: torus.clone(),
        b: BrepTransform::transform_solid(&torus, &torus_upright).expect("upright"),
        note: [
            "実測: 未実装として断られる（交線16本、あぶれ14、非多様体0）。あぶれは端点のずれではなく**相手のいない弧**で、すべて接点 (±16,0,0)/(±8,0,0) に接している（4-132。未解決）",
            "同上",
            "同上",
        ],
    });

    out
}

/// 半径を指定した球。**`sphere` は半径 10 固定**なので、別の大きさが要る
/// ところはここから作ります。
fn sphere_of(radius: f64) -> Solid {
    PrimitiveBuilder::make_sphere(radius).expect("sphere")
}

fn main() {
    let tol = Tolerance::default();
    let case_filter = std::env::var("ZENITH_CONTACT_FILTER").ok();
    let op_filter = std::env::var("ZENITH_CONTACT_OP").ok();
    let mut wrong = 0usize;
    let mut returned = 0usize;
    let mut refused = 0usize;
    let mut mesh_broken = 0usize;

    println!("接触している配置（規約: 接触は、それ自体では位相を作らない）");
    println!();
    println!(
        "{:<30} {:<13} {:>9} {:>7} {:>13}  {}",
        "case", "op", "result", "solids", "nm brep/mesh", "verdict / 実測"
    );
    println!("{}", "-".repeat(118));

    for case in cases().into_iter().filter(|case| {
        case_filter
            .as_deref()
            .map(|needle| case.name.contains(needle))
            .unwrap_or(true)
    }) {
        for (index, (label, op)) in [
            ("union", BooleanOpType::Union),
            ("difference", BooleanOpType::Difference),
            ("intersection", BooleanOpType::Intersection),
        ]
        .into_iter()
        .enumerate()
        .filter(|(_, (label, _))| {
            op_filter
                .as_deref()
                .map(|needle| label.contains(needle))
                .unwrap_or(true)
        }) {
            let outcome = BooleanEngine::boolean_solids_exact_result(&case.a, &case.b, op, &tol);

            let (result, solids, bad_edges, verdict) = match &outcome {
                Err(reason) => {
                    refused += 1;
                    if std::env::var_os("ZENITH_REFUSE_WHY").is_some() {
                        eprintln!("REFUSEWHY {} {label}: {reason}", case.name);
                    }
                    (
                        "REFUSED",
                        "-".to_string(),
                        "-".to_string(),
                        // 断るのは、まだ実装していないだけのことがあります。
                        // ここでは赤にしません。
                        format!("断られた（{}）", case.note[index]),
                    )
                }
                Ok(result) => {
                    returned += 1;
                    // **返ってきたものを測ります。** 稜がちょうど2枚の三角形に
                    // 共有されていなければ、それは多様体ではありません。
                    let bad: usize = result.solids.iter().map(non_manifold_brep_edges).sum();
                    let mesh_bad: usize = result.solids.iter().map(non_manifold_mesh_edges).sum();
                    if bad > 0 {
                        wrong += 1;
                    }
                    if mesh_bad > 0 {
                        mesh_broken += 1;
                    }
                    (
                        "ok",
                        result.solids.len().to_string(),
                        format!("{bad} / {mesh_bad}"),
                        if bad > 0 {
                            format!("WRONG: 非多様体を立体として返した（{}）", case.note[index])
                        } else if mesh_bad > 0 {
                            format!(
                                "ok, B-Rep は多様体。**メッシュが非多様体（{mesh_bad} 本）**——4-83"
                            )
                        } else {
                            format!("ok, 多様体（{}）", case.note[index])
                        },
                    )
                }
            };

            println!(
                "{:<30} {:<13} {:>9} {:>7} {:>13}  {}",
                if index == 0 { case.name } else { "" },
                label,
                result,
                solids,
                bad_edges,
                verdict
            );
        }
        println!("{:<30} {}", "", case.why);
        println!();
    }

    println!("{}", "-".repeat(118));
    println!(
        "{returned} 件が立体を返し、{refused} 件が断られました。**B-Rep が非多様体だったもの {wrong} 件**、メッシュが非多様体だったもの {mesh_broken} 件。"
    );
    println!();
    println!("**断られること自体は、ここでは赤にしません。** 断り方は2種類あります——");
    println!("答えが本当に非多様体で断るもの（円柱の接線の差。場所を名指しします）と、");
    println!("まだ実装していないもの（球の極。3-N-1）です。赤にするのは「非多様体を");
    println!("立体として返した」ほうだけです。括弧の中は**実測**です（4-74）。");

    // B-Rep が非多様体だった場合は絶対に許容しない（即座に赤にする）。
    if wrong > 0 {
        eprintln!("GATE ERROR: B-Rep non-manifold edges detected: {wrong} cases");
        std::process::exit(1);
    }
}
