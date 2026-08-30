//! 球を平面で切った立体の、**円い縁**の厳密フィレット／面取り。
//!
//! # なぜ別に要るのか
//!
//! 円い縁のフィレットは、円柱の蓋（4-92）、円錐の蓋（4-94）、貫通穴の口
//! （4-95）、円筒ボスの根元（4-97）まで来ていました。**球だけが残って
//! いました**——`blend_coverage_probe` は `sphere_capped` を「丸める稜が
//! 4本あるのに 0本」と数えます。断り文は `Edge 5 is not a straight line`
//! で、汎用の経路が直線の稜しか受け取らないためです。
//!
//! # 何が厳密なのか
//!
//! 転がる球（半径 `r`）の中心は、
//!
//! - 蓋の平面から `r` 下がったところ（平面に接する）
//! - 球の中心から `R - r` のところ（球に内接する）
//!
//! の両方を満たします。この2条件の交わりは**軸まわりの円**なので、接触の
//! 断面円を回した**厳密なトーラス**がフィレット面になります。近似は
//! どこにも入りません。有理2次で書けるので、角度方向も断面方向も厳密です。
//!
//! 面取りも同じ枠で、接触の断面が円弧ではなく**直線**になるだけです
//! （厳密な円錐台）。
//!
//! # 局所の記号
//!
//! 縁の円の中心を原点、蓋の外向き法線を `+z` とします（材料は `z <= 0`）。
//!
//! | 記号 | 意味 |
//! | :--- | :--- |
//! | `R` | 球の半径 |
//! | `s` | 球の中心の高さ。半球なら `0` |
//! | `rim` | 縁の半径。`rim^2 + s^2 = R^2` |
//! | `r` | フィレット半径 |
//! | `a` | 転がる球の中心が描く円の半径。`a^2 = (R-r)^2 - (r+s)^2` |
//!
//! 接点は、平面側が `(a, 0)`、球側が `(a R/(R-r), s - R(r+s)/(R-r))` です。
//!
//! # 上限
//!
//! `(r + s) < (R - r)`、つまり **`r < (R - s)/2`**。ここを超えると転がる球が
//! 球面の反対側へ抜けます。

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, SQRT_2, TAU};

use zenith_geom::{ControlPoint3, KnotVector, NurbsCurve3, NurbsSurface3, PlaneSurface3};
use zenith_math::{Point3, Transform3, Vec3, Vec3Ext};
use zenith_topo::{Edge, Face, FaceGeometry, OrientedEdge, Shell, Solid, Vertex, Wire};

use crate::{BlendableEdge, BrepTransform, EdgeBlendReport};

use super::{circle_from_edge, effective_plane_normal, same_edge_geometry};

pub(crate) fn try_fillet_sphere_cap_rim(
    solid: &Solid,
    edge_id: u64,
    fillet_radius: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = PureSphereCapRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let geometry = SphereCapBlend::fillet(site.r_sphere, site.centre_height, fillet_radius)?;
    let canonical = build_blended_sphere_cap(&geometry)?;
    let result = BrepTransform::transform_solid(&canonical, &site.canonical_to_world())?;

    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: site.dihedral().to_degrees(),
            edge_length: TAU * site.rim_radius,
            setback: site.rim_radius - geometry.plane_radius,
            predicted_removed_volume: geometry.removed_volume(),
        },
    )))
}

pub(crate) fn try_chamfer_sphere_cap_rim(
    solid: &Solid,
    edge_id: u64,
    distance: f64,
) -> Result<Option<(Solid, EdgeBlendReport)>, String> {
    let Some(site) = PureSphereCapRim::recognize(solid, edge_id) else {
        return Ok(None);
    };
    let geometry = SphereCapBlend::chamfer(site.r_sphere, site.centre_height, distance)?;
    let canonical = build_blended_sphere_cap(&geometry)?;
    let result = BrepTransform::transform_solid(&canonical, &site.canonical_to_world())?;

    Ok(Some((
        result,
        EdgeBlendReport {
            dihedral_angle_deg: site.dihedral().to_degrees(),
            edge_length: TAU * site.rim_radius,
            setback: distance,
            predicted_removed_volume: geometry.removed_volume(),
        },
    )))
}

pub(crate) fn sphere_cap_rim_blendable(solid: &Solid, edge_id: u64) -> Option<BlendableEdge> {
    let site = PureSphereCapRim::recognize(solid, edge_id)?;
    // r < (R - s)/2 が上限。触れるだけになる手前で止めます。
    let max_fillet = (site.r_sphere - site.centre_height) * 0.5 * 0.999;
    // 面取りは、縁を全部食い切る手前まで。平面側は半径方向に、球側は子午線の
    // 弧長で下がるので、効くのは小さいほうです。
    let max_chamfer = site
        .rim_radius
        .min(site.r_sphere * site.polar_span())
        * 0.999;
    if !(max_fillet > 1e-6 && max_fillet.is_finite() && max_chamfer > 1e-6) {
        return None;
    }
    Some(BlendableEdge {
        edge_id,
        length: TAU * site.rim_radius,
        dihedral_angle_deg: site.dihedral().to_degrees(),
        max_fillet_radius: max_fillet,
        max_chamfer_distance: max_chamfer,
    })
}

/// 球を平面で切っただけの立体と、その縁。
///
/// **「純粋な」形しか受け取りません。** 面が2枚（球1枚と蓋1枚）で、穴も
/// 空洞も無いものだけです。読んだ立体を作り直すので、余計なものが付いて
/// いると作り直しで落ちます。
struct PureSphereCapRim {
    rim_centre: Point3,
    outward_axis: Vec3,
    x_axis: Vec3,
    rim_radius: f64,
    r_sphere: f64,
    /// 球の中心の高さ。縁の中心を原点、`outward_axis` を `+z` として測ります。
    centre_height: f64,
}

impl PureSphereCapRim {
    fn recognize(solid: &Solid, edge_id: u64) -> Option<Self> {
        if !solid.inner_shells.is_empty()
            || solid
                .outer_shell
                .faces
                .iter()
                .any(|face| !face.inner_wires.is_empty())
        {
            return None;
        }
        let faces = &solid.outer_shell.faces;
        if faces.len() != 2 {
            return None;
        }
        let selected = faces
            .iter()
            .flat_map(|face| face.outer_wire.edges.iter())
            .find(|oriented| oriented.edge.id == edge_id)?
            .edge
            .clone();

        let uses: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter_map(|(index, face)| {
                face.outer_wire
                    .edges
                    .iter()
                    .any(|oriented| same_edge_geometry(&selected, &oriented.edge))
                    .then_some(index)
            })
            .collect();
        if uses.len() != 2 {
            return None;
        }

        let cap_index = uses
            .iter()
            .copied()
            .find(|index| matches!(faces[*index].geometry, FaceGeometry::Plane(_)))?;
        let sphere_index = uses
            .iter()
            .copied()
            .find(|index| matches!(faces[*index].geometry, FaceGeometry::Nurbs(_)))?;

        let cap = &faces[cap_index];
        let FaceGeometry::Plane(cap_plane) = &cap.geometry else {
            return None;
        };
        let (rim_centre, rim_radius, x_axis) = circle_from_edge(&selected)?;
        let scale = rim_radius.max(1.0);
        if (rim_centre - cap_plane.origin).dot(&cap_plane.normal).abs() > 1e-7 * scale {
            return None;
        }
        let outward_axis = effective_plane_normal(cap).try_normalize_safe(1e-12)?;

        // 球の中心は軸の上にあります。面の上の標本から高さ `s` を解きます。
        //
        //   rho^2 + (z - s)^2 = R^2  かつ  rim^2 + s^2 = R^2
        //   => s = (rho^2 + z^2 - rim^2) / (2 z)
        //
        // 標本ごとに解いて、揃っていなければ球ではありません。
        let samples = surface_samples(&faces[sphere_index], 7);
        if samples.len() < 9 {
            return None;
        }
        let height = |point: Point3| (point - rim_centre).dot(&outward_axis);
        let radial = |point: Point3| {
            let offset = point - rim_centre;
            (offset - outward_axis * offset.dot(&outward_axis)).norm()
        };

        let mut solved: Option<f64> = None;
        for point in &samples {
            let z = height(*point);
            // 縁のすぐそばの標本は、割り算が効きません。飛ばします。
            if z.abs() <= 1e-3 * scale {
                continue;
            }
            let rho = radial(*point);
            let candidate = (rho * rho + z * z - rim_radius * rim_radius) / (2.0 * z);
            match solved {
                None => solved = Some(candidate),
                Some(existing) => {
                    if (existing - candidate).abs() > 1e-6 * scale {
                        return None;
                    }
                }
            }
        }
        let centre_height = solved?;
        let r_sphere = rim_radius.hypot(centre_height);
        if !(r_sphere > 1e-6) {
            return None;
        }

        // 標本が本当に球の上にあるか、材料の側にあるかを見ます。
        let sphere_centre = rim_centre + outward_axis * centre_height;
        for point in &samples {
            if ((point - sphere_centre).norm() - r_sphere).abs() > 1e-6 * scale {
                return None;
            }
            if height(*point) > 1e-6 * scale {
                // 蓋の外側へ出ている面は、この形ではありません。
                return None;
            }
        }
        // 残っている極が材料の側にあること。
        if centre_height - r_sphere >= -1e-6 * scale {
            return None;
        }

        Some(Self {
            rim_centre,
            outward_axis,
            x_axis,
            rim_radius,
            r_sphere,
            centre_height,
        })
    }

    /// 内側から測った二面角。
    ///
    /// 球の接平面と蓋の平面のなす角です。縁での球の法線は軸から
    /// `acos(-s/R)` 傾いているので、二面角はその補角になります。
    fn dihedral(&self) -> f64 {
        // 外向き法線どうしのなす角の補角。縁での球の法線は `(rim, 0, -s)/R`、
        // 蓋の法線は `(0, 0, 1)` なので、内積は `-s/R` です。
        // 半球（`s = 0`）なら 90 度になります。
        PI - (-self.centre_height / self.r_sphere).clamp(-1.0, 1.0).acos()
    }

    /// 残っている球面が、極から縁まで張る中心角。
    fn polar_span(&self) -> f64 {
        PI - (-self.centre_height / self.r_sphere).clamp(-1.0, 1.0).acos()
    }

    fn canonical_to_world(&self) -> Transform3 {
        let z = self.outward_axis;
        let x = self.x_axis;
        let y = z.cross(&x).normalize();
        let mut matrix = nalgebra::Matrix4::identity();
        for row in 0..3 {
            matrix[(row, 0)] = x[row];
            matrix[(row, 1)] = y[row];
            matrix[(row, 2)] = z[row];
            matrix[(row, 3)] = self.rim_centre[row];
        }
        Transform3 { matrix }
    }
}

/// 面の上の標本点。トリムは見ず、パラメータ矩形を格子で撒きます。
fn surface_samples(face: &Face, steps: usize) -> Vec<Point3> {
    let FaceGeometry::Nurbs(surface) = &face.geometry else {
        return Vec::new();
    };
    let ((u_min, u_max), (v_min, v_max)) = zenith_geom::Surface3::param_range(surface);
    let mut out = Vec::with_capacity((steps + 1) * (steps + 1));
    for i in 0..=steps {
        let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
        for j in 0..=steps {
            let v = v_min + (v_max - v_min) * j as f64 / steps as f64;
            out.push(zenith_geom::Surface3::evaluate(surface, u, v));
        }
    }
    out
}

/// 縁を丸めた（または面取りした）あとの断面。
///
/// 断面は「球の弧 → つなぎ → 蓋の直線」の3本で、つなぎがフィレットなら
/// 円弧、面取りなら直線です。どちらも有理2次で厳密に書けます。
struct SphereCapBlend {
    r_sphere: f64,
    centre_height: f64,
    /// 球側の接点。
    sphere_radius: f64,
    sphere_height: f64,
    /// 蓋側の接点（新しい蓋の半径）。
    plane_radius: f64,
    /// つなぎの断面。
    profile: BlendProfile,
}

enum BlendProfile {
    /// 中心 `(centre_radius, centre_height)`、半径 `r` の円弧。
    Arc {
        centre_radius: f64,
        centre_height: f64,
        radius: f64,
    },
    /// 直線（面取り）。
    Line,
}

impl SphereCapBlend {
    fn fillet(r_sphere: f64, centre_height: f64, fillet: f64) -> Result<Self, String> {
        if !(fillet > 0.0 && fillet.is_finite()) {
            return Err(format!(
                "Sphere cap fillet radius must be positive, got {fillet}"
            ));
        }
        let scale = r_sphere.max(1.0);
        let limit = (r_sphere - centre_height) * 0.5;
        if fillet >= limit - 1e-9 * scale {
            return Err(format!(
                "Sphere cap fillet radius {fillet} reaches the far side of the sphere \
                 (the limit here is {limit:.6})"
            ));
        }
        let ring = r_sphere - fillet;
        let offset = fillet + centre_height;
        let a_squared = ring * ring - offset * offset;
        if a_squared <= 1e-12 * scale * scale {
            return Err(format!(
                "Sphere cap fillet radius {fillet} collapses the rolling-ball circle"
            ));
        }
        let a = a_squared.sqrt();
        Ok(Self {
            r_sphere,
            centre_height,
            sphere_radius: a * r_sphere / ring,
            sphere_height: centre_height - r_sphere * offset / ring,
            plane_radius: a,
            profile: BlendProfile::Arc {
                centre_radius: a,
                centre_height: -fillet,
                radius: fillet,
            },
        })
    }

    fn chamfer(r_sphere: f64, centre_height: f64, distance: f64) -> Result<Self, String> {
        if !(distance > 0.0 && distance.is_finite()) {
            return Err(format!(
                "Sphere cap chamfer distance must be positive, got {distance}"
            ));
        }
        let rim_radius = (r_sphere * r_sphere - centre_height * centre_height)
            .max(0.0)
            .sqrt();
        let scale = r_sphere.max(1.0);
        let plane_radius = rim_radius - distance;
        if plane_radius <= 1e-9 * scale {
            return Err(format!(
                "Sphere cap chamfer distance {distance} eats the whole cap \
                 (the rim radius is {rim_radius:.6})"
            ));
        }
        // 球側は**子午線の弧長**で下がります。円錐の面取りが母線に沿って
        // 下がるのと同じ考え方で、球では母線が弧になるだけです。
        let rim_polar = (-centre_height / r_sphere).clamp(-1.0, 1.0).acos();
        let span = PI - rim_polar;
        let step = distance / r_sphere;
        if step >= span - 1e-9 {
            return Err(format!(
                "Sphere cap chamfer distance {distance} runs past the pole \
                 (the meridian only has {:.6} of arc)",
                span * r_sphere
            ));
        }
        let polar = rim_polar + step;
        Ok(Self {
            r_sphere,
            centre_height,
            sphere_radius: r_sphere * polar.sin(),
            sphere_height: centre_height + r_sphere * polar.cos(),
            plane_radius,
            profile: BlendProfile::Line,
        })
    }

    /// つなぎの断面の、有理2次の中間制御点と重み。
    fn profile_control(&self) -> ((f64, f64), f64) {
        match self.profile {
            BlendProfile::Line => (
                (
                    (self.sphere_radius + self.plane_radius) * 0.5,
                    self.sphere_height * 0.5,
                ),
                1.0,
            ),
            BlendProfile::Arc {
                centre_radius,
                centre_height,
                radius,
            } => {
                let from = (
                    self.sphere_radius - centre_radius,
                    self.sphere_height - centre_height,
                );
                let to = (self.plane_radius - centre_radius, -centre_height);
                let dot = (from.0 * to.0 + from.1 * to.1) / (radius * radius);
                let sweep = dot.clamp(-1.0, 1.0).acos();
                let weight = (sweep * 0.5).cos();
                let sum = (from.0 + to.0, from.1 + to.1);
                let length = sum.0.hypot(sum.1).max(1e-30);
                (
                    (
                        centre_radius + sum.0 / length * radius / weight,
                        centre_height + sum.1 / length * radius / weight,
                    ),
                    weight,
                )
            }
        }
    }

    /// 球の弧の、有理2次の中間制御点と重み。極から球側の接点まで。
    fn sphere_control(&self) -> ((f64, f64), f64) {
        let pole = (0.0, self.centre_height - self.r_sphere);
        let from = (pole.0, pole.1 - self.centre_height);
        let to = (
            self.sphere_radius,
            self.sphere_height - self.centre_height,
        );
        let dot = (from.0 * to.0 + from.1 * to.1) / (self.r_sphere * self.r_sphere);
        let sweep = dot.clamp(-1.0, 1.0).acos();
        let weight = (sweep * 0.5).cos();
        let sum = (from.0 + to.0, from.1 + to.1);
        let length = sum.0.hypot(sum.1).max(1e-30);
        (
            (
                sum.0 / length * self.r_sphere / weight,
                self.centre_height + sum.1 / length * self.r_sphere / weight,
            ),
            weight,
        )
    }

    /// 縁を落としたことで消える体積（正が除去）。
    ///
    /// どちらの断面も回転体なので、円板法で厳密に積めます。
    fn removed_volume(&self) -> f64 {
        // 球の断面の半径の2乗は `R^2 - (z - s)^2`。その原始関数。
        let sphere_primitive = |z: f64| {
            let d = z - self.centre_height;
            self.r_sphere * self.r_sphere * z - d * d * d / 3.0
        };
        let sphere_integral = |from: f64, to: f64| sphere_primitive(to) - sphere_primitive(from);

        let blend_integral = match self.profile {
            BlendProfile::Line => {
                // 半径が z の1次式。円錐台のぶん。
                let dz = -self.sphere_height;
                if dz.abs() <= 1e-30 {
                    0.0
                } else {
                    let slope = (self.plane_radius - self.sphere_radius) / dz;
                    let primitive = |t: f64| {
                        let base = self.sphere_radius;
                        base * base * t + base * slope * t * t + slope * slope * t * t * t / 3.0
                    };
                    primitive(-self.sphere_height) - primitive(0.0)
                }
            }
            BlendProfile::Arc {
                centre_radius,
                centre_height,
                radius,
            } => {
                // 半径は `centre_radius + sqrt(r^2 - (z - centre_height)^2)`。
                let primitive = |z: f64| {
                    let u = z - centre_height;
                    let root = (radius * radius - u * u).max(0.0).sqrt();
                    (centre_radius * centre_radius + radius * radius) * u
                        + centre_radius
                            * (u * root + radius * radius * (u / radius).clamp(-1.0, 1.0).asin())
                        - u * u * u / 3.0
                };
                primitive(0.0) - primitive(self.sphere_height)
            }
        };

        // 元の立体は、球を縁（高さ 0）まで積んだもの。
        let original = sphere_integral(self.centre_height - self.r_sphere, 0.0);
        let blended =
            sphere_integral(self.centre_height - self.r_sphere, self.sphere_height) + blend_integral;
        PI * (original - blended)
    }
}

/// 断面を4象限で回して立体にする。
///
/// 面は 4（球）＋ 4（つなぎ）＋ 1（蓋）の 9 枚です。極では制御点が潰れ、
/// 球の面は3辺のワイヤになります（円錐の頂点と同じ扱い）。
fn build_blended_sphere_cap(geometry: &SphereCapBlend) -> Result<Solid, String> {
    let theta = |index: usize| FRAC_PI_2 * (index % 4) as f64;
    let point = |radial: f64, z: f64, angle: f64| {
        Point3::new(radial * angle.cos(), radial * angle.sin(), z)
    };
    let angular_control = |radial: f64, z: f64, angle: f64| {
        let middle = angle + FRAC_PI_2 * 0.5;
        Point3::new(
            SQRT_2 * radial * middle.cos(),
            SQRT_2 * radial * middle.sin(),
            z,
        )
    };
    let circular_arc =
        |radial: f64, z: f64, index: usize, start: Vertex, end: Vertex| -> Result<Edge, String> {
            Ok(Edge::new(
                NurbsCurve3::new(
                    2,
                    vec![
                        ControlPoint3::unweighted(start.point),
                        ControlPoint3::new(angular_control(radial, z, theta(index)), FRAC_1_SQRT_2),
                        ControlPoint3::unweighted(end.point),
                    ],
                    KnotVector::clamped_uniform(3, 2),
                )?,
                start,
                end,
                1e-6,
            ))
        };

    let pole = Vertex::from_point(Point3::new(
        0.0,
        0.0,
        geometry.centre_height - geometry.r_sphere,
    ));
    let joint: Vec<Vertex> = (0..4)
        .map(|i| {
            Vertex::from_point(point(
                geometry.sphere_radius,
                geometry.sphere_height,
                theta(i),
            ))
        })
        .collect();
    let cap: Vec<Vertex> = (0..4)
        .map(|i| Vertex::from_point(point(geometry.plane_radius, 0.0, theta(i))))
        .collect();

    let (sphere_control, sphere_weight) = geometry.sphere_control();
    let (profile_control, profile_weight) = geometry.profile_control();

    let mut joint_arcs = Vec::with_capacity(4);
    let mut cap_arcs = Vec::with_capacity(4);
    let mut meridian_sphere = Vec::with_capacity(4);
    let mut meridian_blend = Vec::with_capacity(4);

    for i in 0..4 {
        let next = (i + 1) % 4;
        joint_arcs.push(circular_arc(
            geometry.sphere_radius,
            geometry.sphere_height,
            i,
            joint[i].clone(),
            joint[next].clone(),
        )?);
        cap_arcs.push(circular_arc(
            geometry.plane_radius,
            0.0,
            i,
            cap[i].clone(),
            cap[next].clone(),
        )?);
        meridian_sphere.push(Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(pole.point),
                    ControlPoint3::new(
                        point(sphere_control.0, sphere_control.1, theta(i)),
                        sphere_weight,
                    ),
                    ControlPoint3::unweighted(joint[i].point),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            pole.clone(),
            joint[i].clone(),
            1e-6,
        ));
        meridian_blend.push(Edge::new(
            NurbsCurve3::new(
                2,
                vec![
                    ControlPoint3::unweighted(joint[i].point),
                    ControlPoint3::new(
                        point(profile_control.0, profile_control.1, theta(i)),
                        profile_weight,
                    ),
                    ControlPoint3::unweighted(cap[i].point),
                ],
                KnotVector::clamped_uniform(3, 2),
            )?,
            joint[i].clone(),
            cap[i].clone(),
            1e-6,
        ));
    }

    // 角度方向（有理2次）× 断面方向（有理2次）のテンソル積。
    let revolved = |profile: [(f64, f64, f64); 3], index: usize| -> Result<NurbsSurface3, String> {
        let angle = theta(index);
        let rows: Vec<Vec<ControlPoint3>> = (0..3)
            .map(|j| {
                profile
                    .iter()
                    .map(|(radial, z, weight)| match j {
                        0 => ControlPoint3::new(point(*radial, *z, angle), *weight),
                        1 => ControlPoint3::new(
                            angular_control(*radial, *z, angle),
                            *weight * FRAC_1_SQRT_2,
                        ),
                        _ => ControlPoint3::new(
                            point(*radial, *z, angle + FRAC_PI_2),
                            *weight,
                        ),
                    })
                    .collect()
            })
            .collect();
        NurbsSurface3::new(
            2,
            2,
            rows,
            KnotVector::clamped_uniform(3, 2),
            KnotVector::clamped_uniform(3, 2),
        )
    };

    let mut faces = Vec::with_capacity(9);

    for i in 0..4 {
        let next = (i + 1) % 4;
        let surface = revolved(
            [
                (0.0, geometry.centre_height - geometry.r_sphere, 1.0),
                (sphere_control.0, sphere_control.1, sphere_weight),
                (geometry.sphere_radius, geometry.sphere_height, 1.0),
            ],
            i,
        )?;
        let wire = Wire::new(vec![
            OrientedEdge::forward(meridian_sphere[next].clone()),
            OrientedEdge::reversed(joint_arcs[i].clone()),
            OrientedEdge::reversed(meridian_sphere[i].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Nurbs(surface), wire));
    }

    for i in 0..4 {
        let next = (i + 1) % 4;
        let surface = revolved(
            [
                (geometry.sphere_radius, geometry.sphere_height, 1.0),
                (profile_control.0, profile_control.1, profile_weight),
                (geometry.plane_radius, 0.0, 1.0),
            ],
            i,
        )?;
        let wire = Wire::new(vec![
            OrientedEdge::forward(joint_arcs[i].clone()),
            OrientedEdge::forward(meridian_blend[next].clone()),
            OrientedEdge::reversed(cap_arcs[i].clone()),
            OrientedEdge::reversed(meridian_blend[i].clone()),
        ]);
        faces.push(Face::simple(FaceGeometry::Nurbs(surface), wire));
    }

    // `PlaneSurface3::new` は `(原点, u軸, v軸)` を取ります。法線は `u x v` なので、
    // 蓋の外向き法線 `+z` を出すには `u = +x`、`v = +y` です。
    let cap_plane = PlaneSurface3::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .ok_or_else(|| "the cap plane is degenerate".to_string())?;
    let cap_wire = Wire::new(
        (0..4)
            .map(|i| OrientedEdge::forward(cap_arcs[i].clone()))
            .collect(),
    );
    faces.push(Face::simple(FaceGeometry::Plane(cap_plane), cap_wire));

    crate::validated_solid(Shell::closed(faces))
}
