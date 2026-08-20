use crate::orthogonal_boolean::OrthogonalBoxBoolean;
use crate::{BooleanOpType, BrepTransform, ExactBooleanResult, PrimitiveBuilder};
use zenith_math::{Point3, Tolerance, Vec3};
use zenith_topo::{FaceGeometry, Solid};

pub(crate) struct CylinderBoolean;

#[derive(Debug, Clone, Copy, PartialEq)]
struct AxisCylinderBounds {
    radius: f64,
    z_min: f64,
    z_max: f64,
}

impl CylinderBoolean {
    pub(crate) fn boolean_axis_cylinder_and_slab_exact_result(
        solid_a: &Solid,
        solid_b: &Solid,
        op: BooleanOpType,
        tol: &Tolerance,
    ) -> Result<Option<ExactBooleanResult>, String> {
        match op {
            BooleanOpType::Intersection => {
                if let Some(result) = Self::intersect_slab_and_cylinder(solid_a, solid_b, tol)? {
                    return Ok(Some(ExactBooleanResult::single(result)));
                }
                if let Some(result) = Self::intersect_slab_and_cylinder(solid_b, solid_a, tol)? {
                    return Ok(Some(ExactBooleanResult::single(result)));
                }
            }
            BooleanOpType::Difference => {
                if let Some(result) =
                    Self::subtract_slab_from_cylinder_result(solid_a, solid_b, tol)?
                {
                    return Ok(Some(result));
                }
            }
            BooleanOpType::Union => {}
        }

        Ok(None)
    }

    fn intersect_slab_and_cylinder(
        slab_solid: &Solid,
        cylinder_solid: &Solid,
        tol: &Tolerance,
    ) -> Result<Option<Solid>, String> {
        let Some(slab) = OrthogonalBoxBoolean::axis_aligned_box_bounds(slab_solid, tol) else {
            return Ok(None);
        };
        let Some(cylinder) = axis_cylinder_bounds(cylinder_solid, tol) else {
            return Ok(None);
        };
        if slab.min.x > -cylinder.radius + tol.linear
            || slab.max.x < cylinder.radius - tol.linear
            || slab.min.y > -cylinder.radius + tol.linear
            || slab.max.y < cylinder.radius - tol.linear
        {
            return Ok(None);
        }

        let z_min = slab.min.z.max(cylinder.z_min);
        let z_max = slab.max.z.min(cylinder.z_max);
        if z_max - z_min <= tol.linear {
            return Err("Exact cylinder-slab intersection has no positive volume".to_string());
        }

        let cylinder = PrimitiveBuilder::make_cylinder(cylinder.radius, z_max - z_min)?;
        Ok(Some(BrepTransform::translate_solid(
            &cylinder,
            Vec3::new(0.0, 0.0, z_min),
        )))
    }

    fn subtract_slab_from_cylinder_result(
        cylinder_solid: &Solid,
        slab_solid: &Solid,
        tol: &Tolerance,
    ) -> Result<Option<ExactBooleanResult>, String> {
        let Some(cylinder) = axis_cylinder_bounds(cylinder_solid, tol) else {
            return Ok(None);
        };
        let Some(slab) = OrthogonalBoxBoolean::axis_aligned_box_bounds(slab_solid, tol) else {
            return Ok(None);
        };
        if slab.min.x > -cylinder.radius + tol.linear
            || slab.max.x < cylinder.radius - tol.linear
            || slab.min.y > -cylinder.radius + tol.linear
            || slab.max.y < cylinder.radius - tol.linear
        {
            return Ok(None);
        }

        let overlap_min = slab.min.z.max(cylinder.z_min);
        let overlap_max = slab.max.z.min(cylinder.z_max);
        if overlap_max - overlap_min <= tol.linear {
            return Ok(None);
        }
        if overlap_min <= cylinder.z_min + tol.linear && overlap_max >= cylinder.z_max - tol.linear
        {
            return Err("Exact cylinder-slab difference produced an empty result".to_string());
        }

        let (z_min, z_max) = if overlap_min <= cylinder.z_min + tol.linear {
            (overlap_max, cylinder.z_max)
        } else if overlap_max >= cylinder.z_max - tol.linear {
            (cylinder.z_min, overlap_min)
        } else {
            return Self::make_disjoint_cylinder_sections(cylinder, overlap_min, overlap_max, tol)
                .map(Some);
        };
        if z_max - z_min <= tol.linear {
            return Err("Exact cylinder-slab difference produced an empty result".to_string());
        }

        let cylinder = PrimitiveBuilder::make_cylinder(cylinder.radius, z_max - z_min)?;
        Ok(Some(ExactBooleanResult::single(
            BrepTransform::translate_solid(&cylinder, Vec3::new(0.0, 0.0, z_min)),
        )))
    }

    fn make_disjoint_cylinder_sections(
        cylinder: AxisCylinderBounds,
        cut_min: f64,
        cut_max: f64,
        tol: &Tolerance,
    ) -> Result<ExactBooleanResult, String> {
        let mut solids = Vec::new();
        if cut_min - cylinder.z_min > tol.linear {
            let lower = PrimitiveBuilder::make_cylinder(cylinder.radius, cut_min - cylinder.z_min)?;
            solids.push(BrepTransform::translate_solid(
                &lower,
                Vec3::new(0.0, 0.0, cylinder.z_min),
            ));
        }
        if cylinder.z_max - cut_max > tol.linear {
            let upper = PrimitiveBuilder::make_cylinder(cylinder.radius, cylinder.z_max - cut_max)?;
            solids.push(BrepTransform::translate_solid(
                &upper,
                Vec3::new(0.0, 0.0, cut_max),
            ));
        }
        if solids.len() < 2 {
            return Err("Exact cylinder-slab difference produced an empty result".to_string());
        }
        Ok(ExactBooleanResult { solids })
    }
}

fn axis_cylinder_bounds(solid: &Solid, tol: &Tolerance) -> Option<AxisCylinderBounds> {
    if !solid.inner_shells.is_empty() || solid.outer_shell.faces.len() != 6 {
        return None;
    }

    let nurbs_count = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Nurbs(_)))
        .count();
    let plane_count = solid
        .outer_shell
        .faces
        .iter()
        .filter(|face| matches!(face.geometry, FaceGeometry::Plane(_)))
        .count();
    if nurbs_count != 4 || plane_count != 2 {
        return None;
    }

    let points = solid_outer_sample_points(solid);
    if points.is_empty()
        || points
            .iter()
            .any(|point| !point.coords.iter().all(|v| v.is_finite()))
    {
        return None;
    }

    let z_min = points
        .iter()
        .map(|point| point.z)
        .fold(f64::INFINITY, f64::min);
    let z_max = points
        .iter()
        .map(|point| point.z)
        .fold(f64::NEG_INFINITY, f64::max);
    if z_max - z_min <= tol.linear {
        return None;
    }

    let radius = points
        .iter()
        .map(|point| (point.x * point.x + point.y * point.y).sqrt())
        .fold(0.0, f64::max);
    if radius <= tol.linear {
        return None;
    }

    // 側面の標本が「全て」同じ半径にあることまで見る。最大半径にいくつか
    // 乗っているだけでは円柱とは言えない。円錐は底の円がちょうど最大半径に
    // 乗るので、数を数えるだけだった頃は円柱として通り、半径の変わらない
    // 立体として作り直されていた。検証ゲートがその結果を弾いていたので誤答は
    // 出ていないが、正しい経路にも進めなくなっていた。
    let mut lateral_samples = 0usize;
    for face in &solid.outer_shell.faces {
        if !matches!(face.geometry, FaceGeometry::Nurbs(_)) {
            continue;
        }
        for point in face.outer_wire.sample_points(8) {
            let radial_distance = (point.x * point.x + point.y * point.y).sqrt();
            if (radial_distance - radius).abs() > tol.linear * 20.0 {
                return None;
            }
            if point.z < z_min - tol.linear || point.z > z_max + tol.linear {
                return None;
            }
            lateral_samples += 1;
        }
    }
    if lateral_samples < 16 {
        return None;
    }

    Some(AxisCylinderBounds {
        radius,
        z_min,
        z_max,
    })
}

fn solid_outer_sample_points(solid: &Solid) -> Vec<Point3> {
    let mut points = Vec::new();
    for face in &solid.outer_shell.faces {
        for point in face.outer_wire.sample_points(8) {
            points.push(point);
        }
    }
    points
}
