use zenith_algo::{BrepTransform, PrimitiveBuilder};
use zenith_math::Vec3;

#[test]
fn test_solid_bounding_box_exact_dimensions() {
    // 1. 直方体
    let b = PrimitiveBuilder::make_box(10.0, 20.0, 30.0).expect("box");
    let bbox_b = b.bounding_box();
    assert!((bbox_b.min.x - 0.0).abs() < 1e-6);
    assert!((bbox_b.min.y - 0.0).abs() < 1e-6);
    assert!((bbox_b.min.z - 0.0).abs() < 1e-6);
    assert!((bbox_b.max.x - 10.0).abs() < 1e-6);
    assert!((bbox_b.max.y - 20.0).abs() < 1e-6);
    assert!((bbox_b.max.z - 30.0).abs() < 1e-6);

    // 2. 円柱
    let c = PrimitiveBuilder::make_cylinder(5.0, 20.0).expect("cylinder");
    let bbox_c = c.bounding_box();
    assert!((bbox_c.min.x - (-5.0)).abs() < 1e-6);
    assert!((bbox_c.min.y - (-5.0)).abs() < 1e-6);
    assert!((bbox_c.min.z - 0.0).abs() < 1e-6);
    assert!((bbox_c.max.x - 5.0).abs() < 1e-6);
    assert!((bbox_c.max.y - 5.0).abs() < 1e-6);
    assert!((bbox_c.max.z - 20.0).abs() < 1e-6);

    // 3. 球
    let s = PrimitiveBuilder::make_sphere(10.0).expect("sphere");
    let bbox_s = s.bounding_box();
    assert!((bbox_s.min.x - (-10.0)).abs() < 1e-6);
    assert!((bbox_s.min.y - (-10.0)).abs() < 1e-6);
    assert!((bbox_s.min.z - (-10.0)).abs() < 1e-6);
    assert!((bbox_s.max.x - 10.0).abs() < 1e-6);
    assert!((bbox_s.max.y - 10.0).abs() < 1e-6);
    assert!((bbox_s.max.z - 10.0).abs() < 1e-6);

    // 4. スロット柱 (Length 20, Radius 5, Height 15)
    let slot = PrimitiveBuilder::make_slot_prism(20.0, 5.0, 15.0).expect("slot prism");
    let bbox_slot = slot.bounding_box();
    assert!((bbox_slot.min.x - (-15.0)).abs() < 1e-6);
    assert!((bbox_slot.min.y - (-5.0)).abs() < 1e-6);
    assert!((bbox_slot.min.z - 0.0).abs() < 1e-6);
    assert!((bbox_slot.max.x - 15.0).abs() < 1e-6);
    assert!((bbox_slot.max.y - 5.0).abs() < 1e-6);
    assert!((bbox_slot.max.z - 15.0).abs() < 1e-6);

    // 5. 移動後の追従
    let moved = BrepTransform::translate_solid(&b, Vec3::new(100.0, -50.0, 25.0));
    let bbox_moved = moved.bounding_box();
    assert!((bbox_moved.min.x - 100.0).abs() < 1e-6);
    assert!((bbox_moved.min.y - (-50.0)).abs() < 1e-6);
    assert!((bbox_moved.min.z - 25.0).abs() < 1e-6);
    assert!((bbox_moved.max.x - 110.0).abs() < 1e-6);
    assert!((bbox_moved.max.y - (-30.0)).abs() < 1e-6);
    assert!((bbox_moved.max.z - 55.0).abs() < 1e-6);
}

#[test]
fn test_solid_bounding_box_intersection_culling() {
    let b1 = PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("b1");
    let b2 = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("b2"),
        Vec3::new(20.0, 0.0, 0.0),
    );
    let b3 = BrepTransform::translate_solid(
        &PrimitiveBuilder::make_box(10.0, 10.0, 10.0).expect("b3"),
        Vec3::new(5.0, 5.0, 5.0),
    );

    // b1 と b2 は離れているので交差しない
    assert!(!b1.bounding_box().intersects(&b2.bounding_box(), 1e-6));

    // b1 と b3 は重なっているので交差する
    assert!(b1.bounding_box().intersects(&b3.bounding_box(), 1e-6));
}
