//! Zenith Math: CAD幾何演算のための高精度・ロバストな数学ライブラリ

pub mod bbox;
pub mod point;
pub mod polynomial;
pub mod predicates;
pub mod tolerance;
pub mod transform;
pub mod vector;

pub use bbox::BoundingBox3;
pub use point::{Point2, Point3, Point3Ext};
pub use polynomial::BernsteinPolynomial;
pub use predicates::RobustPredicates;
pub use tolerance::Tolerance;
pub use transform::Transform3;
pub use vector::{Vec2, Vec3, Vec3Ext};

/// デフォルトの幾何許容誤差
pub const DEFAULT_TOLERANCE: Tolerance = Tolerance::new(1e-6, 1e-5, 1e-7);
