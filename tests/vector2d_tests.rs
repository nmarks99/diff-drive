use diff_drive::rigid2d::Vector2D;
use diff_drive::utils;

#[test]
fn vector2d_from_polar() {
    let v = Vector2D::from_polar(1.0, utils::deg2rad(90.0));
    assert!(utils::almost_equal(v.x, 0.0, 1e-12));
    assert!(utils::almost_equal(v.y, 1.0, 1e-12));
}

#[test]
fn vector2d_dot() {
    let lhs = Vector2D::new(1.0, 1.0);
    let rhs = Vector2D::new(2.0, 2.0);
    let dot_prod = lhs.dot(&rhs);
    assert_eq!(dot_prod, 4.0);
}

#[test]
fn vector2d_f32_vs_f64() {
    let v1 = Vector2D::new(1.0f32, 1.0f32);
    let v2 = Vector2D::new(2.0f32, 2.0f32);
    let _mag1 = v1.dot(&v2);

    let v1 = Vector2D::new(1.0f64, 1.0f64);
    let v2 = Vector2D::new(2.0f64, 2.0f64);
    let _mag2 = v1.dot(&v2);
}

#[test]
fn vector2d_normalize() {
    let v = Vector2D::new(3.0, 2.0);
    let norm = v.normalize();
    assert!(utils::almost_equal(norm.x, 0.83205029, 1e-4));
    assert!(utils::almost_equal(norm.y, 0.5547002, 1e-4));
}

#[test]
fn vector2d_angle() {
    let v1 = Vector2D::new(1.0, 0.0);
    let v2 = Vector2D::new(0.0, 1.0);
    let angle = v1.angle(&v2);
    assert_eq!(angle, utils::deg2rad(90.0));
}

#[test]
fn vector2d_plus_operator() {
    let v1 = Vector2D::new(1.0, 3.0);
    let v2 = Vector2D::new(2.0, 5.0);
    let sum = v1 + v2;
    assert_eq!(sum.x, 3.0);
    assert_eq!(sum.y, 8.0);
}

#[test]
fn vector2d_distance() {
    let d1 = Vector2D::new(0.0, 0.0).distance(Vector2D::new(1.0, 0.0));
    let d2 = Vector2D::new(1.0, 1.0).distance(Vector2D::new(1.0, 2.0));
    let d3 = Vector2D::new(-1.0, 1.0).distance(Vector2D::new(0.0, 1.0));
    assert!(utils::almost_equal(d1, 1.0, 1e-9));
    assert!(utils::almost_equal(d2, 1.0, 1e-9));
    assert!(utils::almost_equal(d3, 1.0, 1e-9));
}
