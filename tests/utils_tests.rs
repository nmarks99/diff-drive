use diff_drive::rigid2d::Vector2D;
use diff_drive::utils::{almost_equal, distance};

#[test]
fn test_almost_equal() {
    assert!(almost_equal(1.0, 1.0, 1e-6));
    assert_eq!(almost_equal(1.0, 1.001, 1e-6), false);
}

#[test]
fn test_distance() {
    let a = Vector2D::new(0.0, 0.0);
    let b = Vector2D::new(1.0, 0.0);
    let d = distance(a, b);
    assert!(almost_equal(d, 1.0, 1e-6));
}
