use diff_drive::rigid2d::Vector2D;
use diff_drive::utils::{
    almost_equal, arange, distance, linspace, rad_per_sec_to_rpm, rpm_to_rad_per_sec,
};

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

#[test]
fn test_rpm_to_rad_per_sec() {
    let speed_rpm: f32 = 60.0;
    let speed_rad_per_sec: f32 = rpm_to_rad_per_sec(speed_rpm);
    assert!(almost_equal(
        speed_rad_per_sec,
        2.0 * std::f32::consts::PI,
        1e-6,
    ));
}

#[test]
fn test_rad_per_sec_to_rpm() {
    let speed_rad_per_sec: f32 = 2.0 * std::f32::consts::PI;
    let speed_rpm = rad_per_sec_to_rpm(speed_rad_per_sec);
    assert!(almost_equal(speed_rpm, 60.0, 1e-6,))
}

#[test]
fn test_linspace() {
    let x = linspace(0.0, 5.0, 10);
    println!("linspace (0,5,10): {:?}", x);
}

#[test]
fn test_arange() {
    let x = arange(0.0, 11.0, 1.0);
    println!("arange (0,11,1): {:?}", x);
}
