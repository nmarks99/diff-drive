use diff_drive::rigid2d::Pose2D;

#[test]
fn pose2d_new() {
    let p = Pose2D::new(2.0, 3.0, 90.0);
    assert_eq!(p.x, 2.0);
    assert_eq!(p.y, 3.0);
    assert_eq!(p.theta, 90.0);
}
