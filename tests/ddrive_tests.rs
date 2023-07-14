use diff_drive::ddrive::{DiffDrive, WheelState};
use diff_drive::rigid2d::Twist2D;

#[test]
fn wheelstate_new() {
    let wheel_angles = WheelState::new(1.57, 1.57);
    assert_eq!(wheel_angles.left, 1.57);
    assert_eq!(wheel_angles.right, 1.57);
}

#[test]
fn diffdrive_ik() {
    let v = Twist2D::new(1.0, 1.0, 0.0);
    let mut ddrive = DiffDrive::new(1.0, 2.0);
    let speeds: WheelState<f64> = ddrive.inverse_kinematics(v);
    println!("{:?}", speeds);
}
