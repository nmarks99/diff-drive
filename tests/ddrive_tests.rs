use diff_drive::ddrive::{DiffDrive, WheelState};
use diff_drive::rigid2d::Twist2D;
use diff_drive::utils::almost_equal;
use std::f64::consts::PI;

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
    let speeds: WheelState<f64> = ddrive.speeds_from_twist(v);
    println!("{:?}", speeds);
}

#[test]
fn diffdrive_body_twist() {
    let robot = DiffDrive::new(1.0, 1.0);
    let phidot = WheelState::new(1.0, 1.0);
    let v = robot.twist_from_speeds(phidot);
    assert!(almost_equal(v.xdot, 1.0, 1e-6));
}

#[test]
fn diffdrive_fk() {
    const R_TEST: f64 = 1.0;
    const D_TEST: f64 = 1.0;

    // Robot drives forward
    {
        let mut robot = DiffDrive::new(R_TEST, D_TEST * 2.0);
        let phi_new = WheelState::new(1.57, 1.57);
        // let pose0 = Pose2D::new(0.0, 0.0, 0.0);
        let new_pose = robot.forward_kinematics(phi_new);
        assert!(almost_equal(new_pose.x, 1.57, 1e-6));
        assert!(almost_equal(new_pose.y, 0.0, 1e-6));
        assert!(almost_equal(new_pose.theta, 0.0, 1e-6));
    }

    // Robot drives backwards
    {
        let mut robot = DiffDrive::new(R_TEST, D_TEST * 2.0);
        let phi_new = WheelState::new(-1.57, -1.57);
        // let pose0 = Pose2D::new(0.0, 0.0, 0.0);
        let new_pose = robot.forward_kinematics(phi_new);
        assert!(almost_equal(new_pose.x, -1.57, 1e-6));
        assert!(almost_equal(new_pose.y, 0.0, 1e-6));
        assert!(almost_equal(new_pose.theta, 0.0, 1e-6));
    }

    // Robot drives forward and turns
    {
        let mut robot = DiffDrive::new(R_TEST, D_TEST * 2.0);
        let phi_new = WheelState::new(PI, 0.0);
        // let pose0 = Pose2D::new(0.0, 0.0, 0.0);
        let new_pose = robot.forward_kinematics(phi_new);
        assert!(almost_equal(new_pose.x, 1.0, 1e-6));
        assert!(almost_equal(new_pose.y, -1.0, 1e-6));
        assert!(almost_equal(new_pose.theta, -PI / 2.0, 1e-6));
    }

    // Robot spins in place
    {
        let mut robot = DiffDrive::new(R_TEST, D_TEST * 2.0);
        let phi_new = WheelState::new(-PI, PI);
        // let pose0 = Pose2D::new(0.0, 0.0, 0.0);
        let new_pose = robot.forward_kinematics(phi_new);
        assert!(almost_equal(new_pose.x, 0.0, 1e-6));
        assert!(almost_equal(new_pose.y, 0.0, 1e-6));
        assert!(almost_equal(new_pose.theta, PI, 1e-6));
    }
}
