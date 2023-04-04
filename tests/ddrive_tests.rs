use diff_drive::ddrive::WheelState;

#[test]
fn wheelstate_new() {
    let wheel_angles = WheelState::new(1.57,1.57);
    assert_eq!(wheel_angles.left, 1.57);
    assert_eq!(wheel_angles.right, 1.57);
}

