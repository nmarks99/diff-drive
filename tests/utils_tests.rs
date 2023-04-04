use diff_drive::utils::almost_equal;

#[test]
fn test_almost_equal() {
    assert!(almost_equal(1.0,1.0,1e-6));
    assert_eq!(almost_equal(1.0,1.001,1e-6), false);
}

