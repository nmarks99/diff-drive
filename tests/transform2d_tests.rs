use diff_drive::rigid2d::{Transform2D, Twist2D, Vector2D};
use diff_drive::utils::{self, almost_equal};
use std::f64::consts::PI;

#[test]
fn transform2d_new() {
    let angle = utils::deg2rad(90.0);
    let trans = Vector2D::new(1.0, 2.0);
    let tf = Transform2D::new(trans, angle);

    let a = tf.rotation();
    let t = tf.translation();

    println!("a = {}", a);
    println!("t = {}", t);

    assert!(almost_equal(a, PI / 2.0, 1e-6));
    assert!(almost_equal(t.x, 1.0, 1e-6));
    assert!(almost_equal(t.y, 2.0, 1e-6));
}

#[test]
fn transform2d_mul() {
    let v1 = Vector2D::new(5.0, 3.0);
    let v2 = Vector2D::new(2.0, 7.0);
    let tf1 = Transform2D::new(v1, 0.0);
    let tf2 = Transform2D::new(v2, 0.0);
    let tf3 = tf1 * tf2;
    assert!(almost_equal(tf3.rotation(), 0.0, 0.0001));
    assert!(almost_equal(tf3.translation().x, 7.0, 0.0001));
    assert!(almost_equal(tf3.translation().y, 10.0, 0.0001));
}

#[test]
fn transform2d_integrate_twist() {
    // Pure translation
    {
        let v = Twist2D::new(0.0, 1.0, 0.0);
        let tf = Transform2D::new(Vector2D::new(0.0, 0.0), 0.0);
        let tf_new = tf.integrate_twist(v);
        assert!(almost_equal(tf_new.translation().x, 1.0, 1e-6));
        assert!(almost_equal(tf_new.translation().y, 0.0, 1e-6));
        assert!(almost_equal(tf_new.rotation(), 0.0, 1e-6));
    }

    // Pure rotation
    {
        let v = Twist2D::new(PI, 0.0, 0.0);
        let tf = Transform2D::new(Vector2D::new(0.0, 0.0), 0.0);
        let tf_new = tf.integrate_twist(v);
        assert!(almost_equal(tf_new.translation().x, 0.0, 1e-6));
        assert!(almost_equal(tf_new.translation().y, 0.0, 1e-6));
        assert!(almost_equal(tf_new.rotation(), PI, 1e-6));
    }

    // Rotation and Translation
    {
        let v = Twist2D::new(2.0 * PI, PI, PI);
        let tf = Transform2D::new(Vector2D::new(0.0, 0.0), 0.0);
        let tf_new = tf.integrate_twist(v);
        assert!(almost_equal(tf_new.translation().x, 0.0, 1e-6));
        assert!(almost_equal(tf_new.translation().y, 0.0, 1e-6));
        assert!(almost_equal(tf_new.rotation(), 2.0 * PI, 1e-6));
    }
}
