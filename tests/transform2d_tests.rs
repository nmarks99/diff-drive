use diff_drive::rigid2d::{Transform2D,Vector2D};
use diff_drive::utils;
use std::f64::consts::PI;

#[test]
fn transform2d_new() {
    let angle = utils::deg2rad(90.0);
    let trans = Vector2D::new(1.0,2.0);
    let tf = Transform2D::new(trans,angle);

    let a = tf.rotation();
    let t = tf.translation();

    println!("a = {}", a);
    println!("t = {}", t);
        
    assert!(utils::almost_equal(a, PI/2.0, 1e-6));
    assert!(utils::almost_equal(t.x, 1.0, 1e-6));
    assert!(utils::almost_equal(t.y, 2.0, 1e-6));

}
