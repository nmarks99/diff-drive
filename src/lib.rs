#![allow(unused_imports)]

pub mod utils;
pub mod rigid2d;
pub mod ddrive;

use num_traits::Float;
use std::fmt::Display;

use utils::almost_equal;
use rigid2d::{Vector2D, Pose2D};
use ddrive::WheelState;


#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_almost_equal() {
        assert!(almost_equal(1.0,1.0,1e-6));
        assert_eq!(almost_equal(1.0,1.001,1e-6), false);
    }

    #[test]
    fn pose2d_new() {
        let p = Pose2D::new(90.0,2.0,3.0);
        assert_eq!(p.theta, 90.0);
        assert_eq!(p.x, 2.0);
        assert_eq!(p.y, 3.0);
    }

    #[test]
    fn wheelstate_new() {
        let wheel_angles = WheelState::new(1.57,1.57);
        assert_eq!(wheel_angles.left, 1.57);
        assert_eq!(wheel_angles.right, 1.57);
    }

    #[test]
    fn vector2d_dot() {
        let lhs = Vector2D::new(1.0, 1.0);
        let rhs = Vector2D::new(2.0,2.0);
        let dot_prod = lhs.dot(&rhs);
        assert_eq!(dot_prod,4.0);
    }

    #[test]
    fn vector2d_f32_vs_f64() {
        let v1 = Vector2D::new(1.0f32, 1.0f32);
        let v2 = Vector2D::new(2.0f32,2.0f32);
        let _mag1 = v1.dot(&v2);
        
        let v1 = Vector2D::new(1.0f64, 1.0f64);
        let v2 = Vector2D::new(2.0f64,2.0f64);
        let _mag2 = v1.dot(&v2);
    }

    #[test]
    fn vector2d_normalize() {
        let v = Vector2D::new(3.0, 2.0);
        let norm = v.normalize();
        assert!(utils::almost_equal(norm.x, 0.83205029, 1e-4));
        assert!(utils::almost_equal(norm.y, 0.5547002, 1e-4));
    }

}
