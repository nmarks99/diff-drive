use crate::rigid2d::Vector2D;
use num_traits::Float;
use std::fmt::Display;

/// Returns true if two floats are almost equal (within epsilon) otherwise false
pub fn almost_equal<T: Float>(d1: T, d2: T, epsilon: T) -> bool {
    (d1 - d2).abs() < epsilon
}

/// Converts degrees to radians
pub fn deg2rad<T: Float>(deg: T) -> T {
    deg * T::from(std::f64::consts::PI / 180.0).unwrap()
}

/// Converts radians to degrees
pub fn rad2deg<T: Float>(rad: T) -> T {
    rad * T::from(180.0 / std::f64::consts::PI).unwrap()
}

/// Normalizes an angle in radians to be between -pi and pi
pub fn normalize_angle<T: Float>(rad: T) -> T {
    let pi = T::from(std::f64::consts::PI).unwrap();
    if almost_equal(-pi, rad, T::from(1e-6).unwrap()) {
        return pi;
    } else {
        return rad.sin().atan2(rad.cos());
    }
}

/// Computes the linear distance between to points
pub fn distance<T: Float>(a: Vector2D<T>, b: Vector2D<T>) -> T {
    T::sqrt((b.x - a.x).powf(T::from(2.0).unwrap()) + (b.y - a.y).powf(T::from(2.0).unwrap()))
}
