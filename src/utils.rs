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

pub fn rpm_to_rad_per_sec<T: Float>(speed: T) -> T {
    speed * T::from((2.0 * std::f64::consts::PI) / 60.0).unwrap()
}

pub fn rad_per_sec_to_rpm<T: Float>(speed: T) -> T {
    speed * T::from(60.0 / (2.0 * std::f64::consts::PI)).unwrap()
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

/// Creates a linearly spaced vector of floats from start to stop with the given
/// number of points in between
pub fn linspace<T: Float>(start: T, stop: T, num_points: usize) -> Vec<T> {
    let step: T = (stop - start) / T::from(num_points - 1).unwrap();
    (0..num_points)
        .map(|i| start + T::from(i).unwrap() * step)
        .collect()
}

/// Creates a vector of floats from the given start to stop, and separated by the step
pub fn arange<T: Float>(start: T, stop: T, step: T) -> Vec<T> {
    let num_points: usize = (((stop - start) / step) + T::from(1.0).unwrap())
        .to_usize()
        .unwrap();
    (0..num_points)
        .map(|i| start + T::from(i).unwrap() * step)
        .collect()
}

/// Computes the linear distance between to points
pub fn distance<T: Float>(a: Vector2D<T>, b: Vector2D<T>) -> T {
    T::sqrt((b.x - a.x).powf(T::from(2.0).unwrap()) + (b.y - a.y).powf(T::from(2.0).unwrap()))
}
