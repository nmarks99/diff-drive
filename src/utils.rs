use num_traits::Float;
use std::fmt::Display;

pub fn almost_equal<T: Float>(d1: T, d2: T, epsilon: T) -> bool {
    (d1 - d2).abs() < epsilon
}

pub fn deg2rad<T: Float>(deg: T) -> T {
    deg * T::from(std::f64::consts::PI / 180.0).unwrap()
}

pub fn rad2deg<T: Float>(rad: T) -> T {
    rad * T::from(180.0/std::f64::consts::PI).unwrap()
}

pub fn normalize_angle<T: Float>(rad: T) -> T {
    let pi = T::from(std::f64::consts::PI).unwrap();
    if  almost_equal(-pi, rad, T::from(1e-6).unwrap()) {
        return pi;
    }
    else {
        return rad.sin().atan2(rad.cos());
    }
}

