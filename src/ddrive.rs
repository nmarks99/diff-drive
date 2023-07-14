use crate::rigid2d::{Pose2D, Twist2D};
use crate::utils;
use anyhow;
use num_traits::Float;
use std::default::Default;
use std::error::Error;
use std::fmt::Display;

/// State of the left and right wheels which could be position, velocity, etc.
#[derive(Debug, Clone, Copy, Default)]
pub struct WheelState<T: Float + Default> {
    /// State of the left wheel
    pub left: T,

    /// State of the right wheel
    pub right: T,
}

impl<T: Float + Default> WheelState<T> {
    /// construcs a new WheelState from (left,right) which are floats
    pub fn new(left: T, right: T) -> Self {
        WheelState { left, right }
    }
}

impl<T> Display for WheelState<T>
where
    T: Float + Display + Default,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "left:{} right:{}", self.left, self.right)
    }
}

// #[derive(Debug, Clone)]
pub struct DiffDrive<T: Float + Default> {
    /// Radius of the robot's wheels
    wheel_radius: T,

    /// Distance between the wheel centers
    wheel_separation: T,

    /// x,y,theta position in meters, radians
    pose: Pose2D<T>,

    /// Wheel positions in radians
    phi: WheelState<T>,

    /// Wheel speeds in radians per second
    phidot: WheelState<T>,
}

impl<T: Float + Default> DiffDrive<T> {
    pub fn new(wheel_radius: T, wheel_separation: T) -> Self {
        Self {
            wheel_radius,
            wheel_separation,
            pose: Pose2D::default(),
            phi: WheelState::default(),
            phidot: WheelState::default(),
        }
    }

    pub fn inverse_kinematics(&mut self, v: Twist2D<T>) -> WheelState<T> {
        if !utils::almost_equal(v.ydot, T::from(0.0).unwrap(), T::from(0.0001).unwrap()) {
            panic!("Non-zero y component of twist is not possible");
        }

        let d = self.wheel_separation / T::from(2.0).unwrap();
        let r = self.wheel_radius;

        self.phidot.left = (T::from(1.0).unwrap() / r) * (-d * v.thetadot + v.xdot);
        self.phidot.right = (T::from(1.0).unwrap() / r) * (d * v.thetadot + v.xdot);
        self.phidot
    }
}
