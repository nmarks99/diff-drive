#![allow(non_snake_case)]

use crate::rigid2d::{Pose2D, Transform2D, Twist2D, Vector2D};
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

    /// Converts the revolutions per minute from radians per second
    pub fn convert_to_rpm(&mut self) -> Self {
        self.left = utils::rad_per_sec_to_rpm(self.left);
        self.right = utils::rad_per_sec_to_rpm(self.right);
        WheelState {
            left: self.left,
            right: self.right,
        }
    }

    /// Converts from radians per second to revolutions per minute
    pub fn convert_to_rad_per_sec(&mut self) -> Self {
        self.left = utils::rpm_to_rad_per_sec(self.left);
        self.right = utils::rad_per_sec_to_rpm(self.right);
        WheelState {
            left: self.left,
            right: self.right,
        }
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

    /// Computes the wheel speeds needed to obtain the given twist.
    /// this can also be considered inverse kinematics
    /// TODO: do not mutate self here
    pub fn speeds_from_twist(&mut self, v: Twist2D<T>) -> WheelState<T> {
        if !utils::almost_equal(v.ydot, T::from(0.0).unwrap(), T::from(0.0001).unwrap()) {
            panic!("Non-zero y component of twist is not possible");
        }

        let d = self.wheel_separation / T::from(2.0).unwrap();
        let r = self.wheel_radius;

        // WheelState::new(
        //     (T::from(1.0).unwrap() / r) * (-d * v.thetadot + v.xdot),
        //     (T::from(1.0).unwrap() / r) * (d * v.thetadot + v.xdot),
        // )
        self.phidot.left = (T::from(1.0).unwrap() / r) * (-d * v.thetadot + v.xdot);
        self.phidot.right = (T::from(1.0).unwrap() / r) * (d * v.thetadot + v.xdot);
        self.phidot
    }

    /// Computes the body twist for the given wheel speeds
    pub fn twist_from_speeds(&self, phidot: WheelState<T>) -> Twist2D<T> {
        Twist2D::new(
            (self.wheel_radius / self.wheel_separation) * (phidot.right - phidot.left),
            (self.wheel_radius / T::from(2.0).unwrap()) * (phidot.left + phidot.right),
            T::from(0.0).unwrap(),
        )
    }

    /// computes the forward kinematics to find
    /// the new pose of robot given new wheel angles
    pub fn forward_kinematics(&mut self, phi_new: WheelState<T>) -> Pose2D<T> {
        // update the pose with the provided pose
        // self.pose = pose;

        // Compute the new wheel speeds for a single timestep (t=1)
        self.phidot.left = phi_new.left - self.phi.left;
        self.phidot.right = phi_new.right - self.phi.right;

        // Update the wheel angles with the provided ones
        self.phi = phi_new;

        // Get the twist from the new wheel speeds
        let body_twist = self.twist_from_speeds(self.phidot);

        // Define the transform between the world and B frame
        // B is the body frame before achieving the new wheel angles phi_new
        // in other words, self.pose here is the old pose
        let Twb = Transform2D::new(Vector2D::new(self.pose.x, self.pose.y), self.pose.theta);

        // The transform between the B and B' frames can be obtained by
        // integrating the twist forward. B' is the body frame once the
        // robotb has acheieved phi_new
        let Tb_bprime = Twb.integrate_twist(body_twist);

        // Get the B' frame in the world frame by applying this transform
        let Tw_bprime = Twb * Tb_bprime;

        // Get the new pose from Tw_bprime and return it
        self.pose.theta = Tw_bprime.rotation();
        self.pose.x = Tw_bprime.translation().x;
        self.pose.y = Tw_bprime.translation().y;

        self.pose
    }
}
