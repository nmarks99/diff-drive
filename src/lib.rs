//! [![github]](https://github.com/nmarks99/diff-drive)&ensp;
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//!
//! <br>
//!
//! This library provides basic functionality for working with differential
//! drive robots.
//!
//! <br>
//!
//! # Details
//! TODO: explain more here
//!
//! ```
//! use diff_drive::rigid2d::{Vector2D, Pose2D};
//! use diff_drive::ddrive::{DiffDrive};
//!  
//! const WHEEL_SEPARATION: f64 = 0.1;
//! const WHEEL_RADIUS: f64 = 0.03;
//! fn main() {
//!     let v = Vector2D::new(1.0, 1.0);
//!     let p = Pose2D::new(1.0, 2.0, 3.14);
//!     let robot = DiffDrive::new(WHEEL_RADIUS, WHEEL_SEPARATION);
//! }
//! ```
//! `
#![allow(unused_imports)]

pub mod ddrive;
pub mod rigid2d;
pub mod utils;

#[cfg(test)]
mod tests {
    use super::*;
}
