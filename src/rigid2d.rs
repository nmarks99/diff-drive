use num_traits::Float;
use std::fmt::Display;

/// a 2D vector
pub struct Vector2D<T: Float> {
    /// x coordinate
    pub x: T,

    /// y coordinate
    pub y: T
}

impl<T: Float> Vector2D<T> {
    
    /// constructs a Vector2D from floats (x,y)
    pub fn new(x: T, y: T) -> Self {
        Vector2D {x, y}
    }
    
    /// computes the L2 norm of the 2D vector
    pub fn magnitude(&self) -> T {
        let x_sqr = self.x.powf(T::from(2.0).unwrap());
        let y_sqr = self.y.powf(T::from(2.0).unwrap());
        (x_sqr + y_sqr).sqrt()
    }
    
    /// computes the dot product of the vector with another vector
    pub fn dot(&self, rhs: &Vector2D<T>) -> T {
        (self.x * rhs.x) + (self.y * rhs.y)
    }
    
    /// normalizes the vector
    pub fn normalize(&self) -> Self {
        let v_norm: Self;
        if self.x.is_zero() || self.y.is_zero() {
            v_norm = Vector2D::new(T::zero(), T::zero());
        }
        else {
            let mag = self.magnitude();
            v_norm = Vector2D::new(self.x/mag, self.y/mag);
        }
        return v_norm;
    }
}


/// A 2D pose
#[derive(Debug)]
pub struct Pose2D<T: Float> {
    /// Rotation in radians
    pub theta: T,

    /// x position
    pub x: T,

    /// y position
    pub y: T
}

impl<T:Float> Pose2D<T> {

    /// constructs a new Pose2D object from (theta,x,y) which are floats
    pub fn new(theta: T, x: T, y: T)  -> Self {
        Pose2D {theta, x, y}
    }
}

impl<T:Float + Display> Display for Pose2D<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
      write!(f, "deg:{} x:{} y:{}", self.theta, self.x, self.y)
    }
}


