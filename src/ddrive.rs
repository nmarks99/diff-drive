use num_traits::Float;
use std::fmt::Display;

/// State of the left and right wheels which could be position, velocity, etc.
#[derive(Debug)]
pub struct WheelState<T: Float> {

    /// State of the left wheel
    pub left: T,

    /// State of the right wheel
    pub right: T
}

impl<T: Float> WheelState<T> {
    /// construcs a new WheelState from (left,right) which are floats
    pub fn new(left: T, right: T) -> Self {
        WheelState {left, right}
    }
}

impl<T:Float + Display> Display for WheelState<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
      write!(f, "left:{} right:{}", self.left, self.right)
    }
}
