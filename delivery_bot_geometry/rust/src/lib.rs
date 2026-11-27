//! Rust port of the parts of `delivery_bot_geometry` (delivery_bot_core#16) that the step controller
//! (delivery_bot_core#19) needs: angles, SE(2) poses, constant-curvature paths and the arc utilities.
//!
//! The formulas are the same as in the C++ library, and both are checked against the shared test vectors in
//! `../test/vectors/arc_vectors.txt` (`tests/vectors.rs`), so results agree to 1e-9.
//!
//! Conventions (as in C++): m / rad, yaw + = counter-clockwise (ROS FLU). A segment has a signed length (< 0 =
//! reverse) and a steering curvature (+ = left). `s` is the travelled arc length along a path.

mod angle;
mod arc;
mod path;
mod se2;

pub use angle::{angle_diff, interpolate_angle, normalize_angle, PI, TWO_PI};
pub use arc::{
    closest_point, cross_track_error, heading_error, point_at, tracking_error, ClosestPoint,
    TrackingError,
};
pub use path::{advance, Path, Segment};
pub use se2::Pose2;
