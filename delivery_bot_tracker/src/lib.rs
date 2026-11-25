//! Dynamic obstacle detection and tracking with prediction (delivery_bot_core#11).
//!
//! The algorithms live here, free of ROS, so `cargo test` covers them. `main.rs` is the rclrs node around them.

pub mod assign;
pub mod cloud;
pub mod cluster;
pub mod geometry;
pub mod kalman;
pub mod params;
pub mod pipeline;
pub mod static_map;
pub mod tf;
pub mod tracker;


pub use geometry::Pose2;
pub use params::Params;
pub use pipeline::{Pipeline, Source};
pub use tracker::TrackOutput;
