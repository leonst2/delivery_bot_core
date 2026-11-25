//! A small planar TF buffer. Only the ground-plane part (x, y, yaw) of each transform is kept, which is all the
//! tracker needs to bring `base_link` points into `map`. Stamps are nanoseconds of the ROS clock (sim time in the
//! simulation).

use std::collections::{BTreeMap, VecDeque};

use crate::geometry::Pose2;

const MAX_CHAIN: usize = 16;
/// A transform older than the newest one by more than this is a clock reset (/sim/reset), not reordering.
const RESET_JUMP_NS: i64 = 1_000_000_000;

struct Frame {
    parent: String,
    is_static: bool,
    /// Sorted by stamp, unique stamps.
    samples: VecDeque<(i64, Pose2)>,
}

pub struct TfBuffer {
    frames: BTreeMap<String, Frame>,
    history_ns: i64,
    tolerance_ns: i64,
}

fn clean(frame: &str) -> &str {
    frame.trim_start_matches('/')
}

impl TfBuffer {
    /// `history_ns`: how much history to keep per frame. `tolerance_ns`: how far a lookup may lie outside the stored
    /// samples and still use the nearest one (e.g. map → odom from localization lags behind odom → base_link).
    pub fn new(history_ns: i64, tolerance_ns: i64) -> Self {
        Self {
            frames: BTreeMap::new(),
            history_ns,
            tolerance_ns,
        }
    }

    pub fn clear(&mut self) {
        self.frames.clear();
    }

    /// Stores T_parent_child at `stamp`. Static transforms hold for every stamp.
    pub fn insert(&mut self, parent: &str, child: &str, stamp: i64, pose: Pose2, is_static: bool) {
        let (parent, child) = (clean(parent), clean(child));
        if parent.is_empty() || child.is_empty() || parent == child {
            return;
        }
        let frame = self
            .frames
            .entry(child.to_owned())
            .or_insert_with(|| Frame {
                parent: parent.to_owned(),
                is_static,
                samples: VecDeque::new(),
            });
        if frame.parent != parent || frame.is_static != is_static || is_static {
            frame.parent = parent.to_owned();
            frame.is_static = is_static;
            frame.samples.clear();
        }
        if let Some(&(newest, _)) = frame.samples.back() {
            if stamp < newest - RESET_JUMP_NS {
                frame.samples.clear();
            }
        }
        match frame.samples.binary_search_by_key(&stamp, |&(t, _)| t) {
            Ok(i) => frame.samples[i] = (stamp, pose),
            Err(i) => frame.samples.insert(i, (stamp, pose)),
        }
        let newest = frame.samples.back().map_or(stamp, |&(t, _)| t);
        while frame.samples.len() > 1
            && frame
                .samples
                .front()
                .is_some_and(|&(t, _)| t < newest - self.history_ns)
        {
            frame.samples.pop_front();
        }
    }

    fn sample(&self, child: &str, frame: &Frame, stamp: i64) -> Result<Pose2, String> {
        let samples = &frame.samples;
        let (&(first, first_pose), &(last, last_pose)) = match (samples.front(), samples.back()) {
            (Some(f), Some(l)) => (f, l),
            _ => return Err(format!("no transform {} -> {child}", frame.parent)),
        };
        if frame.is_static {
            return Ok(last_pose);
        }
        if stamp >= last {
            return if stamp - last <= self.tolerance_ns {
                Ok(last_pose)
            } else {
                Err(format!(
                    "transform {} -> {child} is {:.3} s older than the requested time",
                    frame.parent,
                    (stamp - last) as f64 * 1e-9
                ))
            };
        }
        if stamp <= first {
            return if first - stamp <= self.tolerance_ns {
                Ok(first_pose)
            } else {
                Err(format!(
                    "transform {} -> {child} has no history at the requested time",
                    frame.parent
                ))
            };
        }
        // first < stamp < last: interpolate between the neighbours.
        let i = match samples.binary_search_by_key(&stamp, |&(t, _)| t) {
            Ok(i) => return Ok(samples[i].1),
            Err(i) => i,
        };
        let (t0, p0) = samples[i - 1];
        let (t1, p1) = samples[i];
        Ok(p0.interpolate(&p1, (stamp - t0) as f64 / (t1 - t0) as f64))
    }

    /// T_target_source at `stamp`, following parents from `source` up to `target`.
    pub fn lookup(&self, target: &str, source: &str, stamp: i64) -> Result<Pose2, String> {
        let (target, source) = (clean(target), clean(source));
        let mut result = Pose2::IDENTITY;
        let mut current = source;
        for _ in 0..MAX_CHAIN {
            if current == target {
                return Ok(result);
            }
            let frame = self.frames.get(current).ok_or_else(|| {
                format!("no transform from '{target}' to '{source}' ('{current}' has no parent)")
            })?;
            result = self.sample(current, frame, stamp)?.compose(&result);
            current = &frame.parent;
        }
        Err(format!(
            "transform chain from '{source}' to '{target}' is too long or cyclic"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const S: i64 = 1_000_000_000;

    fn buffer() -> TfBuffer {
        TfBuffer::new(2 * S, S / 5)
    }

    #[test]
    fn composes_map_odom_base_link() {
        let mut tf = buffer();
        tf.insert(
            "map",
            "odom",
            10 * S,
            Pose2::new(10.0, 0.0, PI / 2.0),
            false,
        );
        tf.insert(
            "odom",
            "base_link",
            10 * S,
            Pose2::new(1.0, 0.0, 0.0),
            false,
        );
        let pose = tf.lookup("map", "base_link", 10 * S).unwrap();
        assert!((pose.x - 10.0).abs() < 1e-9 && (pose.y - 1.0).abs() < 1e-9);
        assert!((pose.yaw - PI / 2.0).abs() < 1e-9);
        assert_eq!(tf.lookup("map", "map", 0).unwrap(), Pose2::IDENTITY);
    }

    #[test]
    fn interpolates_and_respects_the_tolerance() {
        let mut tf = buffer();
        tf.insert(
            "/odom",
            "base_link",
            10 * S,
            Pose2::new(0.0, 0.0, 0.0),
            false,
        );
        tf.insert(
            "odom",
            "/base_link",
            11 * S,
            Pose2::new(2.0, 0.0, 0.2),
            false,
        );
        let mid = tf.lookup("odom", "base_link", 10 * S + S / 4).unwrap();
        assert!((mid.x - 0.5).abs() < 1e-9 && (mid.yaw - 0.05).abs() < 1e-9);
        assert!(tf.lookup("odom", "base_link", 11 * S + S / 10).is_ok());
        assert!(tf.lookup("odom", "base_link", 12 * S).is_err());
        assert!(tf.lookup("map", "base_link", 10 * S).is_err());
    }

    #[test]
    fn prunes_history_and_resets_on_a_clock_jump() {
        let mut tf = buffer();
        for k in 0..50 {
            tf.insert(
                "odom",
                "base_link",
                k * S / 10,
                Pose2::new(k as f64, 0.0, 0.0),
                false,
            );
        }
        assert!(tf.lookup("odom", "base_link", S).is_err());
        assert!(tf.lookup("odom", "base_link", 4 * S).is_ok());
        // /sim/reset: the clock starts again near zero.
        tf.insert(
            "odom",
            "base_link",
            S / 10,
            Pose2::new(-1.0, 0.0, 0.0),
            false,
        );
        assert_eq!(tf.lookup("odom", "base_link", S / 10).unwrap().x, -1.0);
        assert!(tf.lookup("odom", "base_link", 4 * S).is_err());
    }

    #[test]
    fn static_transforms_hold_at_any_time() {
        let mut tf = buffer();
        tf.insert(
            "base_link",
            "lidar_link",
            0,
            Pose2::new(0.3, 0.0, 0.0),
            true,
        );
        assert_eq!(
            tf.lookup("base_link", "lidar_link", 1000 * S).unwrap().x,
            0.3
        );
    }
}
