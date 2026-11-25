//! Acceptance criteria of delivery_bot_core#11, checked on the synthetic world (`synth`). Each test names its
//! criterion. Metrics are measured while the true centre is within LiDAR range.

use crate::geometry::{distance, Pose2};
use crate::params::Params;
use crate::pipeline::{Pipeline, Source};
use crate::synth::{rasterize, scan, Path, Rect, Rng};
use crate::tracker::TrackOutput;

const DT: f64 = 0.1; // 10 Hz LiDAR
const CYCLE_NS: i64 = 100_000_000;
const LIDAR_NOISE: f64 = 0.01; // m, like the Unity LiDAR
/// Scan-to-scan error of the robot pose from localization: x/y (m) and yaw (rad). Every scenario uses it.
const POSE_NOISE: (f64, f64) = (0.02, 0.003);
/// A track counts as following a cyclist when it is this close to the true centre.
const MATCH_DISTANCE: f64 = 1.5;
/// Accuracy is measured from this long after a track is confirmed (the filter needs a few cycles for the velocity).
const WARM_UP: f64 = 1.0;

struct Scenario {
    cyclists: Vec<Path>,
    /// Static objects that are in the map (buildings, parked cars).
    mapped: Vec<Rect>,
    /// Static objects missing from the map.
    unmapped: Vec<Rect>,
    robot: fn(f64) -> Pose2,
    duration: f64,
    /// Localization error: standard deviation of x/y (m) and yaw (rad) of the pose the tracker gets.
    pose_noise: (f64, f64),
    seed: u64,
    shuffle_points: bool,
    params: Params,
}

impl Default for Scenario {
    fn default() -> Self {
        Self {
            cyclists: Vec::new(),
            mapped: Vec::new(),
            unmapped: Vec::new(),
            robot: |_| Pose2::IDENTITY,
            duration: 10.0,
            pose_noise: POSE_NOISE,
            seed: 1,
            shuffle_points: false,
            params: Params::default(),
        }
    }
}

struct Frame {
    t: f64,
    robot: Pose2,
    /// True (position, velocity) per cyclist.
    truth: Vec<([f64; 2], [f64; 2])>,
    outputs: Vec<TrackOutput>,
}

fn run(scenario: &Scenario) -> Vec<Frame> {
    let params = scenario.params.clone();
    let mut pipeline = Pipeline::new(params.clone());
    if !scenario.mapped.is_empty() {
        pipeline.set_map(rasterize(
            &scenario.mapped,
            [-40.0, -40.0],
            [80.0, 80.0],
            0.05,
            params.static_margin,
        ));
    }
    let mut rng = Rng::new(scenario.seed);
    // Its own stream, so a shuffled run sees exactly the same noise as an unshuffled one.
    let mut shuffle_rng = Rng::new(scenario.seed ^ 0x5EED);
    let steps = (scenario.duration / DT).round() as i64;
    let mut frames = Vec::new();
    for k in 0..steps {
        let t = k as f64 * DT;
        let robot = (scenario.robot)(t);
        let states: Vec<_> = scenario.cyclists.iter().map(|p| p.state(t)).collect();
        let mut objects: Vec<Rect> = scenario
            .mapped
            .iter()
            .chain(&scenario.unmapped)
            .copied()
            .collect();
        objects.extend(
            states
                .iter()
                .map(|&(p, heading, _)| Rect::cyclist(p, heading)),
        );
        let mut points = scan(&robot, &objects, LIDAR_NOISE, &mut rng);
        if scenario.shuffle_points {
            shuffle_rng.shuffle(&mut points);
        }
        let (xy, yaw) = scenario.pose_noise;
        let estimate = Pose2::new(
            robot.x + xy * rng.gaussian(),
            robot.y + xy * rng.gaussian(),
            robot.yaw + yaw * rng.gaussian(),
        );
        assert!(pipeline.process(Source::Lidar, k * CYCLE_NS, &estimate, &points));
        frames.push(Frame {
            t,
            robot,
            truth: states.iter().map(|&(p, _, v)| (p, v)).collect(),
            outputs: pipeline.outputs(),
        });
    }
    frames
}

fn in_range(frame: &Frame, position: [f64; 2]) -> bool {
    distance([frame.robot.x, frame.robot.y], position) <= Params::default().max_range
}

/// The reported track nearest to a true position, if one is close enough.
fn matched(frame: &Frame, position: [f64; 2]) -> Option<&TrackOutput> {
    frame
        .outputs
        .iter()
        .filter(|o| distance(o.position, position) <= MATCH_DISTANCE)
        .min_by(|a, b| distance(a.position, position).total_cmp(&distance(b.position, position)))
}

struct Accuracy {
    position_rmse: f64,
    mean_speed_error: f64,
    max_prediction_error: f64,
    samples: usize,
}

/// Accuracy of the track on cyclist `index`, from `WARM_UP` after its first confirmation while it is in range.
fn accuracy(frames: &[Frame], index: usize, path: &Path) -> Accuracy {
    let horizon = Params::default().prediction_horizon;
    let first = frames
        .iter()
        .find(|f| in_range(f, f.truth[index].0) && matched(f, f.truth[index].0).is_some())
        .map(|f| f.t)
        .expect("cyclist was never tracked");
    let (mut sq, mut speed, mut prediction, mut n) = (0.0, 0.0, 0.0_f64, 0);
    for f in frames.iter().filter(|f| f.t >= first + WARM_UP) {
        let (p, v) = f.truth[index];
        if !in_range(f, p) {
            continue;
        }
        let o = matched(f, p).unwrap_or_else(|| panic!("cyclist {index} lost at t = {:.1} s", f.t));
        sq += distance(o.position, p).powi(2);
        speed += (o.velocity[0].hypot(o.velocity[1]) - v[0].hypot(v[1])).abs();
        let e = distance(o.predicted_position, path.state(f.t + horizon).0);
        prediction = prediction.max(e);
        n += 1;
    }
    assert!(n > 10, "only {n} samples");
    Accuracy {
        position_rmse: (sq / n as f64).sqrt(),
        mean_speed_error: speed / n as f64,
        max_prediction_error: prediction,
        samples: n,
    }
}

const SPEEDS: [f64; 3] = [2.0, 5.0, 8.0];

/// Straight paths that pass the robot sideways and head-on (seen end-first), and a curve.
fn paths(speed: f64) -> Vec<(&'static str, Path)> {
    vec![
        (
            "crossing",
            Path::Line {
                start: [-14.0, 4.0],
                heading: 0.0,
                speed,
            },
        ),
        (
            "oncoming",
            Path::Line {
                start: [14.0, 1.5],
                heading: std::f64::consts::PI,
                speed,
            },
        ),
        (
            "diagonal",
            Path::Line {
                start: [-9.0, -9.0],
                heading: std::f64::consts::FRAC_PI_4,
                speed,
            },
        ),
        ("curve", curve(speed)),
    ]
}

/// A left turn that passes 2 m from the robot, starting 12 m of arc before that point. The radius keeps the lateral
/// acceleration at or below 2.5 m/s^2, about what a cyclist rides in a turn (at 8 m/s: r = 25.6 m).
fn curve(speed: f64) -> Path {
    let radius = (speed * speed / 2.5).max(6.0);
    Path::Circle {
        center: [0.0, radius + 2.0],
        radius,
        phase: -std::f64::consts::FRAC_PI_2 - 12.0 / radius,
        speed,
    }
}

fn duration(path: &Path, speed: f64) -> f64 {
    match path {
        Path::Line { .. } => 26.0 / speed,
        Path::Circle { .. } => 24.0 / speed,
    }
}

/// Criterion: position RMSE < 0.3 m and speed error < 0.5 m/s for cyclists at 2-8 m/s.
#[test]
fn position_and_speed_accuracy() {
    let mut failures = Vec::new();
    for speed in SPEEDS {
        for (name, path) in paths(speed) {
            let frames = run(&Scenario {
                cyclists: vec![path],
                duration: duration(&path, speed),
                ..Default::default()
            });
            let a = accuracy(&frames, 0, &path);
            println!(
                "{name} {speed} m/s: rmse {:.3} m, speed error {:.3} m/s, prediction {:.3} m ({} samples)",
                a.position_rmse, a.mean_speed_error, a.max_prediction_error, a.samples
            );
            if a.position_rmse >= 0.3 || a.mean_speed_error >= 0.5 {
                failures.push(format!(
                    "{name} at {speed} m/s: RMSE {:.3} m, speed error {:.3} m/s",
                    a.position_rmse, a.mean_speed_error
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// Criterion: predicted positions within 0.5 m of the truth after 1.5 s for constant-velocity cyclists.
#[test]
fn constant_velocity_prediction() {
    let mut failures = Vec::new();
    for speed in SPEEDS {
        for (name, path) in paths(speed)
            .into_iter()
            .filter(|(_, p)| matches!(p, Path::Line { .. }))
        {
            let frames = run(&Scenario {
                cyclists: vec![path],
                duration: duration(&path, speed),
                ..Default::default()
            });
            let a = accuracy(&frames, 0, &path);
            if a.max_prediction_error >= 0.5 {
                failures.push(format!(
                    "{name} at {speed} m/s: prediction error {:.3} m",
                    a.max_prediction_error
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// Criterion: a new cyclist is confirmed within 3 perception cycles of entering the 12 m range.
#[test]
fn confirmation_within_three_cycles() {
    for speed in SPEEDS {
        for heading in [0.0, 0.6, -0.6] {
            let path = Path::Line {
                start: [-15.0, 2.0 - 10.0 * heading],
                heading,
                speed,
            };
            let frames = run(&Scenario {
                cyclists: vec![path],
                duration: 6.0,
                ..Default::default()
            });
            let entered = frames
                .iter()
                .position(|f| in_range(f, f.truth[0].0))
                .unwrap();
            let confirmed = frames
                .iter()
                .position(|f| matched(f, f.truth[0].0).is_some())
                .expect("never confirmed");
            assert!(
                confirmed <= entered + 2,
                "{speed} m/s, heading {heading}: entered at cycle {entered}, confirmed at {confirmed}"
            );
        }
    }
}

/// Number of times the matched track id changes for cyclist `index`, and the tracked time (s).
fn id_switches(frames: &[Frame], index: usize) -> (usize, f64) {
    let mut last = None;
    let (mut switches, mut tracked) = (0, 0.0);
    for f in frames {
        if let Some(o) = matched(f, f.truth[index].0) {
            if last.is_some_and(|id| id != o.id) {
                switches += 1;
            }
            last = Some(o.id);
            tracked += DT;
        }
    }
    (switches, tracked)
}

/// Criterion: a cyclist keeps its ID while visible (<= 1 ID switch per 100 tracked seconds). Two cyclists circle the
/// robot in opposite directions (2.5 m/s^2 lateral acceleration, like `curve`), so the inner one occludes the outer
/// one twice per lap, and two more cross each other.
#[test]
fn ids_survive_occlusion_and_crossing() {
    let frames = run(&Scenario {
        cyclists: vec![
            Path::Circle {
                center: [0.0, 0.0],
                radius: 6.4,
                phase: 0.0,
                speed: 4.0,
            },
            Path::Circle {
                center: [0.0, 0.0],
                radius: 10.0,
                phase: 1.0,
                speed: -5.0,
            },
        ],
        duration: 150.0,
        seed: 3,
        ..Default::default()
    });
    for index in 0..2 {
        let (switches, tracked) = id_switches(&frames, index);
        println!("circling cyclist {index}: {switches} id switches in {tracked:.0} s");
        assert!(
            tracked > 100.0,
            "cyclist {index} tracked only {tracked:.0} s"
        );
        assert!(
            switches as f64 <= tracked / 100.0,
            "cyclist {index}: {switches} switches in {tracked:.0} s"
        );
    }

    // Perpendicular paths through (6, 3), 0.4 s apart.
    let frames = run(&Scenario {
        cyclists: vec![
            Path::Line {
                start: [-4.0, 3.0],
                heading: 0.0,
                speed: 5.0,
            },
            Path::Line {
                start: [6.0, -9.0],
                heading: std::f64::consts::FRAC_PI_2,
                speed: 5.0,
            },
        ],
        duration: 4.0,
        ..Default::default()
    });
    for index in 0..2 {
        assert_eq!(
            id_switches(&frames, index).0,
            0,
            "crossing cyclist {index} switched ids"
        );
    }
}

/// A small city block: buildings and parked cars in the map, and one bin that the map doesn't know.
fn static_world() -> (Vec<Rect>, Vec<Rect>) {
    let mapped = vec![
        Rect::new([0.0, 12.0], 0.0, 30.0, 8.0),
        Rect::new([0.0, -12.0], 0.0, 30.0, 8.0),
        Rect::new([-20.0, 0.0], 0.0, 6.0, 30.0),
        Rect::new([-6.0, 6.5], 0.0, 4.5, 1.8),
        Rect::new([0.0, 6.5], 0.0, 4.5, 1.8),
        Rect::new([7.0, -6.5], 0.0, 4.5, 1.8),
        Rect::new([12.0, 3.0], 0.3, 4.5, 1.8),
    ];
    let unmapped = vec![Rect::new([4.0, -5.5], 0.0, 0.6, 0.6)];
    (mapped, unmapped)
}

/// Criterion: no false tracks on static-only scenes over 5 minutes; parked cars and buildings are never reported as
/// moving. The robot drives back and forth with a noisy pose estimate. Only the unmapped bin may be reported, and
/// never as moving.
#[test]
fn static_scene_has_no_moving_tracks() {
    let (mapped, unmapped) = static_world();
    let bin = unmapped[0].center;
    let frames = run(&Scenario {
        mapped,
        unmapped,
        robot: |t| Pose2::new(8.0 * (t * std::f64::consts::TAU / 60.0).sin(), 0.0, 0.0),
        duration: 300.0,
        seed: 11,
        ..Default::default()
    });
    for f in &frames {
        for o in &f.outputs {
            assert!(
                distance(o.position, bin) < 1.0,
                "false track at {:?}, t = {:.1} s",
                o.position,
                f.t
            );
            assert_eq!(
                o.velocity,
                [0.0, 0.0],
                "bin reported as moving at t = {:.1} s",
                f.t
            );
        }
    }
    assert!(
        frames.iter().any(|f| !f.outputs.is_empty()),
        "the unmapped bin was never tracked"
    );
}

/// Criterion: identical input gives identical IDs and ordering, also when the points of each scan come in a
/// different order.
#[test]
fn identical_input_gives_identical_output() {
    let (mapped, unmapped) = static_world();
    let scenario = |shuffle_points| Scenario {
        cyclists: vec![
            Path::Line {
                start: [-14.0, 3.0],
                heading: 0.0,
                speed: 6.0,
            },
            Path::Circle {
                center: [0.0, 0.0],
                radius: 4.0,
                phase: 0.0,
                speed: -3.0,
            },
        ],
        mapped: mapped.clone(),
        unmapped: unmapped.clone(),
        duration: 8.0,
        seed: 5,
        shuffle_points,
        ..Default::default()
    };
    let outputs = |frames: Vec<Frame>| frames.into_iter().map(|f| f.outputs).collect::<Vec<_>>();
    let a = outputs(run(&scenario(false)));
    assert_eq!(a, outputs(run(&scenario(false))));
    assert_eq!(a, outputs(run(&scenario(true))));
    assert!(
        a.iter().any(|o| o.len() == 2),
        "expected both cyclists tracked at some point"
    );
}
