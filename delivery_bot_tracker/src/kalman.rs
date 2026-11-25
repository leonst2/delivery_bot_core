//! Constant-velocity Kalman filters in the plane: state [x, y, vx, vy], position measurements, white-acceleration
//! process noise. [`Imm`] runs two of them with different process noise (smooth riding vs. turning) as an interacting
//! multiple model filter, so straight riding gets smooth velocities and turns get little lag.

use std::f64::consts::PI;

use nalgebra::{Matrix2, Matrix2x4, Matrix4, Vector2, Vector4};

#[derive(Clone, Debug, PartialEq)]
pub struct ConstantVelocityKf {
    pub x: Vector4<f64>,
    pub p: Matrix4<f64>,
}

fn h() -> Matrix2x4<f64> {
    Matrix2x4::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
}

impl ConstantVelocityKf {
    pub fn new(position: [f64; 2], position_std: f64, velocity_std: f64) -> Self {
        let (pv, vv) = (position_std * position_std, velocity_std * velocity_std);
        Self {
            x: Vector4::new(position[0], position[1], 0.0, 0.0),
            p: Matrix4::from_diagonal(&Vector4::new(pv, pv, vv, vv)),
        }
    }

    pub fn predict(&mut self, dt: f64, accel_std: f64) {
        if dt <= 0.0 {
            return;
        }
        let mut f = Matrix4::identity();
        f[(0, 2)] = dt;
        f[(1, 3)] = dt;
        let q = accel_std * accel_std;
        let (d2, d3, d4) = (dt * dt * q, dt * dt * dt / 2.0 * q, dt.powi(4) / 4.0 * q);
        let process = Matrix4::new(
            d4, 0.0, d3, 0.0, //
            0.0, d4, 0.0, d3, //
            d3, 0.0, d2, 0.0, //
            0.0, d3, 0.0, d2,
        );
        self.x = f * self.x;
        self.p = f * self.p * f.transpose() + process;
    }

    fn innovation(&self, z: [f64; 2], measurement_std: f64) -> (Vector2<f64>, Matrix2<f64>) {
        let y = Vector2::new(z[0], z[1]) - h() * self.x;
        let s = h() * self.p * h().transpose()
            + Matrix2::identity() * (measurement_std * measurement_std);
        (y, s)
    }

    /// Likelihood of a measurement (Gaussian density of the innovation).
    fn likelihood(&self, z: [f64; 2], measurement_std: f64) -> f64 {
        let (y, s) = self.innovation(z, measurement_std);
        match s.try_inverse() {
            Some(s_inv) => {
                (-0.5 * (y.transpose() * s_inv * y)[(0, 0)]).exp()
                    / (2.0 * PI * s.determinant().sqrt())
            }
            None => 0.0,
        }
    }

    /// Squared Mahalanobis distance of a measurement from the predicted position.
    pub fn mahalanobis2(&self, z: [f64; 2], measurement_std: f64) -> f64 {
        let (y, s) = self.innovation(z, measurement_std);
        s.try_inverse()
            .map_or(f64::INFINITY, |s_inv| (y.transpose() * s_inv * y)[(0, 0)])
    }

    pub fn update(&mut self, z: [f64; 2], measurement_std: f64) {
        let (y, s) = self.innovation(z, measurement_std);
        let Some(s_inv) = s.try_inverse() else {
            return;
        };
        let k = self.p * h().transpose() * s_inv;
        self.x += k * y;
        // Joseph form keeps P symmetric and positive definite.
        let i_kh = Matrix4::identity() - k * h();
        let r = Matrix2::identity() * (measurement_std * measurement_std);
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }

    pub fn position(&self) -> [f64; 2] {
        [self.x[0], self.x[1]]
    }

    pub fn velocity(&self) -> [f64; 2] {
        [self.x[2], self.x[3]]
    }

    /// Row-major 2x2 covariance of x and y.
    pub fn position_covariance(&self) -> [f64; 4] {
        [
            self.p[(0, 0)],
            self.p[(0, 1)],
            self.p[(1, 0)],
            self.p[(1, 1)],
        ]
    }
}

/// Interacting multiple model filter over two constant-velocity models that differ only in their acceleration noise.
#[derive(Clone, Debug, PartialEq)]
pub struct Imm {
    models: [ConstantVelocityKf; 2],
    /// Acceleration noise (m/s^2) of each model.
    accel_std: [f64; 2],
    /// Probability of each model.
    weights: [f64; 2],
    /// Probability of switching models between two updates.
    switch_probability: f64,
    /// Combined estimate.
    combined: ConstantVelocityKf,
}

impl Imm {
    pub fn new(
        position: [f64; 2],
        position_std: f64,
        velocity_std: f64,
        accel_std: [f64; 2],
        switch_probability: f64,
    ) -> Self {
        let kf = ConstantVelocityKf::new(position, position_std, velocity_std);
        Self {
            models: [kf.clone(), kf.clone()],
            accel_std,
            weights: [0.5, 0.5],
            switch_probability,
            combined: kf,
        }
    }

    fn combine(&mut self) {
        let x = self.models[0].x * self.weights[0] + self.models[1].x * self.weights[1];
        let mut p = Matrix4::zeros();
        for (model, w) in self.models.iter().zip(self.weights) {
            let d = model.x - x;
            p += (model.p + d * d.transpose()) * w;
        }
        self.combined = ConstantVelocityKf { x, p };
    }

    pub fn predict(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        // Mixing: each model starts from a blend of both, weighted by how likely it was reached from each.
        let stay = 1.0 - self.switch_probability;
        let transition = [
            [stay, self.switch_probability],
            [self.switch_probability, stay],
        ];
        let mut predicted_weights = [0.0; 2];
        let mut mixed = self.models.clone();
        for j in 0..2 {
            let c: f64 = (0..2).map(|i| transition[i][j] * self.weights[i]).sum();
            predicted_weights[j] = c;
            if c <= 0.0 {
                continue;
            }
            let mix: Vec<f64> = (0..2)
                .map(|i| transition[i][j] * self.weights[i] / c)
                .collect();
            let x = self.models[0].x * mix[0] + self.models[1].x * mix[1];
            let mut p = Matrix4::zeros();
            for (model, weight) in self.models.iter().zip(&mix) {
                let d = model.x - x;
                p += (model.p + d * d.transpose()) * *weight;
            }
            mixed[j] = ConstantVelocityKf { x, p };
        }
        for (model, accel_std) in mixed.iter_mut().zip(self.accel_std) {
            model.predict(dt, accel_std);
        }
        self.models = mixed;
        self.weights = predicted_weights;
        self.combine();
    }

    pub fn mahalanobis2(&self, z: [f64; 2], measurement_std: f64) -> f64 {
        self.combined.mahalanobis2(z, measurement_std)
    }

    pub fn update(&mut self, z: [f64; 2], measurement_std: f64) {
        let likelihoods = [
            self.models[0].likelihood(z, measurement_std),
            self.models[1].likelihood(z, measurement_std),
        ];
        let unnormalised = [
            self.weights[0] * likelihoods[0],
            self.weights[1] * likelihoods[1],
        ];
        let total = unnormalised[0] + unnormalised[1];
        // Far outside both models (underflow): keep the weights rather than divide by zero.
        if total > 1e-300 && total.is_finite() {
            self.weights = [unnormalised[0] / total, unnormalised[1] / total];
        }
        for model in &mut self.models {
            model.update(z, measurement_std);
        }
        self.combine();
    }

    pub fn position(&self) -> [f64; 2] {
        self.combined.position()
    }

    /// Moves the position estimate of every model, leaving velocities and covariances alone.
    pub fn shift(&mut self, delta: [f64; 2]) {
        for kf in self
            .models
            .iter_mut()
            .chain(std::iter::once(&mut self.combined))
        {
            kf.x[0] += delta[0];
            kf.x[1] += delta[1];
        }
    }

    pub fn velocity(&self) -> [f64; 2] {
        self.combined.velocity()
    }

    pub fn position_covariance(&self) -> [f64; 4] {
        self.combined.position_covariance()
    }

    /// Mean variance of vx and vy (m^2/s^2).
    pub fn velocity_variance(&self) -> f64 {
        (self.combined.p[(2, 2)] + self.combined.p[(3, 3)]) / 2.0
    }

    /// Probability of the turning (high-noise) model.
    pub fn manoeuvre_probability(&self) -> f64 {
        self.weights[1]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converges_to_a_constant_velocity() {
        let mut kf = ConstantVelocityKf::new([0.0, 0.0], 0.1, 5.0);
        for k in 1..=40 {
            let t = k as f64 * 0.1;
            kf.predict(0.1, 1.0);
            // Deterministic +-5 cm measurement wobble.
            let wobble = if k % 2 == 0 { 0.05 } else { -0.05 };
            kf.update([3.0 * t + wobble, -t - wobble], 0.1);
        }
        let v = kf.velocity();
        assert!(
            (v[0] - 3.0).abs() < 0.15 && (v[1] + 1.0).abs() < 0.15,
            "velocity {v:?}"
        );
        let p = kf.position();
        assert!(
            (p[0] - 12.0).abs() < 0.1 && (p[1] + 4.0).abs() < 0.1,
            "position {p:?}"
        );
    }

    #[test]
    fn gate_distance_grows_with_uncertainty() {
        let mut kf = ConstantVelocityKf::new([0.0, 0.0], 0.1, 1.0);
        let near = kf.mahalanobis2([0.3, 0.0], 0.1);
        kf.predict(1.0, 2.0);
        assert!(kf.mahalanobis2([0.3, 0.0], 0.1) < near);
        let c = kf.position_covariance();
        assert_eq!(c[1], c[2]);
    }

    fn imm() -> Imm {
        Imm::new([0.0, 0.0], 0.15, 4.0, [0.5, 4.0], 0.05)
    }

    #[test]
    fn imm_prefers_the_smooth_model_on_a_straight_line_and_the_turning_model_in_a_turn() {
        let mut filter = imm();
        for k in 1..=50 {
            filter.predict(0.1);
            let wobble = if k % 2 == 0 { 0.03 } else { -0.03 };
            filter.update([4.0 * k as f64 * 0.1, wobble], 0.15);
        }
        assert!(
            filter.manoeuvre_probability() < 0.3,
            "{}",
            filter.manoeuvre_probability()
        );
        let v = filter.velocity();
        assert!((v[0] - 4.0).abs() < 0.2 && v[1].abs() < 0.2, "{v:?}");
        // Now a tight turn: 5 m/s on a 5 m circle (5 m/s^2).
        let start = filter.position();
        for k in 1..=10 {
            filter.predict(0.1);
            let a = k as f64 * 0.1;
            filter.update([start[0] + 5.0 * a.sin(), 5.0 * (1.0 - a.cos())], 0.15);
        }
        assert!(
            filter.manoeuvre_probability() > 0.5,
            "{}",
            filter.manoeuvre_probability()
        );
    }

    #[test]
    fn imm_keeps_its_weights_normalised_and_its_covariance_symmetric() {
        let mut filter = imm();
        filter.predict(0.1);
        filter.update([100.0, 100.0], 0.15); // far outside: likelihoods underflow
        let w = filter.weights;
        assert!((w[0] + w[1] - 1.0).abs() < 1e-12);
        let c = filter.position_covariance();
        assert_eq!(c[1], c[2]);
        assert!(filter.velocity_variance() > 0.0);
    }
}
