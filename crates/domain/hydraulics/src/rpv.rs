//! Reservoir - pipe - valve, solved by the method of characteristics.
//!
//! Implements `docs/02_physics_of_transients/governing_equations_and_moc.txt`
//! sections 1-2 for one pipe, and validation case V-A1 of
//! `docs/07_validation/validation_strategy.txt`.
//!
//! Scheme: N reaches of length dx, time step dt = dx / a (Courant number 1,
//! exact for the frictionless part), quasi-steady Darcy friction. With
//! B = a / (g A) and R = f dx / (2 g D A^2), for interior node P between
//! neighbours A (left) and B (right) at the previous step:
//!
//! ```text
//!   C+ : H_P = C_P - B Q_P,   C_P = H_A + B Q_A - R Q_A |Q_A|
//!   C- : H_P = C_M + B Q_P,   C_M = H_B - B Q_B + R Q_B |Q_B|
//! ```
//!
//! Boundaries: fixed reservoir head upstream; at the valve the orifice law
//! Q = tau Q0 sqrt((H - `H_down`) / dH0), valid for flow in either direction.
//!
//! Not modelled yet (later milestones S5-S7): unsteady friction,
//! viscoelastic walls, cavitation. Heads below the vapour floor are
//! therefore reported, not corrected, and are not physical.
//!
//! The pipe is taken as horizontal at datum 0, so head is gauge pressure
//! head along the whole pipe.

use jk_kernel::{Acceleration, Flow, Head, Length, Time};

use crate::pipe::positive;
use crate::{HydraulicsError, Pipe, ValveClosure};

/// Upper bound on time steps per run, to turn a mistyped duration into an
/// error instead of an out-of-memory crash.
const MAX_STEPS: f64 = 50_000_000.0;

/// A reservoir feeding a single pipe that ends in a valve.
#[derive(Debug, Clone, PartialEq)]
pub struct ReservoirPipeValve {
    /// Constant head of the upstream reservoir.
    pub reservoir_head: Head,
    /// The pipe.
    pub pipe: Pipe,
    /// Steady flow before the valve moves (>= 0, reservoir towards valve).
    pub initial_flow: Flow,
    /// Head just downstream of the valve (0 for discharge to atmosphere).
    pub downstream_head: Head,
    /// How the valve moves from t = 0.
    pub closure: ValveClosure,
    /// Number of MOC reaches; the time step is L / (reaches * a).
    pub reaches: u32,
    /// Gravitational acceleration.
    pub gravity: Acceleration,
}

/// Head and flow histories at the requested probe nodes.
///
/// Sample k is at time k * dt; sample 0 is the steady state.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    time_step: Time,
    probes: Vec<usize>,
    heads: Vec<Vec<Head>>,
    flows: Vec<Vec<Flow>>,
}

impl Trace {
    /// The time step dt.
    #[must_use]
    pub fn time_step(&self) -> Time {
        self.time_step
    }

    /// Number of samples per probe (steps + 1).
    #[must_use]
    pub fn len(&self) -> usize {
        self.heads.first().map_or(0, Vec::len)
    }

    /// True if nothing was recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Time of sample `k`.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // k < MAX_STEPS, exact in f64
    pub fn time(&self, k: usize) -> Time {
        self.time_step * k as f64
    }

    /// Node indices that were recorded, in request order.
    #[must_use]
    pub fn probes(&self) -> &[usize] {
        &self.probes
    }

    /// Head history of the `i`-th requested probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn head(&self, i: usize) -> &[Head] {
        &self.heads[i]
    }

    /// Flow history of the `i`-th requested probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn flow(&self, i: usize) -> &[Flow] {
        &self.flows[i]
    }

    /// Highest head seen at the `i`-th probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn max_head(&self, i: usize) -> Head {
        self.heads[i]
            .iter()
            .copied()
            .fold(Head::new(f64::NEG_INFINITY), Head::max)
    }

    /// Lowest head seen at the `i`-th probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn min_head(&self, i: usize) -> Head {
        self.heads[i]
            .iter()
            .copied()
            .fold(Head::new(f64::INFINITY), Head::min)
    }
}

impl ReservoirPipeValve {
    /// Index of the valve node (the last node).
    #[must_use]
    pub fn valve_node(&self) -> usize {
        self.reaches as usize
    }

    /// Node nearest to distance `x` from the reservoir.
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // clamped to [0, reaches]
    pub fn node_at(&self, x: Length) -> usize {
        let fraction = (x / self.pipe.length()).clamp(0.0, 1.0);
        (fraction * f64::from(self.reaches)).round() as usize
    }

    /// Reach length dx.
    #[must_use]
    pub fn reach_length(&self) -> Length {
        self.pipe.length() / f64::from(self.reaches)
    }

    /// MOC time step dt = dx / a.
    #[must_use]
    pub fn time_step(&self) -> Time {
        self.reach_length() / self.pipe.wave_speed()
    }

    /// Steady head at the valve before anything moves.
    #[must_use]
    pub fn steady_valve_head(&self) -> Head {
        self.reservoir_head
            - self
                .pipe
                .friction_head_loss(self.initial_flow, self.gravity)
    }

    fn validate(&self) -> Result<f64, HydraulicsError> {
        if self.reaches == 0 {
            return Err(HydraulicsError::invalid("reaches", 0.0, ">= 1"));
        }
        positive("gravity", self.gravity.value())?;
        for (name, value) in [
            ("reservoir_head", self.reservoir_head.value()),
            ("downstream_head", self.downstream_head.value()),
        ] {
            if !value.is_finite() {
                return Err(HydraulicsError::invalid(name, value, "finite"));
            }
        }
        let q0 = self.initial_flow.value();
        if !(q0.is_finite() && q0 >= 0.0) {
            return Err(HydraulicsError::invalid(
                "initial_flow",
                q0,
                "finite and >= 0",
            ));
        }
        self.closure.validate()?;
        let head_drop = (self.steady_valve_head() - self.downstream_head).value();
        if q0 > 0.0 && head_drop <= 0.0 {
            return Err(HydraulicsError::NoHeadDropAtValve { head_drop });
        }
        Ok(head_drop)
    }

    /// Runs the transient for `duration` and records head and flow at the
    /// `probes` (node indices, 0 = reservoir, [`Self::valve_node`] = valve).
    ///
    /// # Errors
    /// Invalid configuration, a non-positive duration, too many steps, or a
    /// probe beyond the last node.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // steps checked below
    pub fn simulate(&self, duration: Time, probes: &[usize]) -> Result<Trace, HydraulicsError> {
        let head_drop = self.validate()?;
        positive("duration", duration.value())?;
        let n = self.valve_node();
        if let Some(&node) = probes.iter().find(|&&p| p > n) {
            return Err(HydraulicsError::ProbeOutOfRange { node, last: n });
        }
        let dt = self.time_step();
        let steps = (duration / dt).ceil();
        if steps > MAX_STEPS {
            return Err(HydraulicsError::invalid(
                "duration / time step",
                steps,
                "<= 5e7 steps",
            ));
        }
        let steps = steps as usize;

        let g = self.gravity.value();
        let area = self.pipe.area().value();
        let b = self.pipe.impedance(self.gravity);
        let r = self.pipe.friction_factor() * self.reach_length().value()
            / (2.0 * g * self.pipe.diameter().value() * area * area);
        let hr = self.reservoir_head.value();
        let hd = self.downstream_head.value();
        let q0 = self.initial_flow.value();

        // Steady state: uniform flow, head falling linearly with friction.
        let mut q = vec![q0; n + 1];
        let mut h: Vec<f64> = (0..=self.reaches)
            .map(|i| hr - f64::from(i) * r * q0 * q0.abs())
            .collect();
        let mut h_new = h.clone();
        let mut q_new = q.clone();

        let mut heads = vec![Vec::with_capacity(steps + 1); probes.len()];
        let mut flows = vec![Vec::with_capacity(steps + 1); probes.len()];
        let mut record = |h: &[f64], q: &[f64]| {
            for (j, &node) in probes.iter().enumerate() {
                heads[j].push(Head::new(h[node]));
                flows[j].push(Flow::new(q[node]));
            }
        };
        record(&h, &q);

        for k in 1..=steps {
            #[allow(clippy::cast_precision_loss)] // k <= MAX_STEPS, exact in f64
            let t = dt * k as f64;

            for i in 1..n {
                let (qa, qb) = (q[i - 1], q[i + 1]);
                let cp = h[i - 1] + b * qa - r * qa * qa.abs();
                let cm = h[i + 1] - b * qb + r * qb * qb.abs();
                h_new[i] = f64::midpoint(cp, cm);
                q_new[i] = (cp - cm) / (2.0 * b);
            }

            // Upstream reservoir: fixed head, C- from node 1.
            let q1 = q[1];
            let cm = h[1] - b * q1 + r * q1 * q1.abs();
            h_new[0] = hr;
            q_new[0] = (hr - cm) / b;

            // Downstream valve: C+ from node n-1 with the orifice law.
            let qa = q[n - 1];
            let cp = h[n - 1] + b * qa - r * qa * qa.abs();
            let tau_q0 = self.closure.opening(t) * q0;
            let cv = if q0 > 0.0 {
                tau_q0 * tau_q0 / (2.0 * head_drop)
            } else {
                0.0
            };
            let qv = valve_flow(cp - hd, b, cv);
            q_new[n] = qv;
            h_new[n] = cp - b * qv;

            std::mem::swap(&mut h, &mut h_new);
            std::mem::swap(&mut q, &mut q_new);
            record(&h, &q);
        }

        Ok(Trace {
            time_step: dt,
            probes: probes.to_vec(),
            heads,
            flows,
        })
    }
}

/// Flow through the valve from its C+ characteristic and the orifice law.
///
/// Solves Q|Q| = 2 Cv (d - B Q) with d = `C_P` - `H_down` and
/// Cv = (tau Q0)^2 / (2 dH0). Written in the form without subtractive
/// cancellation, Q = 2 Cv d / (B Cv + sqrt((B Cv)^2 + 2 Cv |d|)), which
/// covers both flow directions.
fn valve_flow(d: f64, b: f64, cv: f64) -> f64 {
    if cv == 0.0 || d == 0.0 {
        return 0.0;
    }
    let bcv = b * cv;
    2.0 * cv * d / (bcv + (bcv * bcv + 2.0 * cv * d.abs()).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joukowsky;
    use jk_kernel::{STANDARD_GRAVITY, Velocity, WaveSpeed};

    const V0: f64 = 1.0;

    /// 1000 m of DN300 with a = 1000 m/s: 2L/a = 2 s, period 4L/a = 4 s.
    fn system(friction: f64, closure: ValveClosure, reaches: u32) -> ReservoirPipeValve {
        let pipe = Pipe::new(
            Length::new(1000.0),
            Length::new(0.3),
            WaveSpeed::new(1000.0),
            friction,
        )
        .unwrap();
        ReservoirPipeValve {
            reservoir_head: Head::new(100.0),
            initial_flow: Velocity::new(V0) * pipe.area(),
            pipe,
            downstream_head: Head::new(50.0),
            closure,
            reaches,
            gravity: STANDARD_GRAVITY,
        }
    }

    fn surge(s: &ReservoirPipeValve) -> f64 {
        joukowsky::head_change(Velocity::new(-V0), s.pipe.wave_speed(), s.gravity).value()
    }

    /// V-A1: frictionless instantaneous closure gives an exact square wave at
    /// the valve: +dH for 2L/a, then -dH for 2L/a, repeating every 4L/a.
    #[test]
    fn va1_frictionless_instant_closure_is_a_joukowsky_square_wave() {
        let s = system(0.0, ValveClosure::Instantaneous, 20);
        let dh = surge(&s);
        let trace = s.simulate(Time::new(12.0), &[s.valve_node()]).unwrap();
        // 20 reaches -> dt = 0.05 s, so 2L/a = 40 steps and 4L/a = 80.
        // The valve is shut from step 1 (t = dt), so every front arrives one
        // step after its ideal time: high for steps 1..=40, low for 41..=80.
        assert_eq!(trace.time_step(), Time::new(0.05));
        let hr = 100.0;
        for (k, h) in trace.head(0).iter().enumerate() {
            let expected = match k {
                0 => hr,
                _ if (k - 1) % 80 < 40 => hr + dh,
                _ => hr - dh,
            };
            assert!(
                (h.value() - expected).abs() < 1e-9,
                "step {k}: head {h}, expected {expected} m"
            );
        }
        for (k, qv) in trace.flow(0).iter().enumerate().skip(1) {
            assert!(qv.value().abs() < 1e-15, "valve flow at step {k} is {qv}");
        }
    }

    /// The Joukowsky peak does not depend on the grid.
    #[test]
    fn peak_is_grid_independent() {
        for reaches in [1, 7, 20, 64] {
            let s = system(0.0, ValveClosure::Instantaneous, reaches);
            let dh = surge(&s);
            let trace = s.simulate(Time::new(8.0), &[s.valve_node()]).unwrap();
            assert!(
                (trace.max_head(0).value() - (100.0 + dh)).abs() < 1e-9,
                "reaches {reaches}"
            );
            assert!(
                (trace.min_head(0).value() - (100.0 - dh)).abs() < 1e-9,
                "reaches {reaches}"
            );
        }
    }

    /// Nothing moves -> the steady state (with friction) is preserved.
    #[test]
    fn steady_state_is_preserved() {
        let s = system(0.02, ValveClosure::Open, 25);
        let probes = [0, 12, s.valve_node()];
        let trace = s.simulate(Time::new(20.0), &probes).unwrap();
        for (j, &node) in probes.iter().enumerate() {
            let h0 = trace.head(j)[0].value();
            let q0 = trace.flow(j)[0].value();
            for (h, q) in trace.head(j).iter().zip(trace.flow(j)) {
                assert!(
                    (h.value() - h0).abs() < 1e-9,
                    "node {node} head drifted to {h}"
                );
                assert!(
                    (q.value() - q0).abs() < 1e-12,
                    "node {node} flow drifted to {q}"
                );
            }
        }
        assert!((trace.head(2)[0] - s.steady_valve_head()).value().abs() < 1e-9);
    }

    /// With friction the valve head keeps rising after the first front
    /// (line packing), and the oscillation decays.
    #[test]
    fn friction_causes_line_packing_and_damping() {
        let s = system(0.03, ValveClosure::Instantaneous, 20);
        let trace = s.simulate(Time::new(16.0), &[s.valve_node()]).unwrap();
        let h = trace.head(0);
        assert!(h[39] > h[1], "no line packing: {} vs {}", h[39], h[1]);
        let peak = |from: usize, to: usize| {
            h[from..to]
                .iter()
                .copied()
                .fold(Head::new(f64::MIN), Head::max)
        };
        assert!(peak(160, 200) < peak(0, 80), "oscillation did not decay");
    }

    /// V-A2 (coarse): a slow closure (10 x 2L/a) gives a much smaller surge
    /// than Joukowsky, but still a surge.
    #[test]
    fn slow_closure_surge_is_well_below_joukowsky() {
        let s = system(0.0, ValveClosure::linear(Time::new(20.0)), 20);
        let dh = surge(&s);
        let trace = s.simulate(Time::new(30.0), &[s.valve_node()]).unwrap();
        let rise = trace.max_head(0).value() - 100.0;
        assert!(
            rise > 0.05 * dh && rise < 0.5 * dh,
            "rise {rise} m vs Joukowsky {dh} m"
        );
    }

    #[test]
    fn rejects_bad_setups() {
        let mut s = system(0.0, ValveClosure::Instantaneous, 10);
        assert!(s.simulate(Time::ZERO, &[0]).is_err());
        assert!(matches!(
            s.simulate(Time::new(1.0), &[11]),
            Err(HydraulicsError::ProbeOutOfRange { node: 11, last: 10 })
        ));
        s.downstream_head = Head::new(100.0);
        assert!(matches!(
            s.simulate(Time::new(1.0), &[0]),
            Err(HydraulicsError::NoHeadDropAtValve { .. })
        ));
        s.downstream_head = Head::new(50.0);
        s.reaches = 0;
        assert!(s.simulate(Time::new(1.0), &[0]).is_err());
    }

    #[test]
    fn valve_flow_solves_the_orifice_law_both_ways() {
        let (b, cv) = (5000.0, 1e-5);
        for d in [3.0, -3.0, 1e-9, 250.0] {
            let q = valve_flow(d, b, cv);
            let residual = q * q.abs() - 2.0 * cv * (d - b * q);
            assert!(
                residual.abs() <= 1e-12 * 2.0 * cv * d.abs(),
                "d = {d}: residual {residual}"
            );
            assert!(
                q.is_sign_positive() == d.is_sign_positive(),
                "d = {d}: q = {q}"
            );
        }
        assert!(valve_flow(3.0, b, 0.0).abs() < f64::EPSILON);
    }
}
