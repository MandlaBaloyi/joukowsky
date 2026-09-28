//! Reservoir - pipe - valve: the classic single-pipe transient.
//!
//! Validation case V-A1 of `docs/07_validation/validation_strategy.txt`.
//! Built from the shared MOC pieces: [`Grid`], [`Characteristics`], the
//! reservoir and valve boundaries, and [`Trace`].
//!
//! Not modelled yet (milestones S5-S7): unsteady friction, viscoelastic
//! walls, cavitation. Heads below the vapour floor are therefore reported,
//! not corrected, and are not physical.
//!
//! The pipe is taken as horizontal at datum 0, so head is gauge pressure
//! head along the whole pipe.

use jk_kernel::{Acceleration, Flow, Head, Time};

use crate::boundary::{reservoir, valve};
use crate::error::{finite, positive};
use crate::moc::characteristics::Characteristics;
use crate::moc::grid::Grid;
use crate::moc::trace::Trace;
use crate::{HydraulicsError, Pipe, ValveClosure};

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

impl ReservoirPipeValve {
    /// The MOC grid of this system.
    ///
    /// # Errors
    /// If `reaches` is zero.
    pub fn grid(&self) -> Result<Grid, HydraulicsError> {
        Grid::new(&self.pipe, self.reaches)
    }

    /// Index of the valve node (the last node).
    #[must_use]
    pub fn valve_node(&self) -> usize {
        self.reaches as usize
    }

    /// Steady head at the valve before anything moves.
    #[must_use]
    pub fn steady_valve_head(&self) -> Head {
        self.reservoir_head
            - self
                .pipe
                .friction_head_loss(self.initial_flow, self.gravity)
    }

    /// Checks the configuration; returns the grid and the steady head drop
    /// across the valve.
    fn validate(&self) -> Result<(Grid, f64), HydraulicsError> {
        let grid = self.grid()?;
        positive("gravity", self.gravity.value())?;
        finite("reservoir_head", self.reservoir_head.value())?;
        finite("downstream_head", self.downstream_head.value())?;
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
        Ok((grid, head_drop))
    }

    /// Runs the transient for `duration` and records head and flow at the
    /// `probes` (node indices, 0 = reservoir, [`Self::valve_node`] = valve).
    ///
    /// # Errors
    /// Invalid configuration, a non-positive duration, too many steps, or a
    /// probe beyond the last node.
    pub fn simulate(&self, duration: Time, probes: &[usize]) -> Result<Trace, HydraulicsError> {
        let (grid, head_drop) = self.validate()?;
        let n = grid.last_node();
        if let Some(&node) = probes.iter().find(|&&p| p > n) {
            return Err(HydraulicsError::ProbeOutOfRange { node, last: n });
        }
        let steps = grid.steps_for(duration)?;
        let dt = grid.time_step();
        let c = Characteristics::new(&self.pipe, grid.reach_length(), self.gravity);
        let b = c.impedance();
        let hr = self.reservoir_head.value();
        let hd = self.downstream_head.value();
        let q0 = self.initial_flow.value();

        // Steady state: uniform flow, head falling by R Q|Q| per reach.
        let mut q = vec![q0; n + 1];
        let mut h: Vec<f64> = (0..=grid.reaches())
            .map(|i| hr - f64::from(i) * c.reach_head_loss(q0))
            .collect();
        let mut h_new = h.clone();
        let mut q_new = q.clone();

        let mut trace = Trace::new(dt, probes.len(), steps + 1);
        let record = |trace: &mut Trace, h: &[f64], q: &[f64]| {
            for (j, &node) in probes.iter().enumerate() {
                trace.push(j, h[node], q[node]);
            }
        };
        record(&mut trace, &h, &q);

        for k in 1..=steps {
            #[allow(clippy::cast_precision_loss)] // k <= MAX_STEPS, exact in f64
            let t = dt * k as f64;

            for i in 1..n {
                let cp = c.c_plus(h[i - 1], q[i - 1]);
                let cm = c.c_minus(h[i + 1], q[i + 1]);
                (h_new[i], q_new[i]) = c.interior(cp, cm);
            }

            (h_new[0], q_new[0]) = reservoir::upstream(hr, c.c_minus(h[1], q[1]), b);

            let cv = valve::coefficient(self.closure.opening(t), q0, head_drop);
            (h_new[n], q_new[n]) = valve::downstream(c.c_plus(h[n - 1], q[n - 1]), hd, b, cv);

            std::mem::swap(&mut h, &mut h_new);
            std::mem::swap(&mut q, &mut q_new);
            record(&mut trace, &h, &q);
        }

        Ok(trace)
    }
}

#[cfg(test)]
mod tests {
    //! Input checks only. The physics validation of this solver (V-A1, V-A2,
    //! steady state, friction, grid independence) lives in
    //! `tests/03_single_pipe/` at the repository root.

    use super::*;
    use jk_kernel::{Length, STANDARD_GRAVITY, Velocity, WaveSpeed};

    fn system(reaches: u32) -> ReservoirPipeValve {
        let pipe = Pipe::new(
            Length::new(1000.0),
            Length::new(0.3),
            WaveSpeed::new(1000.0),
            0.0,
        )
        .unwrap();
        ReservoirPipeValve {
            reservoir_head: Head::new(100.0),
            initial_flow: Velocity::new(1.0) * pipe.area(),
            pipe,
            downstream_head: Head::new(50.0),
            closure: ValveClosure::Instantaneous,
            reaches,
            gravity: STANDARD_GRAVITY,
        }
    }

    #[test]
    fn rejects_bad_setups() {
        let mut s = system(10);
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
        s.initial_flow = Flow::new(-0.1);
        assert!(s.simulate(Time::new(1.0), &[0]).is_err());
        s.initial_flow = Flow::new(0.07);
        s.reaches = 0;
        assert!(s.simulate(Time::new(1.0), &[0]).is_err());
    }
}
