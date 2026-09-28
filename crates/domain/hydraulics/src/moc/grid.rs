//! Space-time grid for one pipe: N reaches, dx = L / N, dt = dx / a.
//!
//! Courant number 1 (dt = dx / a) makes the frictionless scheme exact: the
//! characteristic lines pass exactly through grid nodes, so there is no
//! interpolation and no numerical damping. On networks each pipe needs a
//! whole number of reaches at one shared dt, which forces small wave-speed
//! adjustments - that belongs to the network grid (milestone S3).

use jk_kernel::{Length, Time};

use crate::error::positive;
use crate::{HydraulicsError, Pipe};

/// Upper bound on time steps per run, to turn a mistyped duration into an
/// error instead of an out-of-memory crash.
pub const MAX_STEPS: usize = 50_000_000;

/// The MOC grid of one pipe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    pipe_length: Length,
    reaches: u32,
    time_step: Time,
}

impl Grid {
    /// Divides `pipe` into `reaches` equal reaches.
    ///
    /// # Errors
    /// [`HydraulicsError::InvalidParameter`] if `reaches` is zero.
    pub fn new(pipe: &Pipe, reaches: u32) -> Result<Self, HydraulicsError> {
        if reaches == 0 {
            return Err(HydraulicsError::invalid("reaches", 0.0, ">= 1"));
        }
        let reach_length = pipe.length() / f64::from(reaches);
        Ok(Self {
            pipe_length: pipe.length(),
            reaches,
            time_step: reach_length / pipe.wave_speed(),
        })
    }

    /// Number of reaches N.
    #[must_use]
    pub fn reaches(&self) -> u32 {
        self.reaches
    }

    /// Index of the last node (N); node 0 is the upstream end.
    #[must_use]
    pub fn last_node(&self) -> usize {
        self.reaches as usize
    }

    /// Reach length dx.
    #[must_use]
    pub fn reach_length(&self) -> Length {
        self.pipe_length / f64::from(self.reaches)
    }

    /// Time step dt = dx / a.
    #[must_use]
    pub fn time_step(&self) -> Time {
        self.time_step
    }

    /// Node nearest to distance `x` from the upstream end.
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // clamped to [0, N]
    pub fn node_at(&self, x: Length) -> usize {
        let fraction = (x / self.pipe_length).clamp(0.0, 1.0);
        (fraction * f64::from(self.reaches)).round() as usize
    }

    /// Number of steps needed to cover `duration`.
    ///
    /// # Errors
    /// A non-positive duration, or more than [`MAX_STEPS`] steps.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // bounded below
    #[allow(clippy::cast_precision_loss)] // MAX_STEPS is exact in f64
    pub fn steps_for(&self, duration: Time) -> Result<usize, HydraulicsError> {
        positive("duration", duration.value())?;
        let steps = (duration / self.time_step).ceil();
        if steps > MAX_STEPS as f64 {
            return Err(HydraulicsError::invalid(
                "duration / time step",
                steps,
                "<= 5e7 steps",
            ));
        }
        Ok(steps as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::WaveSpeed;

    fn pipe() -> Pipe {
        Pipe::new(
            Length::new(1000.0),
            Length::new(0.3),
            WaveSpeed::new(1000.0),
            0.0,
        )
        .unwrap()
    }

    #[test]
    fn courant_one_time_step() {
        let g = Grid::new(&pipe(), 20).unwrap();
        assert_eq!(g.reach_length(), Length::new(50.0));
        assert_eq!(g.time_step(), Time::new(0.05));
        assert_eq!(g.last_node(), 20);
        assert_eq!(g.node_at(Length::new(500.0)), 10);
        assert_eq!(g.node_at(Length::new(-5.0)), 0);
        assert_eq!(g.node_at(Length::new(5000.0)), 20);
    }

    #[test]
    fn step_counts_and_limits() {
        let g = Grid::new(&pipe(), 20).unwrap();
        assert_eq!(g.steps_for(Time::new(1.0)).unwrap(), 20);
        assert!(g.steps_for(Time::ZERO).is_err());
        assert!(g.steps_for(Time::new(1e9)).is_err());
        assert!(Grid::new(&pipe(), 0).is_err());
    }
}
