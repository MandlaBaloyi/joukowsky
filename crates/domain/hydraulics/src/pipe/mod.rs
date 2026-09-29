//! A single pipe: geometry, wave speed, wall friction and - for plastic
//! pipes - a creeping (viscoelastic) wall.
//!
//! Later in this folder: `wave_speed.rs` (Korteweg estimates, used as
//! priors).

pub mod friction;
pub mod viscoelastic;

use jk_kernel::{Acceleration, Area, Flow, Head, Length, Time, WaveSpeed};

use crate::error::positive;
use crate::{HydraulicsError, joukowsky};
use viscoelastic::ViscoelasticWall;

/// One pipe: geometry, wave speed and quasi-steady Darcy friction; a
/// plastic pipe also has a viscoelastic wall.
///
/// The wave speed is given, not derived: on real networks it is uncertain
/// and learned from data (`docs/02_physics_of_transients/wave_speed.txt`).
/// For a plastic pipe it is the INSTANTANEOUS (elastic) wave speed; the
/// wall's creep comes on top ([`viscoelastic`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pipe {
    length: Length,
    diameter: Length,
    wave_speed: WaveSpeed,
    friction_factor: f64,
    wall: Option<ViscoelasticWall>,
}

impl Pipe {
    /// Builds a pipe, rejecting non-physical values.
    ///
    /// # Errors
    /// [`HydraulicsError::InvalidParameter`] if length, internal diameter or
    /// wave speed are not finite and positive, or the Darcy friction factor
    /// is negative or not finite.
    pub fn new(
        length: Length,
        diameter: Length,
        wave_speed: WaveSpeed,
        friction_factor: f64,
    ) -> Result<Self, HydraulicsError> {
        positive("length", length.value())?;
        positive("diameter", diameter.value())?;
        positive("wave_speed", wave_speed.value())?;
        if !(friction_factor.is_finite() && friction_factor >= 0.0) {
            return Err(HydraulicsError::invalid(
                "friction_factor",
                friction_factor,
                "finite and >= 0",
            ));
        }
        Ok(Self {
            length,
            diameter,
            wave_speed,
            friction_factor,
            wall: None,
        })
    }

    /// The same pipe with a creeping (viscoelastic) wall - a plastic pipe.
    #[must_use]
    pub fn with_wall(self, wall: ViscoelasticWall) -> Self {
        Self {
            wall: Some(wall),
            ..self
        }
    }

    /// The viscoelastic wall, if the pipe is plastic.
    #[must_use]
    pub fn wall(&self) -> Option<&ViscoelasticWall> {
        self.wall.as_ref()
    }

    /// Pipe length.
    #[must_use]
    pub fn length(&self) -> Length {
        self.length
    }

    /// Internal diameter.
    #[must_use]
    pub fn diameter(&self) -> Length {
        self.diameter
    }

    /// Pressure-wave speed.
    #[must_use]
    pub fn wave_speed(&self) -> WaveSpeed {
        self.wave_speed
    }

    /// The same pipe with a different wave speed - used by the network grid,
    /// which must nudge wave speeds so every pipe has a whole number of
    /// reaches at one shared time step.
    ///
    /// # Errors
    /// If `wave_speed` is not finite and positive.
    pub fn with_wave_speed(&self, wave_speed: WaveSpeed) -> Result<Self, HydraulicsError> {
        positive("wave_speed", wave_speed.value())?;
        Ok(Self {
            wave_speed,
            ..*self
        })
    }

    /// Darcy-Weisbach friction factor (dimensionless).
    #[must_use]
    pub fn friction_factor(&self) -> f64 {
        self.friction_factor
    }

    /// Internal cross-sectional area.
    #[must_use]
    pub fn area(&self) -> Area {
        Area::of_circle(self.diameter)
    }

    /// Characteristic impedance B = a / (g A) [s/m^2].
    #[must_use]
    pub fn impedance(&self, gravity: Acceleration) -> f64 {
        joukowsky::impedance(self.wave_speed, self.area(), gravity)
    }

    /// One-way wave travel time L / a.
    #[must_use]
    pub fn travel_time(&self) -> Time {
        self.length / self.wave_speed
    }

    /// Round-trip time 2L/a: the closure-time limit for a "rapid" closure.
    #[must_use]
    pub fn round_trip_time(&self) -> Time {
        self.travel_time() * 2.0
    }

    /// Steady friction head loss over the whole pipe, signed with the flow.
    #[must_use]
    pub fn friction_head_loss(&self, flow: Flow, gravity: Acceleration) -> Head {
        friction::quasi_steady::head_loss(
            self.friction_factor,
            self.length,
            self.diameter,
            flow,
            gravity,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_non_physical_values() {
        let ok_d = Length::new(0.3);
        let ok_a = WaveSpeed::new(1000.0);
        assert!(Pipe::new(Length::new(-1.0), ok_d, ok_a, 0.02).is_err());
        assert!(Pipe::new(Length::new(10.0), Length::ZERO, ok_a, 0.02).is_err());
        assert!(Pipe::new(Length::new(10.0), ok_d, WaveSpeed::new(f64::NAN), 0.02).is_err());
        assert!(Pipe::new(Length::new(10.0), ok_d, ok_a, -0.01).is_err());
        assert!(Pipe::new(Length::new(10.0), ok_d, ok_a, 0.0).is_ok());
    }

    #[test]
    fn timing_matches_docs_example() {
        // joukowsky_equation.txt 4(b): L = 1000 m, a = 1250 m/s -> 2L/a = 1.6 s.
        let p = Pipe::new(
            Length::new(1000.0),
            Length::new(0.3),
            WaveSpeed::new(1250.0),
            0.02,
        )
        .unwrap();
        assert_eq!(p.travel_time(), Time::new(0.8));
        assert_eq!(p.round_trip_time(), Time::new(1.6));
    }
}
