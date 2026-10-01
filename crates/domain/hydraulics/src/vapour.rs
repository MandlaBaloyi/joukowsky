//! The vapour floor: the lowest pressure water can hold before it boils
//! (`docs/02_physics_of_transients/column_separation_and_intrusion.txt`
//! section 1).
//!
//! In gauge terms the floor is `(p_vapour - p_atm) / (rho g)`: about -10.1 m
//! at the coast, about -8.5 m on the Highveld, where the air presses less.
//! So a down-surge cavitates sooner in Johannesburg than in Durban, and the
//! engine needs each site's altitude.
//!
//! ```text
//!   p_atm(z) = 101 325 Pa (1 - 0.0065 z / 288.15)^5.25588   (ISA troposphere)
//!   p_vapour = 2 339 Pa at 20 C                              (IAPWS-95)
//! ```

use jk_kernel::{Acceleration, Head, Length, Pressure};

use crate::boundary::pump::WATER_DENSITY;

/// Vapour pressure of water at 20 C [Pa] (IAPWS-95: 2.3393 kPa).
pub const VAPOUR_PRESSURE_20C: Pressure = Pressure::new(2339.3);

/// Air pressure at `altitude` above sea level in the International
/// Standard Atmosphere (troposphere, below 11 km). Weather moves it by a
/// few percent [VERIFY against the site's barometer where one exists].
#[must_use]
pub fn standard_atmosphere(altitude: Length) -> Pressure {
    const SEA_LEVEL: f64 = 101_325.0;
    const LAPSE: f64 = 0.0065; // K/m
    const T0: f64 = 288.15; // K
    const EXPONENT: f64 = 5.255_88; // g M / (R L)
    Pressure::new(SEA_LEVEL * (1.0 - LAPSE * altitude.value() / T0).powf(EXPONENT))
}

/// The vapour floor as a gauge pressure head [m] at `altitude`, for water
/// whose vapour pressure is `vapour_pressure`.
#[must_use]
pub fn vapour_floor(altitude: Length, vapour_pressure: Pressure, gravity: Acceleration) -> Head {
    Head::new(
        (vapour_pressure.value() - standard_atmosphere(altitude).value())
            / (WATER_DENSITY * gravity.value()),
    )
}

/// Free gas carried in the water - dissolved air come out of solution,
/// entrained bubbles - for the discrete GAS cavity model (DGCM,
/// `column_separation_and_intrusion.txt` section 6). Each grid point holds
/// a small gas volume obeying the isothermal gas law between its pressure
/// and the vapour pressure:
///
/// ```text
///   Vg (H - H_floor) = alpha0 Vreach (h0 - floor)      (constant per point)
/// ```
///
/// so it shrinks under pressure and swells as the pressure falls - and
/// near the floor it plays the vapour cavity's part, without DVCM's hard
/// on/off.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FreeGas {
    /// Gas volume per volume of pipe, `alpha0`, when the pressure head is
    /// `at_head`. Laboratory water about 1e-7, field water up to about 1e-4
    /// [VERIFY for SA municipal supply].
    pub void_fraction: f64,
    /// Gauge pressure head at which `void_fraction` holds (0 = atmospheric).
    pub at_head: Head,
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::STANDARD_GRAVITY;

    #[test]
    fn the_coast_boils_lower_than_the_highveld() {
        let floor =
            |z: f64| vapour_floor(Length::new(z), VAPOUR_PRESSURE_20C, STANDARD_GRAVITY).value();
        // Sea level: (2339 - 101325) / (998.2 * 9.80665) = -10.11 m.
        assert!((floor(0.0) + 10.112).abs() < 1e-3, "{}", floor(0.0));
        // Johannesburg, ~1700 m: p_atm ~ 82.5 kPa -> -8.19 m.
        assert!((floor(1700.0) + 8.19).abs() < 0.01, "{}", floor(1700.0));
        assert!((standard_atmosphere(Length::new(1700.0)).value() - 82_500.0).abs() < 100.0);
    }
}
