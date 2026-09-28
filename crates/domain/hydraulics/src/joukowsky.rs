//! The Joukowsky relations: closed-form surge estimates.
//!
//! Implements `docs/02_physics_of_transients/joukowsky_equation.txt`.
//! Valid for a sudden velocity change ("rapid": Tc <= 2L/a) of a single,
//! clean front. Not an absolute upper bound in real systems: column
//! separation, superposed reflections and trapped air can exceed it.

use jk_kernel::{Acceleration, Area, Density, Flow, Head, Pressure, Velocity, WaveSpeed};

/// Head change caused by a sudden velocity change: dH = -(a / g) * dV.
///
/// Slowing the flow (dV < 0) raises the head on the upstream side.
#[must_use]
pub fn head_change(
    velocity_change: Velocity,
    wave_speed: WaveSpeed,
    gravity: Acceleration,
) -> Head {
    Head::new(-(wave_speed.value() / gravity.value()) * velocity_change.value())
}

/// Pressure change caused by a sudden velocity change: dp = -rho * a * dV.
#[must_use]
pub fn pressure_change(
    velocity_change: Velocity,
    wave_speed: WaveSpeed,
    density: Density,
) -> Pressure {
    Pressure::new(-density.value() * wave_speed.value() * velocity_change.value())
}

/// Characteristic impedance of a pipe, B = a / (g A) [s/m^2].
///
/// Converts a flow step into a head step: dH = -B * dQ.
#[must_use]
pub fn impedance(wave_speed: WaveSpeed, area: Area, gravity: Acceleration) -> f64 {
    wave_speed.value() / (gravity.value() * area.value())
}

/// Burst flow inferred from the head drop it produces at the burst point.
///
/// A burst in the middle of a pipe sees two pipe halves in parallel
/// (impedance B/2), so |dH| = (B / 2) * `Q_L`, giving `Q_L` = 2 g A |dH| / a.
/// This is how the engine quantifies a burst from a measured step (P4).
/// Upper-bound estimate: real bursts open over a finite time.
#[must_use]
pub fn burst_flow_from_head_drop(
    head_drop: Head,
    wave_speed: WaveSpeed,
    area: Area,
    gravity: Acceleration,
) -> Flow {
    Flow::new(2.0 * head_drop.abs().value() / impedance(wave_speed, area, gravity))
}

/// Fraction of a wave's amplitude transmitted into EVERY pipe at a junction
/// (section 6): T = 2 (1/`B_in`) / sum(1/`B_i`), where the sum runs over all
/// pipes meeting at the junction, including the incoming one.
///
/// Special cases: n identical pipes (a tee, n = 3) give T = 2/3; a dead end
/// (only the incoming pipe) gives T = 2 - the pressure doubles.
#[must_use]
pub fn junction_transmission(incoming_impedance: f64, all_impedances: &[f64]) -> f64 {
    let admittance: f64 = all_impedances.iter().map(|b| 1.0 / b).sum();
    2.0 / incoming_impedance / admittance
}

/// Fraction of a wave's amplitude reflected back into the incoming pipe at a
/// junction: R = T - 1. A tee reflects -1/3; a dead end +1.
#[must_use]
pub fn junction_reflection(incoming_impedance: f64, all_impedances: &[f64]) -> f64 {
    junction_transmission(incoming_impedance, all_impedances) - 1.0
}

// Validation against the docs worked numbers: tests/hydraulics/analytical/joukowsky/.
