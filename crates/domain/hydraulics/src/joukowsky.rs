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

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::Length;

    /// The docs round g to 9.81 in their worked numbers.
    const G_DOCS: Acceleration = Acceleration::new(9.81);

    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * a.abs().max(b.abs())
    }

    #[test]
    fn stopping_1_m_per_s_in_steel_gives_about_127_m() {
        // joukowsky_equation.txt 4(a): a ~ 1250 m/s, V0 = 1 m/s -> ~127 m.
        let dh = head_change(Velocity::new(-1.0), WaveSpeed::new(1250.0), G_DOCS);
        assert!(close(dh.value(), 127.42, 1e-3));
    }

    #[test]
    fn pressure_and_head_forms_agree() {
        let rho = Density::new(998.0);
        let dv = Velocity::new(-0.8);
        let a = WaveSpeed::new(1010.0);
        let dp = pressure_change(dv, a, rho);
        let dh = head_change(dv, a, G_DOCS);
        assert!(close(
            dp.value(),
            Pressure::from_head(dh, rho, G_DOCS).value(),
            1e-12
        ));
    }

    #[test]
    fn dn150_ac_burst_numbers_from_the_docs() {
        // joukowsky_equation.txt 5: DN150 AC, a = 1000 m/s, B ~ 5770 s/m^2;
        // a 2 l/s burst gives a ~5.8 m step.
        let area = Area::of_circle(Length::new(0.15));
        let a = WaveSpeed::new(1000.0);
        let b = impedance(a, area, G_DOCS);
        assert!(close(b, 5770.0, 1e-3));
        let step = Head::new(b / 2.0 * 0.002);
        assert!(close(step.value(), 5.77, 1e-3));
        let q = burst_flow_from_head_drop(-step, a, area, G_DOCS);
        assert!(close(q.value(), 0.002, 1e-12));
    }
}
