//! How big was it? The flow a burst lets out, from the pressure drops the
//! loggers measured (the "quantify" of the mission; `joukowsky_equation.txt`).
//!
//! A sudden burst letting out Q at a point sends a front of height
//! `F_i` Q to logger i, where `F_i` is the front per unit of flow: (B / 2) at
//! the hole, times every junction's transmission on the way, times what a
//! plastic wall keeps ([`crate::placement::first_fronts`]). With the drops
//! `d_i` each logger measured (noise `sigma_i`), the flow follows by weighted
//! least squares:
//!
//! ```text
//!   Q = sum(F_i d_i / sigma_i^2) / sum(F_i^2 / sigma_i^2)
//!   sigma_Q (noise) = 1 / sqrt(sum(F_i^2 / sigma_i^2))
//! ```
//!
//! B = a / (g A) is proportional to the wave speed, so a wave speed known
//! to +/-10% makes the size uncertain by +/-10% too; that is added in
//! quadrature (P3). The misfit - how far the loggers disagree, in noise
//! units - flags a drop spoiled by an echo or a second event.
//!
//! What it assumes: the burst opened fast compared with the time the front
//! takes to reach the loggers (a slow opening gives smaller fronts, so the
//! size comes out SMALL - a lower bound), and the measured drop is the
//! first front, before reflections return.

use jk_hydraulics::{Network, NodeId};
use jk_kernel::{Flow, Head};

use crate::error::LocalisationError;
use crate::graph::Devices;
use crate::locate::Point;
use crate::placement::first_fronts_with;

/// The estimated size of a burst.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SizeEstimate {
    /// Flow let out [m^3/s].
    pub flow: Flow,
    /// One-standard-deviation uncertainty: noise and wave-speed doubt.
    pub uncertainty: Flow,
    /// Root-mean-square disagreement between loggers, in noise units
    /// (about 1 or below: they agree; much larger: a drop is spoiled).
    pub misfit: f64,
    /// Loggers used.
    pub loggers: usize,
}

/// One logger's measurement: its node, the drop it measured (positive for
/// a fall in pressure) and its noise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drop {
    /// Where the logger is.
    pub node: NodeId,
    /// Pressure fall across the front.
    pub drop: Head,
    /// The logger's noise (one sigma).
    pub noise: Head,
}

/// The flow of a burst at `point` from the `drops` measured, with
/// wave-speed doubt `wave_speed_uncertainty` (relative, one sigma).
///
/// # Errors
/// No logger a front can reach, a non-positive noise, or the point off
/// the network.
pub fn size(
    network: &Network,
    drops: &[Drop],
    point: Point,
    wave_speed_uncertainty: f64,
) -> Result<SizeEstimate, LocalisationError> {
    size_with(
        network,
        &Devices::of(network),
        drops,
        point,
        wave_speed_uncertainty,
    )
}

/// As [`size`], with the pumps and valves in the states `devices` gives.
///
/// # Errors
/// As [`size`].
pub fn size_with(
    network: &Network,
    devices: &Devices,
    drops: &[Drop],
    point: Point,
    wave_speed_uncertainty: f64,
) -> Result<SizeEstimate, LocalisationError> {
    if drops
        .iter()
        .any(|d| d.noise.value().is_nan() || d.noise.value() <= 0.0)
    {
        return Err(LocalisationError::InvalidInput(
            "every logger's noise must be positive".into(),
        ));
    }
    let nodes: Vec<NodeId> = drops.iter().map(|d| d.node).collect();
    let per_unit = first_fronts_with(network, devices, &nodes, point, Flow::new(1.0))?;
    let used: Vec<(f64, f64, f64)> = drops
        .iter()
        .zip(&per_unit)
        .filter(|(_, f)| f.value() > 0.0)
        .map(|(d, f)| (f.value(), d.drop.value(), d.noise.value()))
        .collect();
    if used.is_empty() {
        return Err(LocalisationError::InvalidInput(
            "no logger that saw it can be reached by a front from the point".into(),
        ));
    }
    let (mut num, mut den) = (0.0, 0.0);
    for &(f, d, s) in &used {
        num += f * d / (s * s);
        den += f * f / (s * s);
    }
    let q = num / den;
    let chi2: f64 = used
        .iter()
        .map(|&(f, d, s)| ((d - f * q) / s).powi(2))
        .sum();
    #[allow(clippy::cast_precision_loss)] // a handful of loggers
    let dof = (used.len().max(2) - 1) as f64;
    let noise = 1.0 / den.sqrt();
    Ok(SizeEstimate {
        flow: Flow::new(q),
        uncertainty: Flow::new(noise.hypot(wave_speed_uncertainty * q)),
        misfit: (chi2 / dof).sqrt(),
        loggers: used.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::placement::first_fronts;
    use jk_hydraulics::{Node, Pipe};
    use jk_kernel::{Length, STANDARD_GRAVITY, WaveSpeed};

    #[test]
    fn exact_fronts_give_back_the_flow() {
        // R --400-- S1 --300-- S2 (dead end); burst 100 m from S1.
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(60.0),
        });
        let j = || Node::Junction { demand: Flow::ZERO };
        let (s1, s2) = (net.add_node(j()), net.add_node(j()));
        let p = |l| {
            Pipe::new(
                Length::new(l),
                Length::new(0.15),
                WaveSpeed::new(1000.0),
                0.02,
            )
            .unwrap()
        };
        net.add_pipe(r, s1, p(400.0)).unwrap();
        let pipe = net.add_pipe(s1, s2, p(300.0)).unwrap();
        let point = Point {
            pipe,
            distance: Length::new(100.0),
        };
        let q = 1.5e-3;
        let fronts = first_fronts(&net, &[s1, s2], point, Flow::new(q)).unwrap();
        let drops: Vec<Drop> = [s1, s2]
            .iter()
            .zip(&fronts)
            .map(|(&node, h)| Drop {
                node,
                drop: *h,
                noise: Head::new(0.02),
            })
            .collect();
        let s = size(&net, &drops, point, 0.1).unwrap();
        assert!((s.flow.value() - q).abs() < 1e-12);
        assert!(s.misfit < 1e-9);
        assert!(s.uncertainty.value() >= 0.1 * q);
    }
}
