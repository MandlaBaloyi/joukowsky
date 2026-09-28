//! One time step for the whole network.
//!
//! MOC with Courant number 1 needs dt = dx / a in EVERY pipe, and each pipe
//! needs a whole number of reaches. With one shared dt that is only possible
//! if every pipe's travel time L/a is a whole multiple of dt - which real
//! pipes never are. The standard fix (`governing_equations_and_moc.txt`
//! section 3) is to nudge each pipe's wave speed so that it is:
//!
//! ```text
//!   reaches_i = round( (L_i / a_i) / dt ),   a_i' = L_i / (reaches_i dt)
//! ```
//!
//! The nudge is a KNOWN MODEL ERROR, so it is recorded per pipe and bounded
//! by [`GridOptions::max_wave_speed_adjustment`]. We search dt = `tau_min` / n
//! for n = `min_reaches`, `min_reaches` + 1, ... (`tau_min` = shortest travel
//! time) and take the first dt that keeps every nudge within the bound.
//! Smaller bounds mean smaller dt and more computation.

use jk_kernel::{Time, WaveSpeed};

use crate::HydraulicsError;
use crate::error::positive;
use crate::model::Network;

/// Largest number of reaches tried for the shortest pipe.
const MAX_SUBDIVISIONS: u32 = 10_000;
/// Largest total number of grid nodes, to turn a mistaken setup into an
/// error instead of an out-of-memory crash.
const MAX_TOTAL_NODES: u64 = 20_000_000;

/// How the network grid is chosen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridOptions {
    /// Largest allowed relative change of any pipe's wave speed (0.01 = 1%).
    /// Wave speeds are rarely known better than +/-5-20%, so 1% is well
    /// inside the real uncertainty.
    pub max_wave_speed_adjustment: f64,
    /// Fewest reaches for the pipe with the shortest travel time. Raise it
    /// for finer time resolution (smaller dt).
    pub min_reaches: u32,
}

impl Default for GridOptions {
    fn default() -> Self {
        Self {
            max_wave_speed_adjustment: 0.01,
            min_reaches: 1,
        }
    }
}

/// The chosen time step, reaches per pipe, and the wave-speed nudges.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkGrid {
    time_step: Time,
    reaches: Vec<u32>,
    wave_speeds: Vec<WaveSpeed>,
    adjustments: Vec<f64>,
}

impl NetworkGrid {
    /// Chooses the grid for `network`.
    ///
    /// # Errors
    /// Bad options, or no time step keeps every adjustment within bounds.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // rounded, >= 1, bounded
    pub fn new(network: &Network, options: &GridOptions) -> Result<Self, HydraulicsError> {
        positive(
            "max_wave_speed_adjustment",
            options.max_wave_speed_adjustment,
        )?;
        if options.min_reaches == 0 {
            return Err(HydraulicsError::invalid("min_reaches", 0.0, ">= 1"));
        }
        let taus: Vec<f64> = network
            .pipes()
            .iter()
            .map(|p| p.pipe.travel_time().value())
            .collect();
        let Some(tau_min) = taus.iter().copied().reduce(f64::min) else {
            return Err(HydraulicsError::InvalidNetwork("no pipes".into()));
        };

        for n in options.min_reaches..=MAX_SUBDIVISIONS {
            let dt = tau_min / f64::from(n);
            let reaches: Vec<u32> = taus
                .iter()
                .map(|tau| (tau / dt).round().max(1.0) as u32)
                .collect();
            let adjustments: Vec<f64> = taus
                .iter()
                .zip(&reaches)
                .map(|(tau, &r)| tau / (f64::from(r) * dt) - 1.0)
                .collect();
            let worst = adjustments.iter().fold(0.0_f64, |m, a| m.max(a.abs()));
            if worst > options.max_wave_speed_adjustment {
                continue;
            }
            let total: u64 = reaches.iter().map(|&r| u64::from(r) + 1).sum();
            if total > MAX_TOTAL_NODES {
                break;
            }
            let wave_speeds = network
                .pipes()
                .iter()
                .zip(&adjustments)
                .map(|(p, adj)| p.pipe.wave_speed() * (1.0 + adj))
                .collect();
            return Ok(Self {
                time_step: Time::new(dt),
                reaches,
                wave_speeds,
                adjustments,
            });
        }
        Err(HydraulicsError::InvalidNetwork(format!(
            "no time step keeps every wave-speed adjustment within {:.3}% \
             (pipe travel times are too incommensurate); allow a larger \
             adjustment",
            options.max_wave_speed_adjustment * 100.0
        )))
    }

    /// The shared time step dt.
    #[must_use]
    pub fn time_step(&self) -> Time {
        self.time_step
    }

    /// Reaches per pipe, indexed like [`Network::pipes`].
    #[must_use]
    pub fn reaches(&self) -> &[u32] {
        &self.reaches
    }

    /// Wave speed each pipe is actually simulated with.
    #[must_use]
    pub fn wave_speeds(&self) -> &[WaveSpeed] {
        &self.wave_speeds
    }

    /// Relative wave-speed change per pipe (a'/a - 1): the recorded model
    /// error of the grid.
    #[must_use]
    pub fn adjustments(&self) -> &[f64] {
        &self.adjustments
    }

    /// Largest absolute relative adjustment over all pipes.
    #[must_use]
    pub fn max_adjustment(&self) -> f64 {
        self.adjustments.iter().fold(0.0, |m, a| m.max(a.abs()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Pipe;
    use crate::model::Node;
    use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY};

    fn network(lengths: &[f64]) -> Network {
        let mut net = Network::new(STANDARD_GRAVITY);
        let mut prev = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        for &l in lengths {
            let next = net.add_node(Node::Junction { demand: Flow::ZERO });
            let pipe = Pipe::new(
                Length::new(l),
                Length::new(0.15),
                WaveSpeed::new(1000.0),
                0.0,
            )
            .unwrap();
            net.add_pipe(prev, next, pipe).unwrap();
            prev = next;
        }
        net
    }

    #[test]
    fn commensurate_pipes_need_no_adjustment() {
        // 0.3 s and 0.7 s travel times: dt = 0.3 would need 2.33 reaches in
        // the long pipe; dt = 0.1 fits both exactly (3 and 7 reaches).
        let g = NetworkGrid::new(&network(&[300.0, 700.0]), &GridOptions::default()).unwrap();
        assert!((g.time_step().value() - 0.1).abs() < 1e-12);
        assert!(g.max_adjustment() < 1e-12);
        assert_eq!(g.reaches(), &[3, 7]);
    }

    #[test]
    fn awkward_lengths_are_adjusted_within_the_bound() {
        let opts = GridOptions {
            max_wave_speed_adjustment: 0.005,
            min_reaches: 2,
        };
        let g = NetworkGrid::new(&network(&[137.0, 941.3, 262.9]), &opts).unwrap();
        assert!(g.max_adjustment() <= 0.005);
        assert!(g.reaches()[0] >= 2);
        for ((&r, a), l) in g
            .reaches()
            .iter()
            .zip(g.wave_speeds())
            .zip([137.0, 941.3, 262.9])
        {
            // Courant number 1 in every pipe: a' dt = L / r.
            let dx = l / f64::from(r);
            assert!((a.value() * g.time_step().value() - dx).abs() < 1e-9);
        }
    }

    #[test]
    fn impossible_bound_is_an_error_not_a_hang() {
        let opts = GridOptions {
            max_wave_speed_adjustment: 1e-12,
            min_reaches: 1,
        };
        assert!(NetworkGrid::new(&network(&[100.0, 141.421_356_237]), &opts).is_err());
    }
}
