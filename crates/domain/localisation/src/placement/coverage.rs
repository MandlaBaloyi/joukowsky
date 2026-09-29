//! How well a set of sensors can locate a burst anywhere on the network.

use jk_hydraulics::{Network, NodeId, PipeId};
use jk_kernel::{Flow, Head, Length, Time};

use super::bench::Bench;
use crate::error::{LocalisationError, positive};
use crate::locate::LocaliseOptions;

/// How placements are scored.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlacementOptions {
    /// Bursts are tried every `source_step` along every pipe (at the
    /// middle of each step). Default 20 m.
    pub source_step: Length,
    /// One-sigma timing uncertainty of each arrival: front picking plus the
    /// logger's clock. Default 1 ms - ten times what picking achieved in
    /// simulation, allowing for fronts smeared by plastic pipes [VERIFY on
    /// field data].
    pub timing: Time,
    /// How each burst is located. Its `wave_speed_uncertainty` is the doubt
    /// on the book wave speeds: default 0.1 (one sigma, i.e. +/-20% at two
    /// sigma - `wave_speed.txt`: asset records are unreliable). Candidate
    /// spacing 5 m (plenty against a 20 m target, 2.5x faster than 2 m).
    pub localise: LocaliseOptions,
    /// A burst counts as PINPOINTED when its 90% region is no longer than
    /// this (about one excavation).
    pub pinpoint: Length,
    /// The burst to plan for: only loggers that would FEEL a sudden burst
    /// of this size count ([`mod@super::detectability`]). Default 1 l/s. None:
    /// every logger feels every burst (geometry only).
    pub burst: Option<Flow>,
    /// Each logger's noise (one sigma). Default 2 cm.
    pub noise: Head,
    /// A logger feels a front of at least this many noise sigmas. Default
    /// 6: measured as always caught and timed to 2 samples; 5 is caught
    /// 96% of the time (tests/detection/analytical/sensitivity.rs).
    pub detect_units: f64,
}

impl Default for PlacementOptions {
    fn default() -> Self {
        Self {
            source_step: Length::new(20.0),
            timing: Time::new(1e-3),
            localise: LocaliseOptions {
                spacing: Length::new(5.0),
                wave_speed_uncertainty: 0.1,
                ..LocaliseOptions::default()
            },
            pinpoint: Length::new(20.0),
            burst: Some(Flow::new(1e-3)),
            noise: Head::new(0.02),
            detect_units: 6.0,
        }
    }
}

/// One pipe's figures.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeCoverage {
    /// The pipe.
    pub pipe: PipeId,
    /// Its length.
    pub length: Length,
    /// Share of its length where a burst is pinpointed (0 to 1).
    pub pinpointed: f64,
    /// Median size of the 90% region for a burst on it.
    pub median_region: Length,
}

/// How well a set of sensors covers the network.
#[derive(Debug, Clone, PartialEq)]
pub struct Coverage {
    /// The sensors.
    pub sensors: Vec<NodeId>,
    /// Size of the 90% region, averaged over bursts anywhere (weighted by
    /// pipe length and risk). THE score: smaller is better.
    pub expected_region: Length,
    /// Share of the (weighted) network where a burst is pinpointed.
    pub pinpointed: f64,
    /// Per pipe, in network order.
    pub pipes: Vec<PipeCoverage>,
}

impl Coverage {
    /// Whether this placement is better than `other`: a smaller expected
    /// region; on a tie, more pinpointed.
    #[must_use]
    pub fn beats(&self, other: &Self) -> bool {
        let (a, b) = (self.expected_region.value(), other.expected_region.value());
        if (a - b).abs() > 1e-9 * a.max(b) {
            a < b
        } else {
            self.pinpointed > other.pinpointed
        }
    }
}

/// Scores `sensors` on `network`. `weights` (one per pipe, e.g. burst risk
/// per metre) default to 1: every metre of pipe counts the same.
///
/// # Errors
/// Fewer than two sensors, an unknown or repeated node, weights that do not
/// match the pipes or are negative, or options out of range.
pub fn coverage(
    network: &Network,
    sensors: &[NodeId],
    weights: Option<&[f64]>,
    options: &PlacementOptions,
) -> Result<Coverage, LocalisationError> {
    validate(network, sensors, weights, options)?;
    let mut bench = Bench::new(network, weights, options);
    let state = bench.state(sensors);
    Ok(bench.coverage(sensors, state.regions()))
}

pub(super) fn validate(
    network: &Network,
    sensors: &[NodeId],
    weights: Option<&[f64]>,
    options: &PlacementOptions,
) -> Result<(), LocalisationError> {
    positive("source_step", options.source_step.value())?;
    positive("timing", options.timing.value())?;
    positive("pinpoint", options.pinpoint.value())?;
    positive("noise", options.noise.value())?;
    positive("detect_units", options.detect_units)?;
    if let Some(q) = options.burst {
        positive("burst", q.value())?;
    }
    if sensors.len() < 2 {
        return Err(LocalisationError::InvalidInput(
            "at least two sensors are needed to locate anything".into(),
        ));
    }
    for (i, s) in sensors.iter().enumerate() {
        if s.index() >= network.nodes().len() {
            return Err(LocalisationError::InvalidInput(format!(
                "sensor node {} does not exist",
                s.index()
            )));
        }
        if sensors[..i].contains(s) {
            return Err(LocalisationError::InvalidInput(format!(
                "node {} is listed twice",
                s.index()
            )));
        }
    }
    if let Some(w) = weights
        && (w.len() != network.pipes().len() || w.iter().any(|x| !(x.is_finite() && *x >= 0.0)))
    {
        return Err(LocalisationError::InvalidInput(
            "one finite, non-negative weight per pipe is needed".into(),
        ));
    }
    Ok(())
}
