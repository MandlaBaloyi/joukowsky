//! How well a set of sensors can locate a burst anywhere on the network.

use jk_hydraulics::{Network, NodeId, PipeId};
use jk_kernel::{Flow, Head, Length, Time};

use super::detectability::Fronts;
use crate::error::{LocalisationError, positive};
use crate::graph::TravelGraph;
use crate::locate::{Arrival, LocaliseOptions, Localiser};

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
    let pipes = network.pipes();
    let step = options.source_step.value();
    let mut trials = Trials::new(network, sensors, options);

    let (mut weighted_region, mut weighted_pinpointed, mut total_weight) = (0.0, 0.0, 0.0);
    let mut per_pipe = Vec::with_capacity(pipes.len());
    for (p, link) in pipes.iter().enumerate() {
        let Some(pipe) = network.pipe_id(p) else {
            continue;
        };
        let length = link.pipe.length().value();
        let weight = weights.map_or(1.0, |w| w[p]);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // ceil of a positive ratio
        let n = ((length / step).ceil() as usize).max(1);
        let segment = length / to_f64(n);
        let mut regions: Vec<f64> = (0..n)
            .map(|i| trials.region(p, (to_f64(i) + 0.5) / to_f64(n)))
            .collect();
        let pinpointed = regions
            .iter()
            .filter(|&&r| r <= options.pinpoint.value())
            .count();
        let w = weight * segment;
        weighted_region += w * regions.iter().sum::<f64>();
        weighted_pinpointed += w * to_f64(pinpointed);
        total_weight += w * to_f64(n);
        regions.sort_by(f64::total_cmp);
        per_pipe.push(PipeCoverage {
            pipe,
            length: link.pipe.length(),
            pinpointed: to_f64(pinpointed) / to_f64(n),
            median_region: Length::new(regions[n / 2]),
        });
    }
    let (expected_region, pinpointed) = if total_weight > 0.0 {
        (
            weighted_region / total_weight,
            weighted_pinpointed / total_weight,
        )
    } else {
        (0.0, 0.0)
    };
    Ok(Coverage {
        sensors: sensors.to_vec(),
        expected_region: Length::new(expected_region),
        pinpointed,
        pipes: per_pipe,
    })
}

/// Trial bursts on one placement: which loggers feel each, and how large
/// its 90% region is.
struct Trials<'a> {
    network: &'a Network,
    sensors: &'a [NodeId],
    options: &'a PlacementOptions,
    graph: TravelGraph,
    ends: Vec<(usize, usize)>,
    nominal: Vec<f64>,
    /// Fastest travel times from each sensor to every node.
    times: Vec<Vec<f64>>,
    total_length: f64,
    /// First-front heads, if a burst size is set.
    fronts: Option<Fronts>,
    /// One localiser per set of loggers that feel a burst, prepared once and
    /// reused for every trial burst that set feels.
    localisers: Vec<(Vec<NodeId>, Option<Localiser<'a>>)>,
}

impl<'a> Trials<'a> {
    fn new(network: &'a Network, sensors: &'a [NodeId], options: &'a PlacementOptions) -> Self {
        let graph = TravelGraph::new(network);
        let pipes = network.pipes();
        let nominal: Vec<f64> = pipes.iter().map(|p| p.pipe.travel_time().value()).collect();
        let times = sensors
            .iter()
            .map(|s| graph.times_from(s.index(), &nominal))
            .collect();
        Self {
            network,
            sensors,
            options,
            graph,
            ends: pipes
                .iter()
                .map(|p| (p.from.index(), p.to.index()))
                .collect(),
            nominal,
            times,
            total_length: pipes.iter().map(|p| p.pipe.length().value()).sum(),
            fronts: options.burst.map(|_| Fronts::new(network, sensors)),
            localisers: Vec::new(),
        }
    }

    /// Size of the 90% region for a burst `fraction` along `pipe` [m]; the
    /// whole network's length when fewer than two loggers feel it.
    fn region(&mut self, pipe: usize, fraction: f64) -> f64 {
        let threshold = self.options.detect_units * self.options.noise.value();
        let felt: Vec<bool> = match (&self.fronts, self.options.burst) {
            (Some(f), Some(q)) => f
                .per_unit_flow(pipe, fraction)
                .iter()
                .map(|h| h * q.value() >= threshold)
                .collect(),
            _ => vec![true; self.sensors.len()],
        };
        let arrivals: Vec<Arrival> = self
            .sensors
            .iter()
            .zip(&self.times)
            .zip(&felt)
            .filter(|(_, felt)| **felt)
            .filter_map(|((&s, t), _)| {
                let time = self.graph.point_time(
                    s.index(),
                    t,
                    self.ends[pipe],
                    fraction,
                    self.nominal[pipe],
                );
                time.is_finite().then_some(Arrival {
                    node: s,
                    time: Time::new(time),
                    uncertainty: self.options.timing,
                })
            })
            .collect();
        if arrivals.len() < 2 {
            return self.total_length;
        }
        let set: Vec<NodeId> = arrivals.iter().map(|a| a.node).collect();
        let k = if let Some(k) = self.localisers.iter().position(|(s, _)| *s == set) {
            k
        } else {
            let l = Localiser::new(self.network, &set, &self.options.localise).ok();
            self.localisers.push((set, l));
            self.localisers.len() - 1
        };
        self.localisers[k]
            .1
            .as_ref()
            .and_then(|l| l.credible_length(&arrivals).ok())
            .map_or(self.total_length, Length::value)
    }
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

#[allow(clippy::cast_precision_loss)] // counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}
