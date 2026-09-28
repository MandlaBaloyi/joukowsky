//! The entry point: arrival times in, probability map out.

use jk_hydraulics::{Network, NodeId, PipeId};
use jk_kernel::{Length, Time};

use crate::error::{LocalisationError, positive};
use crate::graph::TravelGraph;
use crate::posterior::credible::{self, PipeProbability, Segment};
use crate::posterior::grid::Grid;
use crate::posterior::score::{Score, score_correlated};

/// A wave front detected at a sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arrival {
    /// The node the sensor sits on.
    pub node: NodeId,
    /// When the front arrived.
    pub time: Time,
    /// One-standard-deviation timing uncertainty: onset picking AND the
    /// sensor's clock, combined in quadrature (see [`crate::clocks`]).
    pub uncertainty: Time,
}

/// How the probability map is computed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocaliseOptions {
    /// Candidate spacing along the pipes.
    pub spacing: Length,
    /// Relative one-sigma uncertainty of every pipe's wave speed (0.05 =
    /// +/-5%), independently per pipe. 0 means the wave speeds are exact.
    /// It is carried into the arrival times through each wave's route, so
    /// sensors whose waves shared a pipe get correlated errors.
    pub wave_speed_uncertainty: f64,
    /// Probability the credible region must hold (0.9 = 90%).
    pub credible_level: f64,
}

impl Default for LocaliseOptions {
    fn default() -> Self {
        Self {
            spacing: Length::new(2.0),
            wave_speed_uncertainty: 0.0,
            credible_level: 0.9,
        }
    }
}

/// A point on the network.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// The pipe.
    pub pipe: PipeId,
    /// Distance from the pipe's `from` end.
    pub distance: Length,
}

/// Where the transient most probably came from.
#[derive(Debug, Clone, PartialEq)]
pub struct Localisation {
    /// The single most probable point (refined between grid points).
    pub most_likely: Point,
    /// When the source emitted the wave, at the most likely point.
    pub emission: Time,
    /// Every pipe with its probability, most likely first.
    pub pipes: Vec<PipeProbability>,
    /// The smallest set of pipe stretches holding `credible_level` of the
    /// probability, most probable first.
    pub credible_region: Vec<Segment>,
    /// Total pipe length of the credible region.
    pub credible_length: Length,
    /// Root-mean-square standardised timing misfit at the most likely
    /// point (with the nominal wave speeds). Near 1 or below: the arrivals
    /// fit one source. Much larger: they do not - two events, a wrong
    /// wave speed, or a bad clock - and the map should not be trusted.
    pub misfit: f64,
}

/// Locates the source of a wave front seen at `arrivals` on `network`.
///
/// # Errors
/// Fewer than two arrivals, an unknown or repeated node, non-positive
/// uncertainties or options, or a sensor no wave can reach.
///
/// # Panics
/// Only if an internal invariant is broken (an empty candidate grid after
/// validation passed) - that would be a bug.
#[allow(clippy::too_many_lines)] // one algorithm, kept together
pub fn localise(
    network: &Network,
    arrivals: &[Arrival],
    options: &LocaliseOptions,
) -> Result<Localisation, LocalisationError> {
    validate(network, arrivals, options)?;
    let graph = TravelGraph::new(network);
    let grid = Grid::new(network, options.spacing.value());
    let pipes = network.pipes();
    let ends: Vec<(usize, usize)> = pipes
        .iter()
        .map(|p| (p.from.index(), p.to.index()))
        .collect();
    let nominal: Vec<f64> = pipes.iter().map(|p| p.pipe.travel_time().value()).collect();
    let observed: Vec<f64> = arrivals.iter().map(|a| a.time.value()).collect();
    let sigma: Vec<f64> = arrivals.iter().map(|a| a.uncertainty.value()).collect();
    let sensors: Vec<usize> = arrivals.iter().map(|a| a.node.index()).collect();

    // Routes from every sensor with the book wave speeds.
    let routes: Vec<(Vec<f64>, Vec<Vec<usize>>)> = sensors
        .iter()
        .map(|&s| graph.routes_from(s, &nominal))
        .collect();
    for (&s, (times, _)) in sensors.iter().zip(&routes) {
        if times
            .iter()
            .enumerate()
            .all(|(n, t)| n == s || !t.is_finite())
        {
            return Err(LocalisationError::Unreachable { node: s });
        }
    }
    let spread2 = options.wave_speed_uncertainty * options.wave_speed_uncertainty;

    // Score of a candidate point: predicted arrival times, and their error
    // covariance = timing noise + each shared pipe's wave-speed error.
    let evaluate = |pipe: usize, fraction: f64| -> Score {
        let n = sensors.len();
        let mut travel = vec![0.0; n];
        let mut legs: Vec<Vec<(usize, f64)>> = Vec::with_capacity(n);
        for (j, &s) in sensors.iter().enumerate() {
            let (times, pipe_routes) = &routes[j];
            let (t, entry) = graph.point_route(s, times, ends[pipe], fraction, nominal[pipe]);
            travel[j] = t;
            let mut leg: Vec<(usize, f64)> = Vec::new();
            if let Some((end, part)) = entry {
                leg.extend(pipe_routes[end].iter().map(|&p| (p, nominal[p])));
                leg.push((pipe, part * nominal[pipe]));
            }
            legs.push(leg);
        }
        let mut cov = vec![0.0; n * n];
        for j in 0..n {
            for k in 0..=j {
                let shared: f64 = legs[j]
                    .iter()
                    .flat_map(|&(pj, wj)| {
                        legs[k]
                            .iter()
                            .filter(move |(pk, _)| *pk == pj)
                            .map(move |&(_, wk)| wj * wk)
                    })
                    .sum();
                let v = spread2 * shared + if j == k { sigma[j] * sigma[j] } else { 0.0 };
                cov[j * n + k] = v;
                cov[k * n + j] = v;
            }
        }
        score_correlated(&observed, &travel, &mut cov)
    };

    let log_posterior: Vec<f64> = grid
        .cells
        .iter()
        .map(|c| evaluate(c.pipe, c.fraction).log_likelihood + c.length.ln())
        .collect();
    let top = log_posterior
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    if !top.is_finite() {
        return Err(LocalisationError::InvalidInput(
            "no candidate point is reachable from every sensor".into(),
        ));
    }
    let mut mass: Vec<f64> = log_posterior.iter().map(|lp| (lp - top).exp()).collect();
    let total: f64 = mass.iter().sum();
    for m in &mut mass {
        *m /= total;
    }

    // Summaries.
    let ids: Vec<PipeId> = (0..pipes.len())
        .map(|i| network.pipe_id(i).expect("index in range"))
        .collect();
    let lengths: Vec<f64> = pipes.iter().map(|p| p.pipe.length().value()).collect();
    let ranked = credible::per_pipe(&grid.cells, &mass, &ids);
    let region = credible::region(&grid.cells, &mass, &ids, &lengths, options.credible_level);
    let credible_length = region.iter().map(|s| (s.to - s.from).value()).sum();

    // Most likely point: the best cell, refined along its pipe (golden-
    // section search on the same score).
    let best = (0..mass.len())
        .max_by(|&a, &b| mass[a].total_cmp(&mass[b]))
        .expect("at least one cell");
    let cell = grid.cells[best];
    let half_cell = 0.5 * cell.length / lengths[cell.pipe];
    let fraction = golden_minimum(
        |f| -evaluate(cell.pipe, f).log_likelihood,
        (cell.fraction - 2.0 * half_cell).max(0.0),
        (cell.fraction + 2.0 * half_cell).min(1.0),
    );
    let at_best = evaluate(cell.pipe, fraction);
    #[allow(clippy::cast_precision_loss)]
    let dof = (arrivals.len() - 1).max(1) as f64;

    Ok(Localisation {
        most_likely: Point {
            pipe: ids[cell.pipe],
            distance: Length::new(fraction * lengths[cell.pipe]),
        },
        emission: Time::new(at_best.emission),
        pipes: ranked,
        credible_region: region,
        credible_length: Length::new(credible_length),
        misfit: (at_best.chi_squared / dof).sqrt(),
    })
}

/// Minimum of `f` on [a, b] by golden-section search (f unimodal there).
fn golden_minimum(f: impl Fn(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    const INV_PHI: f64 = 0.618_033_988_749_894_8;
    let (mut c, mut d) = (b - INV_PHI * (b - a), a + INV_PHI * (b - a));
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..80 {
        if fc < fd {
            b = d;
            (d, fd) = (c, fc);
            c = b - INV_PHI * (b - a);
            fc = f(c);
        } else {
            a = c;
            (c, fc) = (d, fd);
            d = a + INV_PHI * (b - a);
            fd = f(d);
        }
    }
    f64::midpoint(a, b)
}

fn validate(
    network: &Network,
    arrivals: &[Arrival],
    options: &LocaliseOptions,
) -> Result<(), LocalisationError> {
    if arrivals.len() < 2 {
        return Err(LocalisationError::InvalidInput(format!(
            "{} arrival(s): at least two sensors must see the front",
            arrivals.len()
        )));
    }
    if network.pipes().is_empty() {
        return Err(LocalisationError::InvalidInput(
            "the network has no pipes".into(),
        ));
    }
    for (i, a) in arrivals.iter().enumerate() {
        if a.node.index() >= network.nodes().len() {
            return Err(LocalisationError::InvalidInput(format!(
                "sensor node {} does not exist",
                a.node.index()
            )));
        }
        if arrivals[..i].iter().any(|b| b.node == a.node) {
            return Err(LocalisationError::InvalidInput(format!(
                "node {} has two arrivals",
                a.node.index()
            )));
        }
        if !a.time.is_finite() {
            return Err(LocalisationError::invalid(
                "arrival time",
                a.time.value(),
                "finite",
            ));
        }
        positive("arrival uncertainty", a.uncertainty.value())?;
    }
    positive("spacing", options.spacing.value())?;
    if !(options.wave_speed_uncertainty.is_finite() && options.wave_speed_uncertainty >= 0.0) {
        return Err(LocalisationError::invalid(
            "wave_speed_uncertainty",
            options.wave_speed_uncertainty,
            "finite and >= 0",
        ));
    }
    if !(options.credible_level > 0.0 && options.credible_level < 1.0) {
        return Err(LocalisationError::invalid(
            "credible_level",
            options.credible_level,
            "between 0 and 1",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn golden_section_finds_a_parabola_minimum() {
        let x = golden_minimum(|x| (x - 0.3141).powi(2), 0.0, 1.0);
        assert!((x - 0.3141).abs() < 1e-9);
    }
}
