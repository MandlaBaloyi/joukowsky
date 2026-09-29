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
pub fn localise(
    network: &Network,
    arrivals: &[Arrival],
    options: &LocaliseOptions,
) -> Result<Localisation, LocalisationError> {
    validate(network, arrivals, options)?;
    let sensors: Vec<NodeId> = arrivals.iter().map(|a| a.node).collect();
    Localiser::new(network, &sensors, options)?.locate(arrivals)
}

/// The observed times, their uncertainties, and the probability of each
/// candidate cell.
struct Posterior {
    /// Arrival times measured from `origin` (the earliest arrival).
    observed: Vec<f64>,
    /// Time the observed times are measured from [s].
    origin: f64,
    sigma: Vec<f64>,
    mass: Vec<f64>,
}

/// A localiser for one fixed set of sensors.
///
/// Everything that does not depend on the arrival times - the routes from
/// each sensor, the candidate cells, each cell's predicted travel times and
/// the wave-speed part of its error covariance - is computed once. Locating
/// many events with the same sensors (a live site; the placement advisor's
/// thousands of trial bursts) then costs one small factorisation per cell.
/// The arithmetic is exactly that of [`localise`], so the answers are
/// identical.
#[derive(Debug, Clone)]
pub struct Localiser<'a> {
    network: &'a Network,
    options: LocaliseOptions,
    sensors: Vec<NodeId>,
    graph: TravelGraph,
    grid: Grid,
    ends: Vec<(usize, usize)>,
    nominal: Vec<f64>,
    lengths: Vec<f64>,
    ids: Vec<PipeId>,
    routes: Vec<(Vec<f64>, Vec<Vec<usize>>)>,
    /// Per cell, the predicted travel time to each sensor (cells x n)...
    travel: Vec<f64>,
    /// ... and the wave-speed part of the covariance (cells x n x n).
    speed_cov: Vec<f64>,
}

impl<'a> Localiser<'a> {
    /// Prepares to locate fronts seen by `sensors` (in this order).
    ///
    /// # Errors
    /// Fewer than two sensors, an unknown or repeated node, options out of
    /// range, or a sensor no wave can reach.
    ///
    /// # Panics
    /// Only if an internal invariant is broken (a pipe index out of range)
    /// - that would be a bug.
    pub fn new(
        network: &'a Network,
        sensors: &[NodeId],
        options: &LocaliseOptions,
    ) -> Result<Self, LocalisationError> {
        validate_setup(network, sensors, options)?;
        let graph = TravelGraph::new(network);
        let grid = Grid::new(network, options.spacing.value());
        let pipes = network.pipes();
        let ends = pipes
            .iter()
            .map(|p| (p.from.index(), p.to.index()))
            .collect();
        let nominal: Vec<f64> = pipes.iter().map(|p| p.pipe.travel_time().value()).collect();
        let lengths = pipes.iter().map(|p| p.pipe.length().value()).collect();
        let ids = (0..pipes.len())
            .map(|i| network.pipe_id(i).expect("index in range"))
            .collect();
        // Routes from every sensor with the book wave speeds.
        let routes: Vec<(Vec<f64>, Vec<Vec<usize>>)> = sensors
            .iter()
            .map(|s| graph.routes_from(s.index(), &nominal))
            .collect();
        for (s, (times, _)) in sensors.iter().zip(&routes) {
            if times
                .iter()
                .enumerate()
                .all(|(n, t)| n == s.index() || !t.is_finite())
            {
                return Err(LocalisationError::Unreachable { node: s.index() });
            }
        }
        let mut localiser = Self {
            network,
            options: *options,
            sensors: sensors.to_vec(),
            graph,
            grid,
            ends,
            nominal,
            lengths,
            ids,
            routes,
            travel: Vec::new(),
            speed_cov: Vec::new(),
        };
        let n = sensors.len();
        let mut travel = Vec::with_capacity(localiser.grid.cells.len() * n);
        let mut speed_cov = Vec::with_capacity(localiser.grid.cells.len() * n * n);
        for c in &localiser.grid.cells {
            let (t, cov) = localiser.model(c.pipe, c.fraction);
            travel.extend(t);
            speed_cov.extend(cov);
        }
        localiser.travel = travel;
        localiser.speed_cov = speed_cov;
        Ok(localiser)
    }

    /// The network.
    #[must_use]
    pub fn network(&self) -> &Network {
        self.network
    }

    /// The sensors, in the order arrivals must be given.
    #[must_use]
    pub fn sensors(&self) -> &[NodeId] {
        &self.sensors
    }

    /// Predicted travel times from a point to each sensor, and the
    /// wave-speed part of their error covariance: each shared pipe's
    /// travel-time error, spread^2 x (time on it for j) x (time for k).
    fn model(&self, pipe: usize, fraction: f64) -> (Vec<f64>, Vec<f64>) {
        let n = self.sensors.len();
        let spread2 = self.options.wave_speed_uncertainty * self.options.wave_speed_uncertainty;
        let mut travel = vec![0.0; n];
        let mut legs: Vec<Vec<(usize, f64)>> = Vec::with_capacity(n);
        for (j, s) in self.sensors.iter().enumerate() {
            let (times, pipe_routes) = &self.routes[j];
            let (t, entry) = self.graph.point_route(
                s.index(),
                times,
                self.ends[pipe],
                fraction,
                self.nominal[pipe],
            );
            travel[j] = t;
            let mut leg: Vec<(usize, f64)> = Vec::new();
            if let Some((end, part)) = entry {
                leg.extend(pipe_routes[end].iter().map(|&p| (p, self.nominal[p])));
                leg.push((pipe, part * self.nominal[pipe]));
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
                let v = spread2 * shared;
                cov[j * n + k] = v;
                cov[k * n + j] = v;
            }
        }
        (travel, cov)
    }

    /// Scores a candidate: timing noise added to the wave-speed covariance.
    fn score(observed: &[f64], sigma: &[f64], travel: &[f64], speed_cov: &[f64]) -> Score {
        let n = observed.len();
        let mut cov = speed_cov.to_vec();
        for j in 0..n {
            cov[j * n + j] += sigma[j] * sigma[j];
        }
        score_correlated(observed, travel, &mut cov)
    }

    /// The normalised posterior over the candidate cells for `arrivals`.
    fn posterior(&self, arrivals: &[Arrival]) -> Result<Posterior, LocalisationError> {
        if arrivals.len() != self.sensors.len()
            || arrivals
                .iter()
                .zip(&self.sensors)
                .any(|(a, s)| a.node != *s)
        {
            return Err(LocalisationError::InvalidInput(
                "arrivals must match the localiser's sensors, in order".into(),
            ));
        }
        for a in arrivals {
            if !a.time.is_finite() {
                return Err(LocalisationError::invalid(
                    "arrival time",
                    a.time.value(),
                    "finite",
                ));
            }
            positive("arrival uncertainty", a.uncertainty.value())?;
        }
        let n = self.sensors.len();
        // Measure the arrivals from the earliest one. Real loggers stamp
        // clock time (seconds since 1970, about 1.8e9): scored as they are,
        // the squared residuals lose every significant digit (a real
        // logger file gave a misfit of exactly 2^18). The answer does not
        // depend on where time starts, so shifting it changes nothing
        // else.
        let origin = arrivals
            .iter()
            .map(|a| a.time.value())
            .fold(f64::INFINITY, f64::min);
        let observed: Vec<f64> = arrivals.iter().map(|a| a.time.value() - origin).collect();
        let sigma: Vec<f64> = arrivals.iter().map(|a| a.uncertainty.value()).collect();
        let cells = &self.grid.cells;
        let log_posterior: Vec<f64> = cells
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let s = Self::score(
                    &observed,
                    &sigma,
                    &self.travel[i * n..(i + 1) * n],
                    &self.speed_cov[i * n * n..(i + 1) * n * n],
                );
                s.log_likelihood + c.length.ln()
            })
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
        Ok(Posterior {
            observed,
            origin,
            sigma,
            mass,
        })
    }

    /// Only the total length of the 90% (credible-level) region for
    /// `arrivals` - all the placement advisor needs, without refining the
    /// most likely point.
    ///
    /// # Errors
    /// As [`Self::locate`].
    pub fn credible_length(&self, arrivals: &[Arrival]) -> Result<Length, LocalisationError> {
        let mass = self.posterior(arrivals)?.mass;
        let region = credible::region(
            &self.grid.cells,
            &mass,
            &self.ids,
            &self.lengths,
            self.options.credible_level,
        );
        Ok(Length::new(
            region.iter().map(|s| (s.to - s.from).value()).sum(),
        ))
    }

    /// Locates the source of a front seen at `arrivals` - one per sensor,
    /// in the order of [`Self::sensors`].
    ///
    /// # Errors
    /// Arrivals that do not match the sensors, a non-finite time, or a
    /// non-positive uncertainty.
    ///
    /// # Panics
    /// Only if an internal invariant is broken (an empty candidate grid after
    /// validation passed) - that would be a bug.
    pub fn locate(&self, arrivals: &[Arrival]) -> Result<Localisation, LocalisationError> {
        let Posterior {
            observed,
            origin,
            sigma,
            mass,
        } = self.posterior(arrivals)?;
        let cells = &self.grid.cells;
        let evaluate = |pipe: usize, fraction: f64| -> Score {
            let (travel, cov) = self.model(pipe, fraction);
            Self::score(&observed, &sigma, &travel, &cov)
        };

        // Summaries.
        let ranked = credible::per_pipe(cells, &mass, &self.ids);
        let region = credible::region(
            cells,
            &mass,
            &self.ids,
            &self.lengths,
            self.options.credible_level,
        );
        let credible_length = region.iter().map(|s| (s.to - s.from).value()).sum();

        // Most likely point: the best cell, refined along its pipe (golden-
        // section search on the same score).
        let best = (0..mass.len())
            .max_by(|&a, &b| mass[a].total_cmp(&mass[b]))
            .expect("at least one cell");
        let cell = cells[best];
        let half_cell = 0.5 * cell.length / self.lengths[cell.pipe];
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
                pipe: self.ids[cell.pipe],
                distance: Length::new(fraction * self.lengths[cell.pipe]),
            },
            emission: Time::new(origin + at_best.emission),
            pipes: ranked,
            credible_region: region,
            credible_length: Length::new(credible_length),
            misfit: (at_best.chi_squared / dof).sqrt(),
        })
    }
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
    let nodes: Vec<NodeId> = arrivals.iter().map(|a| a.node).collect();
    validate_setup(network, &nodes, options)?;
    for a in arrivals {
        if !a.time.is_finite() {
            return Err(LocalisationError::invalid(
                "arrival time",
                a.time.value(),
                "finite",
            ));
        }
        positive("arrival uncertainty", a.uncertainty.value())?;
    }
    Ok(())
}

fn validate_setup(
    network: &Network,
    sensors: &[NodeId],
    options: &LocaliseOptions,
) -> Result<(), LocalisationError> {
    if sensors.len() < 2 {
        return Err(LocalisationError::InvalidInput(format!(
            "{} sensor(s): at least two are needed to locate anything",
            sensors.len()
        )));
    }
    if network.pipes().is_empty() {
        return Err(LocalisationError::InvalidInput(
            "the network has no pipes".into(),
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
                "node {} has two arrivals",
                s.index()
            )));
        }
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
