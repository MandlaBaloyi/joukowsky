//! The entry point: arrival times in, probability map out.

use std::sync::{Arc, OnceLock};

use jk_hydraulics::{Network, NodeId, PipeId};
use jk_kernel::{Length, Time};

use crate::error::{LocalisationError, positive};
use crate::graph::{Routes, TravelGraph};
use crate::posterior::bound::{Bounds, Scratch};
use crate::posterior::credible::{self, PipeProbability, Segment};
use crate::posterior::grid::Grid;
use crate::posterior::score::{Score, score_correlated_in};

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

/// Candidates this far below the best (in log-probability) are never
/// scored: each would hold less than e^-50 (2e-22) of the best one's
/// probability, so leaving them out changes no answer.
const PRUNE_MARGIN: f64 = 50.0;

/// Cells per chunk: within a pipe worth scoring, stretches of this many
/// cells are bounded (and skipped) on their own.
const CHUNK: usize = 8;

/// The observed times, their uncertainties, and the probability of each
/// candidate cell that holds any.
struct Posterior {
    /// Arrival times measured from `origin` (the earliest arrival).
    observed: Vec<f64>,
    /// Time the observed times are measured from [s].
    origin: f64,
    sigma: Vec<f64>,
    /// (cell index, probability), in cell order; every other cell holds
    /// (practically) nothing.
    mass: Vec<(usize, f64)>,
}

/// One chunk of a pipe's cells: the predicted travel time to each sensor
/// (cells x n) and the wave-speed part of the covariance (cells x n x n).
#[derive(Debug, Clone)]
struct PipeModel {
    travel: Vec<f64>,
    speed_cov: Vec<f64>,
}

/// A localiser for one fixed set of sensors.
///
/// Everything that does not depend on the arrival times - the routes from
/// each sensor, the candidate cells, each cell's predicted travel times and
/// the wave-speed part of its error covariance - is computed once, pipe by
/// pipe as pipes are first needed, and kept. Locating many events with the
/// same sensors (a live site; the placement advisor's thousands of trial
/// bursts) then costs one small factorisation per cell scored.
///
/// Only cells that could matter are scored (branch and bound). For each
/// pipe a cheap bound gives the best log-probability ANY cell on it could
/// reach (`posterior/bound.rs`: each sensor pair's timing miss bounds the
/// misfit below, counting only the parts of their routes that differ).
/// Pipes are scored best bound first, and a pipe whose bound is more than
/// 50 below the best cell found so far is skipped - so on a large network
/// only the neighbourhood of the answer is scored.
/// [`Self::locate_exhaustive`] scores every cell, for checking.
#[derive(Debug, Clone)]
pub struct Localiser<'a> {
    network: &'a Network,
    options: LocaliseOptions,
    sensors: Vec<NodeId>,
    graph: Arc<TravelGraph>,
    grid: Arc<Grid>,
    ends: Vec<(usize, usize)>,
    nominal: Vec<f64>,
    lengths: Vec<f64>,
    ids: Vec<PipeId>,
    routes: Vec<Arc<Routes>>,
    /// Each pipe's cells: a range of `grid.cells`.
    pipe_cells: Vec<(usize, usize)>,
    /// Index of each pipe's first chunk in `models`.
    chunk_base: Vec<usize>,
    /// Each chunk's model, computed when the chunk is first scored.
    models: Vec<OnceLock<PipeModel>>,
    /// What the pruning bound needs, per pipe.
    bounds: Bounds,
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
        let graph = Arc::new(TravelGraph::new(network));
        let grid = Arc::new(Grid::new(network, options.spacing.value()));
        let nominal = nominal_times(network);
        // Routes from every sensor with the book wave speeds.
        let routes: Vec<Arc<Routes>> = sensors
            .iter()
            .map(|s| Arc::new(graph.routes_from(s.index(), &nominal)))
            .collect();
        Self::assemble(network, sensors, options, graph, grid, routes)
    }

    /// As [`Self::new`], sharing the parts that do not depend on the set of
    /// sensors: the graph, the candidate grid (built with
    /// `options.spacing`) and each sensor's routes
    /// ([`TravelGraph::routes_from`] with the book wave speeds). The
    /// placement advisor builds thousands of localisers on one network.
    pub(crate) fn with_shared(
        network: &'a Network,
        sensors: &[NodeId],
        options: &LocaliseOptions,
        graph: Arc<TravelGraph>,
        grid: Arc<Grid>,
        routes: Vec<Arc<Routes>>,
    ) -> Result<Self, LocalisationError> {
        validate_setup(network, sensors, options)?;
        Self::assemble(network, sensors, options, graph, grid, routes)
    }

    fn assemble(
        network: &'a Network,
        sensors: &[NodeId],
        options: &LocaliseOptions,
        graph: Arc<TravelGraph>,
        grid: Arc<Grid>,
        routes: Vec<Arc<Routes>>,
    ) -> Result<Self, LocalisationError> {
        let pipes = network.pipes();
        let ends: Vec<(usize, usize)> = pipes
            .iter()
            .map(|p| (p.from.index(), p.to.index()))
            .collect();
        let nominal = nominal_times(network);
        let lengths = pipes.iter().map(|p| p.pipe.length().value()).collect();
        let ids = (0..pipes.len())
            .map(|i| network.pipe_id(i).expect("index in range"))
            .collect();
        for (s, r) in sensors.iter().zip(&routes) {
            let times = &r.0;
            if times
                .iter()
                .enumerate()
                .all(|(n, t)| n == s.index() || !t.is_finite())
            {
                return Err(LocalisationError::Unreachable { node: s.index() });
            }
        }
        let mut pipe_cells = vec![(0, 0); pipes.len()];
        for (i, c) in grid.cells.iter().enumerate() {
            if pipe_cells[c.pipe] == (0, 0) {
                pipe_cells[c.pipe].0 = i;
            }
            pipe_cells[c.pipe].1 = i + 1;
        }
        let cell_lengths: Vec<f64> = pipe_cells
            .iter()
            .map(|&(first, _)| grid.cells[first].length)
            .collect();
        let mut chunk_base = Vec::with_capacity(pipes.len());
        let mut chunks = 0;
        for &(first, end) in &pipe_cells {
            chunk_base.push(chunks);
            chunks += (end - first).div_ceil(CHUNK);
        }
        let sensor_nodes: Vec<usize> = sensors.iter().map(|s| s.index()).collect();
        let bounds = Bounds::new(
            &graph,
            &sensor_nodes,
            &routes,
            &ends,
            &nominal,
            &cell_lengths,
        );
        Ok(Self {
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
            pipe_cells,
            chunk_base,
            models: (0..chunks).map(|_| OnceLock::new()).collect(),
            bounds,
        })
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
    ///
    /// Summed over the pipes both routes cross in full (`tau_q^2` each,
    /// precomputed per pipe and entry end in [`Bounds`]), plus the burst
    /// pipe itself, where each wave spends its own share.
    fn model(&self, pipe: usize, fraction: f64) -> (Vec<f64>, Vec<f64>) {
        let n = self.sensors.len();
        let (mut travel, mut cov) = (Vec::with_capacity(n), Vec::with_capacity(n * n));
        self.model_into(pipe, fraction, &mut travel, &mut cov, &mut Vec::new());
        (travel, cov)
    }

    /// As [`Self::model`], appending to `travel` (n values) and `cov`
    /// (n x n), with `entry` as scratch space.
    fn model_into(
        &self,
        pipe: usize,
        fraction: f64,
        travel: &mut Vec<f64>,
        cov: &mut Vec<f64>,
        entry: &mut Vec<Option<(usize, f64)>>,
    ) {
        let n = self.sensors.len();
        let spread2 = self.options.wave_speed_uncertainty * self.options.wave_speed_uncertainty;
        let tau = self.nominal[pipe];
        // Per sensor: entry end (0 = from, 1 = to) and time on this pipe.
        entry.clear();
        for (j, s) in self.sensors.iter().enumerate() {
            let (t, via) = self.graph.point_route(
                s.index(),
                &self.routes[j].0,
                self.ends[pipe],
                fraction,
                tau,
            );
            travel.push(t);
            entry.push(via.map(|(end, part)| {
                let k = usize::from(end != self.ends[pipe].0);
                let through = if self.bounds.through(pipe, j, k) {
                    tau
                } else {
                    0.0
                };
                (k, through + part * tau)
            }));
        }
        let base = cov.len();
        cov.resize(base + n * n, 0.0);
        let block = &mut cov[base..];
        for j in 0..n {
            let Some((ej, wj)) = entry[j] else { continue };
            for k in 0..=j {
                let Some((ek, wk)) = entry[k] else { continue };
                let v = spread2 * (self.bounds.shared(pipe, j, ej, k, ek) + wj * wk);
                block[j * n + k] = v;
                block[k * n + j] = v;
            }
        }
    }

    /// The cells of chunk `k` of `pipe`.
    fn chunk_cells(&self, pipe: usize, k: usize) -> (usize, usize) {
        let (first, end) = self.pipe_cells[pipe];
        (first + k * CHUNK, (first + (k + 1) * CHUNK).min(end))
    }

    /// The model of every cell of chunk `k` of `pipe` (computed once).
    fn chunk_model(&self, pipe: usize, k: usize) -> &PipeModel {
        self.models[self.chunk_base[pipe] + k].get_or_init(|| {
            let (first, end) = self.chunk_cells(pipe, k);
            let n = self.sensors.len();
            let mut travel = Vec::with_capacity((end - first) * n);
            let mut speed_cov = Vec::with_capacity((end - first) * n * n);
            let mut entry = Vec::with_capacity(n);
            for c in &self.grid.cells[first..end] {
                self.model_into(pipe, c.fraction, &mut travel, &mut speed_cov, &mut entry);
            }
            PipeModel { travel, speed_cov }
        })
    }

    /// Scores a candidate: timing noise added to the wave-speed covariance.
    fn score(observed: &[f64], sigma: &[f64], travel: &[f64], speed_cov: &[f64]) -> Score {
        let n = observed.len();
        Self::score_in(
            observed,
            sigma,
            travel,
            speed_cov,
            &mut vec![0.0; n * n],
            &mut vec![0.0; 3 * n],
        )
    }

    /// As [`Self::score`], in scratch space `cov` (n x n) and `work` (3n).
    fn score_in(
        observed: &[f64],
        sigma: &[f64],
        travel: &[f64],
        speed_cov: &[f64],
        cov: &mut [f64],
        work: &mut [f64],
    ) -> Score {
        let n = observed.len();
        cov.copy_from_slice(speed_cov);
        for j in 0..n {
            cov[j * n + j] += sigma[j] * sigma[j];
        }
        score_correlated_in(observed, travel, cov, work)
    }

    /// The normalised posterior over the candidate cells for `arrivals`:
    /// the cells that could matter (all of them if `exhaustive`).
    #[allow(clippy::too_many_lines)] // one branch-and-bound search, kept together
    fn posterior(
        &self,
        arrivals: &[Arrival],
        exhaustive: bool,
    ) -> Result<Posterior, LocalisationError> {
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
        let spread2 = self.options.wave_speed_uncertainty * self.options.wave_speed_uncertainty;
        let constant = -0.5 * sigma.iter().map(|s| (s * s).ln()).sum::<f64>()
            - 0.5 * sigma.iter().map(|s| 1.0 / (s * s)).sum::<f64>().ln();

        // Pipes, most promising first.
        let mut scratch = Scratch::default();
        let mut order: Vec<(f64, usize)> = (0..self.pipe_cells.len())
            .map(|p| {
                let b = if exhaustive {
                    f64::INFINITY
                } else {
                    self.bounds.upper(
                        p,
                        (0.0, 1.0),
                        &observed,
                        &sigma,
                        spread2,
                        constant,
                        &mut scratch,
                    )
                };
                (b, p)
            })
            .filter(|(b, _)| *b > f64::NEG_INFINITY)
            .collect();
        order.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        let cells = &self.grid.cells;
        let mut top = f64::NEG_INFINITY;
        let mut scored: Vec<(usize, f64)> = Vec::new();
        let (mut cov, mut work) = (vec![0.0; n * n], vec![0.0; 3 * n]);
        for (b, p) in order {
            if b < top - PRUNE_MARGIN {
                break; // every pipe left is bounded below this
            }
            // Within the pipe, stretches of cells best bound first.
            let (first, end) = self.pipe_cells[p];
            let count = (end - first).div_ceil(CHUNK);
            let mut chunks: Vec<(f64, usize)> = (0..count)
                .map(|k| {
                    if exhaustive || count == 1 {
                        return (b, k);
                    }
                    let (a, z) = self.chunk_cells(p, k);
                    let span = (cells[a].fraction, cells[z - 1].fraction);
                    let bound = self.bounds.upper(
                        p,
                        span,
                        &observed,
                        &sigma,
                        spread2,
                        constant,
                        &mut scratch,
                    );
                    (bound, k)
                })
                .collect();
            chunks.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
            for (bound, k) in chunks {
                if bound < top - PRUNE_MARGIN {
                    break;
                }
                let model = self.chunk_model(p, k);
                let (a, z) = self.chunk_cells(p, k);
                for (c, i) in (a..z).enumerate() {
                    let s = Self::score_in(
                        &observed,
                        &sigma,
                        &model.travel[c * n..(c + 1) * n],
                        &model.speed_cov[c * n * n..(c + 1) * n * n],
                        &mut cov,
                        &mut work,
                    );
                    let lp = s.log_likelihood + cells[i].length.ln();
                    if lp.is_finite() {
                        top = top.max(lp);
                        scored.push((i, lp));
                    }
                }
            }
        }
        if !top.is_finite() {
            return Err(LocalisationError::InvalidInput(
                "no candidate point is reachable from every sensor".into(),
            ));
        }
        scored.sort_by_key(|&(i, _)| i);
        let mut mass: Vec<(usize, f64)> = scored
            .into_iter()
            .map(|(i, lp)| (i, (lp - top).exp()))
            .collect();
        let total: f64 = mass.iter().map(|(_, m)| m).sum();
        for (_, m) in &mut mass {
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
        let mass = self.posterior(arrivals, false)?.mass;
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
        Ok(self.summarise(arrivals, self.posterior(arrivals, false)?))
    }

    /// As [`Self::locate`], but scoring EVERY candidate cell instead of
    /// skipping those that cannot matter. Slower on large networks; the
    /// answers are the same (tests check it) - kept to show that.
    ///
    /// # Errors
    /// As [`Self::locate`].
    ///
    /// # Panics
    /// As [`Self::locate`].
    pub fn locate_exhaustive(
        &self,
        arrivals: &[Arrival],
    ) -> Result<Localisation, LocalisationError> {
        Ok(self.summarise(arrivals, self.posterior(arrivals, true)?))
    }

    fn summarise(&self, arrivals: &[Arrival], posterior: Posterior) -> Localisation {
        let Posterior {
            observed,
            origin,
            sigma,
            mass,
        } = posterior;
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
        let best = mass
            .iter()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .expect("at least one cell")
            .0;
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

        Localisation {
            most_likely: Point {
                pipe: self.ids[cell.pipe],
                distance: Length::new(fraction * self.lengths[cell.pipe]),
            },
            emission: Time::new(origin + at_best.emission),
            pipes: ranked,
            credible_region: region,
            credible_length: Length::new(credible_length),
            misfit: (at_best.chi_squared / dof).sqrt(),
        }
    }
}

/// Each pipe's end-to-end travel time at the book wave speed [s].
pub(crate) fn nominal_times(network: &Network) -> Vec<f64> {
    network
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect()
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
