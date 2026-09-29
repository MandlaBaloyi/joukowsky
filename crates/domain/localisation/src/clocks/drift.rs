//! Clocks that DRIFT: each sensor's offset and rate, from known-source
//! events spread over time (frontier F9, second step).
//!
//! A logger's clock is a crystal oscillator. Off by a few parts per million
//! (ppm), it gains or loses time steadily: 1 ppm is 86 ms a day - 43 m of
//! location error at a = 1000 m/s. A logger that re-synchronises (GNSS,
//! NTP) stays close; one that cannot - underground in a mine, where no
//! satellite reaches it and no cable brings time - drifts until repaired.
//! `time_synchronisation.txt`; typical crystals are tens of ppm, compensated
//! ones (TCXO) 1-2 ppm [VERIFY against logger data sheets].
//!
//! Model, for known-source event e seen by sensor s:
//!
//! ```text
//!   t_es = T0_e + tau_s(source_e) + c_s + r_s (t_es - epoch) + noise
//! ```
//!
//! `c_s` is the offset at the `epoch` and `r_s` the rate (seconds gained per
//! second), evaluated at the arrival itself: t is the arrival's own reading
//! (the clock's error changes the rate term by rate x offset - nanoseconds). As in [`super::calibrate`], one reference sensor has c = r = 0
//! and the rest follow by least squares. The epoch is the middle of the
//! events, where offset and rate are least correlated; the offset then
//! grows uncertain as a time moves away from the events - the correction
//! says so (P3), most of all when it extrapolates beyond the last one.
//!
//! A rate needs the sensor to see known events at two or more different
//! times; otherwise the solve fails and names the sensor.

use jk_hydraulics::{Network, NodeId};
use jk_kernel::Time;
use jk_kernel::linalg::{cholesky, solve_factored};

use super::calibrate::CalibrationEvent;
use crate::error::{LocalisationError, positive};
use crate::graph::TravelGraph;
use crate::locate::{Arrival, nominal_times};

/// A sensor's drifting clock: at time t it reads `offset + rate (t -
/// epoch)` late.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriftEstimate {
    /// The sensor.
    pub node: NodeId,
    /// How late the clock reads at `epoch`.
    pub offset: Time,
    /// Seconds gained per second of the clock's own reading (1e-6 = 1 ppm
    /// = 86 ms a day; per real second it is rate / (1 - rate) - the same to
    /// a few parts in a million of itself).
    pub rate: f64,
    /// The time the offset refers to: the middle of the calibration events.
    pub epoch: Time,
    /// Variances and covariance of (offset [s^2], rate [1], offset x rate [s]).
    covariance: [f64; 3],
}

impl DriftEstimate {
    /// How late the clock reads at time `t`.
    #[must_use]
    pub fn offset_at(&self, t: Time) -> Time {
        Time::new(self.offset.value() + self.rate * (t.value() - self.epoch.value()))
    }

    /// One-standard-deviation uncertainty of [`Self::offset_at`]: smallest
    /// near the events, growing with distance from them.
    #[must_use]
    pub fn uncertainty_at(&self, t: Time) -> Time {
        let dt = t.value() - self.epoch.value();
        let [v_o, v_r, c_or] = self.covariance;
        Time::new((v_o + 2.0 * dt * c_or + dt * dt * v_r).max(0.0).sqrt())
    }
}

/// Estimates every sensor's clock offset and drift rate relative to
/// `reference`, from `events` at different times, with arrival-picking
/// uncertainty `pick_sigma`.
///
/// # Errors
/// No events, an unknown node, a non-positive uncertainty, a source no wave
/// reaches a sensor from, too few arrivals, or a clock the events cannot
/// separate (it never shares an event with the others, or sees known
/// events at only one time, so its rate is unknown).
///
/// # Panics
/// Only if an internal invariant is broken (a sensor missing from the list
/// built from the same events) - that would be a bug.
#[allow(clippy::too_many_lines)] // one least-squares solve, kept together
pub fn estimate_drift(
    network: &Network,
    events: &[CalibrationEvent],
    reference: NodeId,
    pick_sigma: Time,
) -> Result<Vec<DriftEstimate>, LocalisationError> {
    positive("pick_sigma", pick_sigma.value())?;
    if events.is_empty() {
        return Err(LocalisationError::InvalidInput(
            "no calibration events".into(),
        ));
    }
    let n_nodes = network.nodes().len();
    let check = |node: NodeId| {
        if node.index() < n_nodes {
            Ok(())
        } else {
            Err(LocalisationError::InvalidInput(format!(
                "node {} does not exist",
                node.index()
            )))
        }
    };
    check(reference)?;
    let mut sensors: Vec<NodeId> = vec![reference];
    for e in events {
        check(e.source)?;
        if e.arrivals.is_empty() {
            return Err(LocalisationError::InvalidInput(
                "a calibration event with no arrivals".into(),
            ));
        }
        for &(node, time) in &e.arrivals {
            check(node)?;
            if !time.is_finite() {
                return Err(LocalisationError::invalid(
                    "arrival time",
                    time.value(),
                    "finite",
                ));
            }
            if !sensors.contains(&node) {
                sensors.push(node);
            }
        }
    }

    // A rate needs known events at two or more times.
    for &node in &sensors[1..] {
        let seen = events
            .iter()
            .filter(|e| e.arrivals.iter().any(|&(n, _)| n == node))
            .count();
        if seen < 2 {
            return Err(LocalisationError::InvalidInput(format!(
                "the clock of the sensor at node {} is in {seen} calibration event(s): its drift needs known events at two or more different times",
                node.index()
            )));
        }
    }

    // Each event's time (its earliest arrival), measured from the first
    // event: clock time (about 1.8e9 s) would swamp milliseconds. The epoch
    // is their mean, and the rate column is scaled by the spread of the
    // event times so both columns of the solve are of order one.
    let when: Vec<f64> = events
        .iter()
        .map(|e| {
            e.arrivals
                .iter()
                .map(|(_, t)| t.value())
                .fold(f64::INFINITY, f64::min)
        })
        .collect();
    let zero = when.iter().copied().fold(f64::INFINITY, f64::min);
    #[allow(clippy::cast_precision_loss)] // a handful of events
    let epoch = when.iter().map(|w| w - zero).sum::<f64>() / when.len() as f64;
    let span = when
        .iter()
        .map(|w| (w - zero - epoch).abs())
        .fold(0.0, f64::max)
        .max(1.0);

    // Unknowns: one T0 per event, then (offset, scaled rate) per
    // non-reference sensor.
    let n_events = events.len();
    let unknowns = n_events + 2 * (sensors.len() - 1);
    let graph = TravelGraph::new(network);
    let pipe_times = nominal_times(network);
    let mut normal = vec![0.0; unknowns * unknowns];
    let mut rhs = vec![0.0; unknowns];
    let mut rows = 0_usize;
    for (e, event) in events.iter().enumerate() {
        let tau = graph.times_from(event.source.index(), &pipe_times);
        for &(node, time) in &event.arrivals {
            let x = (time.value() - zero - epoch) / span;
            let travel = tau[node.index()];
            if !travel.is_finite() {
                return Err(LocalisationError::Unreachable { node: node.index() });
            }
            let s = sensors
                .iter()
                .position(|&n| n == node)
                .expect("collected above");
            // Row: T0_e (+ c_s + x r'_s unless s is the reference) = t - tau.
            let mut cols: Vec<(usize, f64)> = vec![(e, 1.0)];
            if s > 0 {
                let k = n_events + 2 * (s - 1);
                cols.push((k, 1.0));
                cols.push((k + 1, x));
            }
            let y = time.value() - when[e] - travel;
            for &(i, a) in &cols {
                rhs[i] += a * y;
                for &(j, b) in &cols {
                    normal[i * unknowns + j] += a * b;
                }
            }
            rows += 1;
        }
    }
    if rows <= unknowns {
        return Err(LocalisationError::InvalidInput(format!(
            "{rows} arrivals for {unknowns} unknowns: more calibration events are needed"
        )));
    }
    cholesky(&mut normal, unknowns).map_err(|row| {
        if row < n_events {
            return LocalisationError::InvalidInput(
                "the calibration events cannot be separated from the clocks".into(),
            );
        }
        let who = sensors[(row - n_events) / 2 + 1].index();
        LocalisationError::InvalidInput(format!(
            "the clock of the sensor at node {who} cannot be separated: it must share events \
             with the other sensors, at two or more different times (to measure its drift)"
        ))
    })?;
    let mut solution = rhs;
    solve_factored(&normal, &mut solution, unknowns);

    // Covariances: blocks of (A^T A)^-1 times sigma^2, rate unscaled.
    let sigma2 = pick_sigma.value() * pick_sigma.value();
    let epoch_time = Time::new(zero + epoch);
    let mut out = vec![DriftEstimate {
        node: reference,
        offset: Time::ZERO,
        rate: 0.0,
        epoch: epoch_time,
        covariance: [0.0; 3],
    }];
    for (s, &node) in sensors.iter().enumerate().skip(1) {
        let k = n_events + 2 * (s - 1);
        let column = |i: usize| {
            let mut unit = vec![0.0; unknowns];
            unit[i] = 1.0;
            solve_factored(&normal, &mut unit, unknowns);
            unit
        };
        let (c_offset, c_rate) = (column(k), column(k + 1));
        out.push(DriftEstimate {
            node,
            offset: Time::new(solution[k]),
            rate: solution[k + 1] / span,
            epoch: epoch_time,
            covariance: [
                sigma2 * c_offset[k],
                sigma2 * c_rate[k + 1] / (span * span),
                sigma2 * c_offset[k + 1] / span,
            ],
        });
    }
    Ok(out)
}

/// Turns a sensor's raw arrival (on its own, drifting clock) into a
/// corrected [`Arrival`]: the offset estimated for that moment is removed,
/// and its uncertainty at that moment added (in quadrature) to the picking
/// uncertainty. Sensors without an estimate pass through unchanged with
/// `unknown_clock` uncertainty added.
#[must_use]
pub fn correct_drifting(
    node: NodeId,
    raw: Time,
    pick_sigma: Time,
    estimates: &[DriftEstimate],
    unknown_clock: Time,
) -> Arrival {
    let (offset, clock_sigma) = estimates
        .iter()
        .find(|e| e.node == node)
        .map_or((0.0, unknown_clock.value()), |e| {
            (e.offset_at(raw).value(), e.uncertainty_at(raw).value())
        });
    Arrival {
        node,
        time: Time::new(raw.value() - offset),
        uncertainty: Time::new(pick_sigma.value().hypot(clock_sigma)),
    }
}
