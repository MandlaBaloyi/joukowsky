//! Clock offsets from events with known sources (least squares).
//!
//! For known-source event e seen by sensor s:
//!
//! ```text
//!   t_es = T0_e + tau_s(source_e) + c_s + noise
//! ```
//!
//! `T0_e` (when event e happened) and `c_s` (sensor s's clock offset) are
//! unknown; tau comes from the pipe map. Clocks are only defined relative
//! to one another, so one REFERENCE sensor (ideally GNSS-disciplined) has
//! c = 0. Stacking every (e, s) pair gives an ordinary least-squares
//! problem, solved with the kernel's Cholesky; the same factor gives each
//! offset's uncertainty.
//!
//! A sensor that never shares an event with the rest cannot be separated
//! from its event times - the solve fails loudly and names it, rather than
//! inventing an offset.
//!
//! Caveat, stated: errors in the book wave speeds bias the offsets (a slow
//! pipe looks like a late clock). Sources spread over the network average
//! that out; wave-speed learning (frontier F1) and clock repair will
//! ultimately be estimated together.

use jk_hydraulics::{Network, NodeId};
use jk_kernel::Time;
use jk_kernel::linalg::{cholesky, solve_factored};

use crate::error::{LocalisationError, positive};
use crate::graph::TravelGraph;
use crate::locate::Arrival;

/// An event whose source node is known, and when each sensor saw it.
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationEvent {
    /// Where the event started (e.g. a hydrant or valve operated on purpose).
    pub source: NodeId,
    /// (sensor node, arrival time on that sensor's own clock).
    pub arrivals: Vec<(NodeId, Time)>,
}

/// A sensor's estimated clock offset: its clock reads `offset` late.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockEstimate {
    /// The sensor.
    pub node: NodeId,
    /// Estimated offset (subtract it from the sensor's times).
    pub offset: Time,
    /// One-standard-deviation uncertainty of the offset.
    pub uncertainty: Time,
}

/// Estimates every sensor's clock offset relative to `reference`, from
/// `events`, with arrival-picking uncertainty `pick_sigma`.
///
/// # Errors
/// No events, an unknown node, a non-positive uncertainty, a source no wave
/// reaches a sensor from, or an offset that the events cannot separate.
///
/// # Panics
/// Only if an internal invariant is broken (a sensor missing from the list
/// built from the same events) - that would be a bug.
#[allow(clippy::too_many_lines)] // one least-squares solve, kept together
pub fn estimate_offsets(
    network: &Network,
    events: &[CalibrationEvent],
    reference: NodeId,
    pick_sigma: Time,
) -> Result<Vec<ClockEstimate>, LocalisationError> {
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

    // Sensors seen in any event, reference first.
    let mut sensors: Vec<NodeId> = vec![reference];
    for e in events {
        check(e.source)?;
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

    // Unknowns: one T0 per event, then one offset per non-reference sensor.
    let n_events = events.len();
    let unknowns = n_events + sensors.len() - 1;
    let graph = TravelGraph::new(network);
    let pipe_times: Vec<f64> = network
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let mut normal = vec![0.0; unknowns * unknowns];
    let mut rhs = vec![0.0; unknowns];
    let mut rows = 0_usize;
    for (e, event) in events.iter().enumerate() {
        let tau = graph.times_from(event.source.index(), &pipe_times);
        for &(node, time) in &event.arrivals {
            let travel = tau[node.index()];
            if !travel.is_finite() {
                return Err(LocalisationError::Unreachable { node: node.index() });
            }
            // Row: T0_e (+ c_s unless s is the reference) = t - tau.
            let s = sensors
                .iter()
                .position(|&x| x == node)
                .expect("collected above");
            let mut cols = vec![e];
            if s > 0 {
                cols.push(n_events + s - 1);
            }
            let y = time.value() - travel;
            for &i in &cols {
                rhs[i] += y;
                for &j in &cols {
                    normal[i * unknowns + j] += 1.0;
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
        let who = if row >= n_events {
            sensors[row - n_events + 1].index()
        } else {
            reference.index()
        };
        LocalisationError::InvalidInput(format!(
            "the clock of the sensor at node {who} cannot be separated from the event times: \
             it must share events with the other sensors"
        ))
    })?;
    let mut solution = rhs;
    solve_factored(&normal, &mut solution, unknowns);

    // Offset variances: diagonal of (A^T A)^-1 times sigma^2.
    let sigma = pick_sigma.value();
    let mut out = vec![ClockEstimate {
        node: reference,
        offset: Time::ZERO,
        uncertainty: Time::ZERO,
    }];
    for (s, &node) in sensors.iter().enumerate().skip(1) {
        let k = n_events + s - 1;
        let mut unit = vec![0.0; unknowns];
        unit[k] = 1.0;
        solve_factored(&normal, &mut unit, unknowns);
        out.push(ClockEstimate {
            node,
            offset: Time::new(solution[k]),
            uncertainty: Time::new(sigma * unit[k].max(0.0).sqrt()),
        });
    }
    Ok(out)
}

/// Turns a sensor's raw arrival (on its own clock) into a corrected
/// [`Arrival`]: the estimated offset is removed, and the offset's own
/// uncertainty is added (in quadrature) to the picking uncertainty.
/// Sensors without an estimate are passed through unchanged, with
/// `unknown_clock` uncertainty added.
#[must_use]
pub fn correct(
    node: NodeId,
    raw: Time,
    pick_sigma: Time,
    estimates: &[ClockEstimate],
    unknown_clock: Time,
) -> Arrival {
    let (offset, clock_sigma) = estimates
        .iter()
        .find(|e| e.node == node)
        .map_or((0.0, unknown_clock.value()), |e| {
            (e.offset.value(), e.uncertainty.value())
        });
    Arrival {
        node,
        time: Time::new(raw.value() - offset),
        uncertainty: Time::new(pick_sigma.value().hypot(clock_sigma)),
    }
}
