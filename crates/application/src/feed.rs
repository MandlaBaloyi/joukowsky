//! Source to tap: a canal feeding a pipe network (step R-4 coupling,
//! `docs/02_physics_of_transients/sources_dams_rivers_canals.txt`,
//! section 4).
//!
//! The canal (channels context) and the network (hydraulics context) each
//! run with their own solver; this use case ties them at the intake - the
//! reservoir node the canal fills:
//! - the canal hands the network the intake's water LEVEL over time (bed
//!   level at the outlet + depth there), a level history on that node;
//! - the network hands back the FLOW it draws from the intake over time,
//!   the canal's downstream boundary (water drawn off).
//!
//! Each needs the other, so they take turns until the drawn flow stops
//! changing (a fixed-point iteration). Water is conserved across the
//! interface: what leaves the canal is what the network draws, at every
//! recorded time. When the canal runs dry it gives nothing - and the
//! network sees the intake level fall to the outlet's bed.

use jk_channels::{ChannelOptions, ChannelRun, Initial, Reach, Upstream, simulate};
use jk_hydraulics::steady::extended::{PeriodOptions, PeriodRun};
use jk_hydraulics::{Network, Node, NodeId, SourceLevel};
use jk_kernel::{Flow, Head, Length, Time};

use crate::error::ApplicationError;

/// Most turns before giving up.
const MAX_TURNS: usize = 40;

/// The canal feeding a network, and where.
#[derive(Debug, Clone, PartialEq)]
pub struct CanalFeed {
    /// The canal.
    pub reach: Reach,
    /// What comes into it upstream.
    pub upstream: Upstream,
    /// Its state at the start.
    pub initial: Initial,
    /// Cells along it.
    pub cells: usize,
    /// Bed level at its outlet [m]: the intake's water level is this plus
    /// the depth there.
    pub outlet_bed: Head,
    /// The network's reservoir node the canal fills.
    pub intake: NodeId,
    /// Settled when the drawn flow changes by less than this between turns
    /// [m^3/s].
    pub tolerance: Flow,
}

/// The canal and the network run together.
#[derive(Debug, Clone, PartialEq)]
pub struct FedRun {
    /// The network's extended-period run.
    pub network: PeriodRun,
    /// The canal's run (gauge 0: its outlet).
    pub canal: ChannelRun,
    /// Flow the network draws from the intake, at its recorded times.
    pub drawn: Vec<(Time, Flow)>,
    /// Turns taken.
    pub turns: usize,
    /// Largest change of the drawn flow in the last turn [m^3/s].
    pub change: f64,
    /// When the canal's outlet ran dry while the network still drew water
    /// (None: it never did). From then on the network's demands are not
    /// met: they are fixed in the steady solver (demand-driven), so it keeps
    /// drawing - the run says when that stops being true.
    pub dry_from: Option<Time>,
}

/// Runs `network` (extended period `period`) fed by the canal `feed`.
///
/// # Errors
/// An intake that is not a plain reservoir of the network, a canal or
/// network that cannot run, or no settling within the allowed turns.
pub fn run_fed(
    network: &Network,
    period: &PeriodOptions,
    feed: &CanalFeed,
) -> Result<FedRun, ApplicationError> {
    let bad = |why: String| ApplicationError::InvalidInput(why);
    if !matches!(
        network.nodes().get(feed.intake.index()),
        Some(Node::Reservoir { .. })
    ) {
        return Err(bad(
            "the intake must be a reservoir node of the network".into()
        ));
    }
    let outlet = Length::new(feed.reach.length.value());
    // Flow drawn out of the intake node, from a steady state's pipe flows.
    let out_of_intake = |flows: &[Flow]| -> f64 {
        network
            .pipes()
            .iter()
            .zip(flows)
            .map(|(p, q)| {
                if p.from == feed.intake {
                    q.value()
                } else if p.to == feed.intake {
                    -q.value()
                } else {
                    0.0
                }
            })
            .sum()
    };
    let mut drawn: Vec<(Time, Flow)> = vec![(Time::ZERO, Flow::ZERO)];
    let mut turns = 0;
    loop {
        turns += 1;
        let canal = simulate(
            &feed.reach,
            &feed.upstream,
            &jk_channels::Downstream::Outflow(drawn.clone()),
            &feed.initial,
            &[outlet],
            &ChannelOptions {
                duration: period.duration,
                cells: feed.cells,
                courant: 0.9,
                record_every: period.step,
            },
        )
        .map_err(|e| bad(format!("the canal: {e}")))?;
        let history: Vec<(Time, Head)> = canal
            .times
            .iter()
            .zip(&canal.gauges[0].1)
            .map(|(t, h)| (*t, Head::new(feed.outlet_bed.value() + h.value())))
            .collect();
        let mut fed = network.clone();
        fed.add_level(SourceLevel {
            node: feed.intake,
            history,
        })
        .map_err(|e| bad(format!("the intake: {e}")))?;
        let mut new: Vec<(Time, Flow)> = Vec::new();
        let run = fed
            .extended_observed(period, &mut |t, s| {
                new.push((t, Flow::new(out_of_intake(&s.pipe_flows).max(0.0))));
            })
            .map_err(|e| bad(format!("the network: {e}")))?;
        // Compare with the last turn at the new times.
        let change = new
            .iter()
            .map(|(t, q)| (q.value() - value_at(&drawn, t.value())).abs())
            .fold(0.0, f64::max);
        drawn = new;
        if change <= feed.tolerance.value() || turns >= MAX_TURNS {
            if change > feed.tolerance.value() {
                return Err(bad(format!(
                    "canal and network did not settle in {MAX_TURNS} turns (drawn flow still \
                     changing by {change:.2e} m^3/s)"
                )));
            }
            let dry_from = canal
                .times
                .iter()
                .zip(&canal.gauges[0].1)
                .find(|(t, h)| h.value() <= 1e-3 && value_at(&drawn, t.value()) > 0.0)
                .map(|(t, _)| *t);
            return Ok(FedRun {
                network: run,
                canal,
                drawn,
                turns,
                change,
                dry_from,
            });
        }
    }
}

/// A (time, flow) history at time `t`, linear between points, held beyond.
fn value_at(points: &[(Time, Flow)], t: f64) -> f64 {
    match points.iter().position(|(pt, _)| pt.value() > t) {
        None => points.last().map_or(0.0, |p| p.1.value()),
        Some(0) => points[0].1.value(),
        Some(k) => {
            let ((t0, q0), (t1, q1)) = (points[k - 1], points[k]);
            let w = (t - t0.value()) / (t1.value() - t0.value());
            q0.value() + w * (q1.value() - q0.value())
        }
    }
}
