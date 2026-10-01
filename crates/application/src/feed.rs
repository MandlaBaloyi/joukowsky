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

use jk_channels::{ChannelOptions, ChannelRun, ChannelState, Initial, Reach, Upstream, simulate};
use jk_hydraulics::steady::extended::{PeriodOptions, PeriodRun, Source, TankInlet};
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

/// An intake sump between the canal and the network: the canal fills it
/// through an inlet (orifice law, inside the network's run - stiff, so the
/// network takes the steps the sump needs), the network draws from it.
/// Unlike a fixed level, a sump can run EMPTY: then the network gets only
/// what trickles in, and pressure-driven demand shares that out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sump {
    /// Plan area of its water surface.
    pub area: jk_kernel::Area,
    /// Lowest usable level (its bottom outlet) [m].
    pub floor: Head,
    /// Highest level (it overflows) [m].
    pub ceiling: Head,
    /// Inlet from the canal: Q = inlet sqrt(canal level - max(sump level,
    /// canal bed)) [m^2.5/s], one way. Its sill is the canal's outlet bed.
    pub inlet: f64,
}

/// The canal, sump and network run together.
#[derive(Debug, Clone, PartialEq)]
pub struct SumpRun {
    /// The network's extended-period run (the sump is one of its tanks).
    pub network: PeriodRun,
    /// The canal at the end.
    pub canal: ChannelState,
    /// Water in the canal at the start [m^3].
    pub canal_start: f64,
    /// The sump's level at the network's recorded times.
    pub level: Vec<(Time, Head)>,
    /// Each exchange: (from, to, depth at the canal's outlet at the start,
    /// flow the canal gave over it).
    pub exchanges: Vec<(Time, Time, Length, Flow)>,
    /// Flow the network draws from the sump, at its recorded times.
    pub drawn: Vec<(Time, Flow)>,
}

/// The canal as a live source for the network's run.
struct LiveCanal {
    state: ChannelState,
    bed: f64,
    log: Vec<(Time, Time, Length, Flow)>,
}

impl Source for LiveCanal {
    fn level(&self) -> Head {
        Head::new(self.bed + self.state.outlet_depth().value())
    }

    fn take(&mut self, until: Time, flow: Flow) -> Flow {
        let (from, depth) = (self.state.time(), self.state.outlet_depth());
        let given = self.state.draw(until, flow);
        self.log.push((from, until, depth, given));
        given
    }
}

/// Runs `network` fed by the canal of `feed` through a `sump` at its intake
/// node; the intake reservoir's head is the sump's starting level. The two
/// run side by side: every step of the network's run the canal gives its
/// level, the network moves the sump (the inlet's flow solved with it),
/// then the canal advances over the step giving that flow - or less, if it
/// runs low, and the shortfall comes back out of the sump. Use
/// pressure-driven demand (`period.steady`) so the network's supply falls
/// honestly when the sump runs low.
///
/// # Errors
/// An intake that is not a reservoir of the network, a sump that does not
/// fit it, or a canal or network that cannot run.
///
/// # Panics
/// Never: the sump is found among the tanks just after it is added.
pub fn run_with_sump(
    network: &Network,
    period: &PeriodOptions,
    feed: &CanalFeed,
    sump: Sump,
) -> Result<SumpRun, ApplicationError> {
    let bad = |why: String| ApplicationError::InvalidInput(why);
    if !matches!(
        network.nodes().get(feed.intake.index()),
        Some(Node::Reservoir { .. })
    ) {
        return Err(bad(
            "the intake must be a reservoir node of the network".into()
        ));
    }
    let mut fed = network.clone();
    fed.add_tank(jk_hydraulics::TankStorage {
        node: feed.intake,
        area: sump.area,
        floor: sump.floor,
        ceiling: sump.ceiling,
    })
    .map_err(|e| bad(format!("the sump: {e}")))?;
    let tank = fed
        .tanks()
        .iter()
        .position(|t| t.node == feed.intake)
        .expect("just added");
    let state = ChannelState::new(
        &feed.reach,
        &feed.upstream,
        &jk_channels::Downstream::Outflow(vec![(Time::ZERO, Flow::ZERO)]),
        &feed.initial,
        feed.cells,
        0.9,
    )
    .map_err(|e| bad(format!("the canal: {e}")))?;
    let canal_start = state.volume();
    let mut canal = LiveCanal {
        state,
        bed: feed.outlet_bed.value(),
        log: Vec::new(),
    };
    let mut options = period.clone();
    options.tank_inlets = vec![TankInlet {
        tank: feed.intake,
        source: vec![(Time::ZERO, canal.level())],
        sill: feed.outlet_bed,
        coefficient: sump.inlet,
    }];
    let mut drawn = Vec::new();
    let run = {
        let mut live: [(usize, &mut dyn Source); 1] = [(0, &mut canal)];
        fed.extended_fed(&options, &mut live, &mut |t, s| {
            let out: f64 = fed
                .pipes()
                .iter()
                .zip(&s.pipe_flows)
                .map(|(p, q)| {
                    if p.from == feed.intake {
                        q.value()
                    } else if p.to == feed.intake {
                        -q.value()
                    } else {
                        0.0
                    }
                })
                .sum();
            drawn.push((t, Flow::new(out.max(0.0))));
        })
        .map_err(|e| bad(format!("the network: {e}")))?
    };
    let level = run
        .times
        .iter()
        .zip(&run.tanks[tank].1)
        .map(|(t, h)| (*t, *h))
        .collect();
    Ok(SumpRun {
        network: run,
        canal: canal.state,
        canal_start,
        level,
        exchanges: canal.log,
        drawn,
    })
}
