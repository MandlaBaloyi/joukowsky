//! Extended-period run: hours to days as a chain of steady states, with
//! storage tanks filling and draining in between, and pumps and valves
//! switched by tank levels, clock times and power cuts - the supply side of
//! the network (step R-2, `docs/02_physics_of_transients/sources_dams_
//! rivers_canals.txt`).
//!
//! Tanks drain over hours; waves cross a site in seconds. Simulating hours
//! at wave time steps would take millions of steps, so - as EPANET does -
//! the run solves the steady state every `step` (minutes), with the demands
//! of that moment and every tank at its current level, and moves each tank
//! by its net inflow:
//!
//! ```text
//!   A dh/dt = Q_in - Q_out
//! ```
//!
//! integrated with Heun's method (a predictor step, then the average of the
//! slopes at both ends - second order, so halving the step quarters the
//! error). Reservoirs with a level history follow it; other reservoirs hold
//! their head.
//!
//! Nothing that changes suddenly is smeared across a step: a step is cut
//! short to land exactly where a tank reaches its floor, its ceiling or a
//! control's switching level (found from the step's own rate, then the step
//! is redone to there), and at every clock-time switch, power-cut boundary
//! and demand-pattern boundary. So a pump starts at the moment the tank
//! reaches its switch-on level, whatever the step.
//!
//! What it warns about - the supply events:
//! - a tank falling to its floor stops supplying: from then on it is closed
//!   (a node with no demand), and reopens when the network would push water
//!   back into it;
//! - a tank rising to its ceiling overflows and holds there;
//! - pumps starting and stopping, pipes opening and closing, power going
//!   off and coming back ([`controls`](crate::steady::controls));
//! - when no source can reach part of the network that draws water the run
//!   stops: supply is lost there - exactly when a supply cut would start
//!   draining pipes, and the refilling surge that follows (gap G-14)
//!   becomes a risk.

use jk_kernel::{Flow, Head, Time};

use crate::HydraulicsError;
use crate::error::positive;
use crate::model::{Network, Node, NodeId, PipeId};
use crate::steady::controls::{self, Cause, Control, TIME_TOL, Target};
use crate::steady::rules::{self, Clause, Rule, Situation};
use crate::steady::{Shut, SteadyOptions, SteadySolution};

/// Below this share of the demand delivered, the supply is short.
const SHORT_SHARE: f64 = 0.99;

/// Most sub-steps (cuts at switches) inside one reporting step before the
/// run gives up: a guard against controls that chatter.
const MAX_SUBSTEPS: usize = 1000;

/// How the extended-period run proceeds.
#[derive(Debug, Clone, PartialEq)]
pub struct PeriodOptions {
    /// Time between reported states (and the longest step). Default 5
    /// minutes.
    pub step: Time,
    /// Total time to cover. The run starts at midnight.
    pub duration: Time,
    /// Demand multipliers, one per `pattern_step`, repeating (e.g. 24 hourly
    /// factors for a day), for every junction not in `node_patterns`.
    /// Empty: demands stay as given.
    pub pattern: Vec<f64>,
    /// Patterns of their own for some junctions (homes, a factory, a
    /// hospital follow different days), same `pattern_step`.
    pub node_patterns: Vec<NodePattern>,
    /// Length of each pattern entry. Default one hour.
    pub pattern_step: Time,
    /// Settings of each steady solve.
    pub steady: SteadyOptions,
    /// Pumps and pipes switched by tank levels and clock times, checked in
    /// order: when two act on the same target at once, the later wins. At
    /// the start every pipe is open and every pump running, as the network
    /// stands, and then the controls are applied.
    pub controls: Vec<Control>,
    /// Power cuts (load-shedding), as (start, end): no pump runs during a
    /// cut; when the power returns each pump does what the controls last
    /// asked. Also any window in which pumps must not run (peak tariff).
    pub power_cuts: Vec<(Time, Time)>,
    /// Rule-based controls (IF ... THEN ... ELSE ..., with priorities),
    /// looked at after the simple controls ([`rules`]).
    pub rules: Vec<Rule>,
}

impl Default for PeriodOptions {
    fn default() -> Self {
        Self {
            step: Time::new(300.0),
            duration: Time::new(86_400.0),
            pattern: Vec::new(),
            node_patterns: Vec::new(),
            pattern_step: Time::new(3600.0),
            steady: SteadyOptions::default(),
            controls: Vec::new(),
            power_cuts: Vec::new(),
            rules: Vec::new(),
        }
    }
}

/// A demand pattern followed by some junctions.
#[derive(Debug, Clone, PartialEq)]
pub struct NodePattern {
    /// Demand multipliers, one per `pattern_step`, repeating. Empty: the
    /// demands stay as given.
    pub multipliers: Vec<f64>,
    /// The junctions that follow it.
    pub nodes: Vec<NodeId>,
}

/// Something that happened to the supply.
#[derive(Debug, Clone, PartialEq)]
pub enum SupplyEvent {
    /// A tank fell to its floor and stopped supplying.
    TankEmpty {
        /// The tank.
        tank: NodeId,
        /// When.
        at: Time,
    },
    /// An empty tank started to fill again.
    TankRefilling {
        /// The tank.
        tank: NodeId,
        /// When.
        at: Time,
    },
    /// A tank rose to its ceiling and overflows.
    TankFull {
        /// The tank.
        tank: NodeId,
        /// When.
        at: Time,
    },
    /// A pump started or stopped, or a pipe opened or closed.
    Switched {
        /// What switched.
        target: Target,
        /// Now open/running (true) or closed/stopped (false).
        open: bool,
        /// When.
        at: Time,
        /// Why.
        cause: Cause,
    },
    /// The power failed: every pump stops.
    PowerOff {
        /// When.
        at: Time,
    },
    /// The power came back.
    PowerOn {
        /// When.
        at: Time,
    },
    /// Pressure-driven demand delivers less than 99% of what is asked
    /// (only with [`crate::steady::PressureDemand`]).
    SupplyShort {
        /// When (a recorded time).
        at: Time,
        /// Share of the demand delivered then (0..1).
        delivered: f64,
    },
    /// The demand is met again (99% or more).
    SupplyMet {
        /// When (a recorded time).
        at: Time,
    },
    /// No source reaches part of the network that draws water: the run
    /// stops.
    SupplyLost {
        /// When.
        at: Time,
        /// Why, in words.
        reason: String,
    },
}

/// The result of an extended-period run.
#[derive(Debug, Clone, PartialEq)]
pub struct PeriodRun {
    /// Time of each recorded state.
    pub times: Vec<Time>,
    /// Per tank (in [`Network::tanks`] order): its node and its level at
    /// each recorded time.
    pub tanks: Vec<(NodeId, Vec<Head>)>,
    /// Per pump node (in node order): whether it runs at each recorded
    /// time.
    pub pumps: Vec<(NodeId, Vec<bool>)>,
    /// Head at every node, at each recorded time (NaN where closed pipes
    /// cut a part off with no demand: its pressure is unknown).
    pub node_heads: Vec<Vec<Head>>,
    /// Demand asked and delivered over the whole network, at each recorded
    /// time (equal unless pressure-driven demand cuts it).
    pub supply: Vec<(Flow, Flow)>,
    /// Supply events, in time order.
    pub events: Vec<SupplyEvent>,
}

/// One tank's state during the run.
#[derive(Debug, Clone, Copy)]
struct TankState {
    node: usize,
    area: f64,
    floor: f64,
    ceiling: f64,
    level: f64,
    open: bool,
    /// The way the level last moved: +1 rising, -1 falling, 0 still.
    trend: f64,
}

/// Which pipes are open and which pumps are wanted, and the power.
#[derive(Debug, Clone, PartialEq)]
struct Status {
    pipe_open: Vec<bool>,
    /// Per node: the controls want this pump running (pump nodes only).
    pump_wanted: Vec<bool>,
    pumps: Vec<usize>,
    powered: bool,
}

impl Status {
    fn new(network: &Network) -> Self {
        let pumps: Vec<usize> = network
            .nodes()
            .iter()
            .enumerate()
            .filter(|(_, n)| matches!(n, Node::Pump { .. }))
            .map(|(k, _)| k)
            .collect();
        Self {
            pipe_open: vec![true; network.pipes().len()],
            pump_wanted: vec![true; network.nodes().len()],
            pumps,
            powered: true,
        }
    }

    /// Whether pump node `n` actually runs.
    fn running(&self, n: usize) -> bool {
        self.powered && self.pump_wanted[n]
    }

    /// The target's actual state.
    fn is_open(&self, target: Target) -> bool {
        match target {
            Target::Pipe(p) => self.pipe_open[p.index()],
            Target::Pump(n) => self.running(n.index()),
        }
    }

    /// The state the controls asked for.
    fn wanted(&self, target: Target) -> bool {
        match target {
            Target::Pipe(p) => self.pipe_open[p.index()],
            Target::Pump(n) => self.pump_wanted[n.index()],
        }
    }

    fn set(&mut self, target: Target, open: bool) {
        match target {
            Target::Pipe(p) => self.pipe_open[p.index()] = open,
            Target::Pump(n) => self.pump_wanted[n.index()] = open,
        }
    }

    fn shut(&self) -> Shut {
        Shut {
            pipes: self
                .pipe_open
                .iter()
                .enumerate()
                .filter(|(_, open)| !**open)
                .map(|(p, _)| PipeId(p))
                .collect(),
            pumps: self
                .pumps
                .iter()
                .filter(|&&n| !self.running(n))
                .map(|&n| NodeId(n))
                .collect(),
        }
    }
}

/// Where a step has to stop early: a tank reaching a level.
#[derive(Debug, Clone, Copy)]
struct Crossing {
    /// Fraction of the step at which it happens (0, 1].
    fraction: f64,
    tank: usize,
    level: f64,
}

impl Network {
    /// Runs the network for `options.duration` as a chain of steady states,
    /// tanks filling and draining in between, pumps and valves switched by
    /// the controls.
    ///
    /// # Errors
    /// Invalid options or controls, or a steady state that cannot be solved
    /// at the start (later failures from lost supply end the run with a
    /// [`SupplyEvent::SupplyLost`] instead).
    pub fn extended(&self, options: &PeriodOptions) -> Result<PeriodRun, HydraulicsError> {
        self.extended_observed(options, &mut |_, _| {})
    }

    /// As [`Network::extended`], also handing `observe` the whole steady
    /// state at every recorded time - every flow, every device's state - for
    /// what the run does not keep (e.g. which valves are shut at 03:00).
    ///
    /// # Errors
    /// As [`Network::extended`].
    #[allow(clippy::too_many_lines)] // one time loop with its supply events, kept together
    pub fn extended_observed(
        &self,
        options: &PeriodOptions,
        observe: &mut dyn FnMut(Time, &SteadySolution),
    ) -> Result<PeriodRun, HydraulicsError> {
        positive("step", options.step.value())?;
        positive("duration", options.duration.value())?;
        positive("pattern_step", options.pattern_step.value())?;
        let patterns = std::iter::once(&options.pattern)
            .chain(options.node_patterns.iter().map(|p| &p.multipliers));
        for pattern in patterns {
            if pattern.iter().any(|m| !(m.is_finite() && *m >= 0.0)) {
                return Err(HydraulicsError::invalid(
                    "demand pattern",
                    f64::NAN,
                    "finite multipliers >= 0",
                ));
            }
        }
        for n in options.node_patterns.iter().flat_map(|p| &p.nodes) {
            if !matches!(self.nodes().get(n.index()), Some(Node::Junction { .. })) {
                return Err(HydraulicsError::InvalidNetwork(format!(
                    "node {} given a demand pattern is not a junction",
                    n.index()
                )));
            }
        }
        controls::check(self, &options.controls, &options.power_cuts)?;
        rules::check(self, &options.rules)?;
        let mut tanks: Vec<TankState> = self
            .tanks()
            .iter()
            .map(|t| {
                let Node::Reservoir { head } = self.nodes()[t.node.index()] else {
                    unreachable!("add_tank accepts reservoirs only")
                };
                TankState {
                    node: t.node.index(),
                    area: t.area.value(),
                    floor: t.floor.value(),
                    ceiling: t.ceiling.value(),
                    level: head.value(),
                    open: true,
                    trend: 0.0,
                }
            })
            .collect();
        let dt = options.step.value();
        let end = options.duration.value();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive, bounded
        let steps = (end / dt - 1e-9).ceil() as usize;

        let mut status = Status::new(self);
        let mut run = PeriodRun {
            times: Vec::with_capacity(steps + 1),
            tanks: tanks
                .iter()
                .map(|t| (NodeId(t.node), Vec::with_capacity(steps + 1)))
                .collect(),
            pumps: status
                .pumps
                .iter()
                .map(|&n| (NodeId(n), Vec::with_capacity(steps + 1)))
                .collect(),
            node_heads: Vec::with_capacity(steps + 1),
            supply: Vec::with_capacity(steps + 1),
            events: Vec::new(),
        };

        let mut t = 0.0;
        switch(options, &mut status, &tanks, t, &mut run.events);
        let here = self.settle(t, &mut tanks, &status, options, &mut run.events)?;
        let mut here = self.obey(options, &mut status, &mut tanks, t, here, &mut run.events)?;
        record(&mut run, t, &tanks, &status, &here);
        observe(Time::new(t), &here);
        for k in 1..=steps {
            #[allow(clippy::cast_precision_loss)] // k is small
            let report = (k as f64 * dt).min(end);
            let mut substeps = 0;
            while t < report - TIME_TOL {
                substeps += 1;
                if substeps > MAX_SUBSTEPS {
                    return Err(HydraulicsError::InvalidNetwork(format!(
                        "more than {MAX_SUBSTEPS} switches within one step at t = {t:.0} s: \
                         controls keep switching each other"
                    )));
                }
                // Step to the next report time or timed switch, whichever
                // comes first...
                let boundary = next_boundary(options, t).min(report);
                let mut target = boundary;
                let rates = match self.heun(&here, &tanks, &status, t, target - t, options) {
                    Ok(r) => r,
                    Err(e) => return Ok(lost(run, target, &e)),
                };
                // ...or to where a tank reaches a level that matters, redone
                // from the start with the shorter step.
                let mut rates = rates;
                let crossing = crossing(&tanks, &rates, target - t, &status, options);
                if let Some(c) = crossing
                    && c.fraction < 1.0
                {
                    target = t + c.fraction * (target - t);
                    rates = match self.heun(&here, &tanks, &status, t, target - t, options) {
                        Ok(r) => r,
                        Err(e) => return Ok(lost(run, target, &e)),
                    };
                }
                let h = target - t;
                for (i, tank) in tanks.iter_mut().enumerate() {
                    if !tank.open {
                        continue;
                    }
                    let rate = rates[i];
                    tank.trend = if rate == 0.0 { 0.0 } else { rate.signum() };
                    let mut level = tank.level + h * rate;
                    if let Some(c) = crossing
                        && c.tank == i
                    {
                        level = c.level; // land on it (the redo moves it ~h^3)
                    }
                    if level <= tank.floor && rate < 0.0 {
                        tank.level = tank.floor;
                        tank.open = false;
                        run.events.push(SupplyEvent::TankEmpty {
                            tank: NodeId(tank.node),
                            at: Time::new(target),
                        });
                    } else if level >= tank.ceiling && rate > 0.0 {
                        if tank.level < tank.ceiling {
                            run.events.push(SupplyEvent::TankFull {
                                tank: NodeId(tank.node),
                                at: Time::new(target),
                            });
                        }
                        tank.level = tank.ceiling;
                    } else {
                        tank.level = level;
                    }
                }
                t = target;
                switch(options, &mut status, &tanks, t, &mut run.events);
                here = match self
                    .settle(t, &mut tanks, &status, options, &mut run.events)
                    .and_then(|s| {
                        self.obey(options, &mut status, &mut tanks, t, s, &mut run.events)
                    }) {
                    Ok(s) => s,
                    Err(e) => return Ok(lost(run, t, &e)),
                };
            }
            t = report;
            record(&mut run, t, &tanks, &status, &here);
            observe(Time::new(t), &here);
        }
        Ok(run)
    }

    /// Applies the rules to the state `here` at `t`; if they switch
    /// anything, solves again. Returns the state to go on from.
    fn obey(
        &self,
        options: &PeriodOptions,
        status: &mut Status,
        tanks: &mut [TankState],
        t: f64,
        here: SteadySolution,
        events: &mut Vec<SupplyEvent>,
    ) -> Result<SteadySolution, HydraulicsError> {
        if options.rules.is_empty() {
            return Ok(here);
        }
        let acts = rules::actions(
            &options.rules,
            &Now {
                network: self,
                t,
                tanks,
                here: &here,
                status,
            },
        );
        let before = status.clone();
        let mut by = Vec::new();
        for (a, k) in acts {
            if status.wanted(a.target) != a.open {
                status.set(a.target, a.open);
                by.push((a.target, k));
            }
        }
        let mut changed = false;
        for (target, k) in by {
            let open = status.is_open(target);
            if open != before.is_open(target) {
                changed = true;
                events.push(SupplyEvent::Switched {
                    target,
                    open,
                    at: Time::new(t),
                    cause: Cause::Rule(k),
                });
            }
        }
        if changed {
            self.settle(t, tanks, status, options, events)
        } else {
            Ok(here)
        }
    }

    /// The steady state at `t` for the current tanks and status, reopening
    /// any empty tank the network would now fill.
    fn settle(
        &self,
        t: f64,
        tanks: &mut [TankState],
        status: &Status,
        options: &PeriodOptions,
        events: &mut Vec<SupplyEvent>,
    ) -> Result<SteadySolution, HydraulicsError> {
        loop {
            let state = self.state_at(t, t, tanks, status, options)?;
            let mut reopened = false;
            for tank in tanks.iter_mut().filter(|tank| !tank.open) {
                if state.node_heads[tank.node].value() > tank.floor {
                    tank.open = true;
                    reopened = true;
                    events.push(SupplyEvent::TankRefilling {
                        tank: NodeId(tank.node),
                        at: Time::new(t),
                    });
                }
            }
            if !reopened {
                return Ok(state);
            }
        }
    }

    /// Heun's step of length `h` from `here` (the state at `t`): the rate
    /// of each tank's level [m/s] (0 for closed tanks). Demands are those of
    /// the step's pattern entry at both ends (steps never cross a pattern
    /// boundary).
    fn heun(
        &self,
        here: &SteadySolution,
        tanks: &[TankState],
        status: &Status,
        t: f64,
        h: f64,
        options: &PeriodOptions,
    ) -> Result<Vec<f64>, HydraulicsError> {
        let slope_now = self.net_inflows(here, tanks);
        let mut predicted = tanks.to_vec();
        for (tank, q) in predicted.iter_mut().zip(&slope_now) {
            if tank.open {
                tank.level = (tank.level + h * q / tank.area).min(tank.ceiling);
            }
        }
        let there = self.state_at(t + h, t + 0.5 * h, &predicted, status, options)?;
        let slope_there = self.net_inflows(&there, &predicted);
        Ok(tanks
            .iter()
            .zip(slope_now.iter().zip(&slope_there))
            .map(|(tank, (a, b))| {
                if tank.open {
                    f64::midpoint(*a, *b) / tank.area
                } else {
                    0.0
                }
            })
            .collect())
    }

    /// The steady state at time `t` with the tanks and status as given: each
    /// open tank a fixed head at its level, each closed one a node with no
    /// demand; level histories at their value at `t`; demands scaled by the
    /// pattern entry in force at `pattern_time`.
    fn state_at(
        &self,
        t: f64,
        pattern_time: f64,
        tanks: &[TankState],
        status: &Status,
        options: &PeriodOptions,
    ) -> Result<SteadySolution, HydraulicsError> {
        let mut net = self.clone();
        let mut m = vec![multiplier(&options.pattern, options, pattern_time); self.nodes().len()];
        for p in &options.node_patterns {
            let own = multiplier(&p.multipliers, options, pattern_time);
            for n in &p.nodes {
                m[n.index()] = own;
            }
        }
        for (n, node) in self.nodes().iter().enumerate() {
            match *node {
                Node::Reservoir { head } => net.set_node(
                    n,
                    Node::Reservoir {
                        head: Head::new(self.source_head(n, head, Time::new(t))),
                    },
                ),
                Node::Junction { demand } => net.set_node(
                    n,
                    Node::Junction {
                        demand: demand * m[n],
                    },
                ),
                _ => {}
            }
        }
        net.clear_levels();
        for tank in tanks {
            net.set_node(
                tank.node,
                if tank.open {
                    Node::Reservoir {
                        head: Head::new(tank.level),
                    }
                } else {
                    Node::Junction {
                        demand: jk_kernel::Flow::ZERO,
                    }
                },
            );
        }
        let shut = status.shut();
        if shut.is_empty() {
            net.steady_state(&options.steady)
        } else {
            net.steady_state_shut(&options.steady, &shut)
        }
    }

    /// Net flow INTO each tank [m^3/s] in `state` (0 for closed tanks).
    fn net_inflows(&self, state: &SteadySolution, tanks: &[TankState]) -> Vec<f64> {
        tanks
            .iter()
            .map(|tank| {
                if !tank.open {
                    return 0.0;
                }
                self.pipes()
                    .iter()
                    .zip(&state.pipe_flows)
                    .map(|(p, q)| {
                        if p.to.index() == tank.node {
                            q.value()
                        } else if p.from.index() == tank.node {
                            -q.value()
                        } else {
                            0.0
                        }
                    })
                    .sum()
            })
            .collect()
    }
}

/// The first level inside a step of length `h` that some open tank
/// reaches and that matters: its floor, its ceiling, or the level of a
/// control that would change its target's state.
fn crossing(
    tanks: &[TankState],
    rates: &[f64],
    h: f64,
    status: &Status,
    options: &PeriodOptions,
) -> Option<Crossing> {
    let mut first: Option<Crossing> = None;
    let mut consider = |tank: usize, level: f64| {
        let (l0, change) = (tanks[tank].level, rates[tank] * h);
        if change == 0.0 {
            return;
        }
        let fraction = (level - l0) / change;
        if fraction > 0.0 && fraction < 1.0 && first.is_none_or(|c| fraction < c.fraction) {
            first = Some(Crossing {
                fraction,
                tank,
                level,
            });
        }
    };
    for (i, tank) in tanks.iter().enumerate() {
        if !tank.open {
            continue;
        }
        if rates[i] < 0.0 {
            consider(i, tank.floor);
        } else if tank.level < tank.ceiling {
            consider(i, tank.ceiling);
        }
    }
    for c in &options.controls {
        if status.wanted(c.target) == c.open {
            continue;
        }
        let (node, level, falling) = match c.when {
            controls::Condition::LevelBelow { tank, level } => (tank, level.value(), true),
            controls::Condition::LevelAbove { tank, level } => (tank, level.value(), false),
            _ => continue,
        };
        if let Some(i) = tanks.iter().position(|t| t.node == node.index())
            && tanks[i].open
            && (rates[i] < 0.0) == falling
        {
            consider(i, level);
        }
    }
    // Every level a rule names: its truth may change there.
    for p in options.rules.iter().flat_map(|r| &r.premises) {
        if let Clause::TankLevel { tank, level, .. } = p.clause
            && let Some(i) = tanks.iter().position(|t| t.node == tank.index())
            && tanks[i].open
        {
            consider(i, level.value());
        }
    }
    first
}

/// The next time after `t` at which something switches on the clock: a
/// timed control, a power-cut boundary, a demand-pattern boundary, the
/// end of the run.
fn next_boundary(options: &PeriodOptions, t: f64) -> f64 {
    let mut next = options.duration.value();
    for c in &options.controls {
        if let Some(at) = c.when.next_time(t) {
            next = next.min(at);
        }
    }
    for p in options.rules.iter().flat_map(|r| &r.premises) {
        if let Some(at) = p.clause.next_time(t) {
            next = next.min(at);
        }
    }
    for (from, to) in &options.power_cuts {
        for edge in [from.value(), to.value()] {
            if edge > t + TIME_TOL {
                next = next.min(edge);
            }
        }
    }
    let patterned = !options.pattern.is_empty()
        || options
            .node_patterns
            .iter()
            .any(|p| !p.multipliers.is_empty());
    if patterned {
        let ps = options.pattern_step.value();
        let mut edge = ((t / ps).floor() + 1.0) * ps;
        if edge <= t + TIME_TOL {
            edge += ps;
        }
        next = next.min(edge);
    }
    next
}

/// Applies the power and the controls at time `t`, recording every
/// actual change.
fn switch(
    options: &PeriodOptions,
    status: &mut Status,
    tanks: &[TankState],
    t: f64,
    events: &mut Vec<SupplyEvent>,
) {
    let before = status.clone();
    status.powered = !options
        .power_cuts
        .iter()
        .any(|(from, to)| from.value() - TIME_TOL <= t && t < to.value() - TIME_TOL);
    if status.powered != before.powered {
        events.push(if status.powered {
            SupplyEvent::PowerOn { at: Time::new(t) }
        } else {
            SupplyEvent::PowerOff { at: Time::new(t) }
        });
    }
    let level_of = |n: NodeId| {
        tanks
            .iter()
            .find(|tank| tank.node == n.index())
            .map_or(f64::NAN, |tank| tank.level)
    };
    // Which control last set each target, if any changed it.
    let mut by: Vec<(Target, usize)> = Vec::new();
    for (k, c) in options.controls.iter().enumerate() {
        if c.when.holds(t, level_of) {
            if status.wanted(c.target) != c.open {
                by.retain(|(target, _)| *target != c.target);
                by.push((c.target, k));
            }
            status.set(c.target, c.open);
        }
    }
    let targets = (0..status.pipe_open.len())
        .map(|p| Target::Pipe(PipeId(p)))
        .chain(status.pumps.iter().map(|&n| Target::Pump(NodeId(n))));
    for target in targets {
        let open = status.is_open(target);
        if open == before.is_open(target) {
            continue;
        }
        let cause = by.iter().find(|(x, _)| *x == target).map_or(
            if status.powered {
                Cause::PowerBack
            } else {
                Cause::PowerCut
            },
            |&(_, k)| Cause::Control(k),
        );
        events.push(SupplyEvent::Switched {
            target,
            open,
            at: Time::new(t),
            cause,
        });
    }
}

/// How far past the instant rules look [s], and how far a moving level is
/// taken on [m]: so a rule switches exactly where a level crosses its value
/// or the clock passes its time, whichever way.
const JUST_AFTER_S: f64 = 1e-7;
const JUST_AFTER_M: f64 = 1e-9;

/// The run at one instant, as rules see it.
struct Now<'a> {
    network: &'a Network,
    t: f64,
    tanks: &'a [TankState],
    here: &'a SteadySolution,
    status: &'a Status,
}

impl Situation for Now<'_> {
    fn time(&self) -> f64 {
        // Just after the instant looked at.
        self.t + JUST_AFTER_S
    }

    fn level(&self, tank: NodeId) -> f64 {
        self.tanks
            .iter()
            .find(|x| x.node == tank.index())
            // Just after: a hair on in the way it is moving.
            .map_or(f64::NAN, |x| x.level + x.trend * JUST_AFTER_M)
    }

    fn head(&self, node: NodeId) -> f64 {
        self.here.node_heads[node.index()].value()
    }

    fn flow(&self, target: Target) -> f64 {
        match target {
            Target::Pipe(p) => self.here.pipe_flows[p.index()].value(),
            // Through a pump: its discharge pipe's flow.
            Target::Pump(n) => self
                .network
                .pipes()
                .iter()
                .position(|p| p.from == n)
                .map_or(0.0, |k| self.here.pipe_flows[k].value()),
        }
    }

    fn open(&self, target: Target) -> bool {
        self.status.is_open(target)
    }
}

/// Records the state at a report time.
fn record(
    run: &mut PeriodRun,
    t: f64,
    tanks: &[TankState],
    status: &Status,
    here: &SteadySolution,
) {
    run.times.push(Time::new(t));
    for (i, tank) in tanks.iter().enumerate() {
        run.tanks[i].1.push(Head::new(tank.level));
    }
    for (series, &n) in run.pumps.iter_mut().zip(&status.pumps) {
        series.1.push(status.running(n));
    }
    run.node_heads.push(here.node_heads.clone());
    let asked: f64 = here.requested.iter().map(|q| q.value()).sum();
    let given: f64 = here.delivered.iter().map(|q| q.value()).sum();
    let short = |(a, g): (f64, f64)| a > 0.0 && g < SHORT_SHARE * a;
    let was_short = run
        .supply
        .last()
        .is_some_and(|(a, g)| short((a.value(), g.value())));
    if short((asked, given)) && !was_short {
        run.events.push(SupplyEvent::SupplyShort {
            at: Time::new(t),
            delivered: given / asked,
        });
    } else if !short((asked, given)) && was_short {
        run.events.push(SupplyEvent::SupplyMet { at: Time::new(t) });
    }
    run.supply.push((Flow::new(asked), Flow::new(given)));
}

/// Ends the run: supply lost at `t`.
fn lost(mut run: PeriodRun, t: f64, e: &HydraulicsError) -> PeriodRun {
    run.events.push(SupplyEvent::SupplyLost {
        at: Time::new(t),
        reason: format!("no source reaches part of the network ({e})"),
    });
    run
}

/// The multiplier of `pattern` in force at time `t`.
fn multiplier(pattern: &[f64], options: &PeriodOptions, t: f64) -> f64 {
    if pattern.is_empty() {
        return 1.0;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // t >= 0
    let k = (t / options.pattern_step.value()).floor() as usize;
    pattern[k % pattern.len()]
}
