//! Extended-period run: hours to days as a chain of steady states, with
//! storage tanks filling and draining in between - the supply side of the
//! network (step R-2, `docs/02_physics_of_transients/sources_dams_rivers_
//! canals.txt`).
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
//! What it warns about - the supply events:
//! - a tank falling to its floor stops supplying: from then on it is closed
//!   (a node with no demand), and reopens when the network would push water
//!   back into it;
//! - a tank rising to its ceiling overflows and holds there;
//! - when no source can reach part of the network the run stops: supply is
//!   lost there - exactly when a supply cut would start draining pipes, and
//!   the refilling surge that follows (gap G-14) becomes a risk.

use jk_kernel::{Head, Time};

use crate::HydraulicsError;
use crate::error::positive;
use crate::model::{Network, Node, NodeId};
use crate::steady::{SteadyOptions, SteadySolution};

/// How the extended-period run proceeds.
#[derive(Debug, Clone, PartialEq)]
pub struct PeriodOptions {
    /// Time between steady states. Default 5 minutes.
    pub step: Time,
    /// Total time to cover.
    pub duration: Time,
    /// Demand multipliers, one per `pattern_step`, repeating (e.g. 24 hourly
    /// factors for a day). Empty: demands stay as given.
    pub pattern: Vec<f64>,
    /// Length of each pattern entry. Default one hour.
    pub pattern_step: Time,
    /// Settings of each steady solve.
    pub steady: SteadyOptions,
}

impl Default for PeriodOptions {
    fn default() -> Self {
        Self {
            step: Time::new(300.0),
            duration: Time::new(86_400.0),
            pattern: Vec::new(),
            pattern_step: Time::new(3600.0),
            steady: SteadyOptions::default(),
        }
    }
}

/// Something that happened to the supply.
#[derive(Debug, Clone, PartialEq)]
pub enum SupplyEvent {
    /// A tank fell to its floor and stopped supplying.
    TankEmpty {
        /// The tank.
        tank: NodeId,
        /// When (between steps: where the level crossed the floor).
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
    /// No source reaches part of the network any more: the run stops.
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
    /// Head at every node, at each recorded time.
    pub node_heads: Vec<Vec<Head>>,
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
}

impl Network {
    /// Runs the network for `options.duration` as a chain of steady states,
    /// tanks filling and draining in between.
    ///
    /// # Errors
    /// Invalid options, or a steady state that cannot be solved at the start
    /// (later failures from lost supply end the run with a
    /// [`SupplyEvent::SupplyLost`] instead).
    #[allow(clippy::too_many_lines)] // one time loop with its supply events, kept together
    pub fn extended(&self, options: &PeriodOptions) -> Result<PeriodRun, HydraulicsError> {
        positive("step", options.step.value())?;
        positive("duration", options.duration.value())?;
        positive("pattern_step", options.pattern_step.value())?;
        if options
            .pattern
            .iter()
            .any(|m| !(m.is_finite() && *m >= 0.0))
        {
            return Err(HydraulicsError::invalid(
                "demand pattern",
                f64::NAN,
                "finite multipliers >= 0",
            ));
        }
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
                }
            })
            .collect();
        let dt = options.step.value();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive, bounded
        let steps = (options.duration.value() / dt).ceil() as usize;

        let mut run = PeriodRun {
            times: Vec::with_capacity(steps + 1),
            tanks: tanks
                .iter()
                .map(|t| (NodeId(t.node), Vec::with_capacity(steps + 1)))
                .collect(),
            node_heads: Vec::with_capacity(steps + 1),
            events: Vec::new(),
        };
        let first = self.state_at(Time::ZERO, &tanks, options)?;
        let mut here = first;
        for k in 0..=steps {
            #[allow(clippy::cast_precision_loss)] // k is small
            let t = k as f64 * dt;
            run.times.push(Time::new(t));
            for (i, tank) in tanks.iter().enumerate() {
                run.tanks[i].1.push(Head::new(tank.level));
            }
            run.node_heads.push(here.node_heads.clone());
            if k == steps {
                break;
            }

            // Heun: slopes now, a predicted step, slopes there, average.
            let slope_now = self.net_inflows(&here, &tanks);
            let mut predicted = tanks.clone();
            for (tank, q) in predicted.iter_mut().zip(&slope_now) {
                if tank.open {
                    tank.level += dt * q / tank.area;
                }
            }
            let t_next = Time::new(t + dt);
            let Ok(there) = self.state_at(t_next, &predicted, options) else {
                run.events.push(SupplyEvent::SupplyLost {
                    at: t_next,
                    reason: "no source reaches part of the network".into(),
                });
                break;
            };
            let slope_there = self.net_inflows(&there, &predicted);
            for (i, tank) in tanks.iter_mut().enumerate() {
                if !tank.open {
                    // An empty tank reopens when the network would fill it.
                    let head = there.node_heads[tank.node].value();
                    if head > tank.floor {
                        tank.open = true;
                        run.events.push(SupplyEvent::TankRefilling {
                            tank: NodeId(tank.node),
                            at: t_next,
                        });
                    }
                    continue;
                }
                let rate = f64::midpoint(slope_now[i], slope_there[i]) / tank.area;
                let level = tank.level + dt * rate;
                if level <= tank.floor && rate < 0.0 {
                    let crossing = t + (tank.level - tank.floor) / -rate;
                    tank.level = tank.floor;
                    tank.open = false;
                    run.events.push(SupplyEvent::TankEmpty {
                        tank: NodeId(tank.node),
                        at: Time::new(crossing.min(t + dt)),
                    });
                } else if level >= tank.ceiling && rate > 0.0 {
                    let crossing = t + (tank.ceiling - tank.level) / rate;
                    if tank.level < tank.ceiling {
                        run.events.push(SupplyEvent::TankFull {
                            tank: NodeId(tank.node),
                            at: Time::new(crossing.min(t + dt)),
                        });
                    }
                    tank.level = tank.ceiling;
                } else {
                    tank.level = level;
                }
            }
            let Ok(next) = self.state_at(t_next, &tanks, options) else {
                run.events.push(SupplyEvent::SupplyLost {
                    at: t_next,
                    reason: "no source reaches part of the network".into(),
                });
                break;
            };
            here = next;
        }
        Ok(run)
    }

    /// The steady state at time `t` with the tanks as given: each open tank
    /// a fixed head at its level, each closed one a node with no demand;
    /// level histories at their value at `t`; demands scaled by the pattern.
    fn state_at(
        &self,
        t: Time,
        tanks: &[TankState],
        options: &PeriodOptions,
    ) -> Result<SteadySolution, HydraulicsError> {
        let mut net = self.clone();
        for (n, node) in self.nodes().iter().enumerate() {
            match *node {
                Node::Reservoir { head } => net.set_node(
                    n,
                    Node::Reservoir {
                        head: Head::new(self.source_head(n, head, t)),
                    },
                ),
                Node::Junction { demand } => net.set_node(
                    n,
                    Node::Junction {
                        demand: demand * multiplier(options, t),
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
        net.steady_state(&options.steady)
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

/// The demand multiplier at time `t`.
fn multiplier(options: &PeriodOptions, t: Time) -> f64 {
    if options.pattern.is_empty() {
        return 1.0;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // t >= 0
    let k = (t.value() / options.pattern_step.value()).floor() as usize;
    options.pattern[k % options.pattern.len()]
}
