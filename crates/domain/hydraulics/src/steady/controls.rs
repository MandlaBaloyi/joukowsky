//! Controls: what switches pumps and valves during an extended-period run
//! (`docs/02_physics_of_transients/sources_dams_rivers_canals.txt`, R-2
//! controls note).
//!
//! On a real site almost nothing runs "as the network stands" for a whole
//! day: a pump fills the tank when its level falls to a switch-on level and
//! stops at a switch-off level; a float valve on the tank's inlet closes as
//! it fills; pumps are timed for cheap night tariffs; load-shedding takes
//! the power away. Each such rule is a [`Control`]: a target (a pump or a
//! pipe), what to do (open/run or close/stop) and when (a tank level or a
//! clock time) - EPANET's "simple controls".
//!
//! The run lands exactly on every switch: a level crossing inside a step
//! cuts the step there (as EPANET does), so a pump starts when the tank
//! reaches its switch-on level, not up to a step later. Every switch is a
//! change of flow, and a sudden one is a transient: the switch events are
//! also the list of moments the transient engine should watch.

use jk_kernel::{Head, Time};

use crate::HydraulicsError;
use crate::model::{Network, Node, NodeId, PipeId};

/// Seconds in a day (for daily controls; the run starts at midnight, as the
/// demand pattern does).
pub const DAY: f64 = 86_400.0;

/// What a control switches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    /// A pump node: running or stopped.
    Pump(NodeId),
    /// A pipe: open or closed (a valve in it).
    Pipe(PipeId),
}

/// When a control acts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Condition {
    /// While the tank's level is at or below `level`.
    LevelBelow {
        /// The tank's node.
        tank: NodeId,
        /// The level [m].
        level: Head,
    },
    /// While the tank's level is at or above `level`.
    LevelAbove {
        /// The tank's node.
        tank: NodeId,
        /// The level [m].
        level: Head,
    },
    /// Once, at this time of the run.
    At(Time),
    /// Every day at this time of day (seconds after midnight).
    Daily(Time),
}

/// One rule: when `when` holds, set `target` open/running (`open` true) or
/// closed/stopped (false). Between its conditions a target keeps its state:
/// a pump switched on below 2 m and off above 5 m runs from 2 m up to 5 m
/// and stays off from 5 m down to 2 m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Control {
    /// What it switches.
    pub target: Target,
    /// Open/run (true) or close/stop (false).
    pub open: bool,
    /// When.
    pub when: Condition,
}

impl Control {
    /// The usual pair for filling a tank: open/start `target` when the tank
    /// falls to `low`, close/stop it when it rises to `high`. A pump
    /// filling a tank, or a float (altitude) valve on its inlet.
    #[must_use]
    pub fn fill_between(target: Target, tank: NodeId, low: Head, high: Head) -> [Self; 2] {
        [
            Self {
                target,
                open: true,
                when: Condition::LevelBelow { tank, level: low },
            },
            Self {
                target,
                open: false,
                when: Condition::LevelAbove { tank, level: high },
            },
        ]
    }
}

/// Why a target switched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cause {
    /// Control number `k` in [`crate::steady::extended::PeriodOptions::controls`].
    Control(usize),
    /// A power cut stopped the pump.
    PowerCut,
    /// The power came back and the controls want the pump running.
    PowerBack,
}

/// Time tolerance for landing on switch times [s].
pub(crate) const TIME_TOL: f64 = 1e-6;

impl Condition {
    /// Whether the condition holds at time `t` with tank levels `level_of`.
    pub(crate) fn holds(&self, t: f64, level_of: impl Fn(NodeId) -> f64) -> bool {
        match *self {
            Self::LevelBelow { tank, level } => level_of(tank) <= level.value(),
            Self::LevelAbove { tank, level } => level_of(tank) >= level.value(),
            Self::At(at) => (t - at.value()).abs() < TIME_TOL,
            Self::Daily(at) => {
                let d = (t.rem_euclid(DAY) - at.value()).abs();
                d < TIME_TOL || DAY - d < TIME_TOL
            }
        }
    }

    /// The first time after `t` at which a timed condition holds.
    pub(crate) fn next_time(&self, t: f64) -> Option<f64> {
        match *self {
            Self::At(at) => (at.value() > t + TIME_TOL).then_some(at.value()),
            Self::Daily(at) => {
                let today = t - t.rem_euclid(DAY) + at.value();
                Some(if today > t + TIME_TOL {
                    today
                } else {
                    today + DAY
                })
            }
            Self::LevelBelow { .. } | Self::LevelAbove { .. } => None,
        }
    }
}

/// Checks controls and power cuts against the network.
///
/// # Errors
/// A target that does not exist (a pipe index out of range, a pump node
/// that is not a pump), a level condition on a node that is not a tank, a
/// non-finite level or time, a daily time outside one day, or a power cut
/// that does not end after it starts.
pub(crate) fn check(
    network: &Network,
    controls: &[Control],
    power_cuts: &[(Time, Time)],
) -> Result<(), HydraulicsError> {
    let bad = |k: usize, why: &str| {
        Err(HydraulicsError::InvalidNetwork(format!(
            "control {k}: {why}"
        )))
    };
    for (k, c) in controls.iter().enumerate() {
        match c.target {
            Target::Pipe(p) if p.index() >= network.pipes().len() => {
                return bad(k, "its pipe does not exist");
            }
            Target::Pump(n)
                if !matches!(network.nodes().get(n.index()), Some(Node::Pump { .. })) =>
            {
                return bad(k, "its pump node is not a pump");
            }
            _ => {}
        }
        match c.when {
            Condition::LevelBelow { tank, level } | Condition::LevelAbove { tank, level } => {
                if !network.tanks().iter().any(|t| t.node == tank) {
                    return bad(k, "its level condition is on a node that is not a tank");
                }
                if !level.value().is_finite() {
                    return bad(k, "its level is not finite");
                }
            }
            Condition::At(t) => {
                if !(t.value().is_finite() && t.value() >= 0.0) {
                    return bad(k, "its time is not a finite time >= 0");
                }
            }
            Condition::Daily(t) => {
                if !(0.0..DAY).contains(&t.value()) {
                    return bad(k, "its time of day is not within 0..86400 s");
                }
            }
        }
    }
    for (from, to) in power_cuts {
        if !(from.value().is_finite() && to.value().is_finite() && from.value() >= 0.0 && from < to)
        {
            return Err(HydraulicsError::invalid(
                "power cut",
                from.value(),
                "0 <= start < end, both finite",
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn daily_times_repeat_and_land_exactly() {
        let c = Condition::Daily(Time::new(6.0 * 3600.0));
        assert_eq!(c.next_time(0.0), Some(21_600.0));
        assert_eq!(c.next_time(21_600.0), Some(21_600.0 + DAY));
        assert!(c.holds(21_600.0 + DAY, |_| 0.0));
        assert!(!c.holds(21_601.0, |_| 0.0));
        let midnight = Condition::Daily(Time::ZERO);
        assert!(midnight.holds(DAY - 1e-9, |_| 0.0));
        assert_eq!(midnight.next_time(10.0), Some(DAY));
    }

    #[test]
    fn once_is_once() {
        let c = Condition::At(Time::new(100.0));
        assert_eq!(c.next_time(0.0), Some(100.0));
        assert_eq!(c.next_time(100.0), None);
        assert!(c.holds(100.0, |_| 0.0));
    }
}
