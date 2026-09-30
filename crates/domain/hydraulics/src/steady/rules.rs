//! Rules: EPANET's rule-based controls for the extended-period run
//! (`docs/02_physics_of_transients/sources_dams_rivers_canals.txt`,
//! controls note) - "IF tank 1 is below 2 m AND it is after 22:00 THEN
//! start pump 3 ELSE stop it".
//!
//! A rule is a list of premises joined by AND / OR, THEN actions, optional
//! ELSE actions and a priority. Premises are read left to right as EPANET
//! does: an OR premise is only looked at if what came before is false; an
//! AND premise ends the rule false if what came before is false. So OR
//! binds tighter than AND - "A OR B AND C" is (A OR B) AND C, and "A AND B
//! OR C" is A AND (B OR C) - as the EPANET 2.2 manual states ("the OR
//! operator has higher precedence than AND").
//!
//! When rules ask for different things for one pump or pipe at once, the
//! highest priority wins, and among equals the rule listed first; a rule
//! with no priority (`f64::NEG_INFINITY`) ranks below any that has one.
//!
//! When they are looked at: at every step's end, at every level a rule
//! names (the run cuts its step where a tank reaches it, as for simple
//! controls - EPANET instead looks every "rule time step"), and at every
//! time a rule names. Premises on heads and flows are looked at step by
//! step. Rules act after the simple controls; after they act the state is
//! solved again at the same instant.
//!
//! Rules look at the moment JUST AFTER the instant: a level a hair on in
//! the way it is moving, the clock a hair on. The run stops exactly where a
//! level reaches a rule's value or the clock a rule's time, so a rule
//! switches right there, whichever way the value is crossed. Below and
//! above are strict; equal holds within a hair.

use jk_kernel::{Flow, Head, Time};

use crate::HydraulicsError;
use crate::model::{Network, Node, NodeId};
use crate::steady::controls::{DAY, TIME_TOL, Target};

/// Tolerance on levels and heads in comparisons [m].
const HEAD_TOL: f64 = 1e-6;
/// Tolerance on flows in comparisons [m^3/s].
const FLOW_TOL: f64 = 1e-9;

/// How a value is compared with a rule's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Relation {
    /// Below - EPANET's `<`, `<=`, BELOW (the same, looked at just after
    /// the instant).
    Below,
    /// Above - EPANET's `>`, `>=`, ABOVE.
    Above,
    /// Equal - `=`.
    Equal,
    /// Not equal - `<>`.
    NotEqual,
}

impl Relation {
    fn test(self, value: f64, against: f64, tol: f64) -> bool {
        match self {
            Self::Below => value < against,
            Self::Above => value > against,
            Self::Equal => (value - against).abs() <= tol,
            Self::NotEqual => (value - against).abs() > tol,
        }
    }
}

/// One premise's test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Clause {
    /// A tank's level (as a head: elevation + depth).
    TankLevel {
        /// The tank.
        tank: NodeId,
        /// How compared.
        relation: Relation,
        /// The level [m].
        level: Head,
    },
    /// The head at a node (a pressure is elevation + pressure head).
    Head {
        /// The node.
        node: NodeId,
        /// How compared.
        relation: Relation,
        /// The head [m].
        head: Head,
    },
    /// The flow in a pipe, or through a pump.
    Flow {
        /// The pipe or pump.
        target: Target,
        /// How compared.
        relation: Relation,
        /// The flow [m^3/s].
        flow: Flow,
    },
    /// Whether a pipe is open or a pump running (`is`: IS; false: NOT).
    Status {
        /// The pipe or pump.
        target: Target,
        /// Open / running.
        open: bool,
        /// IS (true) or NOT (false).
        is: bool,
    },
    /// Time since the run started.
    Time {
        /// How compared.
        relation: Relation,
        /// The time.
        at: Time,
    },
    /// Time of day (the run starts at midnight).
    ClockTime {
        /// How compared.
        relation: Relation,
        /// Seconds after midnight.
        at: Time,
    },
}

/// A premise and how it joins the ones before it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Premise {
    /// Joined by OR (true) or AND (false) to what came before; ignored for
    /// the first.
    pub or: bool,
    /// The test.
    pub clause: Clause,
}

/// Open/run (true) or close/stop (false) a pipe or pump.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Action {
    /// What it acts on.
    pub target: Target,
    /// Open/run or close/stop.
    pub open: bool,
}

/// A rule: IF premises THEN actions ELSE actions, with a priority.
#[derive(Debug, Clone, PartialEq)]
pub struct Rule {
    /// The rule's name in the file.
    pub name: String,
    /// IF ... AND / OR ...
    pub premises: Vec<Premise>,
    /// THEN.
    pub then: Vec<Action>,
    /// ELSE (may be empty: nothing is done when the premises are false).
    pub otherwise: Vec<Action>,
    /// Priority: the higher wins a conflict (`f64::NEG_INFINITY` for a rule
    /// given none).
    pub priority: f64,
}

/// What a rule looks at, at one instant.
pub(crate) trait Situation {
    /// Time since the run started [s].
    fn time(&self) -> f64;
    /// A tank's level [m] (NaN if not a tank).
    fn level(&self, tank: NodeId) -> f64;
    /// The head at a node [m].
    fn head(&self, node: NodeId) -> f64;
    /// The flow in a pipe or through a pump [m^3/s].
    fn flow(&self, target: Target) -> f64;
    /// Whether a pipe is open or a pump running.
    fn open(&self, target: Target) -> bool;
}

impl Clause {
    fn holds(&self, s: &impl Situation) -> bool {
        match *self {
            Self::TankLevel {
                tank,
                relation,
                level,
            } => relation.test(s.level(tank), level.value(), HEAD_TOL),
            Self::Head {
                node,
                relation,
                head,
            } => relation.test(s.head(node), head.value(), HEAD_TOL),
            Self::Flow {
                target,
                relation,
                flow,
            } => relation.test(s.flow(target), flow.value(), FLOW_TOL),
            Self::Status { target, open, is } => (s.open(target) == open) == is,
            Self::Time { relation, at } => relation.test(s.time(), at.value(), TIME_TOL),
            Self::ClockTime { relation, at } => {
                relation.test(s.time().rem_euclid(DAY), at.value(), TIME_TOL)
            }
        }
    }

    /// The first time after `t` at which a time premise may change.
    pub(crate) fn next_time(&self, t: f64) -> Option<f64> {
        match *self {
            Self::Time { at, .. } => (at.value() > t + TIME_TOL).then_some(at.value()),
            Self::ClockTime { at, .. } => {
                let today = t - t.rem_euclid(DAY) + at.value();
                Some(if today > t + TIME_TOL {
                    today
                } else {
                    today + DAY
                })
            }
            _ => None,
        }
    }
}

impl Rule {
    /// Whether the premises hold, read left to right as EPANET does.
    pub(crate) fn holds(&self, s: &impl Situation) -> bool {
        let mut result = true;
        for (k, p) in self.premises.iter().enumerate() {
            if k > 0 && p.or {
                if !result {
                    result = p.clause.holds(s);
                }
            } else {
                if !result {
                    return false;
                }
                result = p.clause.holds(s);
            }
        }
        result
    }
}

/// The actions to take now, one per target: of all rules' THEN (premises
/// true) or ELSE (false) actions, the highest priority's, and among equal
/// priorities the first listed rule's. With the index of the rule.
pub(crate) fn actions(rules: &[Rule], s: &impl Situation) -> Vec<(Action, usize)> {
    let mut chosen: Vec<(Action, usize, f64)> = Vec::new();
    for (k, rule) in rules.iter().enumerate() {
        let list = if rule.holds(s) {
            &rule.then
        } else {
            &rule.otherwise
        };
        for a in list {
            match chosen.iter_mut().find(|(c, _, _)| c.target == a.target) {
                Some(slot) if rule.priority > slot.2 => *slot = (*a, k, rule.priority),
                Some(_) => {}
                None => chosen.push((*a, k, rule.priority)),
            }
        }
    }
    chosen.into_iter().map(|(a, k, _)| (a, k)).collect()
}

/// Checks rules against the network.
///
/// # Errors
/// A rule with no premise or no action, a target that does not exist, a
/// level premise on a node that is not a tank, a node that does not exist,
/// or a value that is not finite.
pub(crate) fn check(network: &Network, rules: &[Rule]) -> Result<(), HydraulicsError> {
    let bad = |name: &str, why: &str| {
        Err(HydraulicsError::InvalidNetwork(format!(
            "rule {name}: {why}"
        )))
    };
    let target_ok = |t: Target| match t {
        Target::Pipe(p) => p.index() < network.pipes().len(),
        Target::Pump(n) => matches!(network.nodes().get(n.index()), Some(Node::Pump { .. })),
    };
    for r in rules {
        if r.premises.is_empty() || (r.then.is_empty() && r.otherwise.is_empty()) {
            return bad(&r.name, "needs a premise and an action");
        }
        if r.priority.is_nan() || r.priority == f64::INFINITY {
            return bad(&r.name, "priority is not a number (or none: -infinity)");
        }
        for a in r.then.iter().chain(&r.otherwise) {
            if !target_ok(a.target) {
                return bad(&r.name, "acts on a pipe or pump that does not exist");
            }
        }
        for p in &r.premises {
            let (value, ok) = match p.clause {
                Clause::TankLevel { tank, level, .. } => (
                    level.value(),
                    network.tanks().iter().any(|t| t.node == tank),
                ),
                Clause::Head { node, head, .. } => {
                    (head.value(), node.index() < network.nodes().len())
                }
                Clause::Flow { target, flow, .. } => (flow.value(), target_ok(target)),
                Clause::Status { target, .. } => (0.0, target_ok(target)),
                Clause::Time { at, .. } => (at.value(), at.value() >= 0.0),
                Clause::ClockTime { at, .. } => (at.value(), (0.0..DAY).contains(&at.value())),
            };
            if !(ok && value.is_finite()) {
                return bad(
                    &r.name,
                    "a premise names something that is not there (or not a tank), or a \
                     value out of range",
                );
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A situation of three true/false premises (via the time).
    struct Fixed(f64);
    impl Situation for Fixed {
        fn time(&self) -> f64 {
            self.0
        }
        fn level(&self, _: NodeId) -> f64 {
            0.0
        }
        fn head(&self, _: NodeId) -> f64 {
            0.0
        }
        fn flow(&self, _: Target) -> f64 {
            0.0
        }
        fn open(&self, _: Target) -> bool {
            true
        }
    }

    fn premise(or: bool, truth: bool) -> Premise {
        // TIME above 0 is true at t = 5; TIME above 10 is false.
        Premise {
            or,
            clause: Clause::Time {
                relation: Relation::Above,
                at: Time::new(if truth { 0.0 } else { 10.0 }),
            },
        }
    }

    fn rule(premises: Vec<Premise>) -> Rule {
        Rule {
            name: "r".into(),
            premises,
            then: Vec::new(),
            otherwise: Vec::new(),
            priority: 0.0,
        }
    }

    #[test]
    fn or_binds_tighter_than_and_as_in_epanet() {
        let s = Fixed(5.0);
        // F AND (T OR T) AND T = false (not ((F AND T) OR T) AND T).
        let r = rule(vec![
            premise(false, false),
            premise(false, true),
            premise(true, true),
            premise(false, true),
        ]);
        assert!(!r.holds(&s));
        // T AND (F OR T) = true.
        let r = rule(vec![
            premise(false, true),
            premise(false, false),
            premise(true, true),
        ]);
        assert!(r.holds(&s));
        // (T OR F) AND F = false.
        let r = rule(vec![
            premise(false, true),
            premise(true, false),
            premise(false, false),
        ]);
        assert!(!r.holds(&s));
        // F OR T = true.
        let r = rule(vec![premise(false, false), premise(true, true)]);
        assert!(r.holds(&s));
    }
}
