//! How a file runs its network over time: `[TIMES]`, `[PATTERNS]` and
//! `[CONTROLS]`, turned into options for the engine's extended-period run
//! (`crates/domain/hydraulics/src/steady/extended.rs`, `controls.rs`).
//!
//! - Times: duration, hydraulic step, pattern step. A pattern start is
//!   applied by rotating the patterns; the file's start clock time becomes
//!   t = 0, and clock-time controls are shifted to match.
//! - Patterns: the default pattern (option PATTERN, else pattern "1") for
//!   every junction without one of its own; junctions with their own go in
//!   `node_patterns`. Where the engine has one node for several file
//!   junctions (joined through a valve), or a junction has demand
//!   categories with different patterns, the pattern of the largest demand
//!   is used - with a note.
//! - Controls: `LINK id status IF NODE tank ABOVE|BELOW level`, `AT TIME`,
//!   `AT CLOCKTIME`. A tank level in a file is measured from the tank's
//!   bottom (its elevation); the engine works in heads, so it becomes
//!   elevation + level. Controls on junction pressures, on valves and on
//!   pump speeds are not modelled - each is noted.
//! - Rules: `RULE id / IF ... / AND ... / OR ... / THEN ... / ELSE ... /
//!   PRIORITY p` into the engine's rules (`steady/rules.rs`): premises on
//!   tank levels, node heads and pressures (elevation + pressure), link
//!   flows and statuses, system time and clock time; actions opening or
//!   closing pipes and starting or stopping pumps. A rule with a premise the
//!   engine cannot read (demand, fill or drain time, a setting, a valve) is
//!   left out with a note; an action it cannot take (a setting, a valve) is
//!   dropped with a note.

use std::collections::HashMap;

use jk_hydraulics::steady::Shut;
use jk_hydraulics::steady::controls::{Condition, Control, DAY, Target};
use jk_hydraulics::steady::extended::{NodePattern, PeriodOptions};
use jk_hydraulics::steady::rules::{Action, Clause, Premise, Relation, Rule};
use jk_hydraulics::{Network, NodeId};
use jk_kernel::{Flow, Head, Time};

use super::sections::{Line, Section};
use crate::error::ReadError;

/// How the file runs the network over time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Operations {
    /// Options for [`Network::extended`]: duration, steps, demand patterns
    /// and controls as the file gives them (steady-solve settings and power
    /// cuts left at their defaults).
    pub period: PeriodOptions,
    /// Links out of service at the start (pumps the file starts closed),
    /// for a plain steady solve with [`Network::steady_state_shut`].
    pub shut: Shut,
}

/// `[TIMES]`, in seconds.
#[derive(Debug, Clone, Copy)]
pub(super) struct Times {
    pub duration: f64,
    pub step: f64,
    pub pattern_step: f64,
    pub pattern_start: f64,
    pub start_clock: f64,
}

/// What the operations reader needs from the rest of the build.
pub(super) struct Context<'a> {
    pub network: &'a Network,
    /// Head units per file length unit (m per ft, or 1).
    pub length: f64,
    /// m^3/s per file flow unit.
    pub flow: f64,
    /// Metres of head per file pressure unit.
    pub pressure: f64,
    /// Elevation of each junction and tank, by file node ID (file units).
    pub elevation_of: &'a HashMap<String, f64>,
    /// Engine pipe index of each pipe kept, by file link ID.
    pub pipe_of: &'a HashMap<String, usize>,
    /// Pump node of each modelled pump, by file link ID.
    pub pump_of: &'a HashMap<String, NodeId>,
    /// Why a link cannot be switched (a valve, a joined pump, a pipe left
    /// out), by file link ID.
    pub not_switchable: &'a HashMap<String, String>,
    /// For each file node ID: its engine node, and its elevation if it is a
    /// tank (file units).
    pub node_of: &'a HashMap<String, (NodeId, Option<f64>)>,
    /// Pumps the file starts closed, by file link ID.
    pub closed_pumps: &'a [String],
    /// Engine junction -> (base demand, pattern ID) of each file junction
    /// in it.
    pub demands: &'a HashMap<NodeId, Vec<(f64, Option<String>)>>,
    pub default_pattern: Option<&'a str>,
    /// DEMAND MODEL PDA: demands that fall with pressure.
    pub pressure_demand: Option<jk_hydraulics::PressureDemand>,
}

/// A time as EPANET writes it: `h`, `h.hh`, `h:mm`, `h:mm:ss`, optionally
/// followed by a unit (SEC, MIN, HOURS, DAYS) or AM/PM. Returns seconds and
/// how many fields were used.
///
/// # Errors
/// A field that is not a time.
pub(super) fn time(l: &Line, i: usize, what: &str) -> Result<(f64, usize), ReadError> {
    let s = l
        .fields
        .get(i)
        .ok_or_else(|| ReadError::at(l.number, format!("missing {what}")))?;
    let bad = || ReadError::at(l.number, format!("{what} '{s}' is not a time"));
    let parts: Vec<&str> = s.split(':').collect();
    let mut hours = 0.0;
    for (k, p) in parts.iter().enumerate() {
        let v: f64 = p
            .parse()
            .ok()
            .filter(|v: &f64| v.is_finite() && *v >= 0.0)
            .ok_or_else(bad)?;
        hours += v / 60f64.powi(i32::try_from(k).map_err(|_| bad())?);
    }
    if parts.len() > 3 {
        return Err(bad());
    }
    let unit = l.fields.get(i + 1).map(|u| u.to_ascii_uppercase());
    let (seconds, used) = match unit.as_deref() {
        Some(u) if parts.len() == 1 && u.starts_with("SEC") => (hours, 2),
        Some(u) if parts.len() == 1 && u.starts_with("MIN") => (hours * 60.0, 2),
        Some(u) if parts.len() == 1 && u.starts_with("HOUR") => (hours * 3600.0, 2),
        Some(u) if parts.len() == 1 && u.starts_with("DAY") => (hours * 86_400.0, 2),
        Some("AM") => ((hours % 12.0) * 3600.0, 2),
        Some("PM") => ((hours % 12.0 + 12.0) * 3600.0, 2),
        _ => (hours * 3600.0, 1),
    };
    Ok((seconds, used))
}

/// Reads `[TIMES]`, noting what is not used.
pub(super) fn read_times(
    lines: &[&Line],
    notes: &mut Vec<(Option<usize>, String)>,
) -> Result<Times, ReadError> {
    // EPANET's defaults (manual, [TIMES]).
    let mut t = Times {
        duration: 0.0,
        step: 3600.0,
        pattern_step: 3600.0,
        pattern_start: 0.0,
        start_clock: 0.0,
    };
    for l in lines {
        let key: Vec<String> = l.fields.iter().map(|f| f.to_ascii_uppercase()).collect();
        match (key[0].as_str(), key.get(1).map(String::as_str)) {
            ("DURATION", _) => t.duration = time(l, 1, "duration")?.0,
            ("HYDRAULIC", Some("TIMESTEP")) => t.step = time(l, 2, "hydraulic time step")?.0,
            ("PATTERN", Some("TIMESTEP")) => {
                t.pattern_step = time(l, 2, "pattern time step")?.0;
            }
            ("PATTERN", Some("START")) => t.pattern_start = time(l, 2, "pattern start")?.0,
            ("START", Some("CLOCKTIME")) => t.start_clock = time(l, 2, "start clock time")?.0,
            _ => notes.push((
                Some(l.number),
                format!("[TIMES] {} not used", l.fields.join(" ")),
            )),
        }
    }
    for (what, v) in [
        ("hydraulic time step", t.step),
        ("pattern time step", t.pattern_step),
    ] {
        if v <= 0.0 {
            return Err(ReadError::general(format!(
                "[TIMES] {what} must be above zero"
            )));
        }
    }
    Ok(t)
}

/// Reads `[PATTERNS]`: multipliers by pattern ID, lines appended in order.
pub(super) fn read_patterns(lines: &[&Line]) -> Result<HashMap<String, Vec<f64>>, ReadError> {
    let mut out: HashMap<String, Vec<f64>> = HashMap::new();
    for l in lines {
        let entry = out.entry(l.fields[0].clone()).or_default();
        for (i, f) in l.fields.iter().enumerate().skip(1) {
            let v: f64 = f
                .parse()
                .ok()
                .filter(|v: &f64| v.is_finite() && *v >= 0.0)
                .ok_or_else(|| {
                    ReadError::at(
                        l.number,
                        format!("pattern multiplier {i} '{f}' is not a number >= 0"),
                    )
                })?;
            entry.push(v);
        }
    }
    Ok(out)
}

/// Builds the operations: period options, the shut links at the start and
/// the notes (line, text).
#[allow(clippy::too_many_lines)] // the whole mapping, section by section
pub(super) fn build(
    ctx: &Context,
    times: Times,
    patterns: &HashMap<String, Vec<f64>>,
    controls: &[&Line],
    rules: &[&Line],
    notes: &mut Vec<(Option<usize>, String)>,
) -> Result<Operations, ReadError> {
    let mut period = PeriodOptions {
        step: Time::new(times.step),
        pattern_step: Time::new(times.pattern_step),
        ..PeriodOptions::default()
    };
    if let Some(p) = ctx.pressure_demand {
        period.steady.pressure_demand = Some(p);
        notes.push((
            None,
            format!(
                "DEMAND MODEL PDA: demands fall with pressure - nothing at or below {:.2} m, all \
                 of it from {:.2} m, exponent {} (on each node's ground elevation)",
                p.minimum.value(),
                p.required.value(),
                p.exponent
            ),
        ));
    }
    if times.duration > 0.0 {
        period.duration = Time::new(times.duration);
    } else {
        notes.push((
            None,
            "DURATION 0 (a single steady state in EPANET): extended-period runs default to 24 h"
                .into(),
        ));
    }
    // Pattern start: rotate every pattern so entry 0 is the one in force at
    // t = 0.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // >= 0, small
    let offset = (times.pattern_start / times.pattern_step).round() as usize;
    #[allow(clippy::cast_precision_loss)] // small
    let whole = offset as f64 * times.pattern_step;
    if (times.pattern_start - whole).abs() > 1e-6 {
        notes.push((
            None,
            format!(
                "PATTERN START {:.0} s is not a whole number of pattern steps: rounded to {offset}",
                times.pattern_start
            ),
        ));
    }
    let rotated = |p: &Vec<f64>| {
        let mut p = p.clone();
        if !p.is_empty() {
            let k = offset % p.len();
            p.rotate_left(k);
        }
        p
    };
    if times.start_clock != 0.0 {
        notes.push((
            None,
            format!(
                "the run's t = 0 is the file's start clock time {}; clock-time controls are \
                 shifted to match",
                clock(times.start_clock)
            ),
        ));
    }

    // Demand patterns. EPANET's default: option PATTERN, else pattern "1".
    let default = ctx
        .default_pattern
        .or_else(|| patterns.contains_key("1").then_some("1"));
    if let Some(d) = default {
        period.pattern =
            rotated(patterns.get(d).ok_or_else(|| {
                ReadError::general(format!("default pattern '{d}' is not defined"))
            })?);
    }
    let mut own: HashMap<&str, Vec<NodeId>> = HashMap::new();
    let (mut mixed, mut ids): (usize, Vec<&NodeId>) = (0, ctx.demands.keys().collect());
    ids.sort();
    for node in ids {
        let members = &ctx.demands[node];
        let chosen = members
            .iter()
            .filter(|(d, _)| *d != 0.0)
            .max_by(|a, b| a.0.abs().total_cmp(&b.0.abs()))
            .and_then(|(_, p)| p.as_deref());
        let differs = members
            .iter()
            .filter(|(d, _)| *d != 0.0)
            .any(|(_, p)| p.as_deref() != chosen);
        mixed += usize::from(differs);
        if let Some(p) = chosen
            && Some(p) != default
        {
            if !patterns.contains_key(p) {
                return Err(ReadError::general(format!(
                    "pattern '{p}' is used but not defined"
                )));
            }
            own.entry(p).or_default().push(*node);
        }
    }
    let mut own: Vec<(&str, Vec<NodeId>)> = own.into_iter().collect();
    own.sort_by(|a, b| a.0.cmp(b.0));
    for (p, nodes) in own {
        period.node_patterns.push(NodePattern {
            multipliers: rotated(&patterns[p]),
            nodes,
        });
    }
    if mixed > 0 {
        notes.push((
            None,
            format!(
                "{mixed} engine junction(s) combine demands with different patterns (demand \
                 categories, or junctions joined through a valve): the pattern of the largest \
                 demand is used"
            ),
        ));
    }

    // Pumps the file starts closed: shut at the start (and switched off at
    // t = 0, so a later control can start them).
    let mut shut = Shut::default();
    for id in ctx.closed_pumps {
        if let Some(&node) = ctx.pump_of.get(id) {
            shut.pumps.push(node);
            period.controls.push(Control {
                target: Target::Pump(node),
                open: false,
                when: Condition::At(Time::ZERO),
            });
        }
    }

    // Controls.
    let mut used = 0;
    for l in controls {
        match control(ctx, l, times.start_clock)? {
            Ok(c) => {
                period.controls.push(c);
                used += 1;
            }
            Err(why) => notes.push((Some(l.number), format!("control not modelled: {why}"))),
        }
    }
    if used > 0 {
        notes.push((
            None,
            format!(
                "{used} control(s) switch pumps and pipes in extended-period runs (by tank level \
                 or clock)"
            ),
        ));
    }
    period.rules = read_rules(ctx, rules, times.start_clock, notes)?;
    if !period.rules.is_empty() {
        notes.push((
            None,
            format!(
                "{} rule(s) switch pumps and pipes in extended-period runs (IF / AND / OR / THEN \
                 / ELSE / PRIORITY, read as EPANET does: OR binds tighter than AND)",
                period.rules.len()
            ),
        ));
    }
    Ok(Operations { period, shut })
}

/// Where a rule's lines are: in its premises, its THEN or its ELSE.
#[derive(Clone, Copy, PartialEq)]
enum Part {
    If,
    Then,
    Else,
}

/// Reads `[RULES]`.
///
/// # Errors
/// A line that is not valid EPANET: a clause or action with missing
/// fields, an unknown node or link, a bad number or time.
#[allow(clippy::too_many_lines)] // one grammar, clause by clause
fn read_rules(
    ctx: &Context,
    lines: &[&Line],
    start_clock: f64,
    notes: &mut Vec<(Option<usize>, String)>,
) -> Result<Vec<Rule>, ReadError> {
    // One rule per RULE line: its lines and whether it can be modelled.
    let mut out = Vec::new();
    let mut current: Option<(Rule, Part, Option<String>, usize)> = None;
    let finish = |current: Option<(Rule, Part, Option<String>, usize)>,
                  out: &mut Vec<Rule>,
                  notes: &mut Vec<(Option<usize>, String)>| {
        if let Some((rule, _, why, line)) = current {
            match why {
                Some(why) => notes.push((
                    Some(line),
                    format!("rule {} not modelled: {why}", rule.name),
                )),
                None if rule.then.is_empty() && rule.otherwise.is_empty() => notes.push((
                    Some(line),
                    format!(
                        "rule {} not modelled: none of its actions can be taken",
                        rule.name
                    ),
                )),
                None => out.push(rule),
            }
        }
    };
    for l in lines {
        let up: Vec<String> = l.fields.iter().map(|f| f.to_ascii_uppercase()).collect();
        let bad = |what: &str| ReadError::at(l.number, format!("rule: {what}"));
        match up[0].as_str() {
            "RULE" => {
                finish(current.take(), &mut out, notes);
                let name = l.fields.get(1).ok_or_else(|| bad("RULE needs a name"))?;
                current = Some((
                    Rule {
                        name: name.clone(),
                        premises: Vec::new(),
                        then: Vec::new(),
                        otherwise: Vec::new(),
                        priority: f64::NEG_INFINITY,
                    },
                    Part::If,
                    None,
                    l.number,
                ));
            }
            "PRIORITY" => {
                let (rule, _, _, _) = current
                    .as_mut()
                    .ok_or_else(|| bad("PRIORITY before RULE"))?;
                rule.priority = l
                    .fields
                    .get(1)
                    .and_then(|v| v.parse().ok())
                    .filter(|v: &f64| v.is_finite())
                    .ok_or_else(|| bad("PRIORITY needs a number"))?;
            }
            "IF" | "AND" | "OR" | "THEN" | "ELSE" => {
                let (rule, part, why, _) = current
                    .as_mut()
                    .ok_or_else(|| bad("a clause before RULE"))?;
                match up[0].as_str() {
                    "THEN" => *part = Part::Then,
                    "ELSE" => *part = Part::Else,
                    "IF" => *part = Part::If,
                    _ => {}
                }
                if *part == Part::If {
                    match premise(ctx, l, &up, start_clock)? {
                        Ok(clause) => rule.premises.push(Premise {
                            or: up[0] == "OR",
                            clause,
                        }),
                        Err(reason) => {
                            if why.is_none() {
                                *why = Some(reason);
                            }
                        }
                    }
                } else {
                    match action(ctx, l, &up)? {
                        Ok(a) => {
                            if *part == Part::Then {
                                rule.then.push(a);
                            } else {
                                rule.otherwise.push(a);
                            }
                        }
                        Err(reason) => notes.push((
                            Some(l.number),
                            format!("rule {}: action not taken: {reason}", rule.name),
                        )),
                    }
                }
            }
            other => return Err(bad(&format!("'{other}' is not a rule keyword"))),
        }
    }
    finish(current.take(), &mut out, notes);
    Ok(out)
}

/// A comparison as EPANET writes it.
fn relation(s: &str) -> Option<Relation> {
    Some(match s {
        "=" | "IS" => Relation::Equal,
        "<>" | "NOT" => Relation::NotEqual,
        "<" | "<=" | "BELOW" => Relation::Below,
        ">" | ">=" | "ABOVE" => Relation::Above,
        _ => return None,
    })
}

/// A pipe or pump a rule names, or why it cannot be switched.
fn link(ctx: &Context, l: &Line, id: &str) -> Result<Result<Target, String>, ReadError> {
    if let Some(&p) = ctx.pipe_of.get(id) {
        let pipe = ctx
            .network
            .pipe_id(p)
            .ok_or_else(|| ReadError::at(l.number, "pipe out of range"))?;
        Ok(Ok(Target::Pipe(pipe)))
    } else if let Some(&n) = ctx.pump_of.get(id) {
        Ok(Ok(Target::Pump(n)))
    } else if let Some(why) = ctx.not_switchable.get(id) {
        Ok(Err(format!("link '{id}' {why}")))
    } else {
        Err(ReadError::at(
            l.number,
            format!("rule: unknown link '{id}'"),
        ))
    }
}

/// One premise: the clause, or why the engine cannot read it.
fn premise(
    ctx: &Context,
    l: &Line,
    up: &[String],
    start_clock: f64,
) -> Result<Result<Clause, String>, ReadError> {
    let bad = |what: &str| ReadError::at(l.number, format!("rule: {what}"));
    let field = |i: usize| {
        up.get(i)
            .map(String::as_str)
            .ok_or_else(|| bad("a clause is cut short"))
    };
    let object = field(1)?;
    if object == "SYSTEM" {
        let attribute = field(2)?;
        let rel = relation(field(3)?).ok_or_else(|| bad("unknown comparison"))?;
        return Ok(match attribute {
            "TIME" => Ok(Clause::Time {
                relation: rel,
                at: Time::new(time(l, 4, "rule time")?.0),
            }),
            "CLOCKTIME" => Ok(Clause::ClockTime {
                relation: rel,
                at: Time::new((time(l, 4, "rule clock time")?.0 - start_clock).rem_euclid(DAY)),
            }),
            other => Err(format!("SYSTEM {other} is not read")),
        });
    }
    let id = l
        .fields
        .get(2)
        .ok_or_else(|| bad("a clause is cut short"))?;
    let attribute = field(3)?;
    let rel = relation(field(4)?).ok_or_else(|| bad("unknown comparison"))?;
    let raw = field(5)?;
    let number = || {
        raw.parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| bad(&format!("'{raw}' is not a number")))
    };
    Ok(match (object, attribute) {
        ("LINK" | "PIPE" | "PUMP" | "VALVE", "STATUS") => {
            let open = match raw {
                "OPEN" | "ACTIVE" => true,
                "CLOSED" => false,
                _ => return Err(bad(&format!("status '{raw}'"))),
            };
            match link(ctx, l, id)? {
                Ok(target) => Ok(Clause::Status {
                    target,
                    open,
                    is: rel != Relation::NotEqual,
                }),
                Err(why) => Err(why),
            }
        }
        ("LINK" | "PIPE" | "PUMP" | "VALVE", "FLOW") => match link(ctx, l, id)? {
            Ok(target) => Ok(Clause::Flow {
                target,
                relation: rel,
                flow: Flow::new(number()? * ctx.flow),
            }),
            Err(why) => Err(why),
        },
        ("NODE" | "JUNCTION" | "RESERVOIR" | "TANK", "LEVEL" | "HEAD" | "PRESSURE") => {
            let &(node, tank) = ctx
                .node_of
                .get(id)
                .ok_or_else(|| bad(&format!("unknown node '{id}'")))?;
            match attribute {
                "LEVEL" => match tank {
                    Some(z) if ctx.network.tanks().iter().any(|t| t.node == node) => {
                        Ok(Clause::TankLevel {
                            tank: node,
                            relation: rel,
                            level: Head::new((z + number()?) * ctx.length),
                        })
                    }
                    _ => Err(format!("node '{id}' is not a storage tank")),
                },
                "HEAD" => Ok(Clause::Head {
                    node,
                    relation: rel,
                    head: Head::new(number()? * ctx.length),
                }),
                _ => match ctx.elevation_of.get(id) {
                    Some(z) => Ok(Clause::Head {
                        node,
                        relation: rel,
                        head: Head::new(z * ctx.length + number()? * ctx.pressure),
                    }),
                    None => Err(format!("node '{id}' has no elevation for a pressure")),
                },
            }
        }
        (o, a) => Err(format!("{o} {a} is not read")),
    })
}

/// One action: open/close a pipe or start/stop a pump, or why not.
fn action(ctx: &Context, l: &Line, up: &[String]) -> Result<Result<Action, String>, ReadError> {
    let bad = |what: &str| ReadError::at(l.number, format!("rule: {what}"));
    if up.len() < 6 {
        return Err(bad("an action is cut short"));
    }
    let id = &l.fields[2];
    if up[3] != "STATUS" {
        return Ok(Err(format!(
            "{} {} is not taken (only STATUS)",
            up[1], up[3]
        )));
    }
    let open = match up[5].as_str() {
        "OPEN" | "ACTIVE" => true,
        "CLOSED" => false,
        other => return Err(bad(&format!("status '{other}'"))),
    };
    Ok(link(ctx, l, id)?.map(|target| Action { target, open }))
}

/// One `[CONTROLS]` line: a control, or why the engine cannot model it.
///
/// # Errors
/// A line that is not a valid EPANET control (unknown link or node, a
/// missing field, a bad number or time).
fn control(
    ctx: &Context,
    l: &Line,
    start_clock: f64,
) -> Result<Result<Control, String>, ReadError> {
    let up: Vec<String> = l.fields.iter().map(|f| f.to_ascii_uppercase()).collect();
    let bad = |what: &str| ReadError::at(l.number, format!("control: {what}"));
    if up.first().map(String::as_str) != Some("LINK") || up.len() < 5 {
        return Err(bad("expected LINK id status IF|AT ..."));
    }
    let id = &l.fields[1];
    let target = if let Some(&p) = ctx.pipe_of.get(id) {
        Target::Pipe(
            ctx.network
                .pipe_id(p)
                .ok_or_else(|| bad("pipe out of range"))?,
        )
    } else if let Some(&n) = ctx.pump_of.get(id) {
        Target::Pump(n)
    } else if let Some(why) = ctx.not_switchable.get(id) {
        return Ok(Err(format!("link '{id}' {why}")));
    } else {
        return Err(bad(&format!("unknown link '{id}'")));
    };
    let open = match up[2].as_str() {
        "OPEN" | "ACTIVE" => true,
        "CLOSED" => false,
        s => match s.parse::<f64>() {
            Ok(v) if matches!(target, Target::Pump(_)) && v == 0.0 => false,
            Ok(v) if matches!(target, Target::Pump(_)) && (v - 1.0).abs() < 1e-12 => true,
            Ok(v) => {
                return Ok(Err(format!(
                    "link '{id}' set to {v}: settings (speeds, valve settings) are not modelled"
                )));
            }
            Err(_) => {
                return Err(bad(&format!(
                    "status '{}' is not OPEN, CLOSED or a number",
                    l.fields[2]
                )));
            }
        },
    };
    let when = match (up[3].as_str(), up[4].as_str()) {
        ("IF", "NODE") => {
            let node = l.fields.get(5).ok_or_else(|| bad("missing node ID"))?;
            let &(tank, elevation) = ctx
                .node_of
                .get(node)
                .ok_or_else(|| bad(&format!("unknown node '{node}'")))?;
            let value: f64 = l
                .fields
                .get(7)
                .and_then(|v| v.parse().ok())
                .filter(|v: &f64| v.is_finite())
                .ok_or_else(|| bad("missing or bad level"))?;
            let Some(elevation) =
                elevation.filter(|_| ctx.network.tanks().iter().any(|t| t.node == tank))
            else {
                return Ok(Err(format!(
                    "condition on node '{node}', which is not a storage tank (junction pressures \
                     and reservoir heads are not modelled as conditions)"
                )));
            };
            let level = Head::new((elevation + value) * ctx.length);
            match up.get(6).map(String::as_str) {
                Some("ABOVE") => Condition::LevelAbove { tank, level },
                Some("BELOW") => Condition::LevelBelow { tank, level },
                _ => return Err(bad("expected ABOVE or BELOW")),
            }
        }
        ("AT", "TIME") => Condition::At(Time::new(time(l, 5, "control time")?.0)),
        ("AT", "CLOCKTIME") => {
            let (t, _) = time(l, 5, "control clock time")?;
            Condition::Daily(Time::new((t - start_clock).rem_euclid(DAY)))
        }
        _ => return Err(bad("expected IF NODE, AT TIME or AT CLOCKTIME")),
    };
    Ok(Ok(Control { target, open, when }))
}

/// Seconds after midnight as hh:mm.
fn clock(seconds: f64) -> String {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // one day at most
    let m = (seconds / 60.0).round() as u64;
    format!("{:02}:{:02}", m / 60 % 24, m % 60)
}

/// The lines of a section.
pub(super) fn lines(sections: &[(Section, Vec<Line>)], which: Section) -> Vec<&Line> {
    sections
        .iter()
        .filter(|(s, _)| *s == which)
        .flat_map(|(_, l)| l.iter())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(text: &str) -> Line {
        Line {
            number: 1,
            fields: text.split_whitespace().map(String::from).collect(),
            text: text.into(),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)] // exact by construction
    fn times_in_every_epanet_form() {
        let t = |s: &str| time(&line(s), 0, "t").unwrap().0;
        assert_eq!(t("24:00"), 86_400.0);
        assert_eq!(t("1:30"), 5400.0);
        assert_eq!(t("0:00:30"), 30.0);
        assert_eq!(t("1.5"), 5400.0);
        assert_eq!(t("30 MIN"), 1800.0);
        assert_eq!(t("2 days"), 172_800.0);
        assert_eq!(t("6 AM"), 21_600.0);
        assert_eq!(t("12 AM"), 0.0);
        assert_eq!(t("12 PM"), 43_200.0);
        assert_eq!(t("6:30 PM"), 66_600.0);
        assert!(time(&line("noon"), 0, "t").is_err());
    }
}
