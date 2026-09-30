//! From sections to the engine's network.

use std::collections::HashMap;

use jk_hydraulics::{Network, Node, NodeId, Pipe, TankStorage};
use jk_kernel::{Area, Flow, Head, Length};

use super::friction::{Headloss, darcy_factor};
use super::operations::{self, Context, Operations};
use super::pumps::{self, Curves, Fitted};
use super::sections::{Line, Section, split};
use super::units::FlowUnits;
use super::{InpNetwork, InpOptions, Note};

/// Longest station pipework put either side of a pump [m] (never longer
/// than the file's shortest pipe, so the transient time step is unchanged).
const CONNECTOR_LENGTH: f64 = 10.0;
/// Darcy friction factor of that pipework (its loss is a few cm).
const CONNECTOR_FRICTION: f64 = 0.01;
use crate::error::ReadError;

/// What `[OPTIONS]` sets.
struct Settings {
    units: FlowUnits,
    headloss: Headloss,
    demand_multiplier: f64,
    viscosity: f64,
    /// Option PATTERN: the default demand pattern.
    default_pattern: Option<String>,
}

/// A node as the file gives it (file units).
enum NodeKind {
    Junction {
        elevation: f64,
        demand: f64,
        /// Demand pattern ID, if the junction names one.
        pattern: Option<String>,
    },
    Reservoir {
        head: f64,
    },
    Tank {
        elevation: f64,
        level: f64,
        /// Minimum and maximum level and diameter, when the file gives them.
        storage: Option<(f64, f64, f64)>,
    },
}

struct RawNode {
    id: String,
    kind: NodeKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    Open,
    Closed,
    CheckValve,
}

enum LinkKind {
    Pipe {
        length: f64,
        diameter: f64,
        roughness: f64,
        minor_loss: f64,
    },
    Pump {
        /// Head curve ID (keyword HEAD).
        curve: Option<String>,
        /// Relative speed (keyword SPEED; 1 if absent).
        speed: f64,
        /// Given as constant power (keyword POWER) instead of a curve.
        power: bool,
        /// Has a speed pattern (keyword PATTERN).
        pattern: bool,
    },
    Valve {
        kind: String,
    },
}

struct RawLink {
    id: String,
    from: usize,
    to: usize,
    kind: LinkKind,
    status: Status,
    line: usize,
}

type Sections = Vec<(Section, Vec<Line>)>;

fn records(sections: &Sections, which: Section) -> impl Iterator<Item = &Line> {
    sections
        .iter()
        .filter(move |(s, _)| *s == which)
        .flat_map(|(_, lines)| lines.iter())
}

fn field<'a>(l: &'a Line, i: usize, what: &str) -> Result<&'a str, ReadError> {
    l.fields
        .get(i)
        .map(String::as_str)
        .ok_or_else(|| ReadError::at(l.number, format!("missing {what}")))
}

fn number(l: &Line, i: usize, what: &str) -> Result<f64, ReadError> {
    let s = field(l, i, what)?;
    s.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| ReadError::at(l.number, format!("{what} '{s}' is not a number")))
}

fn optional(l: &Line, i: usize, what: &str) -> Result<Option<f64>, ReadError> {
    if l.fields.len() > i {
        number(l, i, what).map(Some)
    } else {
        Ok(None)
    }
}

fn note(notes: &mut Vec<Note>, line: Option<usize>, text: impl Into<String>) {
    notes.push(Note {
        line,
        text: text.into(),
    });
}

/// A short list of IDs for a note: the first few, then how many more.
fn some_ids(ids: &[String]) -> String {
    let shown: Vec<&str> = ids.iter().take(5).map(String::as_str).collect();
    if ids.len() > 5 {
        format!("{} and {} more", shown.join(", "), ids.len() - 5)
    } else {
        shown.join(", ")
    }
}

#[allow(clippy::too_many_lines)] // the reading order, told once
pub(super) fn build(text: &str, options: &InpOptions) -> Result<InpNetwork, ReadError> {
    let sections = split(text)?;
    let mut notes = Vec::new();
    let settings = read_options(&sections, &mut notes)?;
    let title = records(&sections, Section::Title)
        .map(|l| l.text.clone())
        .collect::<Vec<_>>()
        .join(" ");
    let (mut nodes, index) = read_nodes(&sections, &mut notes)?;
    let categories = read_demands(&sections, &index, &mut nodes, &mut notes)?;
    let mut links = read_links(&sections, &index)?;
    read_status(&sections, &mut links, &mut notes)?;
    let curves = read_curves(&sections)?;
    let fitted = fit_pumps(&links, &curves, &settings, &mut notes);
    let unused_curves: Vec<String> = {
        let mut ids: Vec<String> = curves
            .keys()
            .filter(|c| {
                !links.iter().any(|k| {
                    fitted.contains_key(&k.id)
                        && matches!(&k.kind, LinkKind::Pump { curve: Some(x), .. } if x == *c)
                })
            })
            .cloned()
            .collect();
        ids.sort();
        ids
    };
    if !unused_curves.is_empty() {
        note(
            &mut notes,
            None,
            format!(
                "{} curve(s) not used (efficiency, volume, valve or unmodelled pump curves): {}",
                unused_curves.len(),
                some_ids(&unused_curves)
            ),
        );
    }
    unused_sections(&sections, &mut notes);
    let coordinates = read_coordinates(&sections, &index)?;
    let (mut inp, map) = assemble(
        &sections,
        options,
        &settings,
        title,
        &nodes,
        &links,
        &coordinates,
        &fitted,
        notes,
    )?;

    // How the file runs the network over time.
    let mut op_notes = Vec::new();
    let times =
        operations::read_times(&operations::lines(&sections, Section::Times), &mut op_notes)?;
    let patterns = operations::read_patterns(&operations::lines(&sections, Section::Patterns))?;
    let mut demands: HashMap<NodeId, Vec<(f64, Option<String>)>> = HashMap::new();
    for (i, n) in nodes.iter().enumerate() {
        if let NodeKind::Junction {
            demand, pattern, ..
        } = &n.kind
        {
            let entry = demands.entry(map.node_of_raw[i]).or_default();
            match categories.get(&i) {
                Some(c) => entry.extend(c.iter().cloned()),
                None => entry.push((*demand, pattern.clone())),
            }
        }
    }
    let node_of: HashMap<String, (NodeId, Option<f64>)> = nodes
        .iter()
        .enumerate()
        .map(|(i, n)| {
            let tank = match n.kind {
                NodeKind::Tank { elevation, .. } => Some(elevation),
                _ => None,
            };
            (n.id.clone(), (map.node_of_raw[i], tank))
        })
        .collect();
    let closed_pumps: Vec<String> = links
        .iter()
        .filter(|k| matches!(k.kind, LinkKind::Pump { .. }) && k.status == Status::Closed)
        .map(|k| k.id.clone())
        .collect();
    let operations: Operations = {
        let ctx = Context {
            network: &inp.network,
            length: settings.units.length(),
            pipe_of: &map.pipe_of,
            pump_of: &map.pump_of,
            not_switchable: &map.not_switchable,
            node_of: &node_of,
            closed_pumps: &closed_pumps,
            demands: &demands,
            default_pattern: settings.default_pattern.as_deref(),
        };
        operations::build(
            &ctx,
            times,
            &patterns,
            &operations::lines(&sections, Section::Controls),
            &mut op_notes,
        )?
    };
    for (line, text) in op_notes {
        note(&mut inp.notes, line, text);
    }
    inp.operations = operations;
    Ok(inp)
}

/// Where file IDs ended up in the engine's network.
struct Mapping {
    /// Engine node of each file node.
    node_of_raw: Vec<NodeId>,
    /// Engine pipe index of each pipe kept, by file ID.
    pipe_of: HashMap<String, usize>,
    /// Pump node of each modelled pump, by file ID.
    pump_of: HashMap<String, NodeId>,
    /// Links a control cannot switch, and why, by file ID.
    not_switchable: HashMap<String, String>,
}

fn read_options(sections: &Sections, notes: &mut Vec<Note>) -> Result<Settings, ReadError> {
    let mut s = Settings {
        units: FlowUnits::Gpm,
        headloss: Headloss::HazenWilliams,
        demand_multiplier: 1.0,
        viscosity: 1.0,
        default_pattern: None,
    };
    let mut units_given = false;
    let mut unused = Vec::new();
    for l in records(sections, Section::Options) {
        let key = l.fields[0].to_ascii_uppercase();
        let second = l.fields.get(1).map(|f| f.to_ascii_uppercase());
        match (key.as_str(), second.as_deref()) {
            ("UNITS", _) => {
                let v = field(l, 1, "flow units")?;
                s.units = FlowUnits::parse(v)
                    .ok_or_else(|| ReadError::at(l.number, format!("unknown flow units '{v}'")))?;
                units_given = true;
            }
            ("HEADLOSS", _) => {
                let v = field(l, 1, "head-loss formula")?;
                s.headloss = Headloss::parse(v).ok_or_else(|| {
                    ReadError::at(l.number, format!("unknown head-loss formula '{v}'"))
                })?;
            }
            ("DEMAND", Some("MULTIPLIER")) => {
                s.demand_multiplier = number(l, 2, "demand multiplier")?;
            }
            ("VISCOSITY", _) => s.viscosity = number(l, 1, "viscosity")?,
            ("PATTERN", _) => s.default_pattern = Some(field(l, 1, "pattern ID")?.to_string()),
            ("SPECIFIC", Some("GRAVITY")) => {
                let sg = number(l, 2, "specific gravity")?;
                if (sg - 1.0).abs() > 1e-9 {
                    note(
                        notes,
                        Some(l.number),
                        format!("specific gravity {sg} not used: the engine models water"),
                    );
                }
            }
            _ => unused.push(l.fields.join(" ")),
        }
    }
    if !units_given {
        note(
            notes,
            None,
            "no UNITS option: EPANET's default, GPM (US customary units: feet, inches), assumed",
        );
    }
    if !unused.is_empty() {
        note(
            notes,
            None,
            format!("[OPTIONS] not used by the engine: {}", some_ids(&unused)),
        );
    }
    Ok(s)
}

fn add_node(
    nodes: &mut Vec<RawNode>,
    index: &mut HashMap<String, usize>,
    l: &Line,
    kind: NodeKind,
) -> Result<(), ReadError> {
    let id = field(l, 0, "node ID")?.to_string();
    if index.contains_key(&id) {
        return Err(ReadError::at(
            l.number,
            format!("node ID '{id}' is used twice"),
        ));
    }
    index.insert(id.clone(), nodes.len());
    nodes.push(RawNode { id, kind });
    Ok(())
}

fn read_nodes(
    sections: &Sections,
    notes: &mut Vec<Note>,
) -> Result<(Vec<RawNode>, HashMap<String, usize>), ReadError> {
    let mut nodes = Vec::new();
    let mut index = HashMap::new();
    let (mut patterned, mut curved) = (0, 0);
    for (section, lines) in sections {
        for l in lines {
            match section {
                Section::Junctions => {
                    let kind = NodeKind::Junction {
                        elevation: number(l, 1, "elevation")?,
                        demand: optional(l, 2, "base demand")?.unwrap_or(0.0),
                        pattern: l.fields.get(3).cloned(),
                    };
                    add_node(&mut nodes, &mut index, l, kind)?;
                }
                Section::Reservoirs => {
                    let kind = NodeKind::Reservoir {
                        head: number(l, 1, "head")?,
                    };
                    patterned += usize::from(l.fields.len() > 2);
                    add_node(&mut nodes, &mut index, l, kind)?;
                }
                Section::Tanks => {
                    let min = optional(l, 3, "minimum level")?;
                    let max = optional(l, 4, "maximum level")?;
                    let diameter = optional(l, 5, "diameter")?;
                    curved += usize::from(l.fields.len() > 7);
                    let kind = NodeKind::Tank {
                        elevation: number(l, 1, "elevation")?,
                        level: number(l, 2, "initial level")?,
                        storage: match (min, max, diameter) {
                            (Some(a), Some(b), Some(d)) if a < b && d > 0.0 => Some((a, b, d)),
                            _ => None,
                        },
                    };
                    add_node(&mut nodes, &mut index, l, kind)?;
                }
                _ => {}
            }
        }
    }
    if curved > 0 {
        note(
            notes,
            None,
            format!(
                "{curved} tank volume curve(s) not read: tanks are cylinders of the given diameter"
            ),
        );
    }
    if patterned > 0 {
        note(
            notes,
            None,
            format!(
                "{patterned} reservoir head pattern(s) not used: reservoirs hold their base head"
            ),
        );
    }
    Ok((nodes, index))
}

/// Demand categories of the junctions listed in `[DEMANDS]`: (demand,
/// pattern ID) per category, by file node.
type Categories = HashMap<usize, Vec<(f64, Option<String>)>>;

/// `[DEMANDS]`: a junction listed there takes the sum of its entries,
/// replacing its `[JUNCTIONS]` base demand (EPANET's rule [VERIFY]).
fn read_demands(
    sections: &Sections,
    index: &HashMap<String, usize>,
    nodes: &mut [RawNode],
    notes: &mut Vec<Note>,
) -> Result<Categories, ReadError> {
    let mut sums: HashMap<usize, f64> = HashMap::new();
    let mut categories = Categories::new();
    for l in records(sections, Section::Demands) {
        let id = field(l, 0, "junction ID")?;
        let &n = index
            .get(id)
            .ok_or_else(|| ReadError::at(l.number, format!("demand for unknown node '{id}'")))?;
        let demand = number(l, 1, "demand")?;
        *sums.entry(n).or_insert(0.0) += demand;
        categories
            .entry(n)
            .or_default()
            .push((demand, l.fields.get(2).cloned()));
    }
    if !sums.is_empty() {
        note(
            notes,
            None,
            format!(
                "[DEMANDS] gives {} junction(s) several demand categories: their sum is used",
                sums.len()
            ),
        );
    }
    for (n, total) in sums {
        if let NodeKind::Junction { demand, .. } = &mut nodes[n].kind {
            *demand = total;
        }
    }
    Ok(categories)
}

fn read_links(
    sections: &Sections,
    index: &HashMap<String, usize>,
) -> Result<Vec<RawLink>, ReadError> {
    let mut links: Vec<RawLink> = Vec::new();
    let mut seen: HashMap<String, usize> = HashMap::new();
    for (section, lines) in sections {
        for l in lines {
            let kind = match section {
                Section::Pipes => LinkKind::Pipe {
                    length: number(l, 3, "length")?,
                    diameter: number(l, 4, "diameter")?,
                    roughness: number(l, 5, "roughness")?,
                    minor_loss: optional(l, 6, "minor loss")?.unwrap_or(0.0),
                },
                Section::Pumps => pump_kind(l)?,
                Section::Valves => LinkKind::Valve {
                    kind: field(l, 4, "valve type")?.to_ascii_uppercase(),
                },
                _ => continue,
            };
            let id = field(l, 0, "link ID")?.to_string();
            if seen.insert(id.clone(), l.number).is_some() {
                return Err(ReadError::at(
                    l.number,
                    format!("link ID '{id}' is used twice"),
                ));
            }
            let end = |i: usize| -> Result<usize, ReadError> {
                let n = field(l, i, "node ID")?;
                index.get(n).copied().ok_or_else(|| {
                    ReadError::at(l.number, format!("link '{id}' joins unknown node '{n}'"))
                })
            };
            let (from, to) = (end(1)?, end(2)?);
            if from == to {
                return Err(ReadError::at(
                    l.number,
                    format!("link '{id}' starts and ends at the same node"),
                ));
            }
            let status = match (&kind, l.fields.get(7).map(|s| s.to_ascii_uppercase())) {
                (LinkKind::Pipe { .. }, Some(s)) if s == "CLOSED" => Status::Closed,
                (LinkKind::Pipe { .. }, Some(s)) if s == "CV" => Status::CheckValve,
                _ => Status::Open,
            };
            links.push(RawLink {
                id,
                from,
                to,
                kind,
                status,
                line: l.number,
            });
        }
    }
    Ok(links)
}

/// A `[PUMPS]` record's properties: keyword-value pairs after the nodes.
fn pump_kind(l: &Line) -> Result<LinkKind, ReadError> {
    let (mut curve, mut speed, mut power, mut pattern) = (None, 1.0, false, false);
    let mut i = 3;
    while i < l.fields.len() {
        let key = l.fields[i].to_ascii_uppercase();
        let value = field(l, i + 1, &format!("value of pump {key}"))?;
        match key.as_str() {
            "HEAD" => curve = Some(value.to_string()),
            "SPEED" => speed = number(l, i + 1, "pump speed")?,
            "POWER" => power = true,
            "PATTERN" => pattern = true,
            _ => {
                return Err(ReadError::at(
                    l.number,
                    format!("unknown pump keyword '{}'", l.fields[i]),
                ));
            }
        }
        i += 2;
    }
    Ok(LinkKind::Pump {
        curve,
        speed,
        power,
        pattern,
    })
}

/// `[CURVES]`: points by curve ID, in file order and file units.
fn read_curves(sections: &Sections) -> Result<Curves, ReadError> {
    let mut out = Curves::new();
    for l in records(sections, Section::Curves) {
        let id = field(l, 0, "curve ID")?.to_string();
        let point = (number(l, 1, "curve X")?, number(l, 2, "curve Y")?);
        out.entry(id).or_default().push(point);
    }
    Ok(out)
}

/// Fits every pump with a usable head curve; notes the rest (they become
/// plain connections).
fn fit_pumps(
    links: &[RawLink],
    curves: &Curves,
    s: &Settings,
    notes: &mut Vec<Note>,
) -> HashMap<String, Fitted> {
    let (lu, fu) = (s.units.length(), s.units.to_m3s());
    let mut out = HashMap::new();
    for k in links {
        let LinkKind::Pump {
            curve,
            speed,
            power,
            pattern,
        } = &k.kind
        else {
            continue;
        };
        let why_not = if *power {
            Err("is given as constant power, not a head curve".to_string())
        } else if let Some(c) = curve {
            match curves.get(c) {
                None => Err(format!("names curve '{c}', which is not in [CURVES]")),
                Some(points) => {
                    let si: Vec<(f64, f64)> =
                        points.iter().map(|(q, h)| (q * fu, h * lu)).collect();
                    pumps::fit(&si, *speed).map_err(|e| format!("cannot be modelled: {e}"))
                }
            }
        } else {
            Err("has no head curve".to_string())
        };
        match why_not {
            Ok(fitted) => {
                note(
                    notes,
                    Some(k.line),
                    format!(
                        "pump '{}': {}; efficiency {:.0}%, {:.0} rpm and inertia (Thorley) \
                         assumed - they matter only for pump trips [VERIFY]",
                        k.id,
                        fitted.how,
                        pumps::ASSUMED_EFFICIENCY * 100.0,
                        pumps::ASSUMED_SPEED_RPM
                    ),
                );
                if (*speed - 1.0).abs() > 1e-12 {
                    note(
                        notes,
                        Some(k.line),
                        format!(
                            "pump '{}' runs at relative speed {speed}: curve scaled by the \
                             affinity laws",
                            k.id
                        ),
                    );
                }
                if *pattern {
                    note(
                        notes,
                        Some(k.line),
                        format!("pump '{}': speed pattern not used (constant speed)", k.id),
                    );
                }
                out.insert(k.id.clone(), fitted);
            }
            Err(why) => note(
                notes,
                Some(k.line),
                format!("pump '{}' {why}: modelled as a plain connection", k.id),
            ),
        }
    }
    out
}

fn read_status(
    sections: &Sections,
    links: &mut [RawLink],
    notes: &mut Vec<Note>,
) -> Result<(), ReadError> {
    for l in records(sections, Section::Status) {
        let id = field(l, 0, "link ID")?;
        let value = field(l, 1, "status")?.to_ascii_uppercase();
        let link = links
            .iter_mut()
            .find(|k| k.id == id)
            .ok_or_else(|| ReadError::at(l.number, format!("status for unknown link '{id}'")))?;
        match value.as_str() {
            "CLOSED" => link.status = Status::Closed,
            "OPEN" | "ACTIVE" => {
                if link.status == Status::Closed {
                    link.status = Status::Open;
                }
            }
            _ => note(
                notes,
                Some(l.number),
                format!("setting '{value}' for link '{id}' not used"),
            ),
        }
    }
    Ok(())
}

fn unused_sections(sections: &Sections, notes: &mut Vec<Note>) {
    let why = |s: Section| match s {
        Section::Rules => "rule-based controls (simple [CONTROLS] are modelled; rules not yet)",
        Section::Energy => "energy prices and pump efficiency",
        Section::Emitters | Section::Leakage => "emitters and leakage",
        Section::Quality | Section::Sources | Section::Reactions | Section::Mixing => {
            "water quality"
        }
        Section::Report => "reporting",
        Section::Labels | Section::Backdrop => "map labels and backdrop",
        Section::Tags => "tags",
        _ => "",
    };
    let mut counts: Vec<(Section, usize)> = Vec::new();
    for (s, lines) in sections {
        if why(*s).is_empty() || lines.is_empty() {
            continue;
        }
        match counts.iter_mut().find(|(k, _)| k == s) {
            Some((_, c)) => *c += lines.len(),
            None => counts.push((*s, lines.len())),
        }
    }
    for (s, c) in counts {
        note(
            notes,
            None,
            format!("[{}]: {c} record(s) not used - {}", s.name(), why(s)),
        );
    }
}

fn read_coordinates(
    sections: &Sections,
    index: &HashMap<String, usize>,
) -> Result<HashMap<usize, (f64, f64)>, ReadError> {
    let mut out = HashMap::new();
    for l in records(sections, Section::Coordinates) {
        let id = field(l, 0, "node ID")?;
        let &n = index.get(id).ok_or_else(|| {
            ReadError::at(l.number, format!("coordinates for unknown node '{id}'"))
        })?;
        out.insert(n, (number(l, 1, "X")?, number(l, 2, "Y")?));
    }
    Ok(out)
}

/// Joins nodes connected by open valves and by pumps not modelled
/// (union-find).
fn join(
    nodes: &[RawNode],
    links: &[RawLink],
    fitted: &HashMap<String, Fitted>,
    notes: &mut Vec<Note>,
) -> Vec<usize> {
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..nodes.len()).collect();
    for k in links {
        let what = match &k.kind {
            LinkKind::Pipe { .. } => continue,
            LinkKind::Pump { .. } if fitted.contains_key(&k.id) => continue,
            LinkKind::Pump { .. } => "pump".to_string(),
            LinkKind::Valve { kind } => format!("{kind} valve"),
        };
        let (a, b) = (&nodes[k.from].id, &nodes[k.to].id);
        if k.status == Status::Closed {
            note(
                notes,
                Some(k.line),
                format!(
                    "{what} '{}' is closed: {a} and {b} are not connected through it",
                    k.id
                ),
            );
            continue;
        }
        let (ra, rb) = (root(&mut parent, k.from), root(&mut parent, k.to));
        if ra != rb {
            parent[rb] = ra;
        }
        note(
            notes,
            Some(k.line),
            format!(
                "{what} '{}' between {a} and {b}: modelled as a plain connection (the two nodes are joined); its own hydraulics are not",
                k.id
            ),
        );
    }
    (0..nodes.len()).map(|i| root(&mut parent, i)).collect()
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)] // the assembly, told once
fn assemble(
    sections: &Sections,
    options: &InpOptions,
    s: &Settings,
    title: String,
    nodes: &[RawNode],
    links: &[RawLink],
    coordinates: &HashMap<usize, (f64, f64)>,
    fitted: &HashMap<String, Fitted>,
    mut notes: Vec<Note>,
) -> Result<(InpNetwork, Mapping), ReadError> {
    let roots = join(nodes, links, fitted, &mut notes);
    let mut not_switchable: HashMap<String, String> = HashMap::new();
    for k in links {
        match &k.kind {
            LinkKind::Valve { kind } => {
                not_switchable.insert(
                    k.id.clone(),
                    format!("is a {kind} valve (valves are modelled as connections)"),
                );
            }
            LinkKind::Pump { .. } if !fitted.contains_key(&k.id) => {
                not_switchable.insert(
                    k.id.clone(),
                    "is a pump without a usable head curve (modelled as a connection)".into(),
                );
            }
            _ => {}
        }
    }
    let (lu, fu) = (s.units.length(), s.units.to_m3s());

    // One engine node per group of joined file nodes, in file order.
    let mut group_of_root: HashMap<usize, usize> = HashMap::new();
    let mut members: Vec<Vec<usize>> = Vec::new();
    for (i, r) in roots.iter().enumerate() {
        let g = *group_of_root.entry(*r).or_insert_with(|| {
            members.push(Vec::new());
            members.len() - 1
        });
        members[g].push(i);
    }
    let mut network = Network::new(options.gravity);
    let (mut node_ids, mut elevations, mut positions, mut ids) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::<NodeId>::new());
    let (mut tanks, mut stored) = (0, 0);
    for group in &members {
        let fixed: Vec<f64> = group
            .iter()
            .filter_map(|&i| match nodes[i].kind {
                NodeKind::Reservoir { head } => Some(head * lu),
                NodeKind::Tank {
                    elevation, level, ..
                } => Some((elevation + level) * lu),
                NodeKind::Junction { .. } => None,
            })
            .collect();
        tanks += group
            .iter()
            .filter(|&&i| matches!(nodes[i].kind, NodeKind::Tank { .. }))
            .count();
        let node = if let Some(&head) = fixed.first() {
            if fixed.iter().any(|h| (h - head).abs() > 1e-6) {
                note(
                    &mut notes,
                    None,
                    format!(
                        "fixed heads joined through a pump or valve at {}: the first ({head:.2} m) is used",
                        nodes[group[0]].id
                    ),
                );
            }
            Node::Reservoir {
                head: Head::new(head),
            }
        } else {
            let demand: f64 = group
                .iter()
                .map(|&i| match nodes[i].kind {
                    NodeKind::Junction { demand, .. } => demand,
                    _ => 0.0,
                })
                .sum();
            Node::Junction {
                demand: Flow::new(demand * s.demand_multiplier * fu),
            }
        };
        let id = network.add_node(node);
        ids.push(id);
        // A tank alone in its group, with its levels and diameter given,
        // keeps its storage: extended-period runs move its level.
        if let [only] = group.as_slice()
            && let NodeKind::Tank {
                elevation,
                storage: Some((min, max, diameter)),
                ..
            } = nodes[*only].kind
        {
            let area = std::f64::consts::PI * 0.25 * (diameter * lu).powi(2);
            let added = network.add_tank(TankStorage {
                node: id,
                area: Area::new(area),
                floor: Head::new((elevation + min) * lu),
                ceiling: Head::new((elevation + max) * lu),
            });
            if added.is_ok() {
                stored += 1;
            }
        }
        node_ids.push(
            group
                .iter()
                .map(|&i| nodes[i].id.as_str())
                .collect::<Vec<_>>()
                .join("="),
        );
        elevations.push(group.iter().find_map(|&i| match nodes[i].kind {
            NodeKind::Junction { elevation, .. } | NodeKind::Tank { elevation, .. } => {
                Some(Length::new(elevation * lu))
            }
            NodeKind::Reservoir { .. } => None,
        }));
        positions.push(group.iter().find_map(|&i| coordinates.get(&i).copied()));
    }
    if stored > 0 {
        note(
            &mut notes,
            None,
            format!(
                "{stored} tank(s) with storage: fixed heads in transients, their level simulated in \
                 extended-period runs (cylinders of the given diameter)"
            ),
        );
    }
    if tanks > stored {
        note(
            &mut notes,
            None,
            format!(
                "{} tank(s) modelled as fixed heads at their initial level (levels or diameter \
                 missing, or joined to another fixed head)",
                tanks - stored
            ),
        );
    }
    let group = |raw: usize| group_of_root[&roots[raw]];

    // Pipes.
    let (mut pipe_ids, mut closed, mut checks, mut minor, mut self_loops) =
        (Vec::new(), Vec::new(), Vec::new(), 0, Vec::new());
    let (mut f_lo, mut f_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for k in links {
        let LinkKind::Pipe {
            length,
            diameter,
            roughness,
            minor_loss,
        } = k.kind
        else {
            continue;
        };
        match k.status {
            Status::Closed => {
                closed.push(k.id.clone());
                not_switchable.insert(
                    k.id.clone(),
                    "is closed in the file and left out of the network".into(),
                );
                continue;
            }
            Status::CheckValve => checks.push(k.id.clone()),
            Status::Open => {}
        }
        let (a, b) = (group(k.from), group(k.to));
        if a == b {
            self_loops.push(k.id.clone());
            not_switchable.insert(
                k.id.clone(),
                "was left out (both ends joined through pumps or valves)".into(),
            );
            continue;
        }
        let d = diameter * s.units.diameter();
        let rough = match s.headloss {
            Headloss::DarcyWeisbach => roughness * s.units.roughness(),
            _ => roughness,
        };
        let f = darcy_factor(
            s.headloss,
            rough,
            d,
            options.reference_velocity.value(),
            s.viscosity,
            options.gravity.value(),
        );
        if !(f.is_finite() && f >= 0.0) {
            return Err(ReadError::at(
                k.line,
                format!(
                    "pipe '{}': roughness {roughness} gives no friction factor",
                    k.id
                ),
            ));
        }
        f_lo = f_lo.min(f);
        f_hi = f_hi.max(f);
        minor += usize::from(minor_loss != 0.0);
        let pipe = Pipe::new(
            Length::new(length * lu),
            Length::new(d),
            options.wave_speed,
            f,
        )
        .map_err(|e| ReadError::at(k.line, format!("pipe '{}': {e}", k.id)))?;
        network
            .add_pipe(ids[a], ids[b], pipe)
            .map_err(|e| ReadError::at(k.line, format!("pipe '{}': {e}", k.id)))?;
        pipe_ids.push(k.id.clone());
    }
    if pipe_ids.is_empty() {
        return Err(ReadError::general("the file has no usable pipes"));
    }
    let pipe_of: HashMap<String, usize> = pipe_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.clone(), i))
        .collect();

    // Pumps with their curves: a pump node between two short lengths of
    // station pipework (the engine's pump joins exactly two pipes).
    let shortest = network
        .pipes()
        .iter()
        .map(|p| p.pipe.length().value())
        .fold(f64::INFINITY, f64::min);
    let connector = CONNECTOR_LENGTH.min(shortest);
    let mut largest = vec![0.0_f64; network.nodes().len()];
    for p in network.pipes() {
        let d = p.pipe.diameter().value();
        for n in [p.from.index(), p.to.index()] {
            largest[n] = largest[n].max(d);
        }
    }
    let mut pump_of: HashMap<String, NodeId> = HashMap::new();
    let mut pumps_list = Vec::new();
    for k in links {
        let Some(f) = fitted.get(&k.id) else {
            continue;
        };
        let (a, b) = (group(k.from), group(k.to));
        if a == b {
            note(
                &mut notes,
                Some(k.line),
                format!(
                    "pump '{}' left out: both its ends were joined through valves",
                    k.id
                ),
            );
            not_switchable.insert(k.id.clone(), "was left out".into());
            continue;
        }
        let diameter = |n: usize, other: usize| {
            if largest[n] > 0.0 {
                largest[n]
            } else if largest[other] > 0.0 {
                largest[other]
            } else {
                0.3
            }
        };
        let station = |d: f64| {
            Pipe::new(
                Length::new(connector),
                Length::new(d),
                options.wave_speed,
                CONNECTOR_FRICTION,
            )
            .map_err(|e| ReadError::at(k.line, format!("pump '{}': {e}", k.id)))
        };
        let node = network.add_node(Node::Pump {
            pump: f.pump,
            trip: None,
        });
        network
            .add_pipe(ids[a], node, station(diameter(a, b))?)
            .map_err(|e| ReadError::at(k.line, format!("pump '{}': {e}", k.id)))?;
        network
            .add_pipe(node, ids[b], station(diameter(b, a))?)
            .map_err(|e| ReadError::at(k.line, format!("pump '{}': {e}", k.id)))?;
        pipe_ids.push(format!("{}/suction", k.id));
        pipe_ids.push(format!("{}/discharge", k.id));
        node_ids.push(k.id.clone());
        elevations.push(elevations[a]);
        positions.push(match (positions[a], positions[b]) {
            (Some(p), Some(q)) => Some((f64::midpoint(p.0, q.0), f64::midpoint(p.1, q.1))),
            (p, q) => p.or(q),
        });
        pump_of.insert(k.id.clone(), node);
        pumps_list.push((k.id.clone(), node));
    }
    if !pumps_list.is_empty() {
        note(
            &mut notes,
            None,
            format!(
                "{} pump(s) modelled with their curves, each between two {connector:.1} m lengths \
                 of station pipework (pipes 'ID/suction' and 'ID/discharge')",
                pumps_list.len()
            ),
        );
    }
    note(
        &mut notes,
        None,
        format!(
            ".inp files carry no wave speeds: every pipe is given {:.0} m/s [VERIFY - real wave speeds, by material, are needed to locate bursts well]",
            options.wave_speed.value()
        ),
    );
    note(
        &mut notes,
        None,
        format!(
            "{} roughness turned into one Darcy friction factor per pipe, equal head loss at {:.1} m/s: f from {f_lo:.4} to {f_hi:.4}",
            s.headloss.name(),
            options.reference_velocity.value()
        ),
    );
    if !closed.is_empty() {
        note(
            &mut notes,
            None,
            format!(
                "{} closed pipe(s) left out: {}",
                closed.len(),
                some_ids(&closed)
            ),
        );
    }
    if !checks.is_empty() {
        note(
            &mut notes,
            None,
            format!(
                "{} check-valve pipe(s) kept as plain pipes (the check valve is not modelled): {}",
                checks.len(),
                some_ids(&checks)
            ),
        );
    }
    if !self_loops.is_empty() {
        note(
            &mut notes,
            None,
            format!(
                "{} pipe(s) left out because both ends were joined through pumps or valves: {}",
                self_loops.len(),
                some_ids(&self_loops)
            ),
        );
    }
    if minor > 0 {
        note(
            &mut notes,
            None,
            format!("{minor} pipe(s) have minor-loss coefficients: not used"),
        );
    }
    let mut degree = vec![0usize; network.nodes().len()];
    for p in network.pipes() {
        degree[p.from.index()] += 1;
        degree[p.to.index()] += 1;
    }
    let isolated: Vec<String> = degree
        .iter()
        .enumerate()
        .filter(|(_, d)| **d == 0)
        .map(|(n, _)| node_ids[n].clone())
        .collect();
    if !isolated.is_empty() {
        note(
            &mut notes,
            None,
            format!(
                "{} node(s) connected to no pipe: {}",
                isolated.len(),
                some_ids(&isolated)
            ),
        );
    }

    // Pipe bends, for drawing.
    let pipe_index: HashMap<&str, usize> = pipe_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.as_str(), i))
        .collect();
    let mut vertices = vec![Vec::new(); pipe_ids.len()];
    for l in records(sections, Section::Vertices) {
        let id = field(l, 0, "link ID")?;
        if let Some(&p) = pipe_index.get(id) {
            vertices[p].push((number(l, 1, "X")?, number(l, 2, "Y")?));
        } else if !links.iter().any(|k| k.id == id) {
            return Err(ReadError::at(
                l.number,
                format!("bends for unknown link '{id}'"),
            ));
        }
    }

    let node_of_raw = (0..nodes.len()).map(|i| ids[group(i)]).collect();
    Ok((
        InpNetwork {
            title,
            units: s.units,
            headloss: s.headloss,
            network,
            node_ids,
            pipe_ids,
            elevations,
            positions,
            vertices,
            pumps: pumps_list,
            operations: Operations::default(),
            notes,
        },
        Mapping {
            node_of_raw,
            pipe_of,
            pump_of,
            not_switchable,
        },
    ))
}
