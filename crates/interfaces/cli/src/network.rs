//! `jk network FILE`: read a network file and say what is in it.

use std::path::{Path, PathBuf};

use jk_files::epanet::{InpOptions, read};
use jk_hydraulics::steady::controls::Target;
use jk_hydraulics::steady::extended::SupplyEvent;
use jk_hydraulics::{Node, NodeId};

pub const USAGE: &str = "
    jk network FILE       read an EPANET .inp network file and summarise it,
                          with everything the reader had to assume
";

/// Reads the arguments after `network`.
///
/// # Errors
/// No file name, or more than one.
pub fn parse(mut it: impl Iterator<Item = String>) -> Result<PathBuf, String> {
    let file = it.next().ok_or("jk network needs a file name")?;
    if let Some(extra) = it.next() {
        return Err(format!("unexpected argument '{extra}'"));
    }
    Ok(PathBuf::from(file))
}

/// Reads the file and prints the summary and the reader's notes.
///
/// # Errors
/// A file that cannot be read or is not a valid network.
pub fn run(path: &Path) -> Result<(), String> {
    let inp = read(path, &InpOptions::default()).map_err(|e| format!("{}: {e}", path.display()))?;
    let net = &inp.network;
    let length: f64 = net.pipes().iter().map(|p| p.pipe.length().value()).sum();
    let supplies = net
        .nodes()
        .iter()
        .filter(|n| matches!(n, Node::Reservoir { .. }))
        .count();
    println!("{}", path.display());
    if !inp.title.is_empty() {
        println!("  title        {}", inp.title);
    }
    println!(
        "  units        {} ({} friction)",
        inp.units.name(),
        inp.headloss.name()
    );
    println!(
        "  network      {} nodes ({supplies} supply/fixed-head), {} pipes, {:.2} km of pipe",
        net.nodes().len(),
        net.pipes().len(),
        length / 1000.0
    );
    let drawn = inp.positions.iter().filter(|p| p.is_some()).count();
    println!(
        "  map          coordinates for {drawn} of {} nodes",
        net.nodes().len()
    );
    let period = &inp.operations.period;
    println!(
        "  operation    {} pump(s) with curves, {} tank(s) with storage, {} control(s), {} demand pattern(s)",
        inp.pumps.len(),
        net.tanks().len(),
        period.controls.len(),
        usize::from(!period.pattern.is_empty()) + period.node_patterns.len()
    );
    if !inp.pumps.is_empty() || !net.tanks().is_empty() {
        extended_summary(&inp);
    }
    println!("What the reader assumed or left out ({}):", inp.notes.len());
    for n in &inp.notes {
        match n.line {
            Some(l) => println!("  - line {l}: {}", n.text),
            None => println!("  - {}", n.text),
        }
    }
    Ok(())
}

/// Most events listed from the extended run.
const SHOWN: usize = 12;

/// Runs the file's own schedule (extended period) and lists what happened.
fn extended_summary(inp: &jk_files::epanet::InpNetwork) {
    let period = &inp.operations.period;
    let started = std::time::Instant::now();
    let run = match inp.network.extended(period) {
        Ok(run) => run,
        Err(e) => {
            println!("  extended run failed: {e}");
            return;
        }
    };
    let hours = |t: jk_kernel::Time| t.value() / 3600.0;
    let node = |n: NodeId| inp.node_ids[n.index()].clone();
    let target = |t: Target| match t {
        Target::Pump(n) => format!("pump {}", node(n)),
        Target::Pipe(p) => format!("pipe {}", inp.pipe_ids[p.index()]),
    };
    println!(
        "  extended run {:.0} h in {:.0}-minute steps, as the file schedules it ({:.2} s):",
        hours(period.duration),
        period.step.value() / 60.0,
        started.elapsed().as_secs_f64()
    );
    for e in run.events.iter().take(SHOWN) {
        let line = match e {
            SupplyEvent::TankEmpty { tank, at } => {
                format!("{:7.2} h  tank {} empty", hours(*at), node(*tank))
            }
            SupplyEvent::TankRefilling { tank, at } => {
                format!("{:7.2} h  tank {} refilling", hours(*at), node(*tank))
            }
            SupplyEvent::TankFull { tank, at } => format!(
                "{:7.2} h  tank {} full (overflowing)",
                hours(*at),
                node(*tank)
            ),
            SupplyEvent::Switched {
                target: t,
                open,
                at,
                ..
            } => format!(
                "{:7.2} h  {} {}",
                hours(*at),
                target(*t),
                match (t, open) {
                    (Target::Pump(_), true) => "starts",
                    (Target::Pump(_), false) => "stops",
                    (Target::Pipe(_), true) => "opens",
                    (Target::Pipe(_), false) => "closes",
                }
            ),
            SupplyEvent::PowerOff { at } => format!("{:7.2} h  power off", hours(*at)),
            SupplyEvent::PowerOn { at } => format!("{:7.2} h  power on", hours(*at)),
            SupplyEvent::SupplyLost { at, reason } => {
                format!("{:7.2} h  SUPPLY LOST: {reason}", hours(*at))
            }
        };
        println!("    {line}");
    }
    if run.events.len() > SHOWN {
        println!("    ... and {} more events", run.events.len() - SHOWN);
    }
    if run.events.is_empty() {
        println!("    nothing switched, no tank emptied or filled");
    }
}
