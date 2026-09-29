//! `jk network FILE`: read a network file and say what is in it.

use std::path::{Path, PathBuf};

use jk_files::epanet::{InpOptions, read};
use jk_hydraulics::Node;

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
    println!("What the reader assumed or left out ({}):", inp.notes.len());
    for n in &inp.notes {
        match n.line {
            Some(l) => println!("  - line {l}: {}", n.text),
            None => println!("  - {}", n.text),
        }
    }
    Ok(())
}
