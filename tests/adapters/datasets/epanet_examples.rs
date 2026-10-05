//! D1 - the EPANET example networks (Net1, Net2, Net3; MIT licence;
//! data/README.md). Files written by EPANET's own authors, not by us.
//!
//! Truth: the files themselves. Every junction, reservoir, tank and pipe
//! the file declares must be in the engine's network under its own ID (or
//! be named in a note saying why not), every pump and tank must be modelled,
//! and the file's own schedule (its extended-period run) must play out.

use jk_files::epanet::{InpOptions, read};
use jk_tests::check;

use super::{dataset, section_ids};
use crate::print_notes;

/// Reads one example and checks it against its own declarations.
/// `left_out`: pipes the reader is expected to leave out (closed in the
/// file), each named in a note.
fn example(name: &str, pumps: usize, tanks: usize, left_out: &[&str]) {
    let Some(path) = dataset(&format!("epanet-examples/{name}")) else {
        return;
    };
    let text = std::fs::read_to_string(&path).unwrap();
    let inp = read(&path, &InpOptions::default()).unwrap_or_else(|e| panic!("{name}: {e}"));
    print_notes(&inp);

    let engine_nodes: Vec<&str> = inp.node_ids.iter().flat_map(|id| id.split('=')).collect();
    let mut declared = Vec::new();
    for section in ["[JUNCTIONS]", "[RESERVOIRS]", "[TANKS]"] {
        declared.extend(section_ids(&text, section));
    }
    let missing: Vec<&String> = declared
        .iter()
        .filter(|id| !engine_nodes.contains(&id.as_str()))
        .collect();
    println!(
        "  nodes: the file declares {}, the engine has all but {:?}",
        declared.len(),
        missing
    );
    check(
        "every node the file declares is in the engine",
        missing.is_empty(),
    );

    let file_pipes = section_ids(&text, "[PIPES]");
    let missing: Vec<&str> = file_pipes
        .iter()
        .map(String::as_str)
        .filter(|id| !inp.pipe_ids.iter().any(|p| p == id))
        .collect();
    println!(
        "  pipes: the file declares {}, the engine has all but {missing:?} (expected: {left_out:?})",
        file_pipes.len()
    );
    check("only the expected pipes are left out", missing == left_out);
    for id in left_out {
        check(
            &format!("a note says why pipe {id} is left out"),
            inp.notes.iter().any(|n| n.text.contains(id)),
        );
    }
    println!(
        "  pumps modelled with their curves: {} (file: {pumps}); tanks with storage: {} (file: {tanks})",
        inp.pumps.len(),
        inp.network.tanks().len()
    );
    check("every pump is modelled", inp.pumps.len() == pumps);
    check(
        "every tank stores water",
        inp.network.tanks().len() == tanks,
    );

    let period = &inp.operations.period;
    let run = inp
        .network
        .extended(period)
        .unwrap_or_else(|e| panic!("{name}: its own schedule failed: {e}"));
    println!(
        "  its own schedule: {:.0} h in {:.0}-minute steps, {} event(s)",
        period.duration.value() / 3600.0,
        period.step.value() / 60.0,
        run.events.len()
    );
}

#[test]
fn d1_net1_reads_whole_and_runs_its_day() {
    example("Net1.inp", 1, 1, &[]);
}

#[test]
fn d1_net2_reads_whole_and_runs_its_55_hours() {
    example("Net2.inp", 0, 1, &[]);
}

#[test]
fn d1_net3_reads_whole_and_runs_its_day() {
    // Pipe 330 is CLOSED in the file ([STATUS]) and only ever opened by a
    // control; the engine leaves it out and says so.
    example("Net3.inp", 2, 3, &["330"]);
}
