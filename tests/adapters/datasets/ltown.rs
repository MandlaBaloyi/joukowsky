//! D2 - L-Town, the `BattLeDIM` 2020 leak benchmark (Vrachimis et al.; CC BY
//! 4.0; data/README.md): a town of 782 junctions and 905 pipes, with two
//! years of 5-minute SCADA readings and the true leaks.
//!
//! 1. The model reads whole: every junction, reservoir, tank and pipe the
//!    file declares is in the engine; its pump pumps and its tank fills and
//!    drains (three reader fixes were needed - FN-045).
//! 2. Compared with the truth: the first week of 2018. The engine plays the
//!    week from the model alone; the published readings say when the pump
//!    really ran (its flow, every 5 minutes). Same switches, in the same
//!    order, at nearly the same times.
//!
//! Why not exactly the same times: the organisers made the readings with a
//! "real" model and handed out a nominal one that differs from it by up to
//! 10% in demands, diameters and roughness (their README). A drain or fill
//! of the tank takes 10-15 h, so 10% is 1-1.5 h per half cycle, and the
//! differences carry from one cycle to the next. The limits below (no pair
//! more than 5 h apart, 2.5 h on average) were set from the first run on
//! 2026-10-05 (largest 4.0 h, average 1.7 h): they lock in today's
//! agreement, so a change that makes it worse fails. [VERIFY: with the
//! organisers' "real" model, L-TOWN_Real.inp (177 MB, not downloaded), the
//! times should agree to minutes.]

use jk_files::epanet::{InpOptions, read};
use jk_hydraulics::steady::controls::Target;
use jk_hydraulics::steady::extended::SupplyEvent;
use jk_tests::check;

use super::{dataset, section_ids};
use crate::print_notes;

const HOUR: f64 = 3600.0;
/// The SCADA's reading interval [s].
const READING: f64 = 300.0;

#[test]
fn d2_ltown_reads_whole_with_its_pump_and_tank() {
    let Some(path) = dataset("ltown/L-TOWN.inp") else {
        return;
    };
    let text = std::fs::read_to_string(&path).unwrap();
    let inp = read(&path, &InpOptions::default()).unwrap_or_else(|e| panic!("L-Town: {e}"));
    print_notes(&inp);
    let engine_nodes: Vec<&str> = inp.node_ids.iter().flat_map(|id| id.split('=')).collect();
    let mut declared = Vec::new();
    for section in ["[JUNCTIONS]", "[RESERVOIRS]", "[TANKS]"] {
        declared.extend(section_ids(&text, section));
    }
    let missing = declared
        .iter()
        .filter(|id| !engine_nodes.contains(&id.as_str()))
        .count();
    let pipes = section_ids(&text, "[PIPES]");
    let missing_pipes = pipes.iter().filter(|id| !inp.pipe_ids.contains(id)).count();
    println!(
        "  the file declares {} nodes and {} pipes; missing in the engine: {missing} nodes, {missing_pipes} pipes",
        declared.len(),
        pipes.len()
    );
    check(
        "782 junctions + 2 reservoirs + 1 tank declared",
        declared.len() == 785,
    );
    check("905 pipes declared", pipes.len() == 905);
    check(
        "every node and pipe is in the engine",
        missing == 0 && missing_pipes == 0,
    );
    check(
        "the pump is modelled with its curve (FN-045: it ends at zero head)",
        inp.pumps.len() == 1 && inp.pumps[0].0 == "PUMP_1",
    );
    check("the tank stores water", inp.network.tanks().len() == 1);
    check(
        "both controls act on the pump",
        inp.operations.period.controls.len() == 2,
    );
    check("the three PRVs are modelled", inp.valves.len() == 3);
}

/// When the pump switched in the published readings, in the first `hours`:
/// (time [s], running after). A switch is placed half a reading before the
/// first reading that shows it (+/- 2.5 min).
fn pump_switches_in_readings(csv: &str, hours: f64) -> Vec<(f64, bool)> {
    let mut lines = csv.lines();
    let header: Vec<&str> = lines.next().unwrap().split(';').collect();
    let col = header.iter().position(|h| *h == "PUMP_1").unwrap();
    let mut out = Vec::new();
    let mut was: Option<bool> = None;
    for (i, line) in lines.enumerate() {
        let t = READING * f64::from(u32::try_from(i).unwrap());
        if t > hours * HOUR {
            break;
        }
        let flow: f64 = line
            .split(';')
            .nth(col)
            .unwrap()
            .replace(',', ".")
            .parse()
            .unwrap();
        let running = flow > 1.0; // m^3/h; it delivers about 44 when running
        if was.is_some_and(|w| w != running) {
            out.push((t - READING / 2.0, running));
        }
        was = Some(running);
    }
    out
}

#[test]
#[ignore = "full suite: a week of the town, about a minute in a debug build"]
fn d2_ltown_first_week_pump_switches_like_the_published_data() {
    let (Some(model), Some(flows)) = (
        dataset("ltown/L-TOWN.inp"),
        dataset("ltown/2018_SCADA_Flows.csv"),
    ) else {
        return;
    };
    let inp = read(&model, &InpOptions::default()).unwrap();
    let period = &inp.operations.period;
    let week = period.duration.value() / HOUR;
    let run = inp.network.extended(period).unwrap();
    let engine: Vec<(f64, bool)> = run
        .events
        .iter()
        .filter_map(|e| match e {
            SupplyEvent::Switched {
                target: Target::Pump(_),
                open,
                at,
                ..
            } => Some((at.value(), *open)),
            _ => None,
        })
        .collect();
    let truth = pump_switches_in_readings(&std::fs::read_to_string(flows).unwrap(), week);

    println!("  the first {week:.0} h of 2018 (the model's own run length):");
    println!("    switch    engine [h]   published [h]   difference [h]");
    let mut gaps = Vec::new();
    for (k, (e, t)) in engine.iter().zip(&truth).enumerate() {
        let gap = (e.0 - t.0) / HOUR;
        gaps.push(gap);
        println!(
            "    {:2} {:5}   {:8.2}     {:8.2}        {gap:+6.2}{}",
            k + 1,
            if t.1 { "on" } else { "off" },
            e.0 / HOUR,
            t.0 / HOUR,
            if e.1 == t.1 {
                ""
            } else {
                "   <- not the same switch"
            }
        );
    }
    let longer = if engine.len() > truth.len() {
        &engine
    } else {
        &truth
    };
    for e in longer.iter().skip(gaps.len()) {
        println!(
            "    (only in the {}: {} at {:.2} h, near the end of the week)",
            if engine.len() > truth.len() {
                "engine"
            } else {
                "readings"
            },
            if e.1 { "on" } else { "off" },
            e.0 / HOUR
        );
    }
    #[allow(clippy::cast_precision_loss)] // a handful of switches
    let mean = gaps.iter().map(|g| g.abs()).sum::<f64>() / gaps.len() as f64;
    let worst = gaps.iter().map(|g| g.abs()).fold(0.0, f64::max);
    println!(
        "  switches: engine {}, readings {}; average difference {mean:.2} h, largest {worst:.2} h",
        engine.len(),
        truth.len()
    );
    check(
        "as many switches, give or take one at the end of the week",
        engine.len().abs_diff(truth.len()) <= 1 && gaps.len() >= 10,
    );
    check(
        "every switch is the same kind (on/off) in the same order",
        engine.iter().zip(&truth).all(|(e, t)| e.1 == t.1),
    );
    check("no switch more than 5 h from the readings", worst <= 5.0);
    check("2.5 h or less apart on average", mean <= 2.5);
}
