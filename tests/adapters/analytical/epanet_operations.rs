//! EPANET files that RUN: pumps with their curves, `[CONTROLS]`,
//! `[PATTERNS]` and `[TIMES]` read into the engine's extended-period run
//! (`crates/adapters/files/src/epanet/{pumps, operations}.rs`).
//!
//! 1. The V-A9 system (sump, pump, rising main, 200 m^2 tank, town drawing
//!    60 l/s; pump on at 57 m, off at 64 m) written as a file. The switch
//!    times must match the same independent reference as V-A9: Simpson
//!    quadrature of At dh / (Qpump(h) - D) with the pump curve and the pipe
//!    resistances the file gives (the pump's two 10 m lengths of station
//!    pipework included).
//! 2. Times, patterns and controls in every form a file writes them, each
//!    mapped or noted.

use jk_files::epanet::{InpOptions, parse};
use jk_hydraulics::steady::controls::{Condition, Control, Target};
use jk_hydraulics::steady::extended::SupplyEvent;
use jk_hydraulics::{Network, Node};
use jk_tests::{check, check_close};

use crate::{fixture, print_notes};

const HOUR: f64 = 3600.0;

/// R = f L / (2 g D A^2) of every pipe named, summed.
fn resistance(net: &Network, ids: &[String], names: &[&str]) -> f64 {
    let g = net.gravity().value();
    names
        .iter()
        .map(|name| {
            let k = ids.iter().position(|id| id == name).unwrap();
            let p = &net.pipes()[k].pipe;
            let (d, a) = (p.diameter().value(), p.area().value());
            p.friction_factor() * p.length().value() / (2.0 * g * d * a * a)
        })
        .sum()
}

#[test]
fn a_pump_and_its_controls_read_from_a_file_switch_on_time() {
    let inp = fixture("pumped_tank.inp");
    print_notes(&inp);
    let net = &inp.network;
    let (_, pump) = inp.pumps[0];
    let Node::Pump { pump: p, .. } = net.nodes()[pump.index()] else {
        panic!("PU is a pump node");
    };
    let (a0, a1, a2) = p.curve.coefficients();
    let r = resistance(net, &inp.pipe_ids, &["PU/suction", "PU/discharge", "MAIN"]);
    let tank = net.tanks()[0];
    let area = tank.area.value();
    let demand = 0.060;
    // What the pump delivers into the tank at head h [m^3/s].
    let q_pump = |h: f64| {
        let (qa, qb, qc) = (r - a2, -a1, -(a0 - (h - 10.0)));
        (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa)
    };
    let fill = |h1: f64, h2: f64| {
        let n = 20_000;
        let dh = (h2 - h1) / f64::from(n);
        let f = |h: f64| area / (q_pump(h) - demand);
        let mut sum = f(h1) + f(h2);
        for k in 1..n {
            sum += if k % 2 == 1 { 4.0 } else { 2.0 } * f(h1 + f64::from(k) * dh);
        }
        sum * dh / 3.0
    };
    let drain = (64.0 - 57.0) * area / demand;
    let t1 = fill(60.0, 64.0);
    let t2 = t1 + drain;
    let t3 = t2 + fill(57.0, 64.0);
    let t4 = t3 + drain;

    let period = &inp.operations.period;
    println!(
        "The file's run: {:.0} h in {:.0}-minute steps, {} controls",
        period.duration.value() / HOUR,
        period.step.value() / 60.0,
        period.controls.len()
    );
    check_close(
        "  tank level 50 + 7 in the file = switch-on head [m]",
        match period.controls[0].when {
            Condition::LevelBelow { level, .. } => level.value(),
            _ => f64::NAN,
        },
        57.0,
        1e-12,
    );
    let run = net.extended(period).unwrap();
    let got: Vec<(f64, bool)> = run
        .events
        .iter()
        .filter_map(|e| match e {
            SupplyEvent::Switched { at, open, .. } => Some((at.value(), *open)),
            _ => None,
        })
        .collect();
    println!(
        "  pump delivers {:.1} l/s at 57 m, {:.1} l/s at 64 m",
        q_pump(57.0) * 1000.0,
        q_pump(64.0) * 1000.0
    );
    for ((t, open), (at, is_open)) in [(t1, false), (t2, true), (t3, false), (t4, true)]
        .iter()
        .zip(&got)
    {
        check_close(
            &format!("  pump {} at [h]", if *open { "starts" } else { "stops" }),
            at / HOUR,
            t / HOUR,
            1e-5,
        );
        check("    ... that way round", open == is_open);
    }
    check("  at least four switches in 48 h", got.len() >= 4);
}

#[test]
#[allow(clippy::too_many_lines)] // one file, every mapping checked in turn
fn times_patterns_and_controls_in_every_form_are_mapped_or_noted() {
    let text = "\
[JUNCTIONS]
A  0  1
B  0  1  homes
C  0  1  plant
D  0  0
[RESERVOIRS]
S  5
[TANKS]
TK  40  3  1  6  10  0
[PIPES]
P1  S   A   100  200  0.1
P2  A   B   100  200  0.1
P3  B   TK  100  200  0.1
P4  B   C   100  150  0.1
P5  B   D   100  150  0.1
[PUMPS]
PU1  S2  A  HEAD c1
PU2  S3  B  POWER 5
[RESERVOIRS]
S2  5
S3  5
[CURVES]
c1  20  30
[VALVES]
V1  C  D  150  PRV  30  0
[STATUS]
PU1  Closed
[PATTERNS]
homes  1  2  3  4
plant  5  5
[CONTROLS]
LINK PU1 OPEN IF NODE TK BELOW 2
LINK PU1 CLOSED AT TIME 1:30
LINK P2 CLOSED AT CLOCKTIME 8 AM
LINK P4 OPEN IF NODE C BELOW 20
LINK V1 CLOSED AT TIME 3
LINK PU1 0.8 AT TIME 4
[TIMES]
Duration  2 days
Hydraulic Timestep  15 min
Pattern Timestep  1:00
Pattern Start  2:00
Start ClockTime  6 AM
[OPTIONS]
Units  LPS
Headloss  D-W
Pattern  homes
";
    let inp = parse(text, &InpOptions::default()).unwrap();
    print_notes(&inp);
    let says = |what: &str| inp.notes.iter().any(|n| n.text.contains(what));
    let p = &inp.operations.period;
    // Nodes joined through a valve or pump carry both IDs ("C=D").
    let node = |id: &str| {
        inp.node_ids
            .iter()
            .position(|x| x.split('=').any(|part| part == id))
            .unwrap()
    };
    let pipe = |id: &str| inp.pipe_ids.iter().position(|x| x == id).unwrap();
    println!("[TIMES]");
    check_close("  2 days [h]", p.duration.value() / HOUR, 48.0, 1e-12);
    check_close("  15 min steps [s]", p.step.value(), 900.0, 1e-12);
    println!("[PATTERNS] (pattern start 2:00 = two hourly steps in)");
    check(
        "  option PATTERN homes is the default, rotated: 3 4 1 2",
        p.pattern == [3.0, 4.0, 1.0, 2.0],
    );
    check(
        "  C follows 'plant'; B, whose own pattern is the default, is not listed",
        p.node_patterns.len() == 1
            && p.node_patterns[0].multipliers == [5.0, 5.0]
            && p.node_patterns[0]
                .nodes
                .iter()
                .map(|n| n.index())
                .eq([node("C")]),
    );
    println!("[STATUS] and [CONTROLS]");
    let pu1 = inp.pumps.iter().find(|(id, _)| id == "PU1").unwrap().1;
    check(
        "  PU1 starts closed: shut at the start, switched off at t = 0",
        inp.operations.shut.pumps == [pu1]
            && p.controls[0]
                == Control {
                    target: Target::Pump(pu1),
                    open: false,
                    when: Condition::At(jk_kernel::Time::ZERO),
                },
    );
    let tk = inp.network.node_id(node("TK")).unwrap();
    check(
        "  PU1 on below 2 m of water in TK: head 40 + 2 = 42 m",
        p.controls.iter().any(|c| {
            c.target == Target::Pump(pu1)
                && c.open
                && c.when
                    == Condition::LevelBelow {
                        tank: tk,
                        level: jk_kernel::Head::new(42.0),
                    }
        }),
    );
    check(
        "  PU1 off at 1:30 into the run",
        p.controls
            .iter()
            .any(|c| c.when == Condition::At(jk_kernel::Time::new(5400.0))),
    );
    check(
        "  P2 closed at 8 AM = 2 h after the 6 AM start, every day",
        p.controls.iter().any(|c| {
            c.target == Target::Pipe(inp.network.pipe_id(pipe("P2")).unwrap())
                && !c.open
                && c.when == Condition::Daily(jk_kernel::Time::new(2.0 * HOUR))
        }),
    );
    check(
        "  a junction-pressure condition is noted, not modelled",
        says("condition on node 'C'"),
    );
    check(
        "  a control on a valve is noted",
        says("link 'V1' is a PRV valve"),
    );
    check("  a pump speed setting is noted", says("set to 0.8"));
    check(
        "  the constant-power pump PU2 is a connection, and says so",
        says("pump 'PU2' is given as constant power"),
    );
    check(
        "  the start clock time is noted",
        says("start clock time 06:00"),
    );
    let run = inp.network.extended(p);
    check("  and the whole thing runs", run.is_ok());
    if let Err(e) = run {
        println!("  run failed: {e}");
    }
}
