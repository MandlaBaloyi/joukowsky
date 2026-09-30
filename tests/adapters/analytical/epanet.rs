//! Reading EPANET `.inp` network files: every number known in advance.

use jk_files::epanet::{InpOptions, parse};
use jk_hydraulics::Node;
use jk_hydraulics::steady::controls::{Condition, Control, Target};
use jk_localisation::placement::{PlacementOptions, coverage};
use jk_tests::estate::estate;
use jk_tests::{check, check_close};

use crate::{fixture, print_notes};

/// The estate: R --400-- S1 --300-- T --500-- S2, T --600-- S3, 150 mm.
#[test]
fn the_estate_in_si_units_reads_exactly() {
    let net = fixture("estate_si.inp");
    println!("{}", net.title);
    let n = &net.network;
    check(
        "5 nodes, 4 pipes",
        n.nodes().len() == 5 && n.pipes().len() == 4,
    );
    let lengths: Vec<f64> = n.pipes().iter().map(|p| p.pipe.length().value()).collect();
    check(
        "lengths 400, 300, 500, 600 m",
        lengths == [400.0, 300.0, 500.0, 600.0],
    );
    check(
        "every diameter 150 mm",
        n.pipes()
            .iter()
            .all(|p| (p.pipe.diameter().value() - 0.15).abs() < 1e-12),
    );
    let r = net.node_ids.iter().position(|id| id == "R").unwrap();
    check(
        "R is a 60 m reservoir",
        matches!(n.nodes()[r], Node::Reservoir { head } if (head.value() - 60.0).abs() < 1e-12),
    );
    let s2 = net.node_ids.iter().position(|id| id == "S2").unwrap();
    check(
        "S2 draws 15 l/s",
        matches!(n.nodes()[s2], Node::Junction { demand } if (demand.value() - 0.015).abs() < 1e-12),
    );
    check(
        "S3 is drawn at (700, -600)",
        net.positions[net.node_ids.iter().position(|id| id == "S3").unwrap()]
            == Some((700.0, -600.0)),
    );
    check(
        "elevation of S2: 12 m",
        net.elevations[s2].is_some_and(|e| (e.value() - 12.0).abs() < 1e-12),
    );
    print_notes(&net);
    check(
        "it says wave speeds were assumed",
        net.notes.iter().any(|n| n.text.contains("no wave speeds")),
    );
}

#[test]
fn us_units_give_the_same_network_as_si_units() {
    // The same estate written in gallons per minute, feet and inches
    // (converted exactly): lengths, diameters, demands and heads must come
    // out identical. Friction differs on purpose (Hazen-Williams C = 130
    // instead of Darcy-Weisbach 0.1 mm) - both near f = 0.02.
    // The US file is written to 6 decimals (e.g. 5.905512 in), so it
    // agrees to about 1 part in 10 million - that is the tolerance.
    let (si, us) = (fixture("estate_si.inp"), fixture("estate_us.inp"));
    for (a, b) in si.network.pipes().iter().zip(us.network.pipes()) {
        check_close(
            "pipe length [m]",
            b.pipe.length().value(),
            a.pipe.length().value(),
            1e-7,
        );
        check_close(
            "pipe diameter [m]",
            b.pipe.diameter().value(),
            a.pipe.diameter().value(),
            1e-7,
        );
    }
    for (a, b) in si.network.nodes().iter().zip(us.network.nodes()) {
        match (a, b) {
            (Node::Junction { demand: x }, Node::Junction { demand: y }) => {
                check_close("demand [m3/s]", y.value(), x.value(), 1e-6);
            }
            (Node::Reservoir { head: x }, Node::Reservoir { head: y }) => {
                check_close("reservoir head [m]", y.value(), x.value(), 1e-7);
            }
            _ => check("same kind of node", false),
        }
    }
    let f_si = si.network.pipes()[0].pipe.friction_factor();
    let f_us = us.network.pipes()[0].pipe.friction_factor();
    println!("  friction factor: D-W 0.1 mm gives {f_si:.4}; H-W C = 130 gives {f_us:.4}");
    check(
        "both friction factors plausible (0.015 - 0.025)",
        [f_si, f_us].iter().all(|f| (0.015..0.025).contains(f)),
    );
}

#[test]
#[allow(clippy::too_many_lines)] // one file, every mapping checked in turn
fn everything_approximated_is_reported_never_dropped() {
    let net = fixture("loud.inp");
    print_notes(&net);
    let says = |what: &str| net.notes.iter().any(|n| n.text.contains(what));
    let (_, pump) = net
        .pumps
        .iter()
        .find(|(id, _)| id == "PU1")
        .expect("PU1 is modelled as a pump");
    let Node::Pump { pump: p, .. } = net.network.nodes()[pump.index()] else {
        panic!("PU1's node is a pump");
    };
    check_close(
        "pump PU1, one-point curve 10 l/s at 40 m: shut-off head 4/3 x 40 [m]",
        p.curve.head(0.0, 1.0),
        40.0 * 4.0 / 3.0,
        1e-12,
    );
    check_close(
        "  ... and 40 m at 10 l/s [m]",
        p.curve.head(0.010, 1.0),
        40.0,
        1e-12,
    );
    check(
        "  J1 and J2 stay apart, joined by the pump and its station pipework",
        net.node_ids.iter().any(|id| id == "J1")
            && net.node_ids.iter().any(|id| id == "J2")
            && net.pipe_ids.iter().any(|id| id == "PU1/suction")
            && net.pipe_ids.iter().any(|id| id == "PU1/discharge"),
    );
    check(
        "  ... and says what was assumed",
        says("pump 'PU1': one-point curve"),
    );
    check(
        "the closed valve does not connect J4 and J5",
        says("PRV valve 'V1' is closed"),
    );
    check(
        "the closed pipe is left out",
        says("closed pipe(s) left out: A4") && !net.pipe_ids.iter().any(|p| p == "A4"),
    );
    check(
        "the check-valve pipe is kept, and flagged",
        says("check-valve pipe(s) kept") && net.pipe_ids.iter().any(|p| p == "A3"),
    );
    let tank = net.node_ids.iter().position(|id| id == "TK").unwrap();
    let tank_node = tank;
    check(
        "the tank is a fixed head at 30 + 4 = 34 m (in transients)",
        matches!(net.network.nodes()[tank], Node::Reservoir { head } if (head.value() - 34.0).abs() < 1e-12),
    );
    let storage = net.network.tanks().first().copied().expect("a tank");
    check(
        "  ... with storage: floor 30 + 1, ceiling 30 + 6, 10 m across",
        storage.node.index() == tank
            && (storage.floor.value() - 31.0).abs() < 1e-12
            && (storage.ceiling.value() - 36.0).abs() < 1e-12
            && (storage.area.value() - std::f64::consts::PI * 25.0).abs() < 1e-9,
    );
    check("  ... and says so", says("tank(s) with storage"));
    let j4 = net.node_ids.iter().position(|id| id == "J4").unwrap();
    check(
        "[DEMANDS] replaces J4's demand: 0.5 + 0.7 = 1.2 l/s",
        matches!(net.network.nodes()[j4], Node::Junction { demand } if (demand.value() - 0.0012).abs() < 1e-12),
    );
    let period = &net.operations.period;
    check_close(
        "[TIMES] duration 24:00 [h]",
        period.duration.value() / 3600.0,
        24.0,
        1e-12,
    );
    let j1 = net.node_ids.iter().position(|id| id == "J1").unwrap();
    check(
        "[PATTERNS] J1 follows its own pattern 'daily' (1.0 1.2 0.8)",
        period.node_patterns.len() == 1
            && period.node_patterns[0].multipliers == [1.0, 1.2, 0.8]
            && period.node_patterns[0]
                .nodes
                .iter()
                .any(|n| n.index() == j1),
    );
    check(
        "  ... the others have no pattern (no option PATTERN, no pattern '1')",
        period.pattern.is_empty(),
    );
    let a2 = net.pipe_ids.iter().position(|p| p == "A2").unwrap();
    check(
        "[CONTROLS] A2 closes when tank TK is above 5 m of water: head 30 + 5 = 35 m",
        matches!(
            period.controls.as_slice(),
            [Control {
                target: Target::Pipe(p),
                open: false,
                when: Condition::LevelAbove { tank, level },
            }] if p.index() == a2 && tank.index() == tank_node && (level.value() - 35.0).abs() < 1e-12
        ),
    );
    check("the unused option is listed", says("Quality None"));
    check("the minor loss is listed", says("minor-loss"));
    let a5 = net.pipe_ids.iter().position(|p| p == "A5").unwrap();
    check(
        "pipe A5 keeps its bend at (400, 120)",
        net.vertices[a5] == [(400.0, 120.0)],
    );
}

#[test]
fn broken_files_are_refused_with_the_line_number() {
    let opts = InpOptions::default();
    let cases = [
        (
            "[JUNCTIONS]\nA 1\n[PIPES]\nP1 A B 10 100 0.1\n",
            4,
            "unknown node 'B'",
        ),
        ("[JUNCTIONS]\nA 1\nA 2\n", 3, "used twice"),
        ("[JUNCTIONS]\nA one\n", 2, "not a number"),
        ("[JUNCTIONS]\nA 1\n[PIPEZ]\n", 3, "not an EPANET section"),
        (
            "[JUNCTIONS]\nA 1\nB 1\n[PIPES]\nP1 A B -10 100 0.1\n",
            5,
            "length",
        ),
    ];
    for (text, line, what) in cases {
        let e = parse(text, &opts).unwrap_err();
        println!("  refused: {e}");
        check(
            &format!("line {line}: '{what}'"),
            e.line == Some(line) && e.message.contains(what),
        );
    }
}

#[test]
fn the_engine_works_on_a_network_read_from_a_file() {
    // The placement score of the three estate loggers must be the same on
    // the file's network as on the estate built in code: same geometry,
    // same wave speed (1000 m/s), so the same answer.
    let from_file = fixture("estate_si.inp");
    let id = |s: &str| {
        from_file
            .network
            .node_id(from_file.node_ids.iter().position(|x| x == s).unwrap())
            .unwrap()
    };
    let options = PlacementOptions::default();
    let a = coverage(
        &from_file.network,
        &[id("S1"), id("S2"), id("S3")],
        None,
        &options,
    )
    .unwrap();
    let (e, _) = estate(None);
    let b = coverage(&e.net, &e.sensors, None, &options).unwrap();
    check_close(
        "average 90% search stretch [m]: file vs code",
        a.expected_region.value(),
        b.expected_region.value(),
        1e-9,
    );
}
