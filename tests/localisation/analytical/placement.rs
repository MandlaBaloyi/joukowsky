//! The placement advisor: where the loggers should go.
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2
//!                        |
//!                       600
//!                        |
//!                        S3
//! ```
//!
//! The geometry: a burst is pinpointed only if it lies BETWEEN two
//! sensors. A burst on a branch beyond the outermost sensor reaches all of
//! them through the same junction, so it looks like a burst at that
//! junction. And the supply main R-S1 can never be pinpointed: the
//! reservoir holds the head fixed, so no logger can sit beyond it - the
//! report must say so rather than hide it.
//!
//! What the advisor optimises is what a repair crew walks: the expected
//! size of the 90% region. That is not simply "loggers at the ends" - see
//! the second test.

use jk_hydraulics::{Network, NodeId};
use jk_localisation::placement::{Coverage, PlacementOptions, advise, coverage, detectability};
use jk_tests::check;

use crate::estate;

const PIPE_NAMES: [&str; 4] = ["R-S1 supply main", "S1-T spine", "T-S2 north", "T-S3 south"];

/// Prints a coverage report the way a client would read it.
fn report(what: &str, c: &Coverage, names: &[(NodeId, &str)]) {
    let name = |n: NodeId| {
        names
            .iter()
            .find(|(m, _)| *m == n)
            .map_or_else(|| format!("node {}", n.index()), |(_, s)| (*s).to_string())
    };
    let sensors: Vec<String> = c.sensors.iter().map(|&n| name(n)).collect();
    println!(
        "{what}: {}\n  pinpointed (90% region <= 20 m): {:.0}% of the pipe length; \
         average 90% region {:.0} m",
        sensors.join(", "),
        c.pinpointed * 100.0,
        c.expected_region.value()
    );
    for p in &c.pipes {
        println!(
            "    {:<18} {:>5.0} m   pinpointed {:>4.0}%   typical region {:>5.0} m",
            PIPE_NAMES[p.pipe.index()],
            p.length.value(),
            p.pinpointed * 100.0,
            p.median_region.value()
        );
    }
}

fn estate_nodes() -> (Network, [(NodeId, &'static str); 4]) {
    let (e, _) = estate(None);
    let [s1, s2, s3] = e.sensors;
    let names = [(s1, "S1"), (e.tee, "T (tee)"), (s2, "S2"), (s3, "S3")];
    (e.net, names)
}

/// Exactly known wave speeds: only the geometry limits the answer.
fn exact_speeds() -> PlacementOptions {
    let mut o = PlacementOptions::default();
    o.localise.wave_speed_uncertainty = 0.0;
    o
}

#[test]
fn two_sensors_pinpoint_only_what_lies_between_them() {
    let (net, names) = estate_nodes();
    let (s2, s3) = (names[2].0, names[3].0);
    let c = coverage(&net, &[s2, s3], None, &exact_speeds()).unwrap();
    report(
        "Loggers at the two ends (wave speeds exactly known)",
        &c,
        &names,
    );
    let share = |i: usize| c.pipes[i].pinpointed;
    check(
        "T-S2 and T-S3 (between them): pinpointed",
        share(2) > 0.9 && share(3) > 0.9,
    );
    check(
        "S1-T and R-S1 (beyond them): blind",
        share(0) == 0.0 && share(1) == 0.0,
    );
    // Equally likely anywhere in those 700 m: the 90% region is 90% of
    // them, 630 m (plus one 5 m candidate cell).
    check(
        "  a burst there could be anywhere upstream of T: region 630 m",
        [0, 1]
            .iter()
            .all(|&i| (c.pipes[i].median_region.value() - 630.0).abs() <= 5.0),
    );
}

#[test]
fn the_advisor_minimises_the_search_and_says_what_stays_blind() {
    // With wave speeds known to +/-10% (the default: asset records).
    let (net, names) = estate_nodes();
    let candidates: Vec<NodeId> = names.iter().map(|(n, _)| *n).collect();
    let options = PlacementOptions::default();
    let advice = advise(&net, &candidates, 3, &[], None, &options).unwrap();
    println!("What each extra logger buys (wave speeds known to +/-10%):");
    for step in &advice.steps {
        report(&format!("  {} loggers", step.sensors.len()), step, &names);
    }
    let [(s1, _), _, (s2, _), (s3, _)] = names;
    let ends = coverage(&net, &[s2, s3], None, &options).unwrap();
    report("  for comparison, the two far ends", &ends, &names);
    let pair = &advice.steps[0];
    check(
        "best pair S1 + S3: two small blind zones beat one big one",
        pair.sensors.contains(&s1) && pair.sensors.contains(&s3),
    );
    check(
        "  ... a shorter expected search than the two ends",
        pair.beats(&ends),
    );
    check(
        "3 loggers: S1, S2, S3",
        [s1, s2, s3].iter().all(|s| advice.best.sensors.contains(s)),
    );
    check(
        "each extra logger helps",
        advice.steps.windows(2).all(|w| w[1].beats(&w[0])),
    );
    check(
        "R-S1 stays blind: no logger can sit beyond a reservoir",
        advice.best.pipes[0].pinpointed == 0.0
            && advice.best.pipes[0].median_region.value() >= 300.0,
    );
}

#[test]
fn knowing_the_wave_speeds_is_worth_as_much_as_a_logger() {
    // The same three loggers, with the book wave speeds known to 0 - 20%
    // (one sigma). The region grows with the doubt - so calibrating the
    // pipes from known events (F9) pays like extra hardware.
    let (net, names) = estate_nodes();
    let sensors = [names[0].0, names[2].0, names[3].0];
    let mut last = 0.0;
    let mut grows = true;
    let mut pinpointed = Vec::new();
    for doubt in [0.0, 0.01, 0.02, 0.05, 0.1, 0.2] {
        let mut options = PlacementOptions::default();
        options.localise.wave_speed_uncertainty = doubt;
        let c = coverage(&net, &sensors, None, &options).unwrap();
        println!(
            "  wave speeds known to {:>3.0}%: pinpointed {:>3.0}%, average 90% region {:>4.0} m",
            doubt * 100.0,
            c.pinpointed * 100.0,
            c.expected_region.value()
        );
        grows &= c.expected_region.value() >= last;
        last = c.expected_region.value();
        pinpointed.push(c.pinpointed);
    }
    check("the region only grows as the doubt grows", grows);
    check(
        "exact speeds: all but the supply main pinpointed (78%)",
        (pinpointed[0] - 1400.0 / 1800.0).abs() < 0.02,
    );
}

/// A commercial campus: the municipal connection feeds a ring main (ductile
/// iron, a = 1000 m/s) with plastic branches (uPVC, a = 400 m/s) out to
/// the buildings. 2.31 km of pipe; loggers can go at any junction or
/// building (11 places).
///
/// ```text
///   R(municipal) --150-- M --300-- A --250-- B
///                        |  \      |\        |\
///                       m1   250   a1 a2    b1 b2
///                            C --300---------+
///                            |\
///                           c1 c2
/// ```
fn campus() -> (Network, Vec<(NodeId, &'static str)>, Vec<&'static str>) {
    use jk_hydraulics::Node;
    use jk_kernel::{Flow, Head, STANDARD_GRAVITY};
    use jk_tests::pipe;
    let iron = |l| pipe(l, 0.15, 1000.0, 0.02);
    let upvc = |l| pipe(l, 0.1, 400.0, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r = net.add_node(Node::Reservoir {
        head: Head::new(70.0),
    });
    let names = [
        "M entrance",
        "A",
        "B",
        "C",
        "m1 gatehouse",
        "a1 block A1",
        "a2 block A2",
        "b1 block B1",
        "b2 block B2",
        "c1 block C1",
        "c2 block C2",
    ];
    let n: Vec<NodeId> = names.iter().map(|_| net.add_node(j())).collect();
    let (m, a, b, c) = (n[0], n[1], n[2], n[3]);
    let mut pipes = Vec::new();
    let mut add = |x, y, p, name| {
        net.add_pipe(x, y, p).unwrap();
        pipes.push(name);
    };
    add(r, m, iron(150.0), "R-M connection");
    add(m, a, iron(300.0), "M-A ring");
    add(a, b, iron(250.0), "A-B ring");
    add(b, c, iron(300.0), "B-C ring");
    add(c, m, iron(250.0), "C-M ring");
    add(m, n[4], upvc(100.0), "M-m1");
    add(a, n[5], upvc(120.0), "A-a1");
    add(a, n[6], upvc(180.0), "A-a2");
    add(b, n[7], upvc(200.0), "B-b1");
    add(b, n[8], upvc(90.0), "B-b2");
    add(c, n[9], upvc(150.0), "C-c1");
    add(c, n[10], upvc(220.0), "C-c2");
    let named = n.iter().copied().zip(names).collect();
    (net, named, pipes)
}

#[test]
#[ignore = "full suite: the advisor on a 2.3 km campus takes about a minute"]
fn campus_value_of_each_extra_logger() {
    let (net, names, pipe_names) = campus();
    let candidates: Vec<NodeId> = names.iter().map(|(n, _)| *n).collect();
    let started = std::time::Instant::now();
    let advice = advise(
        &net,
        &candidates,
        6,
        &[],
        None,
        &PlacementOptions::default(),
    )
    .unwrap();
    let seconds = started.elapsed().as_secs_f64();
    let name = |n: NodeId| names.iter().find(|(m, _)| *m == n).map_or("?", |(_, s)| *s);
    println!("Campus, 2.31 km, wave speeds known to +/-10%: what each logger buys");
    for s in &advice.steps {
        let who: Vec<&str> = s.sensors.iter().map(|&n| name(n)).collect();
        println!(
            "  {} loggers: average 90% region {:>4.0} m, pinpointed {:>3.0}%   ({})",
            s.sensors.len(),
            s.expected_region.value(),
            s.pinpointed * 100.0,
            who.join(", ")
        );
    }
    let who: Vec<&str> = advice.best.sensors.iter().map(|&n| name(n)).collect();
    println!(
        "Recommended 6 (after swaps): {}
  average 90% region {:.0} m, pinpointed {:.0}%",
        who.join(", "),
        advice.best.expected_region.value(),
        advice.best.pinpointed * 100.0
    );
    for p in &advice.best.pipes {
        println!(
            "    {:<16} {:>4.0} m   typical region {:>4.0} m",
            pipe_names[p.pipe.index()],
            p.length.value(),
            p.median_region.value()
        );
    }
    println!("  (advice took {seconds:.0} s on this machine)");
    let felt = detectability(&net, &advice.best.sensors, &PlacementOptions::default()).unwrap();
    println!(
        "Would a sudden 1 l/s burst be felt? By at least one logger on {:.0}% of the pipe, \
         by two (so located) on {:.0}%. Smallest burst locatable anywhere on each pipe \
         (upper bound: no plastic-wall damping yet):",
        felt.felt * 100.0,
        felt.locatable * 100.0
    );
    for p in &felt.pipes {
        println!(
            "    {:<16} {:>6.2} l/s",
            pipe_names[p.pipe.index()],
            p.smallest_locatable.value() * 1000.0
        );
    }
    check(
        "each extra logger helps",
        advice.steps.windows(2).all(|w| w[1].beats(&w[0])),
    );
    check(
        "swaps never make it worse",
        !advice.steps[advice.steps.len() - 1].beats(&advice.best),
    );
}
