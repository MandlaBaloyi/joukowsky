//! V-A9: pumps and valves switched by tank levels, clock times and power
//! cuts in an extended-period run (`crates/domain/hydraulics/src/steady/
//! controls.rs`, R-2 controls note in `docs/02_physics_of_transients/
//! sources_dams_rivers_canals.txt`).
//!
//! The reference is independent of the engine: a tank of plan area At fed
//! at Qin(h) and drawn at a steady D rises as
//!
//! ```text
//!   At dh/dt = Qin(h) - D     =>   time from h1 to h2 = integral At / (Qin(h) - D) dh
//! ```
//!
//! evaluated by Simpson's rule on 20 000 intervals; with the inlet shut it
//! falls in a straight line, (h2 - h1) At / D exactly. For a pump, Qin
//! solves pump head = lift + friction, a0 + a1 Q + a2 Q^2 = (h - Hsump) +
//! R Q^2; for a float valve fed from a reservoir, Qin = sqrt((Hr - h) / R),
//! with R = f L / (2 g D A^2) per pipe.

use jk_hydraulics::steady::controls::{Cause, Condition, Control, Target};
use jk_hydraulics::steady::extended::{PeriodOptions, PeriodRun, SupplyEvent};
use jk_hydraulics::steady::rules::{Action, Clause, Premise, Relation, Rule};
use jk_hydraulics::steady::{Shut, SteadyOptions};
use jk_hydraulics::{Network, Node, NodeId, PipeId, TankStorage};
use jk_kernel::{Area, Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const HOUR: f64 = 3600.0;
/// Town demand drawn from the tank [m^3/s].
const DEMAND: f64 = 0.06;
/// Tank plan area [m^2].
const AREA: f64 = 200.0;
/// Pump switch-on and switch-off levels [m].
const LOW: f64 = 57.0;
const HIGH: f64 = 64.0;

/// Friction coefficient R = f L / (2 g D A^2) of a pipe [s^2/m^5].
fn resistance(length: f64, diameter: f64, f: f64) -> f64 {
    let a = std::f64::consts::PI * 0.25 * diameter * diameter;
    f * length / (2.0 * STANDARD_GRAVITY.value() * diameter * a * a)
}

/// Time [s] for the tank to rise from `h1` to `h2` when fed at `q_in(h)`.
fn fill_time(h1: f64, h2: f64, q_in: impl Fn(f64) -> f64) -> f64 {
    let n = 20_000;
    let dh = (h2 - h1) / f64::from(n);
    let f = |h: f64| AREA / (q_in(h) - DEMAND);
    let mut sum = f(h1) + f(h2);
    for k in 1..n {
        let h = h1 + f64::from(k) * dh;
        sum += if k % 2 == 1 { 4.0 } else { 2.0 } * f(h);
    }
    sum * dh / 3.0
}

/// Time [s] to fall from `h1` to `h2` with the inlet shut.
fn drain_time(h1: f64, h2: f64) -> f64 {
    (h1 - h2) * AREA / DEMAND
}

/// Sump (10 m) -> 20 m DN300 -> the station pump -> 2000 m DN300 rising
/// main -> tank (200 m^2, floor 50 m, overflow 66 m) -> town drawing 60 l/s.
fn pumped_tank(start: f64) -> (Network, NodeId, NodeId) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let sump = net.add_node(Node::Reservoir {
        head: Head::new(10.0),
    });
    let pump = net.add_node(Node::Pump {
        pump: super::pumps::station_pump(),
        trip: None,
    });
    let tank = net.add_node(Node::Reservoir {
        head: Head::new(start),
    });
    let town = net.add_node(Node::Junction {
        demand: Flow::new(DEMAND),
    });
    net.add_pipe(sump, pump, pipe(20.0, 0.3, 1000.0, 0.018))
        .unwrap();
    net.add_pipe(pump, tank, pipe(2000.0, 0.3, 1000.0, 0.018))
        .unwrap();
    net.add_pipe(tank, town, pipe(300.0, 0.3, 1000.0, 0.018))
        .unwrap();
    net.add_tank(TankStorage {
        node: tank,
        area: Area::new(AREA),
        floor: Head::new(50.0),
        ceiling: Head::new(66.0),
    })
    .unwrap();
    (net, pump, tank)
}

/// What the station pump delivers into a tank at level h [m^3/s].
fn pump_flow(h: f64) -> f64 {
    let (a0, a1, a2) = super::pumps::station_pump().curve.coefficients();
    let r = resistance(20.0, 0.3, 0.018) + resistance(2000.0, 0.3, 0.018);
    // (r - a2) Q^2 - a1 Q - (a0 - (h - 10)) = 0, the positive root.
    let (qa, qb, qc) = (r - a2, -a1, -(a0 - (h - 10.0)));
    (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa)
}

/// Times [s] of the switches of `target` in a run, with their direction.
fn switches(run: &PeriodRun, target: Target) -> Vec<(f64, bool, Cause)> {
    run.events
        .iter()
        .filter_map(|e| match e {
            SupplyEvent::Switched {
                target: t,
                open,
                at,
                cause,
            } if *t == target => Some((at.value(), *open, *cause)),
            _ => None,
        })
        .collect()
}

#[test]
fn va9_a_pump_fills_its_tank_between_the_switch_levels_on_time() {
    let (net, pump, tank) = pumped_tank(60.0);
    let run = |step: f64| {
        net.extended(&PeriodOptions {
            step: Time::new(step),
            duration: Time::new(48.0 * HOUR),
            controls: Control::fill_between(
                Target::Pump(pump),
                tank,
                Head::new(LOW),
                Head::new(HIGH),
            )
            .to_vec(),
            ..PeriodOptions::default()
        })
        .unwrap()
    };
    let fine = run(300.0);
    let coarse = run(900.0);
    println!(
        "A pump fills a 200 m^2 tank while the town draws 60 l/s; it starts at {LOW} m and \
         stops at {HIGH} m."
    );
    println!(
        "  pump delivers {:.1} l/s at {LOW} m, {:.1} l/s at {HIGH} m (lift and friction rise)",
        pump_flow(LOW) * 1000.0,
        pump_flow(HIGH) * 1000.0
    );
    let t1 = fill_time(60.0, HIGH, pump_flow);
    let t2 = t1 + drain_time(HIGH, LOW);
    let t3 = t2 + fill_time(LOW, HIGH, pump_flow);
    let t4 = t3 + drain_time(HIGH, LOW);
    let expected = [(t1, false), (t2, true), (t3, false), (t4, true)];
    let got = switches(&fine, Target::Pump(pump));
    let got_coarse = switches(&coarse, Target::Pump(pump));
    println!(
        "  the pump switched {} times in 48 h; the first four:",
        got.len()
    );
    check("  at least four switches", got.len() >= 4);
    for (k, ((t, open), (at, on, cause))) in expected.iter().zip(&got).enumerate() {
        let what = if *open { "starts" } else { "stops" };
        check_close(&format!("  pump {what} at [h]"), at / HOUR, t / HOUR, 1e-5);
        check(
            &format!("    ... {what}, by the level control"),
            on == open && matches!(cause, Cause::Control(_)),
        );
        println!(
            "    with 15-minute steps: {:.2} h ({:+.1} s)",
            got_coarse[k].0 / HOUR,
            got_coarse[k].0 - t
        );
    }
    let levels = &fine.tanks[0].1;
    let (lo, hi) = levels
        .iter()
        .skip(1)
        .fold((f64::MAX, f64::MIN), |(lo, hi), h| {
            (lo.min(h.value()), hi.max(h.value()))
        });
    println!("  the level stayed between {lo:.4} m and {hi:.4} m");
    check(
        "  it never passes the switch levels: each step is cut where they are reached",
        lo >= LOW - 1e-9 && hi <= HIGH + 1e-9,
    );
    let running = &fine.pumps[0].1;
    #[allow(clippy::cast_precision_loss)]
    let share = running.iter().filter(|r| **r).count() as f64 / running.len() as f64;
    println!("  the pump ran {:.0}% of the time", share * 100.0);
}

#[test]
fn va9_a_float_valve_on_the_inlet_closes_when_full_and_opens_when_low() {
    // A reservoir at 80 m feeds the tank through 1.5 km of DN200; the town
    // draws 60 l/s... from a tank the valve refills.
    let mut net = Network::new(STANDARD_GRAVITY);
    let source = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let tank = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let town = net.add_node(Node::Junction {
        demand: Flow::new(DEMAND),
    });
    let inlet = net
        .add_pipe(source, tank, pipe(1500.0, 0.25, 1000.0, 0.02))
        .unwrap();
    net.add_pipe(tank, town, pipe(300.0, 0.3, 1000.0, 0.018))
        .unwrap();
    net.add_tank(TankStorage {
        node: tank,
        area: Area::new(AREA),
        floor: Head::new(50.0),
        ceiling: Head::new(66.0),
    })
    .unwrap();
    let r = resistance(1500.0, 0.25, 0.02);
    let q_in = |h: f64| ((80.0 - h) / r).sqrt();
    let run = net
        .extended(&PeriodOptions {
            duration: Time::new(48.0 * HOUR),
            controls: Control::fill_between(
                Target::Pipe(inlet),
                tank,
                Head::new(LOW),
                Head::new(HIGH),
            )
            .to_vec(),
            ..PeriodOptions::default()
        })
        .unwrap();
    println!(
        "A float valve on a tank's inlet: the reservoir at 80 m delivers {:.1} l/s at {LOW} m, \
         {:.1} l/s at {HIGH} m.",
        q_in(LOW) * 1000.0,
        q_in(HIGH) * 1000.0
    );
    let t1 = fill_time(60.0, HIGH, q_in);
    let t2 = t1 + drain_time(HIGH, LOW);
    let t3 = t2 + fill_time(LOW, HIGH, q_in);
    let got = switches(&run, Target::Pipe(inlet));
    for ((t, open), (at, is_open, _)) in [(t1, false), (t2, true), (t3, false)].iter().zip(&got) {
        check_close(
            &format!("  valve {} at [h]", if *open { "opens" } else { "closes" }),
            at / HOUR,
            t / HOUR,
            1e-5,
        );
        check("    ... in that direction", is_open == open);
    }
    check("  three switches", got.len() >= 3);
}

#[test]
fn va9_load_shedding_stops_the_pump_and_it_restarts_when_power_returns() {
    // Start at 63.5 m, pump running. A short cut 0.2-0.3 h stops it
    // mid-fill; it stops by control at 64 m; a long cut 5-9 h covers the
    // moment the tank falls to 57 m, so the pump can only start at 9 h.
    let (net, pump, tank) = pumped_tank(63.5);
    let cuts = vec![
        (Time::new(0.2 * HOUR), Time::new(0.3 * HOUR)),
        (Time::new(5.0 * HOUR), Time::new(9.0 * HOUR)),
    ];
    let run = net
        .extended(&PeriodOptions {
            duration: Time::new(12.0 * HOUR),
            controls: Control::fill_between(
                Target::Pump(pump),
                tank,
                Head::new(LOW),
                Head::new(HIGH),
            )
            .to_vec(),
            power_cuts: cuts,
            ..PeriodOptions::default()
        })
        .unwrap();
    println!("Load-shedding: power off 0.2-0.3 h and 5-9 h.");
    for e in &run.events {
        println!("  {e:?}");
    }
    let got = switches(&run, Target::Pump(pump));
    check(
        "  0.2 h: the cut stops the running pump",
        matches!(got.first(), Some(&(t, false, Cause::PowerCut)) if (t - 0.2 * HOUR).abs() < 1e-6),
    );
    check(
        "  0.3 h: power back, the controls still want it: it restarts",
        matches!(got.get(1), Some(&(t, true, Cause::PowerBack)) if (t - 0.3 * HOUR).abs() < 1e-6),
    );
    let off = got.get(2).expect("the pump stops at the high level");
    check(
        "  then it stops by the level control",
        !off.1 && matches!(off.2, Cause::Control(1)),
    );
    let restart = got.get(3).expect("the pump restarts");
    let reaches_low = off.0 + drain_time(HIGH, LOW);
    println!(
        "  the tank falls to 57 m at {:.2} h, during the cut: the control asks for the pump",
        reaches_low / HOUR
    );
    check(
        "  no switch while the power is off",
        !got.iter()
            .any(|&(t, _, _)| t > 5.0 * HOUR + 1e-6 && t < 9.0 * HOUR - 1e-6)
            && reaches_low > 5.0 * HOUR
            && reaches_low < 9.0 * HOUR,
    );
    check(
        "  but the pump can only start when the power returns, at 9 h",
        restart.1 && restart.2 == Cause::PowerBack && (restart.0 - 9.0 * HOUR).abs() < 1e-6,
    );
    // With the pump off the tank falls in a straight line from 64 m.
    let k = run
        .times
        .iter()
        .position(|t| (t.value() - 9.0 * HOUR).abs() < 1.0)
        .unwrap();
    check_close(
        "  level when power returns [m] (straight fall from 64 m)",
        run.tanks[0].1[k].value(),
        HIGH - DEMAND / AREA * (9.0 * HOUR - off.0),
        1e-9,
    );
    let pumps_in_cut = run
        .times
        .iter()
        .zip(&run.pumps[0].1)
        .filter(|(t, _)| t.value() > 5.0 * HOUR && t.value() < 9.0 * HOUR)
        .any(|(_, running)| *running);
    check("  no pump runs during the cut", !pumps_in_cut);
}

#[test]
fn va9_timed_controls_switch_on_the_clock() {
    // The same pump, run by the clock only: off at 06:00, on again at 10:00
    // every day (a morning peak-tariff window), for two days.
    let (net, pump, _) = pumped_tank(58.0);
    let controls = vec![
        Control {
            target: Target::Pump(pump),
            open: false,
            when: Condition::Daily(Time::new(6.0 * HOUR)),
        },
        Control {
            target: Target::Pump(pump),
            open: true,
            when: Condition::Daily(Time::new(10.0 * HOUR)),
        },
    ];
    let run = net
        .extended(&PeriodOptions {
            step: Time::new(700.0), // not a divisor of an hour: the run still lands on 06:00
            duration: Time::new(48.0 * HOUR),
            controls,
            ..PeriodOptions::default()
        })
        .unwrap();
    let got: Vec<(f64, bool)> = switches(&run, Target::Pump(pump))
        .iter()
        .map(|&(t, open, _)| (t / HOUR, open))
        .collect();
    println!("Pump kept off in the morning peak: switches at {got:?} (h, running)");
    check(
        "  off at 06:00, on at 10:00, off at 06:00 next day, on at 10:00",
        got == vec![(6.0, false), (10.0, true), (30.0, false), (34.0, true)],
    );
}

#[test]
fn va9_closing_a_pipe_is_the_same_as_removing_it() {
    // A reservoir feeds a loop A-B-C; B has a dead-end branch to D (no
    // demand).
    let build = |with_ac: bool, with_bd: bool| {
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        let j = |q: f64| Node::Junction {
            demand: Flow::new(q),
        };
        let (a, b, c, d) = (
            net.add_node(j(0.01)),
            net.add_node(j(0.02)),
            net.add_node(j(0.015)),
            net.add_node(j(0.0)),
        );
        net.add_pipe(r, a, pipe(500.0, 0.3, 1000.0, 0.02)).unwrap();
        net.add_pipe(a, b, pipe(400.0, 0.2, 1000.0, 0.02)).unwrap();
        net.add_pipe(b, c, pipe(300.0, 0.15, 1000.0, 0.02)).unwrap();
        if with_ac {
            net.add_pipe(a, c, pipe(600.0, 0.2, 1000.0, 0.02)).unwrap();
        }
        if with_bd {
            net.add_pipe(b, d, pipe(200.0, 0.1, 1000.0, 0.02)).unwrap();
        }
        net
    };
    let options = SteadyOptions::default();
    let full = build(true, true);
    // Pipes: 0 R-A, 1 A-B, 2 B-C, 3 A-C, 4 B-D.
    let shut = full
        .steady_state_shut(
            &options,
            &Shut {
                pipes: vec![full.pipe_id(3).unwrap()],
                pumps: Vec::new(),
            },
        )
        .unwrap();
    let removed = build(false, true).steady_state(&options).unwrap();
    let worst = (0..5)
        .map(|n| (shut.node_heads[n].value() - removed.node_heads[n].value()).abs())
        .fold(0.0, f64::max);
    println!("Closing pipe A-C of a loop vs building the network without it:");
    check(
        &format!("  largest head difference {worst:.1e} m (rounding only)"),
        worst < 1e-9,
    );
    check(
        "  and the closed pipe carries nothing",
        shut.pipe_flows[3].value() == 0.0,
    );

    // Closing the branch to D: D is cut off with no demand - unknown head.
    let branch: PipeId = full.pipe_id(4).unwrap();
    let cut = full
        .steady_state_shut(
            &options,
            &Shut {
                pipes: vec![branch],
                pumps: Vec::new(),
            },
        )
        .unwrap();
    check(
        "  a dead end closed off with no demand: its head is unknown (NaN), not invented",
        cut.node_heads[4].value().is_nan(),
    );
    // Closing the supply pipe cuts off consumers: supply is lost.
    let err = full
        .steady_state_shut(
            &options,
            &Shut {
                pipes: vec![full.pipe_id(0).unwrap()],
                pumps: Vec::new(),
            },
        )
        .unwrap_err();
    println!("  closing the only supply pipe: {err}");
    check(
        "  cutting consumers off from every source is an error (supply lost)",
        err.to_string().contains("cut it off"),
    );
}

/// V-A12: rule-based controls (`steady/rules.rs`).
fn level_rule(name: &str, tank: NodeId, relation: Relation, level: f64, act: Action) -> Rule {
    Rule {
        name: name.into(),
        premises: vec![Premise {
            or: false,
            clause: Clause::TankLevel {
                tank,
                relation,
                level: Head::new(level),
            },
        }],
        then: vec![act],
        otherwise: Vec::new(),
        priority: 1.0,
    }
}

#[test]
fn va12_rules_that_mirror_the_level_controls_switch_at_the_same_moments() {
    let (net, pump, tank) = pumped_tank(60.0);
    let options = |controls: Vec<Control>, rules: Vec<Rule>| PeriodOptions {
        duration: Time::new(48.0 * HOUR),
        controls,
        rules,
        ..PeriodOptions::default()
    };
    let by_controls = net
        .extended(&options(
            Control::fill_between(Target::Pump(pump), tank, Head::new(LOW), Head::new(HIGH))
                .to_vec(),
            Vec::new(),
        ))
        .unwrap();
    let act = |open: bool| Action {
        target: Target::Pump(pump),
        open,
    };
    let by_rules = net
        .extended(&options(
            Vec::new(),
            vec![
                level_rule("on", tank, Relation::Below, LOW, act(true)),
                level_rule("off", tank, Relation::Above, HIGH, act(false)),
            ],
        ))
        .unwrap();
    let (a, b) = (
        switches(&by_controls, Target::Pump(pump)),
        switches(&by_rules, Target::Pump(pump)),
    );
    println!("The V-A9 pump, run by two rules instead of two controls:");
    check("  as many switches", a.len() == b.len() && !a.is_empty());
    let worst = a
        .iter()
        .zip(&b)
        .map(|(x, y)| (x.0 - y.0).abs())
        .fold(0.0, f64::max);
    check(
        &format!("  at the same moments (largest difference {worst:.1e} s)"),
        worst < 1e-6,
    );
    check(
        "  and each says a rule did it",
        b.iter().all(|s| matches!(s.2, Cause::Rule(_))),
    );
}

#[test]
fn va12_a_higher_priority_rule_holds_the_pump_off_until_its_window_ends() {
    // Peak window 01:00-04:00, priority 5: pump off. Level rules, priority
    // 1. The tank falls to 57 m inside the window: the level rule loses
    // until the window closes.
    let (net, pump, tank) = pumped_tank(58.0);
    let act = |open: bool| Action {
        target: Target::Pump(pump),
        open,
    };
    let clock = |relation: Relation, h: f64, or: bool| Premise {
        or,
        clause: Clause::ClockTime {
            relation,
            at: Time::new(h * HOUR),
        },
    };
    let peak = Rule {
        name: "peak".into(),
        premises: vec![
            clock(Relation::Above, 1.0, false),
            clock(Relation::Below, 4.0, false),
        ],
        then: vec![act(false)],
        otherwise: Vec::new(),
        priority: 5.0,
    };
    let run = net
        .extended(&PeriodOptions {
            duration: Time::new(6.0 * HOUR),
            rules: vec![
                peak,
                level_rule("on", tank, Relation::Below, LOW, act(true)),
                level_rule("off", tank, Relation::Above, HIGH, act(false)),
            ],
            ..PeriodOptions::default()
        })
        .unwrap();
    let got = switches(&run, Target::Pump(pump));
    println!("Pump switches: {got:?}");
    check(
        "  01:00: the peak rule stops the pump",
        matches!(got.first(), Some(&(t, false, Cause::Rule(0))) if (t - HOUR).abs() < 1e-6),
    );
    let k = run
        .times
        .iter()
        .position(|t| (t.value() - HOUR).abs() < 1.0)
        .unwrap();
    let reaches_low = HOUR + (run.tanks[0].1[k].value() - LOW) * AREA / DEMAND;
    println!(
        "  the tank falls to {LOW} m at {:.2} h, inside the window",
        reaches_low / HOUR
    );
    check(
        "  the level rule wants the pump then, but has the lower priority",
        reaches_low < 4.0 * HOUR && got.len() >= 2 && got[1].0 > reaches_low + 1.0,
    );
    check(
        "  04:00: the window ends and the level rule starts the pump",
        matches!(got.get(1), Some(&(t, true, Cause::Rule(1))) if (t - 4.0 * HOUR).abs() < 1e-6),
    );
}
