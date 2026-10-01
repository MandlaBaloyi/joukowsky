//! The scorecard (gap G-05): many events, one ruler - the metrics of
//! `docs/07_validation/validation_strategy.txt` ("Metrics"), measured on
//! the estate so that real data can later be measured the same way.
//!
//! BURSTS: holes of 3-12 mm opening instantly at a random point on a
//! random pipe of the estate, recorded by three 128 S/s loggers (random
//! clock phase, 2 cm noise), through the whole engine. Scored: detected,
//! called a burst, location error along the pipes, whether the truth lies
//! in the 90% credible region, and the size against the true leak flow.
//!
//! QUIET: hours of no event - 60 m with slow wander (demand changing,
//! 0.1 m over about a minute) and 2 cm of noise at each logger - for false
//! alarms per sensor-day.
//!
//! The thresholds checked are regression guards, set from the first
//! scorecard (2026-10-01) with margin; the printed table is the result.

use jk_application::{Alert, CollectingSink, PipelineConfig, Record, process};
use jk_classification::{Context, EventClass};
use jk_detection::Series;
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, NodeId, Probe,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::sensor::{Logger, Rng, sample};
use jk_tests::{check, pipe};

use crate::{RATE, estate_site};

/// The estate's pipes: (from, to, length) as node indices R=0, S1=1, T=2,
/// S2=3, S3=4.
const PIPES: [(usize, usize, f64); 4] =
    [(0, 1, 400.0), (1, 2, 300.0), (2, 3, 500.0), (2, 4, 600.0)];
const BURST_AT: f64 = 3.0;

/// One trial's truth.
struct Truth {
    pipe: usize,
    distance: f64,
    hole_mm: f64,
    /// The pipes' true wave speeds [m/s] (the site assumes 1000).
    speeds: [f64; 4],
}

/// What the engine made of one trial.
struct Score {
    detected: bool,
    burst: bool,
    error: Option<f64>,
    covered: Option<bool>,
    size_error: Option<f64>,
}

/// The estate with pipe `k` split at `d` m from its `from` end by a burst
/// node; simulates 10 s and records the three sensors (and the burst's
/// outflow). Returns the loggers' samples (with phases) and the true leak
/// flow after the opening [m^3/s].
fn simulate(t: &Truth, rng: &mut Rng) -> (Vec<(f64, Vec<f64>)>, f64) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let j = || Node::Junction { demand: Flow::ZERO };
    let nodes: Vec<NodeId> = std::iter::once(r)
        .chain((0..4).map(|_| net.add_node(j())))
        .collect();
    let mut burst_node = None;
    for (k, &(a, b, l)) in PIPES.iter().enumerate() {
        if k == t.pipe {
            let m = net.add_node(j());
            let c = t.speeds[k];
            net.add_pipe(nodes[a], m, pipe(t.distance, 0.15, c, 0.02))
                .unwrap();
            net.add_pipe(m, nodes[b], pipe(l - t.distance, 0.15, c, 0.02))
                .unwrap();
            burst_node = Some(m);
        } else {
            net.add_pipe(nodes[a], nodes[b], pipe(l, 0.15, t.speeds[k], 0.02))
                .unwrap();
        }
    }
    let m = burst_node.expect("split");
    net.add_burst(Burst {
        node: m,
        coefficient: leak::coefficient(
            0.6,
            Area::of_circle(Length::new(t.hole_mm / 1000.0)),
            STANDARD_GRAVITY,
        ),
        outside_head: Head::ZERO,
        start: Time::new(BURST_AT),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(10.0),
            &[
                Probe::Node(nodes[1]),
                Probe::Node(nodes[3]),
                Probe::Node(nodes[4]),
                Probe::Node(m),
            ],
            &GridOptions {
                max_wave_speed_adjustment: 0.001,
                min_reaches: 20,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let records = (0..3)
        .map(|i| {
            let truth: Vec<f64> = run.trace.head(i).iter().map(|h| h.value()).collect();
            let logger = Logger {
                rate: RATE,
                phase: rng.uniform() / RATE,
                noise: 0.02,
            };
            (logger.phase, sample(&truth, dt, &logger, rng))
        })
        .collect();
    // The leak's outflow, averaged over the last 2 s.
    let flows: Vec<f64> = run.trace.flow(3).iter().map(|q| q.value()).collect();
    let tail = &flows[flows.len() * 8 / 10..];
    #[allow(clippy::cast_precision_loss)]
    let leak = tail.iter().sum::<f64>() / tail.len() as f64;
    (records, leak)
}

/// Distance along the estate between `(pipe a, d_a)` and `(pipe b, d_b)`.
fn along(a: (usize, f64), b: (usize, f64)) -> f64 {
    if a.0 == b.0 {
        return (a.1 - b.1).abs();
    }
    // A tree of 5 nodes: node-to-node distances by Floyd-Warshall.
    let mut d = [[f64::INFINITY; 5]; 5];
    for (i, row) in d.iter_mut().enumerate() {
        row[i] = 0.0;
    }
    for &(u, v, l) in &PIPES {
        d[u][v] = l;
        d[v][u] = l;
    }
    for k in 0..5 {
        for i in 0..5 {
            for j in 0..5 {
                d[i][j] = d[i][j].min(d[i][k] + d[k][j]);
            }
        }
    }
    let ends = |(p, x): (usize, f64)| {
        let (u, v, l) = PIPES[p];
        [(u, x), (v, l - x)]
    };
    let mut best = f64::INFINITY;
    for (u, du) in ends(a) {
        for (v, dv) in ends(b) {
            best = best.min(du + d[u][v] + dv);
        }
    }
    best
}

/// Runs one trial through the engine and scores it.
fn trial(t: &Truth, rng: &mut Rng, config: &PipelineConfig) -> Score {
    let (records, leak) = simulate(t, rng);
    let site = estate_site(Context::unknown());
    let recs: Vec<Record<'_>> = records
        .iter()
        .enumerate()
        .map(|(i, (phase, x))| Record {
            sensor: i,
            series: Series {
                samples: x,
                start: Time::new(*phase),
                interval: Time::new(1.0 / RATE),
            },
        })
        .collect();
    let mut sink = CollectingSink::default();
    let alerts = process(&site, &recs, config, &mut sink).unwrap();
    let event: Option<&Alert> = alerts
        .iter()
        .filter(|a| (a.time.value() - BURST_AT).abs() < 1.0)
        .max_by_key(|a| a.revision);
    let Some(a) = event else {
        return Score {
            detected: false,
            burst: false,
            error: None,
            covered: None,
            size_error: None,
        };
    };
    let truth = (t.pipe, t.distance);
    let (error, covered) = a.location.as_ref().map_or((None, None), |loc| {
        let ml = (
            loc.most_likely.pipe.index(),
            loc.most_likely.distance.value(),
        );
        let inside = loc.credible_region.iter().any(|s| {
            s.pipe.index() == t.pipe
                && s.from.value() - 1e-6 <= t.distance
                && t.distance <= s.to.value() + 1e-6
        });
        (Some(along(ml, truth)), Some(inside))
    });
    Score {
        detected: true,
        burst: a.class == EventClass::Burst,
        error,
        covered,
        size_error: a
            .size
            .as_ref()
            .map(|s| (s.flow.value() - leak).abs() / leak),
    }
}

fn median(mut x: Vec<f64>) -> f64 {
    if x.is_empty() {
        return f64::NAN;
    }
    x.sort_by(f64::total_cmp);
    x[x.len() / 2]
}

/// Runs `per_size` trials per hole size, the pipes' true wave speeds
/// spread by `spread` (relative, one sigma) about the 1000 m/s the site
/// assumes, the localiser told `declared`; prints the scorecard, returns
/// (size, detection rate, coverage, median error) per size.
fn bursts(per_size: usize, seed: u64, spread: f64, declared: f64) -> Vec<(f64, f64, f64, f64)> {
    let mut rng = Rng::new(seed);
    let mut config = PipelineConfig::default();
    config.localise.wave_speed_uncertainty = declared;
    println!("  hole    trials  detected  burst  median error  in 90% region  median size error");
    let mut rows = Vec::new();
    for hole_mm in [3.0, 5.0, 8.0, 12.0] {
        let scores: Vec<Score> = (0..per_size)
            .map(|_| {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let pipe = ((rng.uniform() * 4.0) as usize).min(3);
                let l = PIPES[pipe].2;
                let distance = 10.0 + rng.uniform() * (l - 20.0);
                let speeds = [0; 4].map(|_| 1000.0 * (1.0 + spread * rng.normal()));
                let truth = Truth {
                    pipe,
                    distance,
                    hole_mm,
                    speeds,
                };
                trial(&truth, &mut rng, &config)
            })
            .collect();
        #[allow(clippy::cast_precision_loss)]
        let frac = |n: usize| n as f64 / per_size as f64;
        let detected = frac(scores.iter().filter(|s| s.detected).count());
        let burst = frac(scores.iter().filter(|s| s.burst).count());
        let located: Vec<&Score> = scores.iter().filter(|s| s.covered.is_some()).collect();
        #[allow(clippy::cast_precision_loss)]
        let coverage = if located.is_empty() {
            f64::NAN
        } else {
            located.iter().filter(|s| s.covered == Some(true)).count() as f64 / located.len() as f64
        };
        let error = median(scores.iter().filter_map(|s| s.error).collect());
        let size = median(scores.iter().filter_map(|s| s.size_error).collect());
        println!(
            "  {hole_mm:4.0} mm  {per_size:6}  {:7.0}%  {:4.0}%  {error:10.1} m  {:12.0}%  {:15.0}%",
            detected * 100.0,
            burst * 100.0,
            coverage * 100.0,
            size * 100.0
        );
        rows.push((hole_mm, detected, coverage, error));
    }
    rows
}

/// Quiet records of `seconds` at the three loggers: false alerts.
fn quiet(seconds: f64, seed: u64) -> (usize, f64) {
    let mut rng = Rng::new(seed);
    let site = estate_site(Context::unknown());
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (seconds * RATE) as usize;
    let dt = 1.0 / RATE;
    // Slow wander: an Ornstein-Uhlenbeck process, 0.1 m over ~60 s.
    let (sigma, tau) = (0.1, 60.0);
    let records: Vec<Vec<f64>> = (0..3)
        .map(|_| {
            let mut w = 0.0;
            (0..n)
                .map(|_| {
                    w += -w * dt / tau + sigma * (2.0 * dt / tau).sqrt() * rng.normal();
                    60.0 + w + 0.02 * rng.normal()
                })
                .collect()
        })
        .collect();
    let recs: Vec<Record<'_>> = records
        .iter()
        .enumerate()
        .map(|(i, x)| Record {
            sensor: i,
            series: Series {
                samples: x,
                start: Time::ZERO,
                interval: Time::new(dt),
            },
        })
        .collect();
    let mut sink = CollectingSink::default();
    let alerts = process(&site, &recs, &PipelineConfig::default(), &mut sink).unwrap();
    let mut ids: Vec<u32> = alerts.iter().map(|a| a.id).collect();
    ids.dedup();
    let sensor_days = 3.0 * seconds / 86_400.0;
    (ids.len(), sensor_days)
}

fn report_quiet(alarms: usize, sensor_days: f64) {
    if alarms == 0 {
        println!(
            "  quiet: 0 false alerts in {sensor_days:.3} sensor-days - fewer than {:.0} per \
             sensor-day (95%, rule of three)",
            3.0 / sensor_days
        );
    } else {
        #[allow(clippy::cast_precision_loss)]
        let rate = alarms as f64 / sensor_days;
        println!(
            "  quiet: {alarms} false alert(s) in {sensor_days:.3} sensor-days = {rate:.1} per sensor-day"
        );
    }
}

#[test]
fn scorecard_quick() {
    println!(
        "Scorecard (quick): 6 bursts per hole size on the estate, 3 loggers, wave speeds exact:"
    );
    let rows = bursts(6, 71, 0.0, 0.0);
    check(
        "  holes of 8 mm and more: every one detected",
        rows.iter().filter(|r| r.0 >= 8.0).all(|r| r.1 >= 1.0),
    );
    check(
        "  holes of 5 mm and more: median location error under 10 m",
        rows.iter().filter(|r| r.0 >= 5.0).all(|r| r.3 < 10.0),
    );
    let (alarms, sensor_days) = quiet(600.0, 72);
    report_quiet(alarms, sensor_days);
    check(
        "  no false alert in 30 sensor-minutes of quiet",
        alarms == 0,
    );
}

#[test]
#[ignore = "full suite: 3 x 40 bursts per size and 3 hours of quiet"]
fn scorecard_full() {
    println!("A. Wave speeds exact, the localiser told so:");
    let exact = bursts(40, 73, 0.0, 0.0);
    println!("B. Wave speeds off by 3% (random per pipe), the localiser told 3%:");
    let told = bursts(40, 75, 0.03, 0.03);
    println!("C. The same 3% doubt, the localiser told the speeds are exact:");
    let untold = bursts(40, 75, 0.03, 0.0);
    check(
        "  holes of 8 mm and more: at least 97% detected",
        exact.iter().filter(|r| r.0 >= 8.0).all(|r| r.1 >= 0.97),
    );
    let worst = |rows: &[(f64, f64, f64, f64)]| rows.iter().map(|r| r.2).fold(1.0, f64::min);
    println!(
        "  lowest coverage of the 90% region: A {:.0}%, B {:.0}%, C {:.0}%",
        worst(&exact) * 100.0,
        worst(&told) * 100.0,
        worst(&untold) * 100.0
    );
    check(
        "  told the truth about wave speeds, the 90% region holds it at least 80% of the time",
        worst(&exact) >= 0.8 && worst(&told) >= 0.8,
    );
    let (alarms, sensor_days) = quiet(3.0 * 3600.0, 74);
    report_quiet(alarms, sensor_days);
    #[allow(clippy::cast_precision_loss)]
    let rate = alarms as f64 / sensor_days;
    check("  false alerts under 1 per sensor-day", rate < 1.0);
}
