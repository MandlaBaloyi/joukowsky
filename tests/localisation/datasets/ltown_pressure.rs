//! D2 - WHERE are L-Town's leaks? From its 33 pressure sensors and the
//! town's model (`BattLeDIM` 2020, data/README.md; the method:
//! `crates/domain/localisation/src/pressure.rs`, FN-053).
//!
//! The engine plays a 10 m3/h leak at each of the 782 junctions in turn
//! (steady state at 03:00, the town's own demands then) and keeps how far
//! each sensor's pressure falls - a signature per junction. A measured
//! change of the sensors' night pressures (02:00-04:00 means; nights
//! after minus nights before) is matched to every signature; the best fit
//! is where the leak most likely is. Scored as the distance from the best
//! junction to the true leaking pipe, against a junction picked at random.
//!
//! 1. With the TRUE start of each leak choosing the nights - the method's
//!    reach, a reference.
//! 2. With the night WATCH's own alarms choosing them - as the engine
//!    would run: each alarm matched one to one to a true leak (as the
//!    leak tests score), the leak located a week after its rise began.

use jk_balance::{LeakOptions, watch};
use jk_files::epanet::{InpNetwork, InpOptions, read};
use jk_files::scada::{ScadaTable, ScadaUnit};
use jk_hydraulics::{Node, NodeId, SteadyOptions};
use jk_kernel::{Flow, Time};
use jk_localisation::{Signatures, locate_from_pressures};
use jk_tests::ltown::{DAY, M3H, date, match_alarms, table, town_nights, true_leaks};
use jk_tests::{check, dataset};

/// Leak played at each junction for its signature (10 m3/h).
const PROBE: f64 = 10.0 * M3H;

/// Mean pressure of each sensor over 02:00-04:00 of every night, both
/// years, [night][sensor].
fn night_pressures(tables: &[ScadaTable; 2], sensors: &[String]) -> Vec<Vec<f64>> {
    let mut out = Vec::new();
    for t in tables {
        let n = t.columns[0].values.len() / 288;
        for d in 0..n {
            out.push(
                sensors
                    .iter()
                    .map(|s| {
                        let v = &t.column(s).unwrap().values[d * 288 + 24..d * 288 + 48];
                        v.iter().sum::<f64>() / 24.0
                    })
                    .collect(),
            );
        }
    }
    out
}

/// Mean over nights [from, to) of each sensor.
fn mean_nights(p: &[Vec<f64>], from: usize, to: usize) -> Vec<f64> {
    #[allow(clippy::cast_precision_loss)] // small
    let k = (to - from) as f64;
    (0..p[0].len())
        .map(|s| p[from..to].iter().map(|row| row[s]).sum::<f64>() / k)
        .collect()
}

/// Each sensor's night-to-night noise [m]: robust spread of the change
/// from one night to the same weekday's night a week later, at least
/// 2 cm (the files give pressures to the centimetre).
fn sensor_noise(p: &[Vec<f64>]) -> Vec<f64> {
    (0..p[0].len())
        .map(|s| {
            let mut d: Vec<f64> = (7..p.len())
                .map(|k| (p[k][s] - p[k - 7][s]).abs())
                .collect();
            d.sort_by(f64::total_cmp);
            (1.4826 * d[d.len() / 2] / std::f64::consts::SQRT_2).max(0.02)
        })
        .collect()
}

/// Everything both parts share.
struct Town {
    inp: InpNetwork,
    signatures: Signatures,
    pressures: Vec<Vec<f64>>,
    noise: Vec<f64>,
}

/// One location: the best junction, the flow it implies [m3/h], its
/// distance from the leak, and a random junction's (median) [m].
struct Located {
    best: String,
    flow: f64,
    away: f64,
    random: f64,
}

impl Town {
    fn pos(&self, k: usize) -> (f64, f64) {
        self.inp.positions[k].unwrap_or((f64::NAN, f64::NAN))
    }

    /// The middle of pipe `id` on the map.
    fn pipe_at(&self, id: &str) -> (f64, f64) {
        let k = self.inp.pipe_ids.iter().position(|x| x == id).unwrap();
        let p = &self.inp.network.pipes()[k];
        let (a, b) = (self.pos(p.from.index()), self.pos(p.to.index()));
        (f64::midpoint(a.0, b.0), f64::midpoint(a.1, b.1))
    }

    /// Locates the change between nights `before` and `after` ([from,
    /// to)), scored against the nearest of the leaks at `targets`.
    fn locate(
        &self,
        before: (usize, usize),
        after: (usize, usize),
        targets: &[(f64, f64)],
    ) -> Option<Located> {
        let b = mean_nights(&self.pressures, before.0, before.1);
        let a = mean_nights(&self.pressures, after.0, after.1);
        let change: Vec<f64> = a.iter().zip(&b).map(|(x, y)| x - y).collect();
        let best = *locate_from_pressures(&self.signatures, &change, &self.noise)
            .unwrap()
            .first()?;
        let dist = |p: (f64, f64)| {
            targets
                .iter()
                .map(|at| (p.0 - at.0).hypot(p.1 - at.1))
                .fold(f64::INFINITY, f64::min)
        };
        let mut random: Vec<f64> = self
            .signatures
            .candidates()
            .iter()
            .map(|c| dist(self.pos(c.index())))
            .collect();
        random.sort_by(f64::total_cmp);
        Some(Located {
            best: self.inp.node_ids[best.node.index()].clone(),
            flow: best.flow.value() / M3H,
            away: dist(self.pos(best.node.index())),
            random: random[random.len() / 2],
        })
    }
}

/// Reads the town, makes the signatures, reads the night pressures.
fn town(paths: &[std::path::PathBuf]) -> Town {
    let inp = read(&paths[0], &InpOptions::default()).unwrap();
    let tables = [
        table(&paths[1], ScadaUnit::Metres),
        table(&paths[2], ScadaUnit::Metres),
    ];
    let sensors: Vec<String> = tables[0].columns.iter().map(|c| c.name.clone()).collect();
    let node = |id: &str| {
        let k = inp.node_ids.iter().position(|n| n == id).unwrap();
        inp.network.node_id(k).unwrap()
    };
    let sensor_nodes: Vec<NodeId> = sensors.iter().map(|s| node(s)).collect();
    // The town at 03:00 (the patterns' own clock).
    let mut net = inp.network.clone();
    let n = net.nodes().len();
    let m = inp
        .operations
        .period
        .multipliers_at(n, Time::new(3.0 * 3600.0));
    let mut candidates = Vec::new();
    for (k, &mk) in m.iter().enumerate() {
        if let Node::Junction { demand } = net.nodes()[k] {
            let id = net.node_id(k).unwrap();
            net.set_demand(id, demand * mk).unwrap();
            // The file's own junctions, not the engine's device pipework.
            if !inp.node_ids[k].contains('/') && !inp.node_ids[k].contains('=') {
                candidates.push(id);
            }
        }
    }
    let started = std::time::Instant::now();
    let signatures = Signatures::compute(
        &net,
        &sensor_nodes,
        &candidates,
        Flow::new(PROBE),
        &SteadyOptions::default(),
    )
    .unwrap();
    println!(
        "  {} signatures (a 10 m3/h leak at each junction, 03:00) in {:.1} s",
        candidates.len(),
        started.elapsed().as_secs_f64()
    );
    let pressures = night_pressures(&tables, &sensors);
    let noise = sensor_noise(&pressures);
    Town {
        inp,
        signatures,
        pressures,
        noise,
    }
}

/// Prints the distances' summary; returns (within 300 m, median).
fn summary(found: &mut [f64], random: &mut [f64]) -> (usize, f64) {
    found.sort_by(f64::total_cmp);
    random.sort_by(f64::total_cmp);
    let within = |m: f64| found.iter().filter(|&&x| x <= m).count();
    let median = found[found.len() / 2];
    println!(
        "  of {} located: within 300 m of the leak {}, within 500 m {}; median {median:.0} m - a junction picked at random: {:.0} m",
        found.len(),
        within(300.0),
        within(500.0),
        random[random.len() / 2]
    );
    (within(300.0), median)
}

#[test]
#[ignore = "full suite: 782 steady states of the town and two years of readings"]
#[allow(clippy::too_many_lines)] // one story, printed step by step
fn d2_ltown_leaks_located_from_pressure_sensors() {
    let files = [
        "ltown/L-TOWN.inp",
        "ltown/2018_SCADA_Pressures.csv",
        "ltown/2019_SCADA_Pressures.csv",
        "ltown/dataset_configuration.yaml",
        "ltown/2018_SCADA_Flows.csv",
        "ltown/2019_SCADA_Flows.csv",
        "ltown/2018_SCADA_Levels.csv",
        "ltown/2019_SCADA_Levels.csv",
        "ltown/2018_Leakages.csv",
        "ltown/2019_Leakages.csv",
    ];
    let Some(paths) = files.iter().map(|f| dataset(f)).collect::<Option<Vec<_>>>() else {
        return;
    };
    let t = town(&paths);
    let leak_flows = [
        table(&paths[8], ScadaUnit::CubicMetresPerHour),
        table(&paths[9], ScadaUnit::CubicMetresPerHour),
    ];
    let yaml = std::fs::read_to_string(&paths[3]).unwrap();
    let truth = true_leaks(&yaml, &leak_flows);
    let nights = t.pressures.len();

    println!("\n  1. Nights chosen by the TRUE start of each leak (the method's reach):");
    println!("    pipe   kind      peak [m3/h]  best junction  sized [m3/h]  distance [m]");
    let (mut found, mut random) = (Vec::new(), Vec::new());
    for l in &truth {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let start = (l.start / DAY).ceil() as usize;
        if start < 8 || start + 9 > nights {
            continue;
        }
        // A sudden leak: the week after it began; a growing one: the week
        // from its peak (right after it begins it is still small).
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let peak = (l.peak_at / DAY).floor() as usize;
        let from = if l.abrupt {
            start + 1
        } else {
            peak.max(start + 1).min(nights - 8)
        };
        let Some(x) = t.locate(
            (start - 8, start - 1),
            (from, from + 7),
            &[t.pipe_at(&l.pipe)],
        ) else {
            continue;
        };
        println!(
            "    {:5}  {:8}  {:6.1}       {:8}       {:6.1}     {:8.0}",
            l.pipe,
            if l.abrupt { "sudden" } else { "growing" },
            l.peak,
            x.best,
            x.flow,
            x.away
        );
        found.push(x.away);
        random.push(x.random);
    }
    let (near, median) = summary(&mut found, &mut random);
    check("reference: 15 or more within 300 m", near >= 15);
    check("reference: median under 350 m", median < 350.0);

    println!("\n  2. Nights chosen by the night watch's own ALARMS (as the engine runs):");
    let flows = [
        table(&paths[4], ScadaUnit::CubicMetresPerHour),
        table(&paths[5], ScadaUnit::CubicMetresPerHour),
    ];
    let levels = [
        table(&paths[6], ScadaUnit::Metres),
        table(&paths[7], ScadaUnit::Metres),
    ];
    let e0 = flows[0].start.value();
    let w = watch(&town_nights(&flows, &levels), &LeakOptions::default()).unwrap();
    let matched = match_alarms(&w.alarms, &truth, e0);
    println!(
        "    alarm raised  matched leak (peak)  best junction  sized (alarm said)  distance [m]  to a leak that changed [m]"
    );
    // Each true leak's flow at night (02:00-04:00 mean) [m3/h].
    let leak_nights: Vec<Vec<f64>> = truth
        .iter()
        .map(|l| {
            leak_flows
                .iter()
                .flat_map(|tab| {
                    let n = tab.columns[0].values.len() / 288;
                    (0..n)
                        .map(|d| {
                            tab.column(&l.pipe)
                                .map_or(0.0, |c| mean(&c.values[d * 288 + 24..d * 288 + 48]) / M3H)
                        })
                        .collect::<Vec<_>>()
                })
                .collect()
        })
        .collect();
    let (mut found_any, mut random_any) = (Vec::new(), Vec::new());
    let (mut found, mut random) = (Vec::new(), Vec::new());
    for (a, m) in w.alarms.iter().zip(&matched) {
        let Some(i) = *m else { continue };
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let rose = ((a.first_night.value() - e0) / DAY).floor() as usize;
        if rose < 8 || rose + 8 > nights {
            continue;
        }
        // A week of evidence after the rise began; the week before it.
        // Scored against the matched leak - and against the nearest leak
        // whose flow truly changed by 3 m3/h or more between the two weeks
        // (the week can hold a new leak, a repair, another leak growing:
        // the locator finds where the pressure CHANGED).
        let changed: Vec<(f64, f64)> = (0..truth.len())
            .filter(|&k| {
                let f = &leak_nights[k];
                (mean(&f[rose..rose + 7]) - mean(&f[rose - 8..rose - 1])).abs() >= 3.0
            })
            .map(|k| t.pipe_at(&truth[k].pipe))
            .collect();
        let Some(x) = t.locate(
            (rose - 8, rose - 1),
            (rose, rose + 7),
            &[t.pipe_at(&truth[i].pipe)],
        ) else {
            continue;
        };
        let any = if changed.is_empty() {
            None
        } else {
            t.locate((rose - 8, rose - 1), (rose, rose + 7), &changed)
        };
        if let Some(y) = &any {
            found_any.push(y.away);
            random_any.push(y.random);
        }
        println!(
            "    {}    {:5} ({:4.1})        {:8}       {:5.1} ({:5.1})       {:8.0}    {}",
            date(a.raised.value() - e0),
            truth[i].pipe,
            truth[i].peak,
            x.best,
            x.flow,
            a.size.value() / M3H,
            x.away,
            any.map_or("no leak changed by 3 m3/h".to_string(), |y| format!(
                "{:.0}",
                y.away
            ))
        );
        found.push(x.away);
        random.push(x.random);
    }
    println!("  scored against the leak each alarm was matched to:");
    let (near, _) = summary(&mut found, &mut random);
    check(
        "from the alarms: 8 or more within 300 m of the matched leak",
        near >= 8,
    );
    println!("  scored against the nearest leak that truly changed in the two weeks:");
    let (near, median) = summary(&mut found_any, &mut random_any);
    check(
        "from the alarms: 12 or more within 300 m of a changed leak",
        near >= 12,
    );
    check("from the alarms: median under 400 m", median < 400.0);
}

fn mean(v: &[f64]) -> f64 {
    #[allow(clippy::cast_precision_loss)] // small
    let n = v.len().max(1) as f64;
    v.iter().sum::<f64>() / n
}
