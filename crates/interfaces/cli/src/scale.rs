//! `jk scale`: time every part of the engine on synthetic towns of growing
//! size - the scale study V-S4 (`docs/10_findings/detection_range_and_scale.txt`).
//!
//! For each town: the steady state, a burst simulated for a few seconds,
//! locating that burst from the nearest loggers (what a live alert costs),
//! and scoring a logger placement (what the advisor repeats hundreds of
//! times). Build with `--release`: debug builds are 10-30x slower.

use std::time::Instant;

use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, NodeId, Probe, SteadyOptions,
};
use jk_kernel::{Area, Head, Length, Time};
use jk_localisation::graph::TravelGraph;
use jk_localisation::placement::{PlacementOptions, advise, advise_zoned, coverage};
use jk_localisation::{Arrival, LocaliseOptions, Localiser};
use jk_sim::{Town, TownOptions, town};

pub const USAGE: &str = "
    jk scale [--km 2,10,50,100] [--seed N] [--loggers-per-km X] [--no-placement]
             [--advise] [--zone-km X]
                          time every part of the engine on synthetic towns
                          of growing size (build with --release); --advise
                          also runs the placement advisor in full, zone by
                          zone (about X km of pipe per zone, default 5;
                          0: the whole town at once)
";

/// Loggers near the burst that see it in the live-alert timing.
const NEAREST_LOGGERS: usize = 4;
/// Seconds of transient simulated per town.
const SIMULATED: f64 = 5.0;

/// What `jk scale` was asked for.
pub struct Args {
    km: Vec<f64>,
    seed: u64,
    loggers_per_km: f64,
    placement: bool,
    advise: bool,
    zone_km: f64,
}

/// Reads the arguments after `scale`.
///
/// # Errors
/// An unknown option or a value that is not a positive number.
pub fn parse(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        km: vec![2.0, 10.0, 50.0, 100.0],
        seed: 1,
        loggers_per_km: 1.0,
        placement: true,
        advise: false,
        zone_km: 5.0,
    };
    let number = |v: Option<String>, what: &str| -> Result<f64, String> {
        let v = v.ok_or(format!("{what} needs a value"))?;
        v.parse::<f64>()
            .ok()
            .filter(|x| x.is_finite() && *x > 0.0)
            .ok_or(format!("{what}: '{v}' is not a positive number"))
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--km" => {
                let v = it.next().ok_or("--km needs a list, e.g. 2,10,50")?;
                a.km = v
                    .split(',')
                    .map(|x| number(Some(x.trim().to_string()), "--km"))
                    .collect::<Result<_, _>>()?;
            }
            "--seed" => {
                let v = it.next().ok_or("--seed needs a number")?;
                a.seed = v
                    .parse()
                    .map_err(|_| format!("--seed: '{v}' is not a whole number"))?;
            }
            "--loggers-per-km" => a.loggers_per_km = number(it.next(), "--loggers-per-km")?,
            "--no-placement" => a.placement = false,
            "--advise" => a.advise = true,
            "--zone-km" => {
                let v = it.next().ok_or("--zone-km needs a number")?;
                a.zone_km = v
                    .parse::<f64>()
                    .ok()
                    .filter(|x| x.is_finite() && *x >= 0.0)
                    .ok_or(format!("--zone-km: '{v}' is not a number >= 0"))?;
            }
            _ => return Err(format!("unexpected argument '{arg}'")),
        }
    }
    Ok(a)
}

/// Runs the study and prints what each part cost.
///
/// # Errors
/// A town that cannot be generated.
pub fn run(a: &Args) -> Result<(), String> {
    if cfg!(debug_assertions) {
        println!("NOTE: debug build - times are 10-30x slower than --release.\n");
    }
    for &km in &a.km {
        study(km, a)?;
    }
    Ok(())
}

fn study(km: f64, a: &Args) -> Result<(), String> {
    let clock = Instant::now();
    let t = town(&TownOptions {
        length: Length::new(km * 1000.0),
        seed: a.seed,
        ..TownOptions::default()
    })
    .map_err(|e| e.to_string())?;
    let net = &t.network;
    let made = clock.elapsed().as_secs_f64();
    println!(
        "=== Town of {km} km: {} pipes, {} junctions, {} reservoir(s) (made in {})",
        net.pipes().len(),
        t.hydrants.len(),
        t.reservoirs.len(),
        secs(made)
    );

    // 1. Steady state.
    let clock = Instant::now();
    match net.steady_state(&SteadyOptions::default()) {
        Ok(s) => {
            let lowest = s
                .node_heads
                .iter()
                .map(|h| h.value())
                .fold(f64::INFINITY, f64::min);
            println!(
                "  steady state          {:>10}   {} Newton rounds; lowest head {lowest:.1} m",
                secs(clock.elapsed().as_secs_f64()),
                s.iterations
            );
        }
        Err(e) => println!(
            "  steady state          FAILED after {}: {e}",
            secs(clock.elapsed().as_secs_f64())
        ),
    }

    // 2. A burst in the middle of town, simulated.
    let burst_at = middle(&t);
    let loggers = nearest(net, burst_at, NEAREST_LOGGERS, &t.hydrants);
    simulate(&t, burst_at, &loggers);

    // 3. Locating it from the nearest loggers (a live alert).
    locate(net, burst_at, &loggers);

    // 4. Scoring a placement of loggers spread over the town.
    if a.placement {
        place(&t, a.loggers_per_km * km, a.advise, a.zone_km);
    }
    println!();
    Ok(())
}

fn simulate(t: &Town, burst_at: NodeId, loggers: &[NodeId]) {
    let mut net = t.network.clone();
    let hole = Burst {
        node: burst_at,
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.01)), net.gravity()),
        outside_head: Head::ZERO,
        start: Time::new(1.0),
        opening: BurstOpening::Instantaneous,
    };
    if let Err(e) = net.add_burst(hole) {
        println!("  burst simulation      FAILED: {e}");
        return;
    }
    let probes: Vec<Probe> = loggers.iter().map(|&n| Probe::Node(n)).collect();
    let clock = Instant::now();
    match net.simulate(
        &InitialCondition::Steady(SteadyOptions::default()),
        Time::new(SIMULATED),
        &probes,
        &GridOptions::default(),
    ) {
        Ok(run) => {
            let took = clock.elapsed().as_secs_f64();
            let points: u64 = run.grid.reaches().iter().map(|&r| u64::from(r) + 1).sum();
            println!(
                "  burst simulation      {:>10}   {SIMULATED} s simulated: dt {:.2} ms, {points} grid points ({:.1} s per simulated second)",
                secs(took),
                run.grid.time_step().value() * 1000.0,
                took / SIMULATED
            );
        }
        Err(e) => println!(
            "  burst simulation      FAILED after {}: {e}",
            secs(clock.elapsed().as_secs_f64())
        ),
    }
}

fn locate(net: &Network, burst_at: NodeId, loggers: &[NodeId]) {
    let graph = TravelGraph::new(net);
    let nominal: Vec<f64> = net
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let times = graph.times_from(burst_at.index(), &nominal);
    let arrivals: Vec<Arrival> = loggers
        .iter()
        .map(|&n| Arrival {
            node: n,
            time: Time::new(1.0 + times[n.index()]),
            uncertainty: Time::new(1e-3),
        })
        .collect();
    let options = LocaliseOptions {
        wave_speed_uncertainty: 0.1,
        ..LocaliseOptions::default()
    };
    let clock = Instant::now();
    let localiser = match Localiser::new(net, loggers, &options) {
        Ok(l) => l,
        Err(e) => {
            println!("  locate (prepare)      FAILED: {e}");
            return;
        }
    };
    let prepared = clock.elapsed().as_secs_f64();
    let clock = Instant::now();
    match localiser.locate(&arrivals) {
        Ok(l) => println!(
            "  locate (live alert)   {:>10}   prepare {} + locate {} on {} candidate points; 90% region {:.0} m",
            secs(prepared + clock.elapsed().as_secs_f64()),
            secs(prepared),
            secs(clock.elapsed().as_secs_f64()),
            candidate_points(net, options.spacing.value()),
            l.credible_length.value()
        ),
        Err(e) => println!("  locate                FAILED: {e}"),
    }
}

fn place(t: &Town, count: f64, advise_too: bool, zone_km: f64) {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small, positive
    let count = (count.round() as usize).clamp(2, t.hydrants.len());
    let loggers = spread(&t.network, &t.hydrants, count);
    let options = PlacementOptions::default();
    let clock = Instant::now();
    match coverage(&t.network, &loggers, None, &options) {
        Ok(c) => println!(
            "  score a placement     {:>10}   {count} loggers spread out: {:.0}% pinpointed, mean 90% region {:.0} m",
            secs(clock.elapsed().as_secs_f64()),
            c.pinpointed * 100.0,
            c.expected_region.value()
        ),
        Err(e) => println!("  score a placement     FAILED: {e}"),
    }
    if !advise_too {
        return;
    }
    let clock = Instant::now();
    let result = if zone_km > 0.0 {
        advise_zoned(
            &t.network,
            &t.hydrants,
            count,
            &[],
            None,
            &options,
            Length::new(zone_km * 1000.0),
        )
        .map(|z| (z.best, z.zones.len()))
    } else {
        advise(&t.network, &t.hydrants, count, &[], None, &options).map(|a| (a.best, 1))
    };
    match result {
        Ok((best, zones)) => println!(
            "  advise                {:>10}   chose {count} of {} hydrants in {zones} zone(s): {:.0}% pinpointed, mean 90% region {:.0} m",
            secs(clock.elapsed().as_secs_f64()),
            t.hydrants.len(),
            best.pinpointed * 100.0,
            best.expected_region.value()
        ),
        Err(e) => println!("  advise                FAILED: {e}"),
    }
}

/// The hydrant nearest the middle of the town's plan.
fn middle(t: &Town) -> NodeId {
    let n = t.hydrants.len();
    let (cx, cy) = t.hydrants.iter().fold((0.0, 0.0), |(x, y), h| {
        let p = t.positions[h.index()];
        (x + p.0 / to_f64(n), y + p.1 / to_f64(n))
    });
    *t.hydrants
        .iter()
        .min_by(|a, b| {
            let d = |h: &NodeId| {
                let p = t.positions[h.index()];
                (p.0 - cx).hypot(p.1 - cy)
            };
            d(a).total_cmp(&d(b))
        })
        .expect("a town has hydrants")
}

/// The `k` hydrants a wave from `source` reaches first (not `source`).
fn nearest(net: &Network, source: NodeId, k: usize, hydrants: &[NodeId]) -> Vec<NodeId> {
    let graph = TravelGraph::new(net);
    let nominal: Vec<f64> = net
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let times = graph.times_from(source.index(), &nominal);
    let mut by_time: Vec<NodeId> = hydrants.iter().copied().filter(|&h| h != source).collect();
    by_time.sort_by(|a, b| times[a.index()].total_cmp(&times[b.index()]));
    by_time.truncate(k);
    by_time
}

/// `count` hydrants spread out: each next one is the hydrant furthest (by
/// wave travel) from those already chosen.
fn spread(net: &Network, hydrants: &[NodeId], count: usize) -> Vec<NodeId> {
    let graph = TravelGraph::new(net);
    let nominal: Vec<f64> = net
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let mut chosen = vec![hydrants[0]];
    let mut gap = graph.times_from(hydrants[0].index(), &nominal);
    while chosen.len() < count {
        let next = *hydrants
            .iter()
            .max_by(|a, b| gap[a.index()].total_cmp(&gap[b.index()]))
            .expect("hydrants");
        chosen.push(next);
        for (g, t) in gap.iter_mut().zip(graph.times_from(next.index(), &nominal)) {
            *g = g.min(t);
        }
    }
    chosen
}

fn candidate_points(net: &Network, spacing: f64) -> usize {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // ceil of positive
    net.pipes()
        .iter()
        .map(|p| ((p.pipe.length().value() / spacing).ceil() as usize).max(1))
        .sum()
}

fn secs(s: f64) -> String {
    if s < 1e-3 {
        format!("{:.0} us", s * 1e6)
    } else if s < 1.0 {
        format!("{:.0} ms", s * 1e3)
    } else if s < 120.0 {
        format!("{s:.1} s")
    } else if s < 7200.0 {
        format!("{:.1} min", s / 60.0)
    } else if s < 172_800.0 {
        format!("{:.1} h", s / 3600.0)
    } else {
        format!("{:.0} days", s / 86400.0)
    }
}

#[allow(clippy::cast_precision_loss)] // small counts
fn to_f64(i: usize) -> f64 {
    i as f64
}
