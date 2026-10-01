//! V-X1: the estate's response to a reservoir level oscillating at one
//! frequency - the method of characteristics (time steps on a grid)
//! against the transfer-matrix model (exact algebra per frequency).
//!
//! The reservoir R oscillates by 0.5 m about 60 m, eased in over 30
//! periods (so the network's own modes are hardly excited - with no
//! damping at rest, whatever is excited rings for ever; 10 periods left
//! 1% of a nearby mode in the response at 0.71 Hz); after it, the
//! simulator's response at S1, S2, S3 is read by Fourier projection over
//! 60 whole periods and compared - amplitude and phase - with the
//! transfer-matrix prediction on the same (grid-adjusted) wave speeds.
//! At rest, friction is of second order: both models are frictionless to
//! first order, so they must agree closely.

use jk_hydraulics::frequency::response;
use jk_hydraulics::{GridOptions, InitialCondition, Probe, SourceLevel};
use jk_kernel::{Head, Time};
use jk_tests::check;
use jk_tests::estate::estate;

const AMPLITUDE: f64 = 0.5;

/// Complex amplitudes (re, im), one per logger.
type Amplitudes = Vec<(f64, f64)>;

/// The simulator's complex amplitude at each sensor (per metre at R), and
/// the transfer-matrix's, at `f` Hz.
fn both(f: f64) -> (Amplitudes, Amplitudes) {
    let (reaches, ease_periods, points_per_period) = (30, 30.0, 60.0);
    let (e, _) = estate(None);
    let mut net = e.net;
    let r = net.node_id(0).expect("R is node 0");
    let period = 1.0 / f;
    let (ease, steady) = (ease_periods * period, 60.0 * period);
    let total = ease + steady + 2.0 * period;
    let points = (total / period * points_per_period).ceil();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let history = (0..=points as usize)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let t = k as f64 * total / points;
            let w = if t < ease {
                0.5 * (1.0 - (std::f64::consts::PI * t / ease).cos())
            } else {
                1.0
            };
            (
                Time::new(t),
                Head::new(60.0 + AMPLITUDE * w * (std::f64::consts::TAU * f * t).sin()),
            )
        })
        .collect();
    net.add_level(SourceLevel { node: r, history }).unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(total),
            &e.sensors.map(Probe::Node),
            &GridOptions {
                max_wave_speed_adjustment: 0.001,
                min_reaches: reaches,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    // Fourier projection over 60 whole periods after the easing.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let (k0, n) = ((ease / dt).round() as usize, (steady / dt).round() as usize);
    let w = std::f64::consts::TAU * f;
    let simulated = (0..3)
        .map(|j| {
            let h: Vec<f64> = run.trace.head(j).iter().map(|h| h.value()).collect();
            let (mut c, mut s) = (0.0, 0.0);
            for (k, x) in h.iter().enumerate().skip(k0).take(n) {
                #[allow(clippy::cast_precision_loss)]
                let t = k as f64 * dt;
                c += (x - 60.0) * (w * t).cos();
                s += (x - 60.0) * (w * t).sin();
            }
            #[allow(clippy::cast_precision_loss)]
            let scale = 2.0 / n as f64 / AMPLITUDE;
            // h = Re{H e^{iwt}} with the drive sin(wt) = Re{-i e^{iwt}}:
            // H = (c - i s) scale; relative to the drive: H / (-i).
            let (re, im) = (c * scale, -s * scale);
            (-im, re) // divide by -i
        })
        .collect();
    let speeds: Vec<f64> = run.grid.wave_speeds().iter().map(|a| a.value()).collect();
    let tm = response(&net, r, f, Some(&speeds), None).unwrap();
    let model = e
        .sensors
        .iter()
        .map(|s| (tm[s.index()].re, tm[s.index()].im))
        .collect();
    (simulated, model)
}

#[test]
fn vx1_the_simulator_and_the_transfer_matrix_agree_on_the_estate() {
    println!(
        "The estate, its reservoir oscillating by {AMPLITUDE} m: response per metre at R, \
         method of characteristics (MOC) vs transfer matrix (TM)."
    );
    println!("  f [Hz]  logger   |MOC|    |TM|     phase MOC  phase TM   off by");
    let mut worst: f64 = 0.0;
    let mut compared = 0;
    for f in [0.11, 0.17, 0.23, 0.29, 0.37, 0.45, 0.53, 0.61, 0.71, 0.83] {
        let (sim, model) = both(f);
        let peak = model.iter().map(|b| b.0.hypot(b.1)).fold(0.0, f64::max);
        if peak > 3.0 {
            println!(
                "  {f:5.2}   near a natural frequency (response {peak:.1}x): skipped - with no damping \n                 at rest the network's own ringing never dies and beats with the forced response"
            );
            continue;
        }
        compared += 1;
        for (j, (a, b)) in sim.iter().zip(&model).enumerate() {
            let (ma, mb) = (a.0.hypot(a.1), b.0.hypot(b.1));
            let off = (a.0 - b.0).hypot(a.1 - b.1) / mb.max(1.0);
            worst = worst.max(off);
            println!(
                "  {f:5.2}   S{}     {ma:7.3}  {mb:7.3}   {:8.1}   {:8.1}   {off:.1e}",
                j + 1,
                a.1.atan2(a.0).to_degrees(),
                b.1.atan2(b.0).to_degrees()
            );
        }
    }
    check(
        &format!("  {compared} frequencies compared (at least 5)"),
        compared >= 5,
    );
    check(
        &format!("  the two methods agree within 0.5% (worst {worst:.1e})"),
        worst < 0.005,
    );
}
