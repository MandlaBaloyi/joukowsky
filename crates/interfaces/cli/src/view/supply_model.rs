//! The supply demos for the viewer: a tank running dry during a bulk
//! supply outage (extended-period run), and a metered zone's night watch
//! catching a slow leak (water balance). Both simulated here, once.

use jk_balance::{
    Direction, LeakOptions, Meter, Night, NightWindow, Watch, balance, night_flows, watch,
};
use jk_hydraulics::steady::extended::{PeriodOptions, SupplyEvent};
use jk_hydraulics::{Network, Node, Pipe, TankStorage};
use jk_kernel::random::Rng;
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time, WaveSpeed};

const HOUR: f64 = 3600.0;

/// A tank feeding an estate through a bulk-supply outage.
pub struct SupplyModel {
    /// Hours since the outage began, one per step.
    pub hours: Vec<f64>,
    /// Tank level [m].
    pub level: Vec<f64>,
    /// Head at the far end of the estate [m] (NaN once supply is lost).
    pub far_head: Vec<f64>,
    /// The tank's floor and ceiling [m].
    pub floor: f64,
    pub ceiling: f64,
    /// Supply events: (hours, words).
    pub events: Vec<(f64, String)>,
    /// Hours the run was asked to cover (it stops early if supply is lost).
    pub duration: f64,
    /// Average demand [l/s].
    pub demand: f64,
    /// Usable volume at the start [m^3].
    pub volume: f64,
}

impl SupplyModel {
    /// Runs the outage: the municipal feed is off, the estate's tank (400
    /// m^2, starting 6 m above its floor) supplies 18 l/s on average, with
    /// a morning and an evening peak.
    ///
    /// # Errors
    /// Only if the network or run is invalid (a bug).
    pub fn run() -> Result<Self, String> {
        let err = |e: jk_hydraulics::HydraulicsError| e.to_string();
        let pipe = |l: f64, d: f64| {
            Pipe::new(Length::new(l), Length::new(d), WaveSpeed::new(1000.0), 0.02)
                .map_err(|e| e.to_string())
        };
        let mut net = Network::new(STANDARD_GRAVITY);
        let tank = net.add_node(Node::Reservoir {
            head: Head::new(46.0),
        });
        let per = 0.006; // three blocks, 6 l/s each on average
        let j = |q: f64| Node::Junction {
            demand: Flow::new(q),
        };
        let (a, b, c) = (
            net.add_node(j(per)),
            net.add_node(j(per)),
            net.add_node(j(per)),
        );
        net.add_pipe(tank, a, pipe(800.0, 0.25)?).map_err(err)?;
        net.add_pipe(a, b, pipe(600.0, 0.2)?).map_err(err)?;
        net.add_pipe(b, c, pipe(700.0, 0.15)?).map_err(err)?;
        let (floor, ceiling) = (40.0, 48.0);
        net.add_tank(TankStorage {
            node: tank,
            area: Area::new(400.0),
            floor: Head::new(floor),
            ceiling: Head::new(ceiling),
        })
        .map_err(err)?;
        // Night low, morning and evening peaks (average 1).
        let pattern = vec![
            0.4, 0.3, 0.3, 0.3, 0.4, 0.7, 1.5, 1.9, 1.6, 1.2, 1.0, 1.0, 1.1, 1.0, 0.9, 0.9, 1.0,
            1.3, 1.7, 1.8, 1.5, 1.1, 0.7, 0.5,
        ];
        let norm = pattern.iter().sum::<f64>() / 24.0;
        let pattern: Vec<f64> = pattern.iter().map(|m| m / norm).collect();
        let run = net
            .extended(&PeriodOptions {
                duration: Time::new(48.0 * HOUR),
                pattern,
                ..PeriodOptions::default()
            })
            .map_err(err)?;
        let hours = run.times.iter().map(|t| t.value() / HOUR).collect();
        let level = run.tanks[0].1.iter().map(|h| h.value()).collect();
        let far_head = run
            .node_heads
            .iter()
            .map(|h| h[c.index()].value())
            .collect();
        let events = run
            .events
            .iter()
            .map(|e| match e {
                SupplyEvent::TankEmpty { at, .. } => (at.value() / HOUR, "tank empty".into()),
                SupplyEvent::TankRefilling { at, .. } => {
                    (at.value() / HOUR, "tank refilling".into())
                }
                SupplyEvent::TankFull { at, .. } => (at.value() / HOUR, "tank full".into()),
                SupplyEvent::SupplyLost { at, .. } => (at.value() / HOUR, "supply lost".into()),
            })
            .collect();
        Ok(Self {
            hours,
            level,
            far_head,
            floor,
            ceiling,
            events,
            duration: 48.0,
            demand: 3.0 * per * 1000.0,
            volume: (46.0 - floor) * 400.0,
        })
    }
}

/// A metered zone's nights, watched for a slow leak.
pub struct NightModel {
    /// The night flows [l/s], one per night (NaN if missing).
    pub nights: Vec<f64>,
    /// What the watch found.
    pub watch: Watch,
    /// The night the simulated leak began, and its size [l/s].
    pub leak_night: usize,
    pub leak: f64,
    /// Nights whose data was lost (a logger's battery).
    pub missing: Vec<usize>,
    /// Local midnight before the first night [s since 1970].
    start: f64,
}

impl NightModel {
    /// 90 days of 15-minute readings on a zone with an inflow and an
    /// outflow meter; a 1.2 l/s leak starts on day 60; four nights are lost
    /// to a logger battery.
    ///
    /// # Errors
    /// Only if the series are invalid (a bug).
    pub fn run() -> Result<Self, String> {
        let days = 90;
        let (leak_day, leak) = (60, 0.0012);
        let per_day = 96;
        let start = 1_788_220_800.0 - 7200.0; // local midnight, 2026-09-01
        let mut rng = Rng::new(11);
        let n = days * per_day;
        let (mut inflow, mut outflow) = (Vec::with_capacity(n), Vec::with_capacity(n));
        let mut night_use = 0.002;
        let missing: Vec<usize> = vec![33, 34, 71, 72];
        for k in 0..n {
            let day = k / per_day;
            #[allow(clippy::cast_precision_loss)] // under 96
            let hour = (k % per_day) as f64 / 4.0;
            if k % per_day == 0 {
                night_use = 0.002 + 0.0003 * rng.normal();
            }
            let peaks =
                (-((hour - 7.0) / 2.0).powi(2)).exp() + (-((hour - 19.0) / 2.5).powi(2)).exp();
            let leaking = if day > leak_day || (day == leak_day && hour >= 12.0) {
                leak
            } else {
                0.0
            };
            let into = night_use + 0.023 * peaks + leaking + 0.010;
            let lost = missing.contains(&day) && (1.0..5.0).contains(&hour);
            inflow.push(if lost {
                f64::NAN
            } else {
                into * (1.0 + 0.01 * rng.normal())
            });
            outflow.push(0.010 * (1.0 + 0.01 * rng.normal()));
        }
        let meters = [
            Meter {
                name: "bulk in".into(),
                direction: Direction::In,
                flows: inflow,
            },
            Meter {
                name: "to next zone".into(),
                direction: Direction::Out,
                flows: outflow,
            },
        ];
        let b = balance(Time::new(start), Time::new(900.0), &meters, None)
            .map_err(|e| e.to_string())?;
        let nights: Vec<Night> =
            night_flows(&b, NightWindow::default()).map_err(|e| e.to_string())?;
        let w = watch(&nights, &LeakOptions::default()).map_err(|e| e.to_string())?;
        Ok(Self {
            nights: nights.iter().map(|n| n.flow * 1000.0).collect(),
            watch: w,
            leak_night: leak_day + 1,
            leak: leak * 1000.0,
            missing,
            start,
        })
    }

    /// The night index of a time.
    pub fn night_of(&self, t: Time) -> f64 {
        ((t.value() - self.start) / 86_400.0).floor()
    }
}
