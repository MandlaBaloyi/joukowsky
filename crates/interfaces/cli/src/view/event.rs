//! Section 1: a burst, as the engine saw it.
//!
//! The flowing estate with two bursts is simulated; three synthetic
//! loggers record it; the records go to the real live [`Monitor`] in
//! 1-second chunks, as a site would send them. The section shows what the
//! engine said (the alerts, and when it said them), where it thinks each
//! burst is (a map per alert, with the true spot marked), and what each
//! logger recorded (with its detected fronts and the stretches it was not
//! watching at full sensitivity).

use std::fmt::Write as _;

use jk_application::{
    Alert, Chunk, CollectingSink, Level, Monitor, PipelineConfig, SensorSite, Site,
};
use jk_classification::Context;
use jk_kernel::Time;
use jk_kernel::random::Rng;
use jk_localisation::Localisation;

use super::charts::{Marker, TraceData, network_map, trace};
use super::svg::{esc, seq_class, to_f64};
use crate::sim::logger::{Logger, sample};
use crate::sim::sites::{BurstDemo, flowing_estate_two_bursts};

/// Logger rate [S/s].
const RATE: f64 = 128.0;

/// An alert and how far the data had come when it went out.
struct Issued {
    alert: Alert,
    data_time: f64,
}

/// Runs the scenario and returns the section's HTML.
///
/// # Errors
/// Only if the simulation or the pipeline fails (a bug).
#[allow(clippy::too_many_lines)] // one section, told top to bottom
pub fn section() -> Result<String, String> {
    let demo = flowing_estate_two_bursts()?;
    let names: Vec<String> = demo
        .loggers
        .iter()
        .map(|n| demo.site.drawing.node_names[n.index()].clone())
        .collect();

    // What the loggers record.
    let mut rng = Rng::new(3);
    let records: Vec<(f64, Vec<f64>)> = demo
        .truths
        .iter()
        .map(|truth| {
            let logger = Logger {
                rate: RATE,
                phase: rng.uniform() / RATE,
                noise: 0.02,
            };
            (logger.phase, sample(truth, demo.dt, &logger, &mut rng))
        })
        .collect();

    // The engine, live.
    let site = Site {
        network: demo.site.network.clone(),
        sensors: demo
            .loggers
            .iter()
            .zip(&names)
            .map(|(&node, name)| SensorSite {
                node,
                name: name.clone(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
            })
            .collect(),
        pipe_names: demo.site.drawing.pipe_names.clone(),
        context: Context::unknown(),
    };
    let mut monitor = Monitor::new(site, PipelineConfig::default()).map_err(|e| e.to_string())?;
    let mut sink = CollectingSink::default();
    let mut issued = Vec::new();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 128
    let per = RATE as usize;
    let rounds = records
        .iter()
        .map(|(_, x)| x.len().div_ceil(per))
        .max()
        .unwrap_or(0);
    for n in 0..rounds {
        for (sensor, (start, x)) in records.iter().enumerate() {
            let from = n * per;
            if from >= x.len() {
                continue;
            }
            let chunk = Chunk {
                start: Time::new(start + to_f64(from) / RATE),
                interval: Time::new(1.0 / RATE),
                samples: &x[from..(from + per).min(x.len())],
            };
            for alert in monitor
                .ingest(sensor, chunk, &mut sink)
                .map_err(|e| e.to_string())?
            {
                let data_time = monitor.lead().map_or(0.0, Time::value);
                issued.push(Issued { alert, data_time });
            }
        }
    }
    let detections = monitor.detections();
    let impaired: Vec<Vec<(f64, f64)>> = (0..demo.loggers.len())
        .map(|s| {
            monitor
                .impaired(s)
                .iter()
                .map(|(a, b)| (a.value(), b.value()))
                .collect()
        })
        .collect();
    let data_time = monitor.lead().map_or(0.0, Time::value);
    for alert in monitor.finish(&mut sink).map_err(|e| e.to_string())? {
        issued.push(Issued { alert, data_time });
    }

    // The section.
    let mut s = String::new();
    s.push_str(
        r"<h2>1. A burst, as the engine saw it</h2>
<p>A simulated estate: municipal supply R feeds a spine and two branches, consumers draw
15 l/s at the north and south blocks. A 10 mm hole opens 200 m along the south branch at
t = 3 s, and another 150 m along the north branch at t = 30 s. Three loggers (128 samples a
second, 2 cm noise) send their data every second to the live engine.</p>",
    );
    let _ = write!(
        s,
        r#"<p class="note">{} alert(s) went out. Everything below is simulated: the real test is real
data (validation rungs 4 and 5).</p>"#,
        issued.len()
    );
    for (k, i) in issued.iter().enumerate() {
        s.push_str(&alert_card(&i.alert, i.data_time, &demo, k));
    }

    // The records.
    s.push_str(
        r#"<div class="card"><h3>What each logger recorded</h3>
<p class="note">Pressure head at each logger. A labelled line marks each front the detector
picked; shaded stretches are when the logger was not watching at full sensitivity
(learning its noise, deaf just after a front, or listening through the ringing that
follows) - its silence there is not taken as "nothing happened". Hover for values.</p>"#,
    );
    let x_range = (0.0, 40.0);
    for (sensor, (start, x)) in records.iter().enumerate() {
        let times: Vec<f64> = (0..x.len()).map(|k| start + to_f64(k) / RATE).collect();
        let fronts: Vec<f64> = detections
            .iter()
            .filter(|(s, _)| *s == sensor)
            .map(|(_, e)| e.arrival.value())
            .collect();
        let _ = write!(s, "<h3>{}</h3>", esc(&names[sensor]));
        s.push_str(&trace(
            &TraceData {
                times: &times,
                heads: x,
                fronts: &fronts,
                impaired: &impaired[sensor],
            },
            x_range,
            1040.0,
            170.0,
        ));
    }
    s.push_str(
        r#"<div class="legend"><span class="key"><span class="swatch"></span>not watching at full sensitivity</span><span class="key">time axis in seconds; head in metres</span></div></div>"#,
    );
    Ok(s)
}

/// One alert: what, when, how sure, why - and its map.
fn alert_card(alert: &Alert, data_time: f64, demo: &BurstDemo, k: usize) -> String {
    let (status, colour) = match alert.level {
        Level::Alarm => ("Alarm", "#d03b3b"),
        Level::Warning => ("Warning", "#ec835a"),
        Level::Review => ("Review", "#fab219"),
        Level::Info => ("Info", "#898781"),
    };
    let revision = if alert.revision > 0 {
        format!(" (revision {})", alert.revision)
    } else {
        String::new()
    };
    let mut s = String::from(r#"<div class="card">"#);
    let _ = write!(
        s,
        r#"<div class="alert-head"><span class="badge"><span class="dot" style="background:{colour}"></span>{status}</span>
<span class="what">#{}{revision} {} - {:.0}% confident</span>
<span class="meta">started t = {:.3} s &middot; sent when the data reached t = {data_time:.2} s &middot; seen by {}</span></div>"#,
        alert.id,
        esc(&alert.class.to_string().to_uppercase()),
        alert.confidence * 100.0,
        alert.time.value(),
        esc(&alert.sensors.join(", "))
    );
    if let Some(loc) = &alert.location {
        s.push_str(&alert_map(alert, loc, demo));
    } else {
        s.push_str(r#"<p class="note">Not located.</p>"#);
    }
    let _ = write!(
        s,
        r#"<details{}><summary>Why</summary><ul class="reasons">"#,
        if k == 0 { " open" } else { "" }
    );
    for r in &alert.reasons {
        let _ = write!(s, "<li>{}</li>", esc(r));
    }
    s.push_str("</ul></details></div>");
    s
}

/// The map for a located alert: pipes by probability, the 90% search
/// stretch as a halo, the most likely point and the true burst.
fn alert_map(alert: &Alert, loc: &Localisation, demo: &BurstDemo) -> String {
    let mut s = String::new();
    let net = &demo.site.network;
    let names = &demo.site.drawing.pipe_names;
    let pmax = loc.pipes.iter().map(|p| p.probability).fold(0.0, f64::max);
    let mut class = vec!["q0"; net.pipes().len()];
    let mut tip = vec![String::new(); net.pipes().len()];
    for (p, name) in names.iter().enumerate() {
        tip[p] = format!("{name}|probability 0%");
    }
    for p in &loc.pipes {
        let i = p.pipe.index();
        class[i] = seq_class(p.probability / pmax.max(1e-12));
        tip[i] = format!("{}|probability {:.1}%", names[i], p.probability * 100.0);
    }
    let mut markers: Vec<Marker> = demo
        .loggers
        .iter()
        .map(|n| Marker::Logger(n.index()))
        .collect();
    for seg in &loc.credible_region {
        markers.push(Marker::Stretch {
            pipe: seg.pipe.index(),
            from: seg.from.value(),
            to: seg.to.value(),
        });
    }
    let ml = loc.most_likely;
    markers.push(Marker::Point {
        pipe: ml.pipe.index(),
        distance: ml.distance.value(),
        label: format!(
            "most likely: {:.1} m along {}",
            ml.distance.value(),
            names[ml.pipe.index()]
        ),
        truth: false,
    });
    // The true burst nearest in time.
    if let Some(b) = demo.bursts.iter().min_by(|a, b| {
        (a.time - alert.time.value())
            .abs()
            .total_cmp(&(b.time - alert.time.value()).abs())
    }) {
        markers.push(Marker::Point {
            pipe: b.pipe,
            distance: b.distance,
            label: format!("actual burst ({:.0} m along)", b.distance),
            truth: true,
        });
    }
    let _ = write!(
        s,
        r#"<p class="note">Where: {:.1} m along {}; the 90% search stretch (halo) is {:.0} m of pipe.</p>"#,
        ml.distance.value(),
        esc(&names[ml.pipe.index()]),
        loc.credible_length.value()
    );
    s.push_str(&network_map(
        net,
        &demo.site.drawing,
        &class,
        &tip,
        &markers,
        1040.0,
    ));
    s.push_str(
        r#"<div class="legend"><span class="key">pipes: <span class="ramp"><span class="q1"></span><span class="q2"></span><span class="q3"></span><span class="q4"></span><span class="q5"></span><span class="q6"></span><span class="q7"></span><span class="q8"></span></span> less to more likely</span><span class="key">&#9679; logger</span><span class="key">&#9675; most likely point</span><span class="key">&#10005; actual burst</span></div>"#,
    );
    s
}
