//! Real data: the engine's answers next to what really happened
//! (`jk view --real`; the data: data/README.md; the model:
//! `view/real_model.rs`). Every number here is also checked by a test in
//! tests/*/datasets/.

use std::fmt::Write as _;

use jk_detection::Direction;

use super::ChartRecord;
use crate::sim::sites::Drawing;
use crate::view::charts::{Marker, network_map};
use crate::view::real_model::{LabModel, LeakModel, LeakPlaces, Place, RealModel, TownWeek};
use crate::view::shell::Page;
use crate::view::svg::esc;

const SOURCE: &str = "Real data";

fn missing(id: &'static str, title: &str, why: &str) -> Page {
    Page {
        id,
        title: title.into(),
        crumb: "not on this machine".into(),
        source: SOURCE,
        body: format!(
            r#"<div class="card"><h2>Data not found</h2><p class="note">{}</p></div>"#,
            esc(why)
        ),
    }
}

/// The three pages, in menu order.
pub fn pages(m: &RealModel) -> Vec<Page> {
    vec![
        m.lab
            .as_ref()
            .map_or_else(|e| missing("real-lab", "A real valve closure", e), lab_page),
        m.town.as_ref().map_or_else(
            |e| missing("real-town", "L-Town: a week of the town", e),
            town_page,
        ),
        m.leaks.as_ref().map_or_else(
            |e| missing("real-leaks", "L-Town: two years of leaks", e),
            leaks_page,
        ),
        match (&m.leaks, &m.places) {
            (Ok(l), Ok(p)) => where_page(l, p),
            (Err(e), _) | (_, Err(e)) => missing("real-where", "L-Town: where the leaks are", e),
        },
    ]
}

/// Chart records of the lab recordings.
pub fn lab_records(m: &RealModel) -> Vec<ChartRecord<'_>> {
    let Ok(lab) = &m.lab else { return Vec::new() };
    lab.traces
        .iter()
        .map(|t| ChartRecord {
            name: t.name.clone(),
            interval: t.interval,
            runs: vec![(0.0, t.heads.as_slice())],
            fronts: t.events.iter().map(|e| e.arrival.value()).collect(),
            impaired: Vec::new(),
            units: ("s", "m"),
            marks: Vec::new(),
            lines: Vec::new(),
        })
        .collect()
}

fn lab_page(lab: &LabModel) -> Page {
    let p2: Vec<_> = lab
        .traces
        .iter()
        .filter(|t| t.name.ends_with("P2"))
        .collect();
    let seen = p2.iter().filter(|t| t.closure).count();
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Closure seen (far sensor P2)</div><div class="value">{seen} of {}</div><div class="detail">a rise of 0.2 m or more, 16-23 s</div></div>
<div class="stat"><div class="label">False fronts while quiet</div><div class="value">{}</div><div class="detail">first 16 s of all 20 recordings (was 7-17 a run before FN-047)</div></div>
<div class="stat"><div class="label">Recordings</div><div class="value">20</div><div class="detail">10 runs x 2 sensors, 25 600 readings a second, averaged to 128</div></div>
</div>"#,
        p2.len(),
        lab.early.len()
    );
    s.push_str(r#"<div class="card"><h2>What this is</h2><p class="note">A real laboratory pipe network at Texas A&amp;M University: 47 m of 152 mm plastic pipe, looped (LO) or branched (BR), fed by a pump. In each run someone shut the outlet valve by hand at about second 20, so the flow (0.47 litres a second) stopped and the pressure rose. Two sensors: P1 at the inlet beside the pump, P2 at the far corner. The engine's detector, with its default settings, watched each recording as a 128-a-second logger would have recorded it. Lines mark the fronts it picked.</p><ul class="reasons"><li>These sensors measure pressure CHANGES, not the pressure itself - so the levels start near 0.</li><li>The closure is a pulse (up about a metre in half a second, then down below where it started), not a held step.</li><li>Each run's two files were started separately (their lengths differ by up to 3.6 s): they share no clock, so this data cannot test locating - only seeing.</li><li>On the looped layout the inlet sensor P1 shows almost nothing of the closure.</li></ul></div>"#);
    s.push_str(r#"<div class="card"><h2>What the detector saw</h2><div class="table-wrap"><table><thead><tr><th>Run</th><th>Sensor</th><th class="num">Noise unit</th><th>Fronts: time (step)</th></tr></thead><tbody>"#);
    for t in &lab.traces {
        let fronts = if t.events.is_empty() {
            "none".to_string()
        } else {
            t.events
                .iter()
                .map(|e| {
                    format!(
                        "{:.2} s ({:+.2} m {})",
                        e.arrival.value(),
                        e.step.value(),
                        if e.direction == Direction::Rise {
                            "rise"
                        } else {
                            "drop"
                        }
                    )
                })
                .collect::<Vec<_>>()
                .join(", ")
        };
        let (run, sensor) = t.name.split_once(' ').unwrap_or((&t.name, ""));
        let noise = t.events.first().map_or(String::from("-"), |e| {
            format!("{:.1} cm", e.noise.value() * 100.0)
        });
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{}</td><td class="num">{}</td><td>{}</td></tr>"#,
            esc(run),
            esc(sensor),
            esc(&noise),
            esc(&fronts)
        );
    }
    s.push_str("</tbody></table></div></div>");
    s.push_str(r#"<div class="card"><h2>The recordings</h2><p class="note">Each recording on one time axis, seconds from its own start. Drag across a chart to zoom; hover to read.</p><div class="js-traces" id="lab-traces" data-set="lab" data-from="0" data-to="37" data-zoomable></div></div>"#);
    Page {
        id: "real-lab",
        title: "A real valve closure".into(),
        crumb: "Texas A&M testbed, 20 recordings (FN-047)".into(),
        source: SOURCE,
        body: s,
    }
}

/// Chart records of the town's week.
pub fn town_records(m: &RealModel) -> Vec<ChartRecord<'_>> {
    let Ok(w) = &m.town else { return Vec::new() };
    let hours = w.interval / 3600.0;
    let mut out = vec![
        ChartRecord {
            name: "Tank T1 level - the engine (from the model alone)".into(),
            interval: hours,
            runs: vec![(0.0, w.tank.0.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("h", "m"),
            marks: Vec::new(),
            lines: vec![(3.9, "pump stops".into()), (2.4, "pump starts".into())],
        },
        ChartRecord {
            name: "Tank T1 level - published readings".into(),
            interval: hours,
            runs: vec![(0.0, w.tank.1.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("h", "m"),
            marks: Vec::new(),
            lines: vec![(3.9, "pump stops".into()), (2.4, "pump starts".into())],
        },
    ];
    for (id, engine, published) in &w.pressures {
        for (who, v) in [("the engine", engine), ("published", published)] {
            out.push(ChartRecord {
                name: format!("Pressure at {id} - {who}"),
                interval: hours,
                runs: vec![(0.0, v.as_slice())],
                fronts: Vec::new(),
                impaired: Vec::new(),
                units: ("h", "m"),
                marks: Vec::new(),
                lines: Vec::new(),
            });
        }
    }
    out
}

fn town_page(w: &TownWeek) -> Page {
    let gaps: Vec<f64> = w
        .switches
        .iter()
        .map(|s| (s.engine - s.published).abs())
        .collect();
    #[allow(clippy::cast_precision_loss)] // a handful
    let mean = gaps.iter().sum::<f64>() / gaps.len().max(1) as f64;
    let worst = gaps.iter().copied().fold(0.0, f64::max);
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Pump switches matched</div><div class="value">{}</div><div class="detail">same order, on and off alike</div></div>
<div class="stat"><div class="label">Apart, on average</div><div class="value">{mean:.1} h</div><div class="detail">largest {worst:.1} h, with no tuning</div></div>
<div class="stat"><div class="label">Pressures, engine vs published</div><div class="value">{:.2} m</div><div class="detail">apart on average over the week, all {} sensors; the worst sensor {:.2} m ({})</div></div>
<div class="stat"><div class="label">Town</div><div class="value">43 km</div><div class="detail">782 junctions, 905 pipes, 1 pump, 1 tank, 3 pressure valves</div></div>
</div>"#,
        w.switches.len(),
        w.pressure_fit.mean,
        w.pressure_fit.sensors,
        w.pressure_fit.worst,
        esc(&w.pressure_fit.worst_at)
    );
    s.push_str(r#"<div class="card"><h2>What this is</h2><p class="note">L-Town, the town of the BattLeDIM 2020 leak competition (University of Cyprus and partners): 43 km of pipe laid out on a real town. The engine read the model the organisers handed out and played the town's first week of 2018 from it alone - every 5 minutes, the tank filling and draining, the pump switched by the tank's level. Next to it, the readings the organisers published. They made those with a "real" model that differs from the handed-out one by up to 10% in demands and pipe data (on purpose: competitors had to cope), so the two should be close, not equal. The readings are themselves simulated by the organisers - real layout, not field measurements.</p></div>"#);
    s.push_str(r#"<div class="card"><h2>When the pump switched</h2><div class="table-wrap"><table><thead><tr><th>#</th><th>Switch</th><th class="num">Engine [h]</th><th class="num">Published [h]</th><th class="num">Difference [h]</th></tr></thead><tbody>"#);
    for (k, sw) in w.switches.iter().enumerate() {
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{}</td><td class="num">{:.2}</td><td class="num">{:.2}</td><td class="num">{:+.2}</td></tr>"#,
            k + 1,
            if sw.on { "on" } else { "off" },
            sw.engine,
            sw.published,
            sw.engine - sw.published
        );
    }
    s.push_str("</tbody></table></div></div>");
    s.push_str(r#"<div class="card"><h2>The week, side by side</h2><p class="note">Hours from 2018-01-01 00:00. The tank's sawtooth is the pump filling it and the town drawing it down; the pressures are three of the town's 33 pressure sensors (metres of water above ground). Drag to zoom; hover to read.</p><div class="js-traces" id="town-traces" data-set="town" data-unit="h" data-from="0" data-to="168" data-zoomable></div></div>"#);
    Page {
        id: "real-town",
        title: "L-Town: a week of the town".into(),
        crumb: "the engine from the model alone, next to the published readings (FN-045)".into(),
        source: SOURCE,
        body: s,
    }
}

/// Chart records of the two years of nights.
pub fn leak_records(m: &RealModel) -> Vec<ChartRecord<'_>> {
    let Ok(l) = &m.leaks else { return Vec::new() };
    // Few labels per chart, or they pile up: alarms on the flow, falls on
    // the evidence; the leaks' starts are in the table.
    let alarms: Vec<(f64, String)> = l
        .watch
        .alarms
        .iter()
        .map(|a| (l.day(a.raised), "alarm".into()))
        .collect();
    let falls: Vec<(f64, String)> = l
        .watch
        .falls
        .iter()
        .map(|f| (l.day(f.raised), "fall".into()))
        .collect();
    vec![
        ChartRecord {
            name: "L-Town night flow, 02:00-04:00 [m3/h]".into(),
            interval: 1.0,
            runs: vec![(0.0, l.nights.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("day", "m3/h"),
            marks: alarms,
            lines: Vec::new(),
        },
        ChartRecord {
            name: "Evidence of a rise (CUSUM, in spreads)".into(),
            interval: 1.0,
            runs: vec![(0.0, l.watch.cusum.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("day", ""),
            marks: falls,
            lines: vec![(6.0, "alarm level".into())],
        },
    ]
}

/// The date of a watch time (seconds since 1970, the town's clock).
fn date(t: jk_kernel::Time) -> String {
    jk_kernel::clock::stamp(t.value(), None)[..10].to_string()
}

fn leaks_page(l: &LeakModel) -> Page {
    let n = l.truth.len();
    let within = |days: f64, big: bool| {
        l.truth
            .iter()
            .filter(|t| !big || t.peak >= 15.0)
            .filter(|t| t.caught_after.is_some_and(|d| d <= days))
            .count()
    };
    let big = l.truth.iter().filter(|t| t.peak >= 15.0).count();
    let false_alarms = l.alarm_leaks.iter().filter(|c| c.is_empty()).count();
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Leaks caught within a month</div><div class="value">{} of {n}</div><div class="detail">{} of the {big} reaching 15 m3/h; within a week {}</div></div>
<div class="stat"><div class="label">Alarms</div><div class="value">{}</div><div class="detail">{false_alarms} false, in two years</div></div>
<div class="stat"><div class="label">Repairs seen</div><div class="value">{}</div><div class="detail">lasting falls in night flow</div></div>
<div class="stat"><div class="label">Smallest leak caught in a week</div><div class="value">{:.1} m3/h</div><div class="detail">nights vary by {:.1} m3/h about their weekday</div></div>
</div>"#,
        within(30.0, false),
        within(30.0, true),
        within(7.0, false),
        l.watch.alarms.len(),
        l.watch.falls.len(),
        l.watch.smallest_in_a_week.value() * 3600.0,
        l.watch.spread.value() * 3600.0
    );
    s.push_str(r#"<div class="card"><h2>What this is</h2><p class="note">L-Town's two years of meter readings (2018-2019, every 5 minutes) given to the engine's night watch exactly as a utility would have them: the two inflows from the reservoirs and the tank's level. Between 02:00 and 04:00 almost nobody uses water, so the night flow is mostly leakage; the watch raises an alarm when nights stay above what is expected of them, and reports a lasting fall (a repair). The organisers published the truth: 33 leaks, when each began and was repaired, how big. The first run caught 3 within a week and went blind for 341 days after a repair (FN-048); this is the watch after the fix.</p><ul class="reasons"><li>The whole town is one zone: leaks under about 8 m3/h cannot be told from how much nights vary - smaller metered zones can.</li><li>Holidays use water like no ordinary night: the nights that open and follow Cyprus's public holidays are not judged (G-24).</li><li>Scored one to one: each alarm is matched to the newest leak running when it was raised; L-Town nearly always has a leak running, so false alarms are judged in simulation instead (one per ~560 nights).</li></ul></div>"#);
    s.push_str(r#"<div class="card"><h2>Two years of nights</h2><p class="note">Days from 2018-01-01. The first chart marks the alarms, the second the lasting falls (repairs) the watch saw; when each true leak began is in the table below. Drag to zoom; hover to read.</p><div class="js-traces" id="leak-traces" data-set="leaks" data-unit="day" data-from="0" data-to="730" data-zoomable></div></div>"#);
    s.push_str(r#"<div class="card"><h2>The alarms</h2><div class="table-wrap"><table><thead><tr><th>#</th><th>Rise from</th><th>Raised on</th><th class="num">Size [m3/h]</th><th>The leak it is matched to</th></tr></thead><tbody>"#);
    for (k, (a, caught)) in l.watch.alarms.iter().zip(&l.alarm_leaks).enumerate() {
        let what = if caught.is_empty() {
            "none - a FALSE alarm".to_string()
        } else {
            caught
                .iter()
                .map(|&i| format!("{} ({:.0} m3/h)", l.truth[i].pipe, l.truth[i].peak))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{}</td><td>{}</td><td class="num">{:.1} &plusmn; {:.1}</td><td>{}</td></tr>"#,
            k + 1,
            esc(&date(a.first_night)),
            esc(&date(a.raised)),
            a.size.value() * 3600.0,
            a.uncertainty.value() * 3600.0,
            esc(&what)
        );
    }
    s.push_str("</tbody></table></div></div>");
    s.push_str(r#"<div class="card"><h2>The true leaks</h2><div class="table-wrap"><table><thead><tr><th>Pipe</th><th class="num">Began (day)</th><th>Kind</th><th class="num">Peak [m3/h]</th><th class="num">Caught after [days]</th></tr></thead><tbody>"#);
    for t in &l.truth {
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td class="num">{:.0}</td><td>{}</td><td class="num">{:.1}</td><td class="num">{}</td></tr>"#,
            esc(&t.pipe),
            t.start,
            if t.abrupt { "sudden" } else { "growing" },
            t.peak,
            t.caught_after
                .map_or("not caught".to_string(), |d| format!("{d:.0}"))
        );
    }
    s.push_str("</tbody></table></div></div>");
    Page {
        id: "real-leaks",
        title: "L-Town: two years of leaks".into(),
        crumb: "the night watch against the 33 true leaks (FN-048)".into(),
        source: SOURCE,
        body: s,
    }
}

/// A pipe at node `n` and how far along it the node is (to draw a point
/// at a junction).
fn at_node(p: &LeakPlaces, n: usize) -> Option<(usize, f64)> {
    p.inp
        .network
        .pipes()
        .iter()
        .enumerate()
        .find_map(|(k, pipe)| {
            if pipe.from.index() == n {
                Some((k, 0.0))
            } else if pipe.to.index() == n {
                Some((k, pipe.pipe.length().value()))
            } else {
                None
            }
        })
}

#[allow(clippy::too_many_lines)] // the page top to bottom
fn where_page(l: &LeakModel, p: &LeakPlaces) -> Page {
    let located: Vec<(usize, &Place)> = p
        .places
        .iter()
        .enumerate()
        .filter_map(|(k, x)| x.as_ref().map(|x| (k, x)))
        .collect();
    let changed: Vec<f64> = located.iter().filter_map(|(_, x)| x.to_changed).collect();
    let near = changed.iter().filter(|&&d| d <= 300.0).count();
    let mut sorted = changed.clone();
    sorted.sort_by(f64::total_cmp);
    let median = sorted.get(sorted.len() / 2).copied().unwrap_or(f64::NAN);
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Pointed within 300 m</div><div class="value">{near} of {}</div><div class="detail">of a leak that truly changed in those two weeks</div></div>
<div class="stat"><div class="label">Typical miss</div><div class="value">{median:.0} m</div><div class="detail">median; a junction picked at random: about 840 m</div></div>
<div class="stat"><div class="label">Town</div><div class="value">2.7 x 1.5 km</div><div class="detail">33 pressure sensors; 782 leak signatures in {:.1} s</div></div>
</div>"#,
        changed.len(),
        p.seconds
    );
    s.push_str(r#"<div class="card"><h2>What this is</h2><p class="note">The night watch says a leak began somewhere in the town; it cannot say where. L-Town also has 33 pressure sensors, read every 5 minutes. A leak draws water through the pipes to it, so the pressure falls most near it. The engine plays "a leak here" at each of the 782 junctions in turn and notes how much each sensor would fall - every spot's fingerprint. For each alarm it compares the sensors' night pressures in the week the rise began with the week before, and the fingerprint that fits best is where it points.</p><ul class="reasons"><li>Scored against a leak whose flow truly changed in those two weeks (the truth file's flows). Some alarms come when no leak changed that much - the alarm's timing, not the locator, then has nothing to point at.</li><li>Big, sudden leaks are found best; small, slowly growing ones barely move the sensors (FN-053).</li><li>Distances are straight lines on the map, in metres.</li></ul></div>"#);
    // The map: sensors, the true leaks matched, and where the engine pointed.
    let inp = &p.inp;
    let drawing = Drawing {
        positions: inp
            .positions
            .iter()
            .map(|q| q.unwrap_or((0.0, 0.0)))
            .collect(),
        node_names: inp.node_ids.clone(),
        pipe_names: inp.pipe_ids.clone(),
        bends: inp.vertices.clone(),
        y_up: true,
    };
    let mut markers: Vec<Marker> = p.sensors.iter().map(|&k| Marker::Logger(k)).collect();
    for (k, place) in &located {
        if let Some((pipe, distance)) = at_node(p, place.node) {
            markers.push(Marker::Point {
                pipe,
                distance,
                label: format!("#{}", k + 1),
                truth: false,
            });
        }
        if let Some(&i) = l.alarm_leaks[*k].first()
            && let Some(pipe) = inp.pipe_ids.iter().position(|x| *x == l.truth[i].pipe)
        {
            markers.push(Marker::Point {
                pipe,
                distance: inp.network.pipes()[pipe].pipe.length().value() / 2.0,
                label: l.truth[i].pipe.clone(),
                truth: true,
            });
        }
    }
    let tips: Vec<String> = inp.pipe_ids.iter().map(|id| format!("pipe {id}")).collect();
    let class = vec!["q5"; inp.network.pipes().len()];
    s.push_str(r#"<div class="card"><h2>The map</h2><p class="note">North up. Filled circles: the 33 pressure sensors. Rings (#n): where the engine pointed for alarm n. Crosses: the true leaking pipe that alarm was matched to. Hover a pipe for its name.</p>"#);
    s.push_str(&network_map(
        &inp.network,
        &drawing,
        &class,
        &tips,
        &markers,
        1000.0,
    ));
    s.push_str("</div>");
    s.push_str(r#"<div class="card"><h2>Alarm by alarm</h2><div class="table-wrap"><table><thead><tr><th>#</th><th>Raised on</th><th>Matched leak</th><th>Engine points to</th><th class="num">Sized [m3/h]</th><th class="num">To the matched leak [m]</th><th class="num">To a leak that changed [m]</th></tr></thead><tbody>"#);
    for (k, place) in &located {
        let a = &l.watch.alarms[*k];
        let matched = l.alarm_leaks[*k].first().map_or("-".to_string(), |&i| {
            format!("{} ({:.0} m3/h)", l.truth[i].pipe, l.truth[i].peak)
        });
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td class="num">{:.1}</td><td class="num">{:.0}</td><td class="num">{}</td></tr>"#,
            k + 1,
            esc(&date(a.raised)),
            esc(&matched),
            esc(&inp.node_ids[place.node]),
            place.flow,
            place.to_matched,
            place
                .to_changed
                .map_or("no leak changed".to_string(), |d| format!("{d:.0}"))
        );
    }
    s.push_str("</tbody></table></div></div>");
    Page {
        id: "real-where",
        title: "L-Town: where the leaks are".into(),
        crumb: "located from the 33 pressure sensors and the model (FN-053)".into(),
        source: SOURCE,
        body: s,
    }
}
