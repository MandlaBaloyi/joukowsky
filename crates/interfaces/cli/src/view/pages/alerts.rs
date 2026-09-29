//! Alerts: a list, and every alert in full - facts, map, the arrival at
//! each logger (the time differences the engine located it from), why.

use std::fmt::Write as _;

use jk_localisation::Localisation;

use super::{badge, class_name, ramp_legend};
use crate::view::charts::{Marker, network_map};
use crate::view::model::{EventModel, Issued};
use crate::view::shell::Page;
use crate::view::svg::{esc, seq_class};

pub fn page(event: &EventModel) -> Page {
    let mut s = String::from(r#"<div class="master"><div class="list">"#);
    for i in &event.issued {
        let a = &i.alert;
        let revision = if a.revision > 0 {
            format!(" (rev. {})", a.revision)
        } else {
            String::new()
        };
        let _ = write!(
            s,
            r##"<a class="alert-item" href="#alerts/{id}" data-alert="{id}"><div class="t1">{}<span class="note">t = {:.2} s</span></div><div class="t2"><b>#{id}{revision} {}</b> - {:.0}% confident</div></a>"##,
            badge(a.level),
            a.time.value(),
            esc(&class_name(a)),
            a.confidence * 100.0,
            id = a.id
        );
    }
    s.push_str("</div><div>");
    for i in &event.issued {
        s.push_str(&detail(event, i));
    }
    if event.issued.is_empty() {
        s.push_str(r#"<div class="card"><p>No alerts.</p></div>"#);
    }
    s.push_str("</div></div>");
    Page {
        id: "alerts",
        title: "Alerts".into(),
        crumb: format!("{} alert(s) from the live engine", event.issued.len()),
        body: s,
    }
}

/// One alert in full.
fn detail(event: &EventModel, issued: &Issued) -> String {
    let a = &issued.alert;
    let names = &event.demo.site.drawing.pipe_names;
    let mut s = format!(
        r#"<div class="alert-detail" data-alert="{}" hidden><div class="card">"#,
        a.id
    );
    let _ = write!(
        s,
        r#"<div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap">{}<h2 style="margin:0">#{} {}{} - {:.0}% confident</h2></div>"#,
        badge(a.level),
        a.id,
        esc(&class_name(a)),
        if a.revision > 0 {
            format!(" (revision {})", a.revision)
        } else {
            String::new()
        },
        a.confidence * 100.0
    );
    let (place, stretch) =
        a.location
            .as_ref()
            .map_or(("not located".to_string(), "-".to_string()), |l| {
                (
                    format!(
                        "{:.1} m along {}",
                        l.most_likely.distance.value(),
                        names[l.most_likely.pipe.index()]
                    ),
                    format!("{:.0} m of pipe", l.credible_length.value()),
                )
            });
    let _ = write!(
        s,
        r#"<div class="facts">
<div><div class="k">Started</div><div class="v">{:.3} s</div></div>
<div><div class="k">Alert out (data time)</div><div class="v">{:.2} s</div></div>
<div><div class="k">Time to alert</div><div class="v">{:.1} s</div></div>
<div><div class="k">Most likely</div><div class="v">{}</div></div>
<div><div class="k">90% search stretch</div><div class="v">{}</div></div>
<div><div class="k">Seen by</div><div class="v">{}</div></div>
</div></div>"#,
        a.time.value(),
        issued.data_time,
        issued.data_time - a.time.value(),
        esc(&place),
        esc(&stretch),
        esc(&a.sensors.join(", "))
    );
    if let Some(loc) = &a.location {
        s.push_str(&map_card(event, a.time.value(), loc));
    }
    s.push_str(&arrivals_card(event, issued));
    s.push_str(r#"<div class="card"><h2>Why</h2><ul class="reasons">"#);
    for r in &a.reasons {
        let _ = write!(s, "<li>{}</li>", esc(r));
    }
    s.push_str("</ul></div></div>");
    s
}

/// The map: pipes by probability, the 90% stretch, estimate and truth.
fn map_card(event: &EventModel, time: f64, loc: &Localisation) -> String {
    let demo = &event.demo;
    let net = &demo.site.network;
    let names = &demo.site.drawing.pipe_names;
    let pmax = loc.pipes.iter().map(|p| p.probability).fold(0.0, f64::max);
    let mut class = vec!["q0"; net.pipes().len()];
    let mut tip: Vec<String> = names
        .iter()
        .map(|n| format!("{n}|probability 0%"))
        .collect();
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
        label: format!("most likely: {:.1} m along", ml.distance.value()),
        truth: false,
    });
    let truth = demo
        .bursts
        .iter()
        .min_by(|x, y| (x.time - time).abs().total_cmp(&(y.time - time).abs()));
    let mut check = String::new();
    if let Some(b) = truth {
        markers.push(Marker::Point {
            pipe: b.pipe,
            distance: b.distance,
            label: format!("actual burst ({:.0} m along)", b.distance),
            truth: true,
        });
        check = if b.pipe == ml.pipe.index() {
            format!(
                "Check against the simulation: the burst really was {:.0} m along {} - the estimate is {:.1} m off.",
                b.distance,
                names[b.pipe],
                (ml.distance.value() - b.distance).abs()
            )
        } else {
            format!(
                "Check against the simulation: the burst really was {:.0} m along {} - a different pipe.",
                b.distance, names[b.pipe]
            )
        };
    }
    let mut s = String::from(
        r#"<div class="card"><h2>Where</h2><p class="note">Pipes coloured by how likely the burst is on them; the halo is the 90% search stretch - the length a crew would search.</p>"#,
    );
    s.push_str(&network_map(
        net,
        &demo.site.drawing,
        &class,
        &tip,
        &markers,
        700.0,
    ));
    let _ = write!(
        s,
        r#"<div class="legend">{}<span class="key"><span class="mark">&#9679;</span>logger</span><span class="key"><span class="mark">&#9711;</span>most likely point</span><span class="key"><span class="mark">&#10005;</span>actual burst (simulation)</span></div><p class="note" style="margin-top:8px">{}</p></div>"#,
        ramp_legend("pipes", "less", "more likely"),
        esc(&check)
    );
    s
}

/// The arrival at each logger: a zoomed view around the fronts, and the
/// table of times the engine located the burst from.
fn arrivals_card(event: &EventModel, issued: &Issued) -> String {
    let fronts = event.arrivals(&issued.alert);
    let times: Vec<f64> = fronts.iter().flatten().map(|f| f.arrival).collect();
    let first = times.iter().copied().fold(f64::INFINITY, f64::min);
    let last = times.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let (from, to) = if first.is_finite() {
        (first - 0.3, last + 0.7)
    } else {
        let t = issued.alert.time.value();
        (t - 0.5, t + 1.5)
    };
    let mut s = format!(
        r#"<div class="card"><h2>Arrival at each logger</h2><p class="note">The pressure wave reaches the loggers one after another. Those time differences are what the engine locates the burst from: the closer logger feels it first.</p><div class="js-traces" id="arrivals-{}" data-from="{from:.3}" data-to="{to:.3}"></div>"#,
        issued.alert.id
    );
    s.push_str(r#"<div class="table-wrap" style="margin-top:12px"><table><thead><tr><th>Logger</th><th class="num">Front at</th><th class="num">After the first</th><th class="num">Pressure step</th></tr></thead><tbody>"#);
    for (r, f) in event.records.iter().zip(&fronts) {
        match f {
            Some(f) => {
                let _ = write!(
                    s,
                    r#"<tr><td>{}</td><td class="num">{:.4} s</td><td class="num">{:.1} ms</td><td class="num">{:+.2} m</td></tr>"#,
                    esc(&r.name),
                    f.arrival,
                    (f.arrival - first) * 1000.0,
                    f.step
                );
            }
            None => {
                let _ = write!(
                    s,
                    r#"<tr><td>{}</td><td class="num">-</td><td class="num">-</td><td class="num">-</td></tr>"#,
                    esc(&r.name)
                );
            }
        }
    }
    s.push_str("</tbody></table></div></div>");
    s
}
