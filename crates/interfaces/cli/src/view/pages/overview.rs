//! Overview: the headline figures, the event timeline, the alerts.

use std::fmt::Write as _;

use jk_application::Level;

use super::{badge, class_name, status};
use crate::view::charts::{TimelineAlert, timeline};
use crate::view::model::{EventModel, PlacementModel};
use crate::view::shell::Page;
use crate::view::svg::esc;

pub fn page(event: &EventModel, placement: Option<&PlacementModel>) -> Page {
    let net = &event.demo.site.network;
    let length: f64 = net.pipes().iter().map(|p| p.pipe.length().value()).sum();
    let count = |l: Level| event.issued.iter().filter(|i| i.alert.level == l).count();
    let slowest = event
        .issued
        .iter()
        .map(|i| i.data_time - i.alert.time.value())
        .fold(0.0, f64::max);
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Alerts</div><div class="value">{}</div><div class="detail">{} alarm &middot; {} warning &middot; {} review &middot; {} info</div></div>
<div class="stat"><div class="label">Time to alert</div><div class="value">{slowest:.1} s</div><div class="detail">after the burst, in data time (slowest alert)</div></div>
<div class="stat"><div class="label">Loggers</div><div class="value">{}</div><div class="detail">128 samples a second, 2 cm noise</div></div>
<div class="stat"><div class="label">Network</div><div class="value">{:.1} km</div><div class="detail">{} pipes, {} nodes</div></div>"#,
        event.issued.len(),
        count(Level::Alarm),
        count(Level::Warning),
        count(Level::Review),
        count(Level::Info),
        event.records.len(),
        length / 1000.0,
        net.pipes().len(),
        net.nodes().len()
    );
    if let Some(p) = placement {
        let _ = write!(
            s,
            r##"<a class="stat" href="#placement" style="text-decoration:none"><div class="label">Placement advice (campus)</div><div class="value">{} loggers</div><div class="detail">{:.0} m average search stretch</div></a>"##,
            p.advice.best.sensors.len(),
            p.advice.best.expected_region.value()
        );
    }
    s.push_str("</div>");

    // Timeline.
    let alerts: Vec<TimelineAlert> = event
        .issued
        .iter()
        .map(|i| {
            let (label, class) = status(i.alert.level);
            TimelineAlert {
                id: i.alert.id,
                status: class,
                tip: format!(
                    "{label} #{} - {}|started at {:.3} s|alert out at {:.2} s ({:.1} s later)|click for the details",
                    i.alert.id,
                    class_name(&i.alert),
                    i.alert.time.value(),
                    i.data_time,
                    i.data_time - i.alert.time.value()
                ),
                start: i.alert.time.value(),
                sent: i.data_time,
            }
        })
        .collect();
    let loggers: Vec<(String, Vec<f64>)> = event
        .records
        .iter()
        .map(|r| (r.name.clone(), r.fronts.iter().map(|f| f.arrival).collect()))
        .collect();
    s.push_str(r#"<div class="card"><h2>Event timeline</h2><p class="note">Each dot is where an event started; its line runs to the moment the alert went out (data time). Below, every front each logger's detector picked. Hover for details; click an alert to open it.</p>"#);
    s.push_str(r#"<div class="table-wrap">"#);
    s.push_str(&timeline(&alerts, &loggers, event.span(), 1080.0));
    s.push_str("</div></div>");

    // Alerts.
    s.push_str(r#"<div class="card"><h2>Alerts</h2><div class="table-wrap"><table><thead><tr><th>Level</th><th>Event</th><th>Where</th><th class="num">Started</th><th class="num">Alert out</th><th>Seen by</th></tr></thead><tbody>"#);
    for i in &event.issued {
        let a = &i.alert;
        let place = a.location.as_ref().map_or("not located".to_string(), |l| {
            format!(
                "{:.1} m along {}",
                l.most_likely.distance.value(),
                event.demo.site.drawing.pipe_names[l.most_likely.pipe.index()]
            )
        });
        let _ = write!(
            s,
            r##"<tr><td>{}</td><td><a href="#alerts/{}">#{} {} - {:.0}%</a></td><td>{}</td><td class="num">{:.3} s</td><td class="num">{:.2} s</td><td>{}</td></tr>"##,
            badge(a.level),
            a.id,
            a.id,
            esc(&class_name(a)),
            a.confidence * 100.0,
            esc(&place),
            a.time.value(),
            i.data_time,
            esc(&a.sensors.join(", "))
        );
    }
    s.push_str("</tbody></table></div></div>");
    Page {
        id: "overview",
        title: "Overview".into(),
        crumb: "Flowing estate, two bursts - the live engine fed 1 s chunks".into(),
        body: s,
    }
}
