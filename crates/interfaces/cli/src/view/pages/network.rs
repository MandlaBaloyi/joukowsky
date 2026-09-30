//! Network: the site's map, its pipes and its nodes.

use std::fmt::Write as _;

use jk_hydraulics::Node;

use crate::view::charts::{Marker, network_map};
use crate::view::model::EventModel;
use crate::view::shell::Page;
use crate::view::svg::esc;

pub fn page(event: &EventModel) -> Page {
    let demo = &event.demo;
    let net = &demo.site.network;
    let d = &demo.site.drawing;
    let tips: Vec<String> = net
        .pipes()
        .iter()
        .zip(&d.pipe_names)
        .map(|(p, name)| {
            format!(
                "{name}|{:.0} m, {:.0} mm|wave speed {:.0} m/s|a wave crosses it in {:.0} ms",
                p.pipe.length().value(),
                p.pipe.diameter().value() * 1000.0,
                p.pipe.wave_speed().value(),
                p.pipe.travel_time().value() * 1000.0
            )
        })
        .collect();
    let class = vec!["q5"; net.pipes().len()];
    let markers: Vec<Marker> = demo
        .loggers
        .iter()
        .map(|n| Marker::Logger(n.index()))
        .collect();
    let mut s = String::from(
        r#"<div class="card"><h2>Map</h2><p class="note">As the engine knows the site. Hover a pipe for its size and wave timing.</p>"#,
    );
    s.push_str(&network_map(net, d, &class, &tips, &markers, 1000.0));
    s.push_str(r#"<div class="legend"><span class="key"><span class="mark">&#9679;</span>logger</span><span class="key"><span class="mark">&#9650;</span>reservoir / supply (holds the pressure)</span></div></div>"#);

    s.push_str(r#"<div class="cols"><div class="card"><h2>Pipes</h2><div class="table-wrap"><table><thead><tr><th>Pipe</th><th class="num">Length</th><th class="num">Diameter</th><th class="num">Wave speed</th><th class="num">Wave crosses in</th></tr></thead><tbody>"#);
    for (p, name) in net.pipes().iter().zip(&d.pipe_names) {
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td class="num">{:.0} m</td><td class="num">{:.0} mm</td><td class="num">{:.0} m/s</td><td class="num">{:.0} ms</td></tr>"#,
            esc(name),
            p.pipe.length().value(),
            p.pipe.diameter().value() * 1000.0,
            p.pipe.wave_speed().value(),
            p.pipe.travel_time().value() * 1000.0
        );
    }
    s.push_str(r#"</tbody></table></div></div><div class="card"><h2>Nodes</h2><div class="table-wrap"><table><thead><tr><th>Node</th><th>Kind</th><th class="num">Use</th><th>Logger</th></tr></thead><tbody>"#);
    for (n, node) in net.nodes().iter().enumerate() {
        let (kind, use_) = match node {
            Node::Reservoir { head } => ("supply", format!("{:.0} m head", head.value())),
            Node::Junction { demand } if demand.value() > 0.0 => {
                ("consumer", format!("{:.1} l/s", demand.value() * 1000.0))
            }
            Node::Junction { .. } => {
                let pipes = net
                    .pipes()
                    .iter()
                    .filter(|p| p.from.index() == n || p.to.index() == n)
                    .count();
                (if pipes == 1 { "dead end" } else { "junction" }, "-".into())
            }
            Node::Valve { .. } => ("valve", "-".into()),
            Node::CheckValve => ("check valve", "-".into()),
            Node::Pump { .. } => ("pump", "-".into()),
            Node::ControlValve { mode, .. } => (
                "control valve",
                match mode {
                    jk_hydraulics::ValveMode::Prv { setting } => {
                        format!("PRV, holds {:.1} m below", setting.value())
                    }
                    jk_hydraulics::ValveMode::Psv { setting } => {
                        format!("PSV, keeps {:.1} m above", setting.value())
                    }
                    jk_hydraulics::ValveMode::Fcv { flow } => {
                        format!("FCV, at most {:.1} l/s", flow.value() * 1000.0)
                    }
                    jk_hydraulics::ValveMode::Tcv => "TCV, fixed throttle".into(),
                },
            ),
        };
        let logger = demo.loggers.iter().any(|l| l.index() == n);
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{kind}</td><td class="num">{}</td><td>{}</td></tr>"#,
            esc(&d.node_names[n]),
            esc(&use_),
            if logger { "yes" } else { "" }
        );
    }
    s.push_str("</tbody></table></div></div></div>");
    Page {
        id: "network",
        title: "Network".into(),
        crumb: "The estate, as the engine knows it".into(),
        source: "Simulated data",
        body: s,
    }
}
