//! A network read from an EPANET `.inp` file: what it holds, what the
//! reader had to assume, and the map from the file's own coordinates.

use std::fmt::Write as _;

use jk_files::epanet::InpNetwork;
use jk_hydraulics::Node;

use crate::sim::sites::Drawing;
use crate::view::charts::network_map;
use crate::view::shell::Page;
use crate::view::svg::esc;

/// Most rows shown per table (the rest are counted).
const MAX_ROWS: usize = 300;

pub fn page(inp: &InpNetwork, file_name: &str) -> Page {
    let net = &inp.network;
    let length: f64 = net.pipes().iter().map(|p| p.pipe.length().value()).sum();
    let supplies = net
        .nodes()
        .iter()
        .filter(|n| matches!(n, Node::Reservoir { .. }))
        .count();
    let demand: f64 = net
        .nodes()
        .iter()
        .map(|n| match n {
            Node::Junction { demand } => demand.value(),
            _ => 0.0,
        })
        .sum();
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Pipes</div><div class="value">{}</div><div class="detail">{:.2} km in all</div></div>
<div class="stat"><div class="label">Nodes</div><div class="value">{}</div><div class="detail">{supplies} supply / fixed-head</div></div>
<div class="stat"><div class="label">Total demand</div><div class="value">{:.1} l/s</div><div class="detail">base values, patterns not applied</div></div>
<div class="stat"><div class="label">File units</div><div class="value">{}</div><div class="detail">{} friction</div></div>
</div>"#,
        net.pipes().len(),
        length / 1000.0,
        net.nodes().len(),
        demand * 1000.0,
        inp.units.name(),
        inp.headloss.name()
    );

    s.push_str(&notes_card(inp));
    s.push_str(&map_card(inp));
    s.push_str(&tables(inp));
    Page {
        id: "file",
        title: if inp.title.is_empty() {
            file_name.to_string()
        } else {
            inp.title.clone()
        },
        crumb: format!("Read from {file_name} - a network file (EPANET .inp)"),
        source: "Network file",
        body: s,
    }
}

/// Everything the reader assumed or left out - the honest part, first.
fn notes_card(inp: &InpNetwork) -> String {
    let mut s = String::new();
    // The honest part first.
    let _ = write!(
        s,
        r#"<div class="card"><h2>What the reader assumed or left out ({})</h2><p class="note">An .inp file describes a network for steady flow; the engine needs more. Everything it approximated or did not use is listed here - nothing is dropped silently.</p><ul class="reasons">"#,
        inp.notes.len()
    );
    for n in &inp.notes {
        match n.line {
            Some(l) => {
                let _ = write!(s, "<li><b>line {l}</b>: {}</li>", esc(&n.text));
            }
            None => {
                let _ = write!(s, "<li>{}</li>", esc(&n.text));
            }
        }
    }
    s.push_str("</ul></div>");

    s
}

/// The map from the file's own coordinates (north up).
fn map_card(inp: &InpNetwork) -> String {
    let mut s = String::new();
    let net = &inp.network;
    // The map.
    s.push_str(r#"<div class="card"><h2>Map</h2>"#);
    let missing = inp.positions.iter().filter(|p| p.is_none()).count();
    if missing == 0 {
        let drawing = Drawing {
            positions: inp
                .positions
                .iter()
                .map(|p| p.unwrap_or((0.0, 0.0)))
                .collect(),
            node_names: inp.node_ids.clone(),
            pipe_names: inp.pipe_ids.clone(),
            bends: inp.vertices.clone(),
            y_up: true,
        };
        let tips: Vec<String> = net
            .pipes()
            .iter()
            .zip(&inp.pipe_ids)
            .map(|(p, id)| {
                format!(
                    "pipe {id}|{:.0} m, {:.0} mm|friction factor {:.4}",
                    p.pipe.length().value(),
                    p.pipe.diameter().value() * 1000.0,
                    p.pipe.friction_factor()
                )
            })
            .collect();
        let class = vec!["q5"; net.pipes().len()];
        s.push_str(r#"<p class="note">Drawn from the file's own coordinates, north up. Hover a pipe for its size.</p>"#);
        s.push_str(&network_map(net, &drawing, &class, &tips, &[], 1000.0));
        s.push_str(r#"<div class="legend"><span class="key"><span class="mark">&#9650;</span>supply / fixed head</span></div>"#);
    } else {
        let _ = write!(
            s,
            "<p>The file gives no coordinates for {missing} node(s), so the map cannot be drawn.</p>"
        );
    }
    s.push_str("</div>");

    s
}

/// Pipes and nodes, the first few hundred of each.
fn tables(inp: &InpNetwork) -> String {
    let mut s = String::new();
    let net = &inp.network;
    // Tables.
    s.push_str(r#"<div class="cols"><div class="card"><h2>Pipes</h2><div class="table-wrap"><table><thead><tr><th>ID</th><th>From - to</th><th class="num">Length</th><th class="num">Diameter</th><th class="num">Friction f</th></tr></thead><tbody>"#);
    for (p, id) in net.pipes().iter().zip(&inp.pipe_ids).take(MAX_ROWS) {
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{} - {}</td><td class="num">{:.0} m</td><td class="num">{:.0} mm</td><td class="num">{:.4}</td></tr>"#,
            esc(id),
            esc(&inp.node_ids[p.from.index()]),
            esc(&inp.node_ids[p.to.index()]),
            p.pipe.length().value(),
            p.pipe.diameter().value() * 1000.0,
            p.pipe.friction_factor()
        );
    }
    s.push_str("</tbody></table></div>");
    if net.pipes().len() > MAX_ROWS {
        let _ = write!(
            s,
            r#"<p class="note">First {MAX_ROWS} of {} pipes shown.</p>"#,
            net.pipes().len()
        );
    }
    s.push_str(r#"</div><div class="card"><h2>Nodes</h2><div class="table-wrap"><table><thead><tr><th>ID</th><th>Kind</th><th class="num">Elevation</th><th class="num">Demand / head</th></tr></thead><tbody>"#);
    for (n, node) in net.nodes().iter().enumerate().take(MAX_ROWS) {
        let (kind, value) = match node {
            Node::Reservoir { head } => ("supply", format!("{:.2} m head", head.value())),
            Node::Junction { demand } => {
                ("junction", format!("{:.2} l/s", demand.value() * 1000.0))
            }
            _ => ("device", "-".into()),
        };
        let elevation = inp.elevations[n].map_or("-".into(), |e| format!("{:.1} m", e.value()));
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td>{kind}</td><td class="num">{elevation}</td><td class="num">{value}</td></tr>"#,
            esc(&inp.node_ids[n])
        );
    }
    s.push_str("</tbody></table></div>");
    if net.nodes().len() > MAX_ROWS {
        let _ = write!(
            s,
            r#"<p class="note">First {MAX_ROWS} of {} nodes shown.</p>"#,
            net.nodes().len()
        );
    }
    s.push_str("</div></div>");
    s
}
