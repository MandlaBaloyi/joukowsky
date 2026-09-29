//! Placement: the recommendation, what each logger buys, pipe by pipe.

use std::fmt::Write as _;

use jk_hydraulics::NodeId;

use super::ramp_legend;
use crate::view::charts::{Marker, columns, network_map};
use crate::view::model::PlacementModel;
use crate::view::shell::Page;
use crate::view::svg::{esc, seq_class};

const CRUMB: &str =
    "Commercial campus, 2.31 km - wave speeds known to +/-10%, planning for a sudden 1 l/s burst";

fn names(m: &PlacementModel, nodes: &[NodeId]) -> Vec<String> {
    nodes
        .iter()
        .map(|n| m.site.drawing.node_names[n.index()].clone())
        .collect()
}

/// The recommendation: figures and the map.
pub fn recommendation(m: &PlacementModel) -> Page {
    let best = &m.advice.best;
    let net = &m.site.network;
    let pipe_names = &m.site.drawing.pipe_names;
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Loggers recommended</div><div class="value">{}</div><div class="detail">from {} places a logger could go</div></div>
<div class="stat"><div class="label">Average search stretch</div><div class="value">{:.0} m</div><div class="detail">90% region, averaged over the network</div></div>
<div class="stat"><div class="label">Pinpointed to 20 m</div><div class="value">{:.0}%</div><div class="detail">of the pipe length</div></div>
<div class="stat"><div class="label">Smallest burst located anywhere</div><div class="value">{:.2} l/s</div><div class="detail">sudden burst; upper bound (no plastic damping yet)</div></div>
</div>"#,
        best.sensors.len(),
        m.candidates.len(),
        best.expected_region.value(),
        best.pinpointed * 100.0,
        m.smallest_anywhere() * 1000.0
    );
    let span = (10.0_f64, 400.0_f64);
    let class: Vec<&str> = best
        .pipes
        .iter()
        .map(|p| {
            let r = p.median_region.value().clamp(span.0, span.1);
            seq_class((r / span.0).ln() / (span.1 / span.0).ln())
        })
        .collect();
    let tip: Vec<String> = best
        .pipes
        .iter()
        .zip(&m.felt.pipes)
        .map(|(p, d)| {
            format!(
                "{}|{:.0} m of pipe|typical search stretch {:.0} m|smallest burst located {:.2} l/s",
                pipe_names[p.pipe.index()],
                p.length.value(),
                p.median_region.value(),
                d.smallest_locatable.value() * 1000.0
            )
        })
        .collect();
    let mut markers: Vec<Marker> = best
        .sensors
        .iter()
        .map(|n| Marker::Logger(n.index()))
        .collect();
    markers.extend(
        m.candidates
            .iter()
            .filter(|n| !best.sensors.contains(n))
            .map(|n| Marker::Candidate(n.index())),
    );
    s.push_str(r#"<div class="card"><h2>Where the loggers go</h2><p class="note">Each pipe is coloured by the stretch a crew would search for a burst on it - see the legend. Hover a pipe for its figures.</p><div class="split"><div>"#);
    s.push_str(&network_map(
        net,
        &m.site.drawing,
        &class,
        &tip,
        &markers,
        640.0,
    ));
    let _ = write!(
        s,
        r#"<div class="legend">{}<span class="key"><span class="mark">&#9679;</span>logger</span><span class="key"><span class="mark">&#9675;</span>could take a logger</span><span class="key"><span class="mark">&#9650;</span>municipal supply</span></div></div><div class="side">"#,
        ramp_legend("pipes", "10 m", "400 m+ search")
    );
    s.push_str(r#"<h3>Recommended loggers</h3><ol class="plain">"#);
    for name in names(m, &best.sensors) {
        let _ = write!(s, "<li>{}</li>", esc(&name));
    }
    s.push_str(r#"</ol><h3 style="margin-top:16px">Blind or weak</h3><ul class="plain">"#);
    let mut weak = 0;
    for p in &best.pipes {
        let (r, len) = (p.median_region.value(), p.length.value());
        if r > 1.5 * len || r > 100.0 {
            weak += 1;
            let _ = write!(
                s,
                "<li><b>{}</b> ({len:.0} m): a crew would search about {r:.0} m</li>",
                esc(&pipe_names[p.pipe.index()])
            );
        }
    }
    if weak == 0 {
        s.push_str("<li>none</li>");
    }
    s.push_str(r#"</ul><p class="note" style="margin-top:14px">What limits the search most is how well the wave speeds are known (&plusmn;10% here): calibrating them from known events shrinks every stretch.</p></div></div></div>"#);
    Page {
        id: "placement",
        title: "Where the loggers go".into(),
        crumb: CRUMB.into(),
        body: s,
    }
}

/// What each extra logger buys.
pub fn value(m: &PlacementModel) -> Page {
    let items: Vec<(String, f64)> = m
        .advice
        .steps
        .iter()
        .map(|c| (c.sensors.len().to_string(), c.expected_region.value()))
        .collect();
    let mut s = String::from(
        r#"<div class="cols"><div class="card"><h2>Average search stretch</h2><p class="note">With 2, 3, ... loggers, each added where it helps most.</p>"#,
    );
    s.push_str(&columns(&items, "m", 520.0, 300.0));
    s.push_str(r#"</div><div class="card"><h2>Step by step</h2><div class="table-wrap"><table><thead><tr><th class="num">Loggers</th><th>Added</th><th class="num">Average search</th><th class="num">Pinpointed</th></tr></thead><tbody>"#);
    let mut previous: Vec<NodeId> = Vec::new();
    for c in &m.advice.steps {
        let added: Vec<NodeId> = c
            .sensors
            .iter()
            .filter(|n| !previous.contains(n))
            .copied()
            .collect();
        let _ = write!(
            s,
            r#"<tr><td class="num">{}</td><td>{}</td><td class="num">{:.0} m</td><td class="num">{:.0}%</td></tr>"#,
            c.sensors.len(),
            esc(&names(m, &added).join(" + ")),
            c.expected_region.value(),
            c.pinpointed * 100.0
        );
        previous.clone_from(&c.sensors);
    }
    let best = &m.advice.best;
    let _ = write!(
        s,
        r#"<tr><td class="num">{}</td><td><b>after swaps</b> - the recommendation</td><td class="num"><b>{:.0} m</b></td><td class="num"><b>{:.0}%</b></td></tr></tbody></table></div><p class="note" style="margin-top:8px">Adding loggers one at a time is a good start, but the best pair need not be part of the best six: the final swaps move loggers from the ring junctions out to the building ends.</p></div></div>"#,
        best.sensors.len(),
        best.expected_region.value(),
        best.pinpointed * 100.0
    );
    Page {
        id: "placement-value",
        title: "What each extra logger buys".into(),
        crumb: CRUMB.into(),
        body: s,
    }
}

/// Pipe by pipe, the longest searches first.
pub fn pipes(m: &PlacementModel) -> Page {
    let best = &m.advice.best;
    let pipe_names = &m.site.drawing.pipe_names;
    let mut rows: Vec<usize> = (0..best.pipes.len()).collect();
    rows.sort_by(|&a, &b| {
        best.pipes[b]
            .median_region
            .value()
            .total_cmp(&best.pipes[a].median_region.value())
    });
    let longest = best
        .pipes
        .iter()
        .map(|p| p.median_region.value())
        .fold(1.0, f64::max);
    let mut s = String::from(
        r#"<div class="card"><div class="table-wrap"><table><thead><tr><th>Pipe</th><th class="num">Length</th><th class="num">Typical search stretch</th><th class="num">Pinpointed</th><th class="num">Smallest burst located</th></tr></thead><tbody>"#,
    );
    for i in rows {
        let (p, d) = (&best.pipes[i], &m.felt.pipes[i]);
        let r = p.median_region.value();
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td class="num">{:.0} m</td><td class="num"><div class="inline-bar">{r:.0} m<span class="track"><span class="bar" style="width:{:.0}%"></span></span></div></td><td class="num">{:.0}%</td><td class="num">{:.2} l/s</td></tr>"#,
            esc(&pipe_names[p.pipe.index()]),
            p.length.value(),
            r / longest * 100.0,
            p.pinpointed * 100.0,
            d.smallest_locatable.value() * 1000.0
        );
    }
    s.push_str(r#"</tbody></table></div><p class="note" style="margin-top:10px">A search stretch much longer than its pipe means the pipe is blind to these loggers: a burst on it looks the same as one at the junction it hangs from. No logger can sit beyond the municipal connection (the supply holds the pressure fixed there), so that pipe stays blind. "Smallest burst located" is a sudden burst that two loggers still feel - an upper bound until plastic-pipe damping is modelled.</p></div>"#);
    Page {
        id: "placement-pipes",
        title: "Pipe by pipe".into(),
        crumb: CRUMB.into(),
        body: s,
    }
}
