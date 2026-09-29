//! Section 2: where the loggers should go.
//!
//! The placement advisor chooses loggers for the commercial campus; the
//! section shows the recommendation on a map (each pipe coloured by how
//! long a stretch a crew would search for a burst on it), what each extra
//! logger buys, and per pipe the smallest burst that can still be located.

use std::fmt::Write as _;

use jk_localisation::placement::{PlacementOptions, advise, detectability};

use super::charts::{Marker, columns, network_map};
use super::svg::{esc, seq_class};
use crate::sim::sites::campus;

/// Runs the advisor for `count` loggers and returns the section's HTML.
///
/// # Errors
/// Only if the site or the advisor fails (a bug), or `count` is out of range.
#[allow(clippy::too_many_lines)] // one section, told top to bottom
pub fn section(count: usize) -> Result<String, String> {
    let (site, candidates) = campus()?;
    let options = PlacementOptions::default();
    let net = &site.network;
    let names = &site.drawing.pipe_names;
    eprintln!(
        "jk view: choosing {count} loggers for the campus (about 10 s per logger on a small laptop)..."
    );
    let advice = advise(net, &candidates, count, &[], None, &options).map_err(|e| e.to_string())?;
    let best = &advice.best;
    let felt = detectability(net, &best.sensors, &options).map_err(|e| e.to_string())?;
    let smallest = felt
        .pipes
        .iter()
        .map(|p| p.smallest_locatable.value())
        .fold(0.0, f64::max);
    let node_name = |n: jk_hydraulics::NodeId| site.drawing.node_names[n.index()].clone();

    let mut s = String::new();
    s.push_str(
        r"<h2>2. Where should the loggers go?</h2>
<p>A commercial campus: the municipal connection feeds an iron ring main with plastic branches to
the buildings, 2.31 km of pipe. The placement advisor tries a burst every 20 m along every pipe,
asks the real locator how long a stretch a crew would have to search, and chooses the loggers that
make that search shortest - counting only loggers that would actually feel a sudden 1 l/s burst.</p>",
    );
    let who: Vec<String> = best.sensors.iter().map(|&n| node_name(n)).collect();
    let _ = write!(
        s,
        r#"<div class="card"><div class="stats">
<div class="stat"><div class="label">Loggers recommended</div><div class="value">{}</div></div>
<div class="stat"><div class="label">Average search stretch (90%)</div><div class="value">{:.0} m</div></div>
<div class="stat"><div class="label">Pipe pinpointed to 20 m</div><div class="value">{:.0}%</div></div>
<div class="stat"><div class="label">Smallest burst located anywhere</div><div class="value">{:.2} l/s</div></div>
</div><p class="note" style="margin-top:10px">At: {}. Book wave speeds are taken as known to &plusmn;10%, which is what
limits the search stretch most; calibrating them from known events shrinks it. "Smallest burst" is an upper
bound until plastic-pipe damping is modelled.</p></div>"#,
        best.sensors.len(),
        best.expected_region.value(),
        best.pinpointed * 100.0,
        smallest * 1000.0,
        esc(&who.join(", "))
    );

    // The map.
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
        .zip(&felt.pipes)
        .map(|(p, d)| {
            format!(
                "{}|{:.0} m of pipe|typical search stretch {:.0} m|smallest burst located {:.2} l/s",
                names[p.pipe.index()],
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
        candidates
            .iter()
            .filter(|n| !best.sensors.contains(n))
            .map(|n| Marker::Candidate(n.index())),
    );
    s.push_str(r#"<div class="grid2"><div class="card"><h3>The recommendation</h3>"#);
    s.push_str(&network_map(
        net,
        &site.drawing,
        &class,
        &tip,
        &markers,
        620.0,
    ));
    s.push_str(
        r#"<div class="legend"><span class="key">pipes: <span class="ramp"><span class="q1"></span><span class="q2"></span><span class="q3"></span><span class="q4"></span><span class="q5"></span><span class="q6"></span><span class="q7"></span><span class="q8"></span></span> short to long search (10 m to 400 m+)</span><span class="key">&#9679; logger</span><span class="key">&#9675; could take a logger</span></div></div>"#,
    );

    // What each logger buys.
    let items: Vec<(String, f64)> = advice
        .steps
        .iter()
        .map(|c| (c.sensors.len().to_string(), c.expected_region.value()))
        .collect();
    s.push_str(r#"<div class="card"><h3>What each extra logger buys</h3><p class="note">Average 90% search stretch with 2, 3, ... loggers, each added where it helps most (before the final swaps).</p>"#);
    s.push_str(&columns(&items, "m", 420.0, 260.0));
    s.push_str("</div></div>");

    // Per pipe.
    s.push_str(
        r#"<div class="card"><h3>Pipe by pipe</h3><table><thead><tr><th>Pipe</th><th class="num">Length</th><th class="num">Typical search stretch</th><th class="num">Pinpointed</th><th class="num">Smallest burst located</th></tr></thead><tbody>"#,
    );
    for (p, d) in best.pipes.iter().zip(&felt.pipes) {
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td class="num">{:.0} m</td><td class="num">{:.0} m</td><td class="num">{:.0}%</td><td class="num">{:.2} l/s</td></tr>"#,
            esc(&names[p.pipe.index()]),
            p.length.value(),
            p.median_region.value(),
            p.pinpointed * 100.0,
            d.smallest_locatable.value() * 1000.0
        );
    }
    s.push_str(
        r#"</tbody></table><p class="note" style="margin-top:8px">A stretch much longer than its pipe means the pipe is blind to these loggers: a burst on it
looks the same as one at the junction it hangs from. No logger can sit beyond the municipal connection
(the supply holds the pressure fixed there), so that pipe stays blind.</p></div>"#,
    );
    Ok(s)
}
