//! About the data: what is simulated, what has been proven, what is next.

use crate::view::shell::Page;

pub fn page() -> Page {
    let s = r#"<div class="card"><h2>Everything here is simulated</h2>
<p>The pressure records come from the engine's own pipe simulator (method of characteristics), recorded by
synthetic loggers that behave like real transient loggers: 128 samples a second, each averaged over its
interval, a random clock phase, 2 cm of noise. The live engine then sees them exactly as it would see a real
site's data - in 1-second chunks.</p>
<p>That proves the engine is consistent with physics. It does not yet prove it works on real pipes: real noise,
plastic pipe walls that smear the wave, wave speeds nobody measured, network drawings that do not match the
ground. That is the next part of the validation ladder.</p></div>
<div class="card"><h2>The validation ladder</h2><ul class="plain">
<li><b>1. Analytical</b> - closed-form physics (Joukowsky, junction rules, 2L/a): passed.</li>
<li><b>2. Invariants</b> - laws that must always hold (steady stays steady, the grid does not change the answer): passed.</li>
<li><b>3. Simulation</b> - bursts simulated, recorded, detected, located (studies V-S1, V-S2): passed, as shown here.</li>
<li><b>4. Published and partner data</b> - real recordings: next, once the data readers exist.</li>
<li><b>5. Live shadow mode</b> - the engine listening to a real site without sending alerts: later.</li>
</ul></div>
<div class="card"><h2>This page</h2>
<p>Made by <code>jk view</code>. One file, no server, nothing downloaded; it works offline. To make it again:</p>
<p><code>cargo run --release -p jk-cli -- view</code> &nbsp; (a few seconds; options: <code>--loggers N</code>, <code>--no-placement</code>, <code>--out FILE</code>)</p>
<p class="note">Next: readers for network files (EPANET .inp, which carry map coordinates) and logger exports, so real recordings appear here instead of simulations.</p></div>"#;
    Page {
        id: "about",
        title: "About the data".into(),
        crumb: "What is simulated, what is proven, what is next".into(),
        source: "Simulated data",
        body: s.to_string(),
    }
}
