//! The page around the pictures: theme colours, type, cards, tooltips.
//!
//! One self-contained file - no server, no downloads, nothing to install:
//! open it in any browser. Colours are defined once per theme as CSS
//! custom properties (light, and dark from the reader's system setting);
//! the palette is the validated reference palette of the charting method
//! (sequential blue ramp, reserved status colours with a text label).

/// A complete HTML page with `title` and `body` (already escaped HTML).
pub fn page(title: &str, body: &str) -> String {
    format!(
        r#"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<style>{STYLE}</style>
</head>
<body>
<div class="viz-root">
{body}
</div>
<div id="tip" class="tip" hidden></div>
<script>{SCRIPT}</script>
</body>
</html>
"#
    )
}

const STYLE: &str = r#"
.viz-root {
  color-scheme: light;
  --page: #f9f9f7; --surface-1: #fcfcfb; --border: rgba(11,11,11,0.10);
  --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7; --wash: #f0efec;
  --series-1: #2a78d6; --halo: rgba(42,120,214,0.22);
  --q0: #c3c2b7; --q1: #86b6ef; --q2: #6da7ec; --q3: #5598e7; --q4: #3987e5;
  --q5: #2a78d6; --q6: #256abf; --q7: #184f95; --q8: #0d366b;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) .viz-root {
    color-scheme: dark;
    --page: #0d0d0d; --surface-1: #1a1a19; --border: rgba(255,255,255,0.10);
    --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
    --grid: #2c2c2a; --axis: #383835; --wash: #2c2c2a;
    --series-1: #3987e5; --halo: rgba(57,135,229,0.30);
    --q0: #383835; --q1: #184f95; --q2: #1c5cab; --q3: #256abf; --q4: #2a78d6;
    --q5: #3987e5; --q6: #6da7ec; --q7: #9ec5f4; --q8: #cde2fb;
  }
}
:root[data-theme="dark"] .viz-root {
  color-scheme: dark;
  --page: #0d0d0d; --surface-1: #1a1a19; --border: rgba(255,255,255,0.10);
  --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --wash: #2c2c2a;
  --series-1: #3987e5; --halo: rgba(57,135,229,0.30);
  --q0: #383835; --q1: #184f95; --q2: #1c5cab; --q3: #256abf; --q4: #2a78d6;
  --q5: #3987e5; --q6: #6da7ec; --q7: #9ec5f4; --q8: #cde2fb;
}
html, body { margin: 0; }
body { background: #f9f9f7; }
@media (prefers-color-scheme: dark) { body { background: #0d0d0d; } }
.viz-root {
  background: var(--page); color: var(--text-primary);
  font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif;
  padding: 28px 16px 60px; min-height: 100vh; box-sizing: border-box;
}
.wrap { max-width: 1080px; margin: 0 auto; }
h1 { font-size: 26px; margin: 0 0 4px; }
h2 { font-size: 20px; margin: 36px 0 6px; }
h3 { font-size: 15px; margin: 0 0 6px; }
p { margin: 0 0 10px; color: var(--text-secondary); max-width: 760px; }
.card {
  background: var(--surface-1); border: 1px solid var(--border);
  border-radius: 12px; padding: 16px 18px; margin: 14px 0;
}
.note { font-size: 13px; color: var(--text-muted); }
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; }
.stat .label { font-size: 13px; color: var(--text-secondary); }
.stat .value { font-size: 26px; font-weight: 600; }
.badge { display: inline-flex; align-items: center; gap: 6px; font-weight: 600; }
.badge .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
.alert-head { display: flex; flex-wrap: wrap; gap: 6px 18px; align-items: baseline; }
.alert-head .what { font-weight: 600; }
.alert-head .meta { color: var(--text-secondary); font-size: 14px; }
ul.reasons { margin: 8px 0 0; padding-left: 18px; color: var(--text-secondary); font-size: 14px; }
.legend { display: flex; flex-wrap: wrap; gap: 6px 16px; align-items: center; font-size: 13px; color: var(--text-secondary); margin: 6px 0 0; }
.legend .ramp { display: inline-flex; gap: 2px; }
.legend .ramp span { width: 18px; height: 10px; border-radius: 2px; display: inline-block; }
.legend .key { display: inline-flex; align-items: center; gap: 6px; }
.legend .swatch { width: 18px; height: 10px; border-radius: 2px; display: inline-block; background: var(--wash); border: 1px solid var(--axis); }
.grid2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 14px; }
table { border-collapse: collapse; width: 100%; font-size: 14px; }
th { text-align: left; font-weight: 600; color: var(--text-secondary); border-bottom: 1px solid var(--axis); padding: 6px 8px; }
td { border-bottom: 1px solid var(--grid); padding: 6px 8px; }
td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; }
svg.chart { width: 100%; height: auto; display: block; overflow: visible; }
svg .tick { fill: var(--text-muted); font-size: 11px; font-variant-numeric: tabular-nums; }
svg .label { fill: var(--text-secondary); font-size: 12px; }
svg .label.strong { fill: var(--text-primary); font-weight: 600; }
svg .point-label { fill: var(--text-primary); }
svg .value { fill: var(--text-secondary); font-size: 12px; }
svg .front-label { fill: var(--text-secondary); font-size: 11px; }
svg .grid { stroke: var(--grid); stroke-width: 1; }
svg .axis { stroke: var(--axis); stroke-width: 1; }
svg .impaired { fill: var(--wash); }
svg .series { fill: none; stroke: var(--series-1); stroke-width: 1.5; stroke-linejoin: round; stroke-linecap: round; }
svg .front { stroke: var(--text-secondary); stroke-width: 1.5; }
svg .crosshair { stroke: var(--text-muted); stroke-width: 1; }
svg .hit { fill: transparent; pointer-events: all; }
svg .pipe { stroke-width: 5; stroke-linecap: round; }
svg .pipe.q0 { stroke: var(--q0); } svg .pipe.q1 { stroke: var(--q1); } svg .pipe.q2 { stroke: var(--q2); }
svg .pipe.q3 { stroke: var(--q3); } svg .pipe.q4 { stroke: var(--q4); } svg .pipe.q5 { stroke: var(--q5); }
svg .pipe.q6 { stroke: var(--q6); } svg .pipe.q7 { stroke: var(--q7); } svg .pipe.q8 { stroke: var(--q8); }
svg .pipe-hit { stroke: transparent; stroke-width: 18; stroke-linecap: round; pointer-events: stroke; }
svg .stretch { stroke: var(--halo); stroke-width: 18; stroke-linecap: round; }
svg .logger { fill: var(--text-primary); stroke: var(--surface-1); stroke-width: 2; }
svg .candidate { fill: var(--surface-1); stroke: var(--text-muted); stroke-width: 1.5; }
svg .reservoir { fill: var(--text-muted); }
svg .likely { fill: var(--surface-1); stroke: var(--text-primary); stroke-width: 3; }
svg .truth { stroke: var(--text-primary); stroke-width: 2; stroke-linecap: round; fill: none; }
svg .bar { fill: var(--series-1); }
svg .bar:hover, svg .pipe-hit:hover { opacity: 0.85; }
.ramp .q1 { background: var(--q1); } .ramp .q2 { background: var(--q2); } .ramp .q3 { background: var(--q3); }
.ramp .q4 { background: var(--q4); } .ramp .q5 { background: var(--q5); } .ramp .q6 { background: var(--q6); }
.ramp .q7 { background: var(--q7); } .ramp .q8 { background: var(--q8); }
.tip {
  position: fixed; z-index: 10; pointer-events: none; max-width: 280px;
  background: #1a1a19; color: #ffffff; border-radius: 8px; padding: 8px 10px;
  font: 13px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; box-shadow: 0 4px 16px rgba(0,0,0,0.2);
}
.tip .tip-value { font-weight: 600; font-size: 14px; }
.tip div + div { color: #c3c2b7; }
"#;

const SCRIPT: &str = r"
(() => {
  const tip = document.getElementById('tip');
  const show = (lines, x, y) => {
    tip.replaceChildren();
    lines.forEach((line, i) => {
      const d = document.createElement('div');
      d.textContent = line;
      if (i === 0) d.className = 'tip-value';
      tip.appendChild(d);
    });
    tip.hidden = false;
    const w = tip.offsetWidth, h = tip.offsetHeight;
    tip.style.left = Math.min(x + 14, window.innerWidth - w - 8) + 'px';
    tip.style.top = Math.min(y + 14, window.innerHeight - h - 8) + 'px';
  };
  const hide = () => { tip.hidden = true; };
  document.querySelectorAll('[data-tip]').forEach(el => {
    el.addEventListener('pointermove', e => show(el.dataset.tip.split('|'), e.clientX, e.clientY));
    el.addEventListener('pointerleave', hide);
  });
  document.querySelectorAll('svg.trace').forEach(svg => {
    const t = JSON.parse(svg.dataset.t), h = JSON.parse(svg.dataset.h);
    const [x0, x1, t0, t1] = svg.dataset.sx.split(',').map(Number);
    const hair = svg.querySelector('.crosshair');
    const hit = svg.querySelector('.hit');
    hit.addEventListener('pointermove', e => {
      const pt = svg.createSVGPoint();
      pt.x = e.clientX; pt.y = e.clientY;
      const p = pt.matrixTransform(svg.getScreenCTM().inverse());
      const time = t0 + (p.x - x0) / (x1 - x0) * (t1 - t0);
      let lo = 0, hi = t.length - 1;
      while (hi - lo > 1) { const m = (lo + hi) >> 1; if (t[m] < time) lo = m; else hi = m; }
      const i = (time - t[lo] < t[hi] - time) ? lo : hi;
      const x = x0 + (t[i] - t0) / (t1 - t0) * (x1 - x0);
      hair.setAttribute('x1', x); hair.setAttribute('x2', x);
      hair.setAttribute('visibility', 'visible');
      show([h[i].toFixed(2) + ' m of head', 't = ' + t[i].toFixed(3) + ' s'], e.clientX, e.clientY);
    });
    hit.addEventListener('pointerleave', () => { hair.setAttribute('visibility', 'hidden'); hide(); });
  });
})();
";
