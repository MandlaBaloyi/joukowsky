// joukowsky viewer - navigation, theme, tooltips, and the pressure charts.
// Plain JavaScript, no libraries. Every piece of text from the data is put
// on the page with textContent, never as markup.
(() => {
  'use strict';
  const NS = 'http://www.w3.org/2000/svg';
  const data = JSON.parse(document.getElementById('jk-data').textContent);

  // ---------------------------------------------------------------- tooltip
  const tip = document.getElementById('tip');
  // rows: [{text, strong}] or [{key, label, value}]
  function showTip(rows, x, y) {
    tip.replaceChildren();
    for (const r of rows) {
      const d = document.createElement('div');
      if (r.value !== undefined) {
        d.className = 'row';
        const k = document.createElement('span'); k.className = 'key';
        const l = document.createElement('span'); l.textContent = r.label;
        const v = document.createElement('b'); v.textContent = r.value;
        d.append(k, v, l);
      } else {
        d.textContent = r.text;
        if (r.strong) d.className = 'strong';
      }
      tip.appendChild(d);
    }
    tip.hidden = false;
    const w = tip.offsetWidth, h = tip.offsetHeight;
    tip.style.left = Math.max(8, Math.min(x + 14, window.innerWidth - w - 8)) + 'px';
    tip.style.top = Math.max(8, Math.min(y + 14, window.innerHeight - h - 8)) + 'px';
  }
  const hideTip = () => { tip.hidden = true; };
  document.querySelectorAll('[data-tip]').forEach(el => {
    const rows = () => el.dataset.tip.split('|').map((t, i) => ({ text: t, strong: i === 0 }));
    el.addEventListener('pointermove', e => showTip(rows(), e.clientX, e.clientY));
    el.addEventListener('pointerleave', hideTip);
  });

  // ------------------------------------------------------------------ theme
  function applyTheme(choice) {
    if (choice === 'light' || choice === 'dark') document.documentElement.dataset.theme = choice;
    else delete document.documentElement.dataset.theme;
    document.querySelectorAll('[data-theme-choice]').forEach(b =>
      b.setAttribute('aria-pressed', String(b.dataset.themeChoice === choice)));
    try { localStorage.setItem('jk-theme', choice); } catch (e) { /* private mode */ }
  }
  document.querySelectorAll('[data-theme-choice]').forEach(b =>
    b.addEventListener('click', () => applyTheme(b.dataset.themeChoice)));
  let saved = 'system';
  try { saved = localStorage.getItem('jk-theme') || 'system'; } catch (e) { /* private mode */ }
  applyTheme(saved);

  // ------------------------------------------------------------- navigation
  const pages = [...document.querySelectorAll('.page')];
  const links = [...document.querySelectorAll('[data-nav]')];
  document.getElementById('menu-button').addEventListener('click', () =>
    document.body.classList.toggle('menu-open'));

  function selectAlert(id) {
    const items = [...document.querySelectorAll('.alert-item')];
    const chosen = id || (items[0] && items[0].dataset.alert);
    items.forEach(i => i.classList.toggle('active', i.dataset.alert === chosen));
    document.querySelectorAll('.alert-detail').forEach(d => { d.hidden = d.dataset.alert !== chosen; });
  }

  function route() {
    const [name, arg] = (location.hash.slice(1) || 'overview').split('/');
    // Pages are ids "page-<name>", so the address never makes the browser
    // jump to an element on its own.
    const page = pages.find(p => p.dataset.page === name) || pages[0];
    pages.forEach(p => { p.hidden = p !== page; });
    links.forEach(a => a.classList.toggle('active', a.dataset.nav === page.dataset.page));
    document.getElementById('page-title').textContent = page.dataset.title;
    document.getElementById('page-crumb').textContent = page.dataset.crumb || '';
    document.getElementById('page-source').textContent = page.dataset.source || '';
    document.title = page.dataset.title + ' - joukowsky viewer';
    if (page.dataset.page === 'alerts') selectAlert(arg);
    document.body.classList.remove('menu-open');
    renderVisible();
  }
  window.addEventListener('hashchange', () => { route(); window.scrollTo(0, 0); });

  // ----------------------------------------------------------------- charts
  function el(name, attrs, parent) {
    const e = document.createElementNS(NS, name);
    for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
    if (parent) parent.appendChild(e);
    return e;
  }
  // Round ticks (1, 2, 5 x 10^n) and the decimals they need.
  function ticks(min, max, count) {
    // At least three labels, so every axis can be read.
    for (let c = count; c < count + 6; c++) {
      const t = ticksOnce(min, max, c);
      if (t.values.length >= 3) return t;
    }
    return ticksOnce(min, max, count + 6);
  }
  function ticksOnce(min, max, count) {
    const span = Math.max(max - min, 1e-9), raw = span / count;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 5, 10].map(m => m * mag).find(s => s >= raw);
    const out = [];
    for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-9; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
    return { values: out, decimals: step >= 1 ? 0 : Math.ceil(-Math.log10(step)) };
  }
  // A record: { name, dt, runs: [{ start, h: [...] }], fronts, impaired }.
  // Runs are continuous stretches; between them the data never arrived,
  // and inside them NaN (null in JSON) marks an invalid sample - both are
  // drawn as breaks in the line, never joined up (principle P6).
  const valueAt = (rec, t) => {
    for (const run of rec.runs) {
      const i = Math.round((t - run.start) / rec.dt);
      if (i >= 0 && i < run.h.length) return run.h[i];
    }
    return null;
  };

  // Draws every record of `records` between range[0] and range[1] seconds,
  // stacked, sharing the time axis. onZoom(range) is called when the
  // reader drags across a chart (if given). With `now` (playback) only
  // what had happened by then is drawn, and a line marks the moment.
  function drawStack(container, records, range, onZoom, now) {
    const upTo = (now === undefined || now === null) ? Infinity : now;
    container.replaceChildren();
    const width = Math.max(320, container.clientWidth);
    const height = 150, left = 52, right = 12, top = 20, bottom = 24;
    const [t0, t1] = range;
    const sx = t => left + (t - t0) / (t1 - t0) * (width - left - right);
    const tOf = x => t0 + (x - left) / (width - left - right) * (t1 - t0);
    const charts = [];
    for (const rec of records) {
      const title = document.createElement('div');
      title.className = 'chart-title';
      title.textContent = rec.name;
      container.appendChild(title);
      const svg = el('svg', { class: 'chart trace', viewBox: `0 0 ${width} ${height}`, role: 'img' }, container);
      svg.setAttribute('aria-label', 'Pressure head at ' + rec.name);
      const spans = rec.runs.map(run => ({
        run,
        i0: Math.max(0, Math.floor((t0 - run.start) / rec.dt)),
        i1: Math.min(run.h.length - 1, Math.ceil((t1 - run.start) / rec.dt)),
      }));
      let lo = Infinity, hi = -Infinity;
      for (const { run, i0, i1 } of spans) {
        for (let i = i0; i <= i1; i++) {
          const h = run.h[i];
          if (h === null || !isFinite(h)) continue;
          lo = Math.min(lo, h); hi = Math.max(hi, h);
        }
      }
      for (const l of rec.lines || []) { lo = Math.min(lo, l.v); hi = Math.max(hi, l.v); }
      if (!isFinite(lo)) { lo = 0; hi = 1; }
      const pad = Math.max((hi - lo) * 0.08, 0.05);
      const yt = ticks(lo - pad, hi + pad, 4);
      const y0 = Math.min(yt.values[0], lo - pad), y1 = Math.max(yt.values[yt.values.length - 1], hi + pad);
      const sy = h => top + (y1 - h) / (y1 - y0) * (height - top - bottom);
      for (const [a, b] of rec.impaired) {
        const xa = sx(Math.max(a, t0)), xb = sx(Math.min(b, t1));
        if (xb > xa) el('rect', { class: 'impaired', x: xa, y: top, width: xb - xa, height: height - top - bottom }, svg);
      }
      for (const v of yt.values) {
        el('line', { class: 'grid', x1: left, x2: width - right, y1: sy(v), y2: sy(v) }, svg);
        el('text', { class: 'tick', x: left - 6, y: sy(v) + 4, 'text-anchor': 'end' }, svg).textContent = v.toFixed(yt.decimals);
      }
      const xt = ticks(t0, t1, Math.max(2, Math.floor((width - left - right) / 90)));
      for (const v of xt.values) {
        el('text', { class: 'tick', x: sx(v), y: height - 6, 'text-anchor': 'middle' }, svg).textContent = v.toFixed(xt.decimals);
      }
      el('line', { class: 'axis', x1: left, x2: width - right, y1: height - bottom, y2: height - bottom }, svg);
      // The record: min and max per pixel column (exact look, small path);
      // a new sub-path after every break.
      let d = '', col = null, cmin = null, cmax = null, broken = true;
      const flush = () => {
        if (col === null) return;
        const a = cmin.i < cmax.i ? cmin : cmax, b = cmin.i < cmax.i ? cmax : cmin;
        d += (broken ? 'M' : 'L') + sx(a.t).toFixed(1) + ',' + sy(a.h).toFixed(1);
        if (b !== a) d += 'L' + sx(b.t).toFixed(1) + ',' + sy(b.h).toFixed(1);
        broken = false;
        col = null;
      };
      for (const { run, i0, i1 } of spans) {
        flush(); broken = true;
        for (let i = i0; i <= i1; i++) {
          const t = run.start + i * rec.dt, h = run.h[i], c = Math.floor(sx(t));
          if (t > upTo) break;
          if (h === null || !isFinite(h)) { flush(); broken = true; continue; }
          const p = { i, t, h };
          if (c !== col) { flush(); col = c; cmin = p; cmax = p; }
          else { if (h < cmin.h) cmin = p; if (h > cmax.h) cmax = p; }
        }
        flush();
      }
      for (const l of rec.lines || []) {
        el('line', { class: 'refline', x1: left, x2: width - right, y1: sy(l.v), y2: sy(l.v) }, svg);
        el('text', { class: 'refline-label', x: width - right - 4, y: sy(l.v) - 4, 'text-anchor': 'end' }, svg).textContent = l.label;
      }
      el('path', { class: 'series', d }, svg);
      // Labelled moments; a label too close to the one before drops a
      // line so they never overlap.
      let lastEnd = -Infinity, row = 0;
      for (const m of rec.marks || []) {
        if (m.t < t0 || m.t > t1 || m.t > upTo) continue;
        const x = sx(m.t);
        el('line', { class: 'front', x1: x, x2: x, y1: top, y2: height - bottom }, svg);
        row = x < lastEnd + 8 ? row + 1 : 0;
        const label = el('text', { class: 'front-label', x: x + 4, y: top - 6 + row * 13 }, svg);
        label.textContent = m.label;
        lastEnd = x + 4 + 6.5 * m.label.length;
      }
      for (const f of rec.fronts) {
        if (f < t0 || f > t1 || f > upTo) continue;
        el('line', { class: 'front', x1: sx(f), x2: sx(f), y1: top, y2: height - bottom }, svg);
        el('text', { class: 'front-label', x: sx(f) + 4, y: top - 6 }, svg).textContent = 'front ' + f.toFixed(3) + ' s';
      }
      if (upTo < Infinity && upTo >= t0 && upTo <= t1) {
        el('line', { class: 'now', x1: sx(upTo), x2: sx(upTo), y1: top, y2: height - bottom }, svg);
      }
      const cross = el('line', { class: 'crosshair', x1: 0, x2: 0, y1: top, y2: height - bottom, visibility: 'hidden' }, svg);
      const sel = el('rect', { class: 'selection', x: 0, y: top, width: 0, height: height - top - bottom, visibility: 'hidden' }, svg);
      const hit = el('rect', { class: 'hit', x: left, y: top, width: width - left - right, height: height - top - bottom }, svg);
      charts.push({ svg, rec, cross, sel, hit });
    }

    const localX = (svg, e) => {
      const pt = svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY;
      return pt.matrixTransform(svg.getScreenCTM().inverse()).x;
    };
    let drag = null;
    for (const c of charts) {
      c.hit.addEventListener('pointermove', e => {
        const x = Math.max(left, Math.min(width - right, localX(c.svg, e)));
        const t = tOf(x);
        for (const o of charts) {
          o.cross.setAttribute('x1', x); o.cross.setAttribute('x2', x);
          o.cross.setAttribute('visibility', 'visible');
        }
        const tunit = (charts[0] && charts[0].rec.tunit) || 's';
        const rows = [{ text: 't = ' + t.toFixed(tunit === 's' ? 3 : 2) + ' ' + tunit, strong: true }];
        for (const o of charts) {
          const v = valueAt(o.rec, t);
          rows.push({ label: o.rec.name, value: (v === null || !isFinite(v)) ? 'no data' : v.toFixed(2) + ' ' + (o.rec.vunit || 'm') });
        }
        showTip(rows, e.clientX, e.clientY);
        if (drag && drag.chart === c) {
          const a = Math.min(drag.x, x), b = Math.max(drag.x, x);
          c.sel.setAttribute('x', a); c.sel.setAttribute('width', b - a); c.sel.setAttribute('visibility', 'visible');
        }
      });
      c.hit.addEventListener('pointerleave', () => {
        for (const o of charts) o.cross.setAttribute('visibility', 'hidden');
        hideTip();
      });
      if (onZoom) {
        c.hit.addEventListener('pointerdown', e => {
          drag = { chart: c, x: localX(c.svg, e) };
          c.hit.setPointerCapture(e.pointerId);
        });
        c.hit.addEventListener('pointerup', e => {
          if (!drag) return;
          const x = localX(c.svg, e), from = drag.x;
          drag = null;
          c.sel.setAttribute('visibility', 'hidden');
          if (Math.abs(x - from) > 6) onZoom([tOf(Math.min(x, from)), tOf(Math.max(x, from))]);
        });
      }
    }
  }

  // Containers: <div class="js-traces" data-set data-from data-to
  // [data-zoomable]>; data-set names the records to draw (default "event").
  const ranges = new Map();
  const nows = new Map();
  function render(container) {
    if (!ranges.has(container)) ranges.set(container, [Number(container.dataset.from), Number(container.dataset.to)]);
    const zoomable = container.hasAttribute('data-zoomable');
    const records = data.sets[container.dataset.set || 'event'] || [];
    drawStack(container, records, ranges.get(container), zoomable ? r => setRange(container, r, null) : null, nows.get(container));
  }

  // Playback: a [data-play=SECONDS] button in a container's controls plays
  // its whole range in that many seconds; the container's data-events
  // ([{t, text, kind}], in time order) appear in its feed
  // (#<container id>-feed) as their moment passes - as the engine would
  // have raised them.
  function setupPlay(container) {
    const bar = document.querySelector(`[data-controls="${container.id}"]`);
    const btn = bar && bar.querySelector('[data-play]');
    if (!btn) return;
    const feed = document.getElementById(container.id + '-feed');
    const events = JSON.parse(container.dataset.events || '[]');
    const unit = container.dataset.unit || 's';
    let timer = null, shown = 0;
    const stop = () => { clearInterval(timer); timer = null; btn.textContent = 'Play again'; };
    const add = e => {
      if (!feed) return;
      const row = document.createElement('div');
      row.className = 'feed-row ' + (e.kind || '');
      const at = document.createElement('span');
      at.className = 'feed-t';
      at.textContent = e.at || (e.t.toFixed(unit === 's' ? 2 : 0) + ' ' + unit);
      row.append(at, document.createTextNode(' ' + e.text));
      feed.prepend(row);
    };
    btn.addEventListener('click', () => {
      if (timer) { clearInterval(timer); timer = null; btn.textContent = 'Play'; return; }
      const [a, b] = [Number(container.dataset.from), Number(container.dataset.to)];
      ranges.set(container, [a, b]);
      let now = nows.get(container);
      if (now === undefined || now >= b) {
        now = a; shown = 0;
        if (feed) feed.replaceChildren();
      }
      const total = Number(btn.dataset.play) || 30;
      btn.textContent = 'Pause';
      let last = performance.now();
      timer = setInterval(() => {
        const t = performance.now();
        now = Math.min(b, now + (b - a) * (t - last) / 1000 / total);
        last = t;
        nows.set(container, now);
        render(container);
        while (shown < events.length && events[shown].t <= now) { add(events[shown]); shown++; }
        const hint = bar.querySelector('.hint');
        if (hint) hint.textContent = `playing: ${now.toFixed(unit === 's' ? 1 : 0)} ${unit} of ${b.toFixed(0)}`;
        if (now >= b) stop();
      }, 100);
    });
  }
  document.querySelectorAll('.js-traces').forEach(setupPlay);
  function setRange(container, range, preset) {
    ranges.set(container, range);
    const bar = document.querySelector(`[data-controls="${container.id}"]`);
    if (bar) {
      bar.querySelectorAll('[data-range]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.range === preset)));
      const shown = bar.querySelector('.hint');
      const unit = container.dataset.unit || 's';
      if (shown) shown.textContent = `showing ${range[0].toFixed(2)} - ${range[1].toFixed(2)} ${unit}` + (preset ? '' : ' (zoomed)') + ' - drag across a chart to zoom';
    }
    render(container);
  }
  document.querySelectorAll('[data-controls]').forEach(bar => {
    const container = document.getElementById(bar.dataset.controls);
    bar.querySelectorAll('[data-range]').forEach(b => b.addEventListener('click', () =>
      setRange(container, b.dataset.range.split(',').slice(1).map(Number), b.dataset.range)));
  });
  function renderVisible() {
    document.querySelectorAll('.js-traces').forEach(c => { if (c.offsetParent !== null) render(c); });
  }
  let resizeTimer = null;
  window.addEventListener('resize', () => { clearTimeout(resizeTimer); resizeTimer = setTimeout(renderVisible, 150); });

  route();
  window.scrollTo(0, 0);
})();
