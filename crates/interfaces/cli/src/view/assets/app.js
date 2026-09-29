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
  const valueAt = (rec, t) => {
    const i = Math.round((t - rec.start) / rec.dt);
    return (i >= 0 && i < rec.h.length) ? rec.h[i] : null;
  };

  // Draws every logger's record between range[0] and range[1] seconds,
  // stacked, sharing the time axis. onZoom(range) is called when the
  // reader drags across a chart (if given).
  function drawStack(container, range, onZoom) {
    container.replaceChildren();
    const width = Math.max(320, container.clientWidth);
    const height = 150, left = 52, right = 12, top = 20, bottom = 24;
    const [t0, t1] = range;
    const sx = t => left + (t - t0) / (t1 - t0) * (width - left - right);
    const tOf = x => t0 + (x - left) / (width - left - right) * (t1 - t0);
    const charts = [];
    for (const rec of data.records) {
      const title = document.createElement('div');
      title.className = 'chart-title';
      title.textContent = rec.name;
      container.appendChild(title);
      const svg = el('svg', { class: 'chart trace', viewBox: `0 0 ${width} ${height}`, role: 'img' }, container);
      svg.setAttribute('aria-label', 'Pressure head at ' + rec.name);
      const i0 = Math.max(0, Math.floor((t0 - rec.start) / rec.dt));
      const i1 = Math.min(rec.h.length - 1, Math.ceil((t1 - rec.start) / rec.dt));
      let lo = Infinity, hi = -Infinity;
      for (let i = i0; i <= i1; i++) { lo = Math.min(lo, rec.h[i]); hi = Math.max(hi, rec.h[i]); }
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
      // The record: min and max per pixel column (exact look, small path).
      let d = '', col = null, cmin = null, cmax = null;
      const flush = () => {
        if (col === null) return;
        const a = cmin.i < cmax.i ? cmin : cmax, b = cmin.i < cmax.i ? cmax : cmin;
        d += (d ? 'L' : 'M') + sx(a.t).toFixed(1) + ',' + sy(a.h).toFixed(1);
        if (b !== a) d += 'L' + sx(b.t).toFixed(1) + ',' + sy(b.h).toFixed(1);
      };
      for (let i = i0; i <= i1; i++) {
        const t = rec.start + i * rec.dt, h = rec.h[i], c = Math.floor(sx(t));
        const p = { i, t, h };
        if (c !== col) { flush(); col = c; cmin = p; cmax = p; }
        else { if (h < cmin.h) cmin = p; if (h > cmax.h) cmax = p; }
      }
      flush();
      el('path', { class: 'series', d }, svg);
      for (const f of rec.fronts) {
        if (f < t0 || f > t1) continue;
        el('line', { class: 'front', x1: sx(f), x2: sx(f), y1: top, y2: height - bottom }, svg);
        el('text', { class: 'front-label', x: sx(f) + 4, y: top - 6 }, svg).textContent = 'front ' + f.toFixed(3) + ' s';
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
        const rows = [{ text: 't = ' + t.toFixed(3) + ' s', strong: true }];
        for (const o of charts) {
          const v = valueAt(o.rec, t);
          rows.push({ label: o.rec.name, value: v === null ? '-' : v.toFixed(2) + ' m' });
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

  // Containers: <div class="js-traces" data-from data-to [data-zoomable]>.
  const ranges = new Map();
  function render(container) {
    if (!ranges.has(container)) ranges.set(container, [Number(container.dataset.from), Number(container.dataset.to)]);
    const zoomable = container.hasAttribute('data-zoomable');
    drawStack(container, ranges.get(container), zoomable ? r => setRange(container, r, null) : null);
  }
  function setRange(container, range, preset) {
    ranges.set(container, range);
    const bar = document.querySelector(`[data-controls="${container.id}"]`);
    if (bar) {
      bar.querySelectorAll('[data-range]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.range === preset)));
      const shown = bar.querySelector('.hint');
      if (shown) shown.textContent = `showing ${range[0].toFixed(2)} - ${range[1].toFixed(2)} s` + (preset ? '' : ' (zoomed)') + ' - drag across a chart to zoom';
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
