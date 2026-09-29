//! The frame every page sits in: sidebar menu, title bar, theme switch.
//!
//! One self-contained file - no server, nothing downloaded: the stylesheet
//! and the script (`assets/`) are built into the `jk` binary and written
//! into the page. The data the charts draw is embedded as JSON.

use std::fmt::Write as _;

use super::svg::esc;

const STYLE: &str = include_str!("assets/style.css");
const SCRIPT: &str = include_str!("assets/app.js");

/// One entry of the sidebar menu.
pub struct NavItem {
    /// Page id (the part after `#`).
    pub id: &'static str,
    pub label: &'static str,
    /// A count shown on the right (e.g. number of alerts).
    pub count: Option<usize>,
}

/// A group of menu entries, under a heading (and the site it is about).
pub struct NavGroup {
    pub heading: &'static str,
    pub site: Option<String>,
    pub items: Vec<NavItem>,
}

/// One page: shown when its menu entry is chosen.
pub struct Page {
    pub id: &'static str,
    pub title: String,
    /// Small text next to the title (which site, which data).
    pub crumb: String,
    /// The page's HTML.
    pub body: String,
}

/// The complete document.
pub fn document(nav: &[NavGroup], pages: &[Page], data_json: &str) -> String {
    let mut menu = String::new();
    for g in nav {
        let _ = write!(menu, r#"<div class="nav-group">{}"#, esc(g.heading));
        if let Some(site) = &g.site {
            let _ = write!(menu, r#" <span class="site">&middot; {}</span>"#, esc(site));
        }
        menu.push_str("</div>");
        for i in &g.items {
            let count = i.count.map_or(String::new(), |c| {
                format!(r#"<span class="count">{c}</span>"#)
            });
            let _ = write!(
                menu,
                r##"<a href="#{id}" data-nav="{id}"><span>{}</span>{count}</a>"##,
                esc(i.label),
                id = i.id
            );
        }
    }
    let mut body = String::new();
    for p in pages {
        let _ = write!(
            body,
            r#"<section class="page" id="page-{}" data-page="{}" data-title="{}" data-crumb="{}" hidden>{}</section>"#,
            p.id,
            p.id,
            esc(&p.title),
            esc(&p.crumb),
            p.body
        );
    }
    // JSON inside a script element must never contain "</".
    let data = data_json.replace("</", "<\\/");
    format!(
        r#"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>joukowsky viewer</title>
<style>{STYLE}</style>
</head>
<body>
<div class="app">
<aside class="sidebar" aria-label="Menu">
  <div class="brand"><div class="name">joukowsky</div><div class="sub">engine viewer</div></div>
  <nav class="nav">{menu}</nav>
  <div class="spacer"></div>
  <div class="theme"><div class="label">Theme</div>
    <div class="segmented" role="group" aria-label="Theme">
      <button type="button" data-theme-choice="system">System</button>
      <button type="button" data-theme-choice="light">Light</button>
      <button type="button" data-theme-choice="dark">Dark</button>
    </div>
  </div>
</aside>
<main>
  <header class="topbar">
    <button type="button" id="menu-button" aria-label="Open the menu">Menu</button>
    <div><h1 id="page-title">Overview</h1><div class="crumb" id="page-crumb"></div></div>
    <div class="grow"></div>
    <span class="pill">Simulated data</span>
  </header>
  <div class="content">{body}</div>
</main>
</div>
<div id="tip" class="tip" hidden></div>
<script type="application/json" id="jk-data">{data}</script>
<script>{SCRIPT}</script>
</body>
</html>
"#
    )
}
