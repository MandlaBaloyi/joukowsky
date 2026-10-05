//! Rung 4 - real data: files published by others, read and run by the
//! engine, and compared with the truth that comes with them.
//!
//! The files live in `data/incoming/` (data/README.md: what each dataset
//! is, its licence, its fingerprint). They are not in git. When a dataset
//! is not on this machine its tests print "SKIPPED" and pass - they never
//! fail for missing data and never fetch anything. `.\data\verify.ps1`
//! checks the files are unchanged before trusting these results.

mod epanet_examples;
mod ltown;

use std::path::PathBuf;

/// Path of `data/incoming/<rel>` if the file is on this machine; otherwise
/// says the test is skipped and why.
#[must_use]
pub fn dataset(rel: &str) -> Option<PathBuf> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("data")
        .join("incoming")
        .join(rel);
    if path.exists() {
        println!("  dataset file: data/incoming/{rel}");
        Some(path)
    } else {
        println!(
            "  SKIPPED: data/incoming/{rel} is not on this machine (see data/README.md to get it)"
        );
        None
    }
}

/// The IDs of the records of an `.inp` section, as the file writes them.
#[must_use]
pub fn section_ids(text: &str, section: &str) -> Vec<String> {
    let mut inside = false;
    let mut ids = Vec::new();
    for line in text.lines() {
        let line = line.split(';').next().unwrap_or("").trim();
        if line.starts_with('[') {
            inside = line.eq_ignore_ascii_case(section);
            continue;
        }
        if inside && let Some(id) = line.split_whitespace().next() {
            ids.push(id.to_string());
        }
    }
    ids
}
