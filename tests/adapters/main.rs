//! Adapter tests: getting real data in.
//!
//! Code under test: `crates/adapters/files` (reading EPANET `.inp` network
//! files). The files in `fixtures/` are written by hand for these tests -
//! nothing is downloaded - so every expected number is known exactly.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only adapters`

mod analytical;

use std::path::PathBuf;

use jk_files::epanet::{InpNetwork, InpOptions, read};

/// Reads `fixtures/<name>` with the default options.
///
/// # Panics
/// If the file cannot be read (a test bug).
#[must_use]
pub fn fixture(name: &str) -> InpNetwork {
    read(&fixture_path(name), &InpOptions::default()).unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// Path of `fixtures/<name>`.
#[must_use]
pub fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("adapters")
        .join("fixtures")
        .join(name)
}

/// Prints the notes the reader made, as a reader of any output would see
/// them.
pub fn print_notes(net: &InpNetwork) {
    println!("  reader's notes ({}):", net.notes.len());
    for n in &net.notes {
        match n.line {
            Some(l) => println!("    - line {l}: {}", n.text),
            None => println!("    - {}", n.text),
        }
    }
}
