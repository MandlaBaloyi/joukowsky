//! `jk logger FILE`: read a logger export and say what is in it.

use std::path::PathBuf;

use jk_files::logger::{CsvOptions, PressureUnit, read};

pub const USAGE: &str = "
    jk logger FILE [--unit UNIT] [--month-first]
                          read a logger export (CSV) and summarise it, with
                          everything the reader had to decide or repair
";

/// What `jk logger` was asked for.
pub struct Args {
    file: PathBuf,
    options: CsvOptions,
}

/// Reads the arguments after `logger`.
///
/// # Errors
/// No file name, or an unknown option or unit.
pub fn parse(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut file = None;
    let mut options = CsvOptions::default();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--month-first" => options.day_first = false,
            "--unit" => {
                let v = it.next().ok_or("--unit needs a unit")?;
                options.unit = Some(PressureUnit::parse(&v).ok_or_else(|| {
                    format!("--unit: '{v}' is not bar, mbar, kPa, MPa, Pa, psi or m")
                })?);
            }
            _ if file.is_none() && !a.starts_with("--") => file = Some(PathBuf::from(a)),
            _ => return Err(format!("unexpected argument '{a}'")),
        }
    }
    Ok(Args {
        file: file.ok_or("jk logger needs a file name")?,
        options,
    })
}

/// Reads the file and prints the summary and the reader's notes.
///
/// # Errors
/// A file that cannot be read or is not a logger export.
pub fn run(a: &Args) -> Result<(), String> {
    let r = read(&a.file, &a.options).map_err(|e| format!("{}: {e}", a.file.display()))?;
    let dt = r.interval.value();
    #[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
    let seconds = r.samples() as f64 * dt;
    println!("{}", a.file.display());
    println!("  first sample {}", r.first_stamp);
    let every = if dt > 1.0 {
        format!("one sample every {dt:.0} s")
    } else {
        format!("{:.2} samples a second", 1.0 / dt)
    };
    println!(
        "  sampling     {every}, {} samples ({seconds:.1} s) in {} continuous run(s)",
        r.samples(),
        r.runs.len()
    );
    for c in &r.channels {
        println!("  channel      {} ({})", c.name, c.unit.name());
    }
    println!("What the reader decided or repaired ({}):", r.notes.len());
    for n in &r.notes {
        match n.line {
            Some(l) => println!("  - line {l}: {}", n.text),
            None => println!("  - {}", n.text),
        }
    }
    Ok(())
}
