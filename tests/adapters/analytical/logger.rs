//! Reading logger exports: hand-made files, every number known.

use jk_files::logger::{CsvOptions, PressureUnit, read};
use jk_tests::{check, check_close};

use crate::fixture_path;

/// Metres of head in one bar (998.2 kg/m^3, g = 9.81).
const BAR: f64 = 1e5 / (998.2 * 9.81);

fn show_notes(notes: &[jk_files::Note]) {
    println!("  reader's notes ({}):", notes.len());
    for n in notes {
        match n.line {
            Some(l) => println!("    - line {l}: {}", n.text),
            None => println!("    - {}", n.text),
        }
    }
}

#[test]
fn a_typical_export_with_a_gap_an_empty_cell_and_a_step() {
    // 128 S/s, ms-rounded UTC timestamps, 2 lines of metadata, pressure in
    // bar next to a temperature column; 64 rows (0.5 s) lost at t = 2 s;
    // one empty cell; the pressure drops 0.5 bar at t = 1.5 s.
    let r = read(&fixture_path("logger_bar.csv"), &CsvOptions::default()).unwrap();
    show_notes(&r.notes);
    let says = |s: &str| r.notes.iter().any(|n| n.text.contains(s));
    check_close(
        "sampling interval [s]",
        r.interval.value(),
        1.0 / 128.0,
        1e-12,
    );
    check(
        "one pressure channel (temperature left out)",
        r.channels.len() == 1 && says("looks like temperature"),
    );
    check(
        "two runs: the lost half second splits it",
        r.runs.len() == 2 && says("64 sample(s)"),
    );
    check_close(
        "second run starts at 10:15:02.500 (s after the first)",
        r.runs[1].start.value() - r.runs[0].start.value(),
        2.5,
        1e-9,
    );
    let first = &r.runs[0].heads[0];
    check_close("4.5 bar in metres of head", first[0], 4.5 * BAR, 1e-12);
    check(
        "the empty cell is a gap (NaN), not filled",
        first[10].is_nan() && says("1 empty"),
    );
    let step = first[200] - first[100];
    check_close("the 0.5 bar drop, in metres", step, -0.5 * BAR, 1e-9);
    check(
        "rounded timestamps noted",
        says("off the logger's regular clock"),
    );
    check("time zone noted: UTC", says("times are UTC"));
    check("metadata lines skipped", says("2 line(s) of metadata"));
}

#[test]
fn a_european_export_semicolons_decimal_commas_date_and_time_columns() {
    // 1 S/s in kPa, day-first dates crossing midnight, one repeated row.
    let r = read(&fixture_path("logger_eu.csv"), &CsvOptions::default()).unwrap();
    show_notes(&r.notes);
    let says = |s: &str| r.notes.iter().any(|n| n.text.contains(s));
    check_close("1 sample a second", r.interval.value(), 1.0, 1e-12);
    check("one continuous run across midnight", r.runs.len() == 1);
    check(
        "the repeated row is dropped and noted",
        says("repeat an earlier time") && r.samples() == 10,
    );
    check_close(
        "451.2 kPa in metres of head",
        r.runs[0].heads[0][0],
        451.2 * 1e3 / (998.2 * 9.81),
        1e-12,
    );
    check("decimal comma noted", says("decimal comma"));
    check("no time zone: noted", says("no time zone"));
}

#[test]
fn a_column_without_a_unit_is_refused_unless_the_unit_is_given() {
    let path = fixture_path("logger_nounit.csv");
    let e = read(&path, &CsvOptions::default()).unwrap_err();
    println!("  refused: {e}");
    check(
        "refused: units are never guessed",
        e.message.contains("never guessed"),
    );
    let given = CsvOptions {
        unit: Some(PressureUnit::Bar),
        ..CsvOptions::default()
    };
    let r = read(&path, &given).unwrap();
    show_notes(&r.notes);
    check(
        "read when the unit is given, and noted",
        r.notes.iter().any(|n| n.text.contains("as instructed")),
    );
    check_close("2 samples a second", r.interval.value(), 0.5, 1e-12);
}
