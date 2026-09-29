//! Splitting an `.inp` file into its sections.
//!
//! An `.inp` file is plain text: section headers in brackets (`[PIPES]`),
//! one record per line, fields separated by white space, comments after a
//! `;`. Keywords are case-insensitive. `[END]` ends the file.

use crate::error::ReadError;

/// The sections of the EPANET 2.2 / 2.3 format; each variant is the
/// section of that name.
#[allow(missing_docs)] // the names say it
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Section {
    Title,
    Junctions,
    Reservoirs,
    Tanks,
    Pipes,
    Pumps,
    Valves,
    Tags,
    Demands,
    Status,
    Patterns,
    Curves,
    Controls,
    Rules,
    Energy,
    Emitters,
    Leakage,
    Quality,
    Sources,
    Reactions,
    Mixing,
    Times,
    Report,
    Options,
    Coordinates,
    Vertices,
    Labels,
    Backdrop,
}

impl Section {
    fn parse(name: &str) -> Option<Self> {
        Some(match name.to_ascii_uppercase().as_str() {
            "TITLE" => Self::Title,
            "JUNCTIONS" => Self::Junctions,
            "RESERVOIRS" => Self::Reservoirs,
            "TANKS" => Self::Tanks,
            "PIPES" => Self::Pipes,
            "PUMPS" => Self::Pumps,
            "VALVES" => Self::Valves,
            "TAGS" => Self::Tags,
            "DEMANDS" => Self::Demands,
            "STATUS" => Self::Status,
            "PATTERNS" => Self::Patterns,
            "CURVES" => Self::Curves,
            "CONTROLS" => Self::Controls,
            "RULES" => Self::Rules,
            "ENERGY" => Self::Energy,
            "EMITTERS" => Self::Emitters,
            "LEAKAGE" => Self::Leakage,
            "QUALITY" => Self::Quality,
            "SOURCES" => Self::Sources,
            "REACTIONS" => Self::Reactions,
            "MIXING" => Self::Mixing,
            "TIMES" => Self::Times,
            "REPORT" => Self::Report,
            "OPTIONS" => Self::Options,
            "COORDINATES" => Self::Coordinates,
            "VERTICES" => Self::Vertices,
            "LABELS" => Self::Labels,
            "BACKDROP" => Self::Backdrop,
            _ => return None,
        })
    }

    /// The section's name as written in files.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Title => "TITLE",
            Self::Junctions => "JUNCTIONS",
            Self::Reservoirs => "RESERVOIRS",
            Self::Tanks => "TANKS",
            Self::Pipes => "PIPES",
            Self::Pumps => "PUMPS",
            Self::Valves => "VALVES",
            Self::Tags => "TAGS",
            Self::Demands => "DEMANDS",
            Self::Status => "STATUS",
            Self::Patterns => "PATTERNS",
            Self::Curves => "CURVES",
            Self::Controls => "CONTROLS",
            Self::Rules => "RULES",
            Self::Energy => "ENERGY",
            Self::Emitters => "EMITTERS",
            Self::Leakage => "LEAKAGE",
            Self::Quality => "QUALITY",
            Self::Sources => "SOURCES",
            Self::Reactions => "REACTIONS",
            Self::Mixing => "MIXING",
            Self::Times => "TIMES",
            Self::Report => "REPORT",
            Self::Options => "OPTIONS",
            Self::Coordinates => "COORDINATES",
            Self::Vertices => "VERTICES",
            Self::Labels => "LABELS",
            Self::Backdrop => "BACKDROP",
        }
    }
}

/// One record: where it is and its fields (comment removed).
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    /// 1-based line number in the file.
    pub number: usize,
    /// White-space separated fields.
    pub fields: Vec<String>,
    /// The line as written, comment removed and trimmed (for titles).
    pub text: String,
}

/// Every section in file order, with its records.
///
/// # Errors
/// Data before the first section header, or a header that is not an
/// EPANET section.
pub fn split(text: &str) -> Result<Vec<(Section, Vec<Line>)>, ReadError> {
    let mut out: Vec<(Section, Vec<Line>)> = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let number = i + 1;
        let content = raw.split(';').next().unwrap_or("").trim();
        if content.is_empty() {
            continue;
        }
        if let Some(name) = content.strip_prefix('[').and_then(|c| c.strip_suffix(']')) {
            if name.trim().eq_ignore_ascii_case("END") {
                break;
            }
            let section = Section::parse(name.trim()).ok_or_else(|| {
                ReadError::at(
                    number,
                    format!(
                        "[{}] is not an EPANET section - is this an .inp file?",
                        name.trim()
                    ),
                )
            })?;
            out.push((section, Vec::new()));
            continue;
        }
        let Some((_, lines)) = out.last_mut() else {
            return Err(ReadError::at(
                number,
                "data before the first [SECTION] header - is this an .inp file?",
            ));
        };
        lines.push(Line {
            number,
            fields: content.split_whitespace().map(str::to_string).collect(),
            text: content.to_string(),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_sections_drops_comments_stops_at_end() {
        let text = "; a comment\n[TITLE]\nMy net ; trailing\n[pipes]\nP1 A B 100 150 0.1\n\n[END]\n[JUNK]\n";
        let s = split(text).unwrap();
        assert_eq!(s.len(), 2);
        assert_eq!(s[0].0, Section::Title);
        assert_eq!(s[0].1[0].text, "My net");
        assert_eq!(s[1].0, Section::Pipes);
        assert_eq!(s[1].1[0].fields, ["P1", "A", "B", "100", "150", "0.1"]);
        assert_eq!(s[1].1[0].number, 5);
    }

    #[test]
    fn refuses_unknown_sections_and_stray_data() {
        assert_eq!(split("[PIPEZ]\n").unwrap_err().line, Some(1));
        assert_eq!(split("P1 A B\n").unwrap_err().line, Some(1));
    }
}
