/// Something in a file the engine approximated, did not use, or had to
/// repair - reported, never silently dropped (principles P6, P8).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Note {
    /// The line it concerns (1-based), if one.
    pub line: Option<usize>,
    /// What happened, in plain words.
    pub text: String,
}
