use std::fmt;

/// Why a hydraulic model could not be built or run.
///
/// Bad input is reported, never silently corrected (principle P6).
#[derive(Debug, Clone, PartialEq)]
pub enum HydraulicsError {
    /// A parameter is outside its physically meaningful range.
    InvalidParameter {
        /// Parameter name as used in the docs.
        name: &'static str,
        /// The rejected value, in SI units.
        value: f64,
        /// What the value must satisfy.
        requirement: &'static str,
    },
    /// The valve has flow through it but no head drop across it, so its
    /// discharge coefficient is undefined.
    NoHeadDropAtValve {
        /// Head drop across the valve at steady state [m].
        head_drop: f64,
    },
    /// A requested probe node does not exist on the grid.
    ProbeOutOfRange {
        /// The requested node index.
        node: usize,
        /// Index of the last node.
        last: usize,
    },
}

impl HydraulicsError {
    pub(crate) fn invalid(name: &'static str, value: f64, requirement: &'static str) -> Self {
        Self::InvalidParameter {
            name,
            value,
            requirement,
        }
    }
}

impl fmt::Display for HydraulicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter {
                name,
                value,
                requirement,
            } => write!(f, "invalid {name} = {value}: must be {requirement}"),
            Self::NoHeadDropAtValve { head_drop } => write!(
                f,
                "valve passes flow but its steady head drop is {head_drop} m; \
                 the downstream head must be below the head at the valve"
            ),
            Self::ProbeOutOfRange { node, last } => {
                write!(f, "probe node {node} does not exist (last node is {last})")
            }
        }
    }
}

impl std::error::Error for HydraulicsError {}
