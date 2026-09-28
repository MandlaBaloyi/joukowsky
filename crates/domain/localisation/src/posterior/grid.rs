//! Candidate source points: cells every few metres along every pipe.

use jk_hydraulics::Network;

/// One candidate cell: a stretch of one pipe, represented by its centre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cell {
    /// Pipe index.
    pub pipe: usize,
    /// Centre, as a fraction of the pipe from its `from` end.
    pub fraction: f64,
    /// Length of pipe the cell covers [m].
    pub length: f64,
}

/// All candidate cells of a network.
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    /// The cells, pipe by pipe, in order along each pipe.
    pub cells: Vec<Cell>,
}

impl Grid {
    /// Cells no longer than `spacing` metres along every pipe.
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // ceil of positive
    pub fn new(network: &Network, spacing: f64) -> Self {
        let mut cells = Vec::new();
        for (pipe, p) in network.pipes().iter().enumerate() {
            let length = p.pipe.length().value();
            let n = ((length / spacing).ceil() as u32).max(1);
            let cell_length = length / f64::from(n);
            for i in 0..n {
                cells.push(Cell {
                    pipe,
                    fraction: (f64::from(i) + 0.5) / f64::from(n),
                    length: cell_length,
                });
            }
        }
        Self { cells }
    }
}
