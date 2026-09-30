//! Sparse symmetric positive-definite systems: our own envelope (profile)
//! Cholesky with a reverse Cuthill-McKee ordering.
//!
//! The steady solver's nodal matrix has one row per junction, but each row
//! has only as many non-zeros as the junction has pipes. Stored dense it
//! costs n^2 memory and n^3/3 work - 2000 junctions (about 180 km of town)
//! was the practical limit. Here:
//!
//! 1. Reverse `Cuthill-McKee` ordering (1969; George 1971): number
//!    the unknowns breadth-first from a far-out node, neighbours in order of
//!    degree, then reverse. Connected nodes get close numbers, so every
//!    row's non-zeros sit in a narrow band left of the diagonal.
//! 2. Envelope storage: row i keeps its entries from its first non-zero
//!    column to the diagonal. Cholesky creates no fill outside that
//!    envelope, so the factor fits in the same storage.
//! 3. Row-by-row (bordering) Cholesky and the two triangular solves.
//!
//! For a street grid of n junctions the band is about sqrt(n) wide: work
//! ~ n^2 instead of n^3, memory ~ n^1.5 instead of n^2. The pattern is set
//! up once and refilled every Newton round.
//!
//! Own code (decisions TD-011, TD-018: the brain is ours); checked against
//! the dense [`crate::linalg::cholesky`] in the tests.

use std::collections::VecDeque;

/// A symmetric matrix pattern and values, in envelope storage, rows and
/// columns renumbered by reverse Cuthill-McKee.
#[derive(Debug, Clone)]
pub struct Envelope {
    /// New number of each original row.
    new_of: Vec<usize>,
    /// Original row of each new number.
    old_of: Vec<usize>,
    /// Per new row: first stored column.
    first: Vec<usize>,
    /// Per new row: where its entries start in `values`.
    start: Vec<usize>,
    /// Row i holds columns first[i]..=i at start[i]...
    values: Vec<f64>,
}

impl Envelope {
    /// The pattern of an n x n symmetric matrix whose off-diagonal
    /// non-zeros are at the `edges` (i, j) - either order, repeats allowed.
    #[must_use]
    pub fn new(n: usize, edges: &[(usize, usize)]) -> Self {
        let mut neighbours = vec![Vec::new(); n];
        for &(i, j) in edges {
            if i != j {
                neighbours[i].push(j);
                neighbours[j].push(i);
            }
        }
        for list in &mut neighbours {
            list.sort_unstable();
            list.dedup();
        }
        let old_of = reverse_cuthill_mckee(&neighbours);
        let mut new_of = vec![0; n];
        for (new, &old) in old_of.iter().enumerate() {
            new_of[old] = new;
        }
        let mut first: Vec<usize> = (0..n).collect();
        for (i, list) in neighbours.iter().enumerate() {
            let a = new_of[i];
            for &j in list {
                let b = new_of[j];
                if b < a {
                    first[a] = first[a].min(b);
                }
            }
        }
        let mut start = Vec::with_capacity(n + 1);
        let mut total = 0;
        for (i, &f) in first.iter().enumerate() {
            start.push(total);
            total += i - f + 1;
        }
        start.push(total);
        Self {
            new_of,
            old_of,
            first,
            start,
            values: vec![0.0; total],
        }
    }

    /// Number of rows.
    #[must_use]
    pub fn len(&self) -> usize {
        self.first.len()
    }

    /// True for a 0 x 0 matrix.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.first.is_empty()
    }

    /// Stored entries (the envelope's size).
    #[must_use]
    pub fn stored(&self) -> usize {
        self.values.len()
    }

    /// Sets every value to zero (the pattern stays).
    pub fn clear(&mut self) {
        self.values.fill(0.0);
    }

    /// Adds `v` to entry (i, j) - and by symmetry (j, i) - in the ORIGINAL
    /// numbering. (i, j) must be on the diagonal or one of the edges.
    ///
    /// # Panics
    /// If (i, j) is outside the pattern.
    pub fn add(&mut self, i: usize, j: usize, v: f64) {
        let (a, b) = (self.new_of[i], self.new_of[j]);
        let (row, col) = if a >= b { (a, b) } else { (b, a) };
        assert!(col >= self.first[row], "entry outside the pattern");
        self.values[self.start[row] + col - self.first[row]] += v;
    }

    /// Factors A = L L^T in place.
    ///
    /// # Errors
    /// The ORIGINAL row at which A turned out not to be positive definite.
    pub fn factor(&mut self) -> Result<(), usize> {
        for i in 0..self.len() {
            let (fi, si) = (self.first[i], self.start[i]);
            for j in fi..i {
                let (fj, sj) = (self.first[j], self.start[j]);
                let from = fi.max(fj);
                // Row i and row j over columns from..j.
                let dot: f64 = self.values[si + from - fi..si + j - fi]
                    .iter()
                    .zip(&self.values[sj + from - fj..sj + j - fj])
                    .map(|(a, b)| a * b)
                    .sum();
                let diag = self.values[sj + j - fj];
                let v = &mut self.values[si + j - fi];
                *v = (*v - dot) / diag;
            }
            let row = &self.values[si..si + i - fi];
            let d = self.values[si + i - fi] - row.iter().map(|l| l * l).sum::<f64>();
            if !(d > 0.0 && d.is_finite()) {
                return Err(self.old_of[i]);
            }
            self.values[si + i - fi] = d.sqrt();
        }
        Ok(())
    }

    /// Solves A x = b after [`Self::factor`]: `b` (original numbering)
    /// becomes x.
    pub fn solve(&self, b: &mut [f64]) {
        let n = self.len();
        let mut y: Vec<f64> = (0..n).map(|i| b[self.old_of[i]]).collect();
        for i in 0..n {
            let fi = self.first[i];
            let row = &self.values[self.start[i]..=self.start[i] + i - fi];
            let dot: f64 = row[..i - fi]
                .iter()
                .zip(&y[fi..i])
                .map(|(l, v)| l * v)
                .sum();
            y[i] = (y[i] - dot) / row[i - fi];
        }
        for i in (0..n).rev() {
            let fi = self.first[i];
            let row = &self.values[self.start[i]..=self.start[i] + i - fi];
            y[i] /= row[i - fi];
            let yi = y[i];
            for (l, v) in row[..i - fi].iter().zip(&mut y[fi..i]) {
                *v -= l * yi;
            }
        }
        for (i, v) in y.into_iter().enumerate() {
            b[self.old_of[i]] = v;
        }
    }
}

/// Reverse Cuthill-McKee order of a graph given by sorted neighbour lists:
/// the original node of each new number. Every connected component is
/// numbered from a pseudo-peripheral node of least degree.
fn reverse_cuthill_mckee(neighbours: &[Vec<usize>]) -> Vec<usize> {
    let n = neighbours.len();
    let mut order = Vec::with_capacity(n);
    let mut seen = vec![false; n];
    let degree = |i: usize| neighbours[i].len();
    let mut by_degree: Vec<usize> = (0..n).collect();
    by_degree.sort_by_key(|&i| (degree(i), i));
    for &seed in &by_degree {
        if seen[seed] {
            continue;
        }
        let root = far_node(neighbours, seed);
        seen[root] = true;
        let mut queue = VecDeque::from([root]);
        while let Some(v) = queue.pop_front() {
            order.push(v);
            let mut next: Vec<usize> = neighbours[v]
                .iter()
                .copied()
                .filter(|&w| !seen[w])
                .collect();
            next.sort_by_key(|&w| (degree(w), w));
            for w in next {
                seen[w] = true;
                queue.push_back(w);
            }
        }
    }
    order.reverse();
    order
}

/// A pseudo-peripheral node of `start`'s component (George & Liu): the
/// least-degree node of the last breadth-first level, repeated while the
/// levels get deeper.
fn far_node(neighbours: &[Vec<usize>], start: usize) -> usize {
    let mut root = start;
    let mut depth = 0;
    loop {
        let levels = levels_from(neighbours, root);
        let last = levels.last().expect("the root is a level");
        let candidate = *last
            .iter()
            .min_by_key(|&&w| (neighbours[w].len(), w))
            .expect("levels are not empty");
        if levels.len() <= depth || candidate == root {
            return root;
        }
        depth = levels.len();
        root = candidate;
    }
}

fn levels_from(neighbours: &[Vec<usize>], root: usize) -> Vec<Vec<usize>> {
    let mut depth = vec![usize::MAX; neighbours.len()];
    depth[root] = 0;
    let mut levels = vec![vec![root]];
    loop {
        let mut next = Vec::new();
        for &v in levels.last().expect("at least the root") {
            for &w in &neighbours[v] {
                if depth[w] == usize::MAX {
                    depth[w] = levels.len();
                    next.push(w);
                }
            }
        }
        if next.is_empty() {
            return levels;
        }
        levels.push(next);
    }
}

#[cfg(test)]
mod tests {
    // Random indices from uniform numbers: small, positive, in range.
    #![allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]

    use super::*;
    use crate::linalg::cholesky_solve;
    use crate::random::Rng;

    /// A random connected graph Laplacian plus a positive diagonal (a
    /// nodal matrix with some fixed heads), and its edges.
    fn laplacian(n: usize, rng: &mut Rng) -> (Vec<f64>, Vec<(usize, usize)>) {
        let mut a = vec![0.0; n * n];
        let mut edges = Vec::new();
        let mut link = |i: usize, j: usize, g: f64, a: &mut Vec<f64>| {
            a[i * n + i] += g;
            a[j * n + j] += g;
            a[i * n + j] -= g;
            a[j * n + i] -= g;
            edges.push((i, j));
        };
        for i in 1..n {
            let j = (rng.uniform() * i as f64) as usize;
            link(i, j, 0.5 + rng.uniform(), &mut a);
        }
        for _ in 0..n / 2 {
            let (i, j) = (
                (rng.uniform() * n as f64) as usize,
                (rng.uniform() * n as f64) as usize,
            );
            if i != j {
                link(i, j, 0.5 + rng.uniform(), &mut a);
            }
        }
        for i in 0..n {
            if rng.uniform() < 0.1 || i == 0 {
                a[i * n + i] += 1.0; // a pipe to a reservoir
            }
        }
        (a, edges)
    }

    #[test]
    fn solves_what_the_dense_cholesky_solves() {
        let mut rng = Rng::new(21);
        for n in [1, 2, 5, 40, 150] {
            let (a, edges) = laplacian(n, &mut rng);
            let b: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
            let mut env = Envelope::new(n, &edges);
            for i in 0..n {
                env.add(i, i, a[i * n + i]);
            }
            // Fill off-diagonals from the dense matrix, once per pair.
            let mut done = std::collections::HashSet::new();
            for &(i, j) in &edges {
                let key = (i.min(j), i.max(j));
                if i != j && done.insert(key) {
                    env.add(key.0, key.1, a[key.0 * n + key.1]);
                }
            }
            env.factor().unwrap();
            let mut x = b.clone();
            env.solve(&mut x);
            let (mut dense, mut y) = (a.clone(), b.clone());
            cholesky_solve(&mut dense, &mut y, n).unwrap();
            for (u, v) in x.iter().zip(&y) {
                assert!(
                    (u - v).abs() <= 1e-10 * v.abs().max(1.0),
                    "n = {n}: {u} vs {v}"
                );
            }
        }
    }

    #[test]
    fn a_grid_gets_a_band_about_as_wide_as_its_side() {
        // 30 x 30 grid, numbered badly (a random permutation).
        let side = 30;
        let n = side * side;
        let mut rng = Rng::new(5);
        let mut perm: Vec<usize> = (0..n).collect();
        for i in (1..n).rev() {
            let j = (rng.uniform() * (i + 1) as f64) as usize;
            perm.swap(i, j.min(i));
        }
        let mut edges = Vec::new();
        for r in 0..side {
            for c in 0..side {
                if c + 1 < side {
                    edges.push((perm[r * side + c], perm[r * side + c + 1]));
                }
                if r + 1 < side {
                    edges.push((perm[r * side + c], perm[(r + 1) * side + c]));
                }
            }
        }
        let env = Envelope::new(n, &edges);
        // A band of about `side` per row: ~ n * side stored, not n^2 / 2.
        assert!(env.stored() < 2 * n * side, "stored {}", env.stored());
    }

    #[test]
    fn a_singular_matrix_names_its_row() {
        // Two nodes joined, neither tied to a fixed head: singular.
        let mut env = Envelope::new(2, &[(0, 1)]);
        env.add(0, 0, 1.0);
        env.add(1, 1, 1.0);
        env.add(0, 1, -1.0);
        assert!(env.factor().is_err());
    }
}
