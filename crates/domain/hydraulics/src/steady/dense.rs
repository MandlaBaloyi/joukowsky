//! Dense Cholesky solver for symmetric positive-definite systems.
//!
//! The steady-state solver's nodal matrix is symmetric and positive
//! definite whenever every part of the network reaches a fixed head, so
//! Cholesky (A = L L^T) is exact, stable without pivoting, and fails
//! loudly - with the row where it broke - when that assumption is violated.
//!
//! Dense on purpose (decision TD-011, build over buy): our first targets are
//! site networks of tens to hundreds of nodes, where this takes
//! microseconds to milliseconds. Cost grows as n^3/3, so the steady solver
//! caps n; a sparse factorisation comes with municipal-scale networks.

/// Solves A x = b in place: on return `b` holds x and `a` holds L.
///
/// `a` is n x n, row-major; only its lower triangle is read.
///
/// # Errors
/// The row at which A turned out not to be positive definite.
pub(crate) fn cholesky_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Result<(), usize> {
    debug_assert_eq!(a.len(), n * n);
    debug_assert_eq!(b.len(), n);

    // Factor: A = L L^T, L stored in the lower triangle of `a`.
    for j in 0..n {
        let row_j = j * n;
        let mut diag = a[row_j + j];
        for k in 0..j {
            diag -= a[row_j + k] * a[row_j + k];
        }
        if !(diag > 0.0 && diag.is_finite()) {
            return Err(j);
        }
        let l_jj = diag.sqrt();
        a[row_j + j] = l_jj;
        for i in (j + 1)..n {
            let row_i = i * n;
            let mut s = a[row_i + j];
            for k in 0..j {
                s -= a[row_i + k] * a[row_j + k];
            }
            a[row_i + j] = s / l_jj;
        }
    }

    // Forward: L y = b.
    for i in 0..n {
        let row_i = i * n;
        let mut s = b[i];
        for k in 0..i {
            s -= a[row_i + k] * b[k];
        }
        b[i] = s / a[row_i + i];
    }
    // Back: L^T x = y.
    for i in (0..n).rev() {
        let mut s = b[i];
        for k in (i + 1)..n {
            s -= a[k * n + i] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_a_known_system() {
        // A = [[4, 12, -16], [12, 37, -43], [-16, -43, 98]] (a textbook
        // example with L = [[2,0,0],[6,1,0],[-8,5,3]]); x = [1, 2, 3].
        let mut a = vec![4.0, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0];
        let x = [1.0, 2.0, 3.0];
        let mut b = vec![
            4.0 * x[0] + 12.0 * x[1] - 16.0 * x[2],
            12.0 * x[0] + 37.0 * x[1] - 43.0 * x[2],
            -16.0 * x[0] - 43.0 * x[1] + 98.0 * x[2],
        ];
        cholesky_solve(&mut a, &mut b, 3).unwrap();
        for (got, want) in b.iter().zip(x) {
            assert!((got - want).abs() < 1e-12, "{got} vs {want}");
        }
        assert!((a[0] - 2.0).abs() < 1e-15 && (a[3] - 6.0).abs() < 1e-15);
    }

    #[test]
    fn reports_a_matrix_that_is_not_positive_definite() {
        // Singular: a node whose head nothing pins down.
        let mut a = vec![1.0, -1.0, -1.0, 1.0];
        let mut b = vec![0.0, 0.0];
        assert_eq!(cholesky_solve(&mut a, &mut b, 2), Err(1));
    }
}
