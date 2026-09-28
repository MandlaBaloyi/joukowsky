//! Dense Cholesky factorisation for symmetric positive-definite systems.
//!
//! Shared by the steady-state solver (its nodal matrix) and localisation
//! (the covariance of arrival-time errors). Both matrices are symmetric and
//! positive definite whenever the physics is well posed, so Cholesky
//! (A = L L^T) is exact, stable without pivoting, and fails loudly - with
//! the row where it broke - when that assumption is violated.
//!
//! Dense on purpose (decisions TD-011, TD-018: our own): the matrices so far
//! are small (tens to a few hundred rows). Cost grows as n^3/3; a sparse
//! factorisation comes with municipal-scale networks.

/// Factors A = L L^T in place: on return the lower triangle of `a` (n x n,
/// row-major) holds L. Only the lower triangle of A is read.
///
/// # Errors
/// The row at which A turned out not to be positive definite.
pub fn cholesky(a: &mut [f64], n: usize) -> Result<(), usize> {
    debug_assert_eq!(a.len(), n * n);
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
    Ok(())
}

/// Solves L L^T x = b in place (`b` becomes x), given the factor from
/// [`cholesky`].
pub fn solve_factored(l: &[f64], b: &mut [f64], n: usize) {
    debug_assert_eq!(b.len(), n);
    for i in 0..n {
        let row_i = i * n;
        let mut s = b[i];
        for k in 0..i {
            s -= l[row_i + k] * b[k];
        }
        b[i] = s / l[row_i + i];
    }
    for i in (0..n).rev() {
        let mut s = b[i];
        for k in (i + 1)..n {
            s -= l[k * n + i] * b[k];
        }
        b[i] = s / l[i * n + i];
    }
}

/// log det A from its Cholesky factor: 2 sum ln `L_ii`.
#[must_use]
pub fn log_det_factored(l: &[f64], n: usize) -> f64 {
    2.0 * (0..n).map(|i| l[i * n + i].ln()).sum::<f64>()
}

/// Solves A x = b in place: on return `b` holds x and `a` holds L.
///
/// # Errors
/// The row at which A turned out not to be positive definite.
pub fn cholesky_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Result<(), usize> {
    cholesky(a, n)?;
    solve_factored(a, b, n);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_a_known_system() {
        // A = [[4, 12, -16], [12, 37, -43], [-16, -43, 98]] (textbook example
        // with L = [[2,0,0],[6,1,0],[-8,5,3]]); x = [1, 2, 3].
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
        // det A = (2 * 1 * 3)^2 = 36.
        assert!((log_det_factored(&a, 3) - 36.0_f64.ln()).abs() < 1e-12);
    }

    #[test]
    fn reports_a_matrix_that_is_not_positive_definite() {
        let mut a = vec![1.0, -1.0, -1.0, 1.0];
        let mut b = vec![0.0, 0.0];
        assert_eq!(cholesky_solve(&mut a, &mut b, 2), Err(1));
    }
}
