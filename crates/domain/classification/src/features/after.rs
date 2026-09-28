//! What the signal did after the front.
//!
//! PERSISTENCE: the mean deviation from the old level over the window, as a
//! fraction of the swing. A valve closure seen upstream makes the head
//! oscillate around its old level (persistence near 0); a burst far from
//! any supply leaves it lower (near 1). The value depends on the network
//! (nearby reservoirs restore pressure), so version 1 records it but does
//! not lean on it - Stage B learns how much it means on each network.

/// Mean of (x - level) over `window`, divided by `swing`.
#[must_use]
pub fn persistence(window: &[f64], level: f64, swing: f64) -> f64 {
    if swing == 0.0 || window.is_empty() {
        return 0.0;
    }
    #[allow(clippy::cast_precision_loss)] // window lengths are small
    let n = window.len() as f64;
    window.iter().map(|x| x - level).sum::<f64>() / n / swing
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn staying_down_is_one_and_symmetric_ringing_is_zero() {
        assert!((persistence(&[55.0; 10], 60.0, -5.0) - 1.0).abs() < 1e-12);
        let ring = [65.0, 55.0, 65.0, 55.0];
        assert!(persistence(&ring, 60.0, 5.0).abs() < 1e-12);
    }
}
