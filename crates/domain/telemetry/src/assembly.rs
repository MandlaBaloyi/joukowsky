//! Chunks in; continuous stretches of valid samples and explicit gaps out.
//!
//! A chunk continues the stream when it starts where the last one ended,
//! within half a sampling interval (loggers stamp chunks with a little
//! jitter). If it starts later, the samples in between never arrived: a
//! GAP. If it starts earlier, it overlaps data already received - a
//! duplicate upload or a clock that jumped back - and it is refused.
//! Non-finite samples (NaN, infinity) are gaps too. After any gap a NEW
//! stretch begins: whatever consumes the samples (a detector) must learn
//! the sensor afresh, because the level may have moved while nobody
//! watched (a pump stopped during load-shedding).
//!
//! Sample k of a chunk is the average over the interval ending at
//! `start + k * interval`. Continuing stretches keep their own time base,
//! so chunk jitter never moves a sample.

use jk_kernel::Time;

use crate::error::TelemetryError;

/// A piece of one sensor's data as delivered.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Chunk<'a> {
    /// Time of the first sample.
    pub start: Time,
    /// Sampling interval.
    pub interval: Time,
    /// Pressure head samples (m); NaN marks an invalid sample.
    pub samples: &'a [f64],
}

/// Why data is missing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GapCause {
    /// No samples arrived for this time (power cut, lost upload).
    NoData,
    /// Samples arrived but were not valid numbers.
    Invalid,
}

/// Time with no valid data: from the first missing sample to the first
/// valid sample after it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gap {
    /// Time of the first missing or invalid sample.
    pub from: Time,
    /// Time of the first valid sample after the gap.
    pub to: Time,
    /// Why (the first cause seen, if several ran together).
    pub cause: GapCause,
}

/// What an accepted chunk turns into, in time order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Piece<'a> {
    /// Valid samples.
    Samples {
        /// Time of the first of them.
        start: Time,
        /// The samples.
        samples: &'a [f64],
        /// True if they begin a new stretch (the first data, or the first
        /// after a gap); false if they continue the previous piece.
        new_stretch: bool,
    },
    /// A gap that has just closed.
    Gap(Gap),
}

/// Assembles one sensor's chunks.
#[derive(Debug, Clone, PartialEq)]
pub struct Assembler {
    interval: f64,
    /// Time the next sample is due, once anything has arrived.
    next: Option<f64>,
    /// Whether the last sample received was valid (a stretch is open).
    open: bool,
    /// An open gap: when it began and why.
    gap: Option<(f64, GapCause)>,
}

impl Assembler {
    /// An assembler for a stream sampled every `interval`.
    ///
    /// # Errors
    /// A non-finite or non-positive interval.
    pub fn new(interval: Time) -> Result<Self, TelemetryError> {
        let dt = interval.value();
        if !(dt.is_finite() && dt > 0.0) {
            return Err(TelemetryError::InvalidInterval(interval));
        }
        Ok(Self {
            interval: dt,
            next: None,
            open: false,
            gap: None,
        })
    }

    /// The stream's sampling interval.
    #[must_use]
    pub fn interval(&self) -> Time {
        Time::new(self.interval)
    }

    /// Time of the last sample received, valid or not (None before any).
    #[must_use]
    pub fn received_until(&self) -> Option<Time> {
        self.next.map(|n| Time::new(n - self.interval))
    }

    /// A gap still open - no valid sample since `from` - and why.
    #[must_use]
    pub fn open_gap(&self) -> Option<(Time, GapCause)> {
        self.gap.map(|(from, cause)| (Time::new(from), cause))
    }

    /// Accepts the next chunk.
    ///
    /// # Errors
    /// A changed sampling interval, or a chunk overlapping data already
    /// received. A refused chunk changes nothing.
    pub fn accept<'a>(&mut self, chunk: Chunk<'a>) -> Result<Vec<Piece<'a>>, TelemetryError> {
        let dt = self.interval;
        if (chunk.interval.value() - dt).abs() > 1e-9 * dt {
            return Err(TelemetryError::IntervalChanged {
                expected: self.interval(),
                got: chunk.interval,
            });
        }
        let start = chunk.start.value();
        let base = match self.next {
            Some(next) => {
                let offset = (start - next) / dt;
                if offset < -0.5 {
                    return Err(TelemetryError::Overlap {
                        expected: Time::new(next),
                        got: chunk.start,
                    });
                }
                if offset > 0.5 {
                    // The samples in between never arrived.
                    if self.open {
                        self.gap = Some((next, GapCause::NoData));
                        self.open = false;
                    }
                    start
                } else {
                    next
                }
            }
            None => start,
        };
        let x = chunk.samples;
        let mut out = Vec::new();
        let mut i = 0;
        while i < x.len() {
            let t = base + to_f64(i) * dt;
            let valid = x[i].is_finite();
            let run = x[i..]
                .iter()
                .position(|v| v.is_finite() != valid)
                .map_or(x.len(), |n| i + n);
            if valid {
                if let Some((from, cause)) = self.gap.take() {
                    out.push(Piece::Gap(Gap {
                        from: Time::new(from),
                        to: Time::new(t),
                        cause,
                    }));
                }
                out.push(Piece::Samples {
                    start: Time::new(t),
                    samples: &x[i..run],
                    new_stretch: !self.open,
                });
                self.open = true;
            } else {
                if self.gap.is_none() {
                    self.gap = Some((t, GapCause::Invalid));
                }
                self.open = false;
            }
            i = run;
        }
        if !x.is_empty() {
            self.next = Some(base + to_f64(x.len()) * dt);
        }
        Ok(out)
    }
}

#[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.25;

    fn chunk(start: f64, samples: &[f64]) -> Chunk<'_> {
        Chunk {
            start: Time::new(start),
            interval: Time::new(DT),
            samples,
        }
    }

    /// Samples pieces as "S start xlength `new_stretch`", gaps as
    /// "G from-to cause".
    fn describe(pieces: &[Piece<'_>]) -> Vec<String> {
        pieces
            .iter()
            .map(|p| match p {
                Piece::Samples {
                    start,
                    samples,
                    new_stretch,
                } => format!("S {} x{} {new_stretch}", start.value(), samples.len()),
                Piece::Gap(g) => format!("G {}-{} {:?}", g.from.value(), g.to.value(), g.cause),
            })
            .collect()
    }

    #[test]
    fn back_to_back_chunks_are_one_stretch_despite_jitter() {
        let mut a = Assembler::new(Time::new(DT)).unwrap();
        let x = [1.0; 4];
        assert_eq!(
            describe(&a.accept(chunk(10.0, &x)).unwrap()),
            ["S 10 x4 true"]
        );
        // Due at 11.0; stamped 0.1 s late - within half an interval.
        assert_eq!(
            describe(&a.accept(chunk(11.1, &x)).unwrap()),
            ["S 11 x4 false"]
        );
        assert_eq!(a.received_until(), Some(Time::new(11.75)));
    }

    #[test]
    fn a_missing_chunk_is_a_gap_and_a_new_stretch() {
        let mut a = Assembler::new(Time::new(DT)).unwrap();
        let x = [1.0; 4];
        a.accept(chunk(10.0, &x)).unwrap();
        // 11.0 - 12.0 never arrived.
        assert_eq!(
            describe(&a.accept(chunk(12.0, &x)).unwrap()),
            ["G 11-12 NoData", "S 12 x4 true"]
        );
    }

    #[test]
    fn invalid_samples_are_a_gap_even_across_chunks() {
        let mut a = Assembler::new(Time::new(DT)).unwrap();
        let n = f64::NAN;
        assert_eq!(
            describe(&a.accept(chunk(0.0, &[1.0, 1.0, n, n])).unwrap()),
            ["S 0 x2 true"]
        );
        assert_eq!(a.open_gap(), Some((Time::new(0.5), GapCause::Invalid)));
        assert_eq!(
            describe(&a.accept(chunk(1.0, &[n, 2.0, 2.0, 2.0])).unwrap()),
            ["G 0.5-1.25 Invalid", "S 1.25 x3 true"]
        );
        assert_eq!(a.open_gap(), None);
    }

    #[test]
    fn overlaps_and_interval_changes_are_refused_and_change_nothing() {
        let mut a = Assembler::new(Time::new(DT)).unwrap();
        a.accept(chunk(10.0, &[1.0; 4])).unwrap();
        let before = a.clone();
        assert!(matches!(
            a.accept(chunk(10.5, &[1.0; 4])),
            Err(TelemetryError::Overlap { .. })
        ));
        let wrong = Chunk {
            interval: Time::new(0.5),
            ..chunk(11.0, &[1.0; 4])
        };
        assert!(matches!(
            a.accept(wrong),
            Err(TelemetryError::IntervalChanged { .. })
        ));
        assert_eq!(a, before);
        assert!(Assembler::new(Time::new(0.0)).is_err());
    }
}
