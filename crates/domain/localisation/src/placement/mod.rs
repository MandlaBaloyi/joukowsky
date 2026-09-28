//! Where should the loggers go? The placement advisor.
//!
//! `docs/04_detection_and_localisation/burst_localisation.txt` section 4:
//! choose the sensor locations that make the expected 90% credible region
//! of a burst - anywhere on the network, weighted by pipe risk - as small
//! as possible, among the places an installer can actually use. Frontier
//! F10: robust to wave speeds that are not well known.
//!
//! Why placement decides everything. A wave front reaches the sensors in
//! an order and with time differences that point back to its source - but
//! only between sensors. A burst on a branch BEYOND the outermost sensor
//! reaches every sensor through the same junction, so every point on that
//! branch gives the same time differences as the junction itself: the
//! branch is BLIND, however good the timing. Example, the test estate:
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2        sensors at S2 and S3 only:
//!                        |                   T-S2 and T-S3 are pinpointed,
//!                       600                  but a burst anywhere on R-S1
//!                        |                   or S1-T looks like one at T.
//!                        S3
//! ```
//!
//! How a placement is scored ([`coverage`]): bursts are tried every
//! `source_step` (20 m) along every pipe; for each, the exact arrival times at the
//! sensors (with the realistic timing and wave-speed doubts of
//! [`PlacementOptions`]) are handed to the real localiser, and the size of
//! its 90% region is recorded. The score is that size, averaged over the
//! network (weighted by pipe length and, optionally, risk); the report also
//! gives the share of pipe length PINPOINTED (region no longer than a dig,
//! 20 m by default) and each pipe's own figures.
//!
//! How sensors are chosen ([`advise`]): greedily - the best pair, then the
//! sensor that helps most, and so on, recording what each extra logger
//! buys - then improved by trying swaps until none helps.
//!
//! What the score rewards, found on the estate: not simply "loggers at the
//! ends". Two loggers at S2 and S3 leave ONE blind zone of 700 m (R-S1 and
//! S1-T, both beyond T); at S1 and S3 they leave two smaller ones (400 m
//! and 500 m), so the expected search length is shorter - and that is what
//! a repair crew walks. And with book wave speeds known only to +/-10%, the
//! 90% regions are 60-100 m, not 2 m: calibrating wave speeds from known
//! events (F9) is worth as much as another logger.
//!
//! Not yet: whether a small burst is DETECTABLE at the chosen sensors
//! (study V-S2: wave amplitude after junctions and friction), and robust
//! placement over a posterior of wave speeds rather than one doubt level.

mod advise;
mod coverage;

pub use advise::{Advice, advise};
pub use coverage::{Coverage, PipeCoverage, PlacementOptions, coverage};
