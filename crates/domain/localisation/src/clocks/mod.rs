//! Sensor clocks: how wrong they are, and repairing them in software.
//!
//! `docs/03_sensing_and_signal/time_synchronisation.txt`: localisation
//! compares arrival times across sensors, so it is only as good as the
//! clocks agree - every 1 ms of disagreement moves a burst ~0.5 m
//! (a = 1000 m/s). A GNSS-disciplined logger is good to microseconds; an
//! NTP-synchronised one over cellular to 10-100 ms, i.e. 5-50 m.
//!
//! Frontier item F9 (`09_technologies/11_frontier_what_we_must_invent.txt`):
//! repair bad clocks in software. Events with a KNOWN source - an operator
//! opening a hydrant at a logged place, a scheduled pump start - reach every
//! sensor after a travel time the pipe map predicts, so whatever is left
//! over, event after event, is the sensor's clock offset.
//!
//! - [`calibrate`]: estimate each sensor's offset from known-source events.

pub mod calibrate;

pub use calibrate::{CalibrationEvent, ClockEstimate, correct, estimate_offsets};
