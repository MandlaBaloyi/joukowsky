//! Balance bounded context: the water balance of metered zones, and slow
//! leaks - the ones that send no wave
//! (`docs/04_detection_and_localisation/slow_leaks_and_water_balance.txt`).
//!
//! Pure: series in, findings out; no clock reads (times are inputs).
//!
//! - [`zone`]: the balance of a zone from its meters (and storage).
//! - [`night`]: one night flow per night, missing nights left missing.
//! - [`leak`]: a lasting rise in night flow - the leak alarm - and the
//!   smallest leak a zone can reveal.

pub mod leak;
pub mod night;
pub mod zone;

mod error;

pub use error::BalanceError;
pub use leak::{LeakAlarm, LeakOptions, Watch, smallest_detectable, watch};
pub use night::{Night, NightWindow, night_flows};
pub use zone::{Direction, Meter, Series, balance};
