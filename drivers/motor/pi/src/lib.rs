//! PI (Physik Instrumente) legacy per-model motor controller drivers
//! (`motorPI`), distinct from the already-ported GCS2 generic stage
//! controller path (`motor-pi-gcs2`). Module-per-controller.

pub mod c663;
pub mod c862;
pub mod ioc;

pub use c663::{PIC663Axis, PIC663Controller};
pub use c862::{PIC862Axis, PIC862Controller};
