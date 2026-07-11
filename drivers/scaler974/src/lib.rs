//! Ortec 974 counter/timer asyn port driver, ported from `epics-modules/scaler`
//! (`drvScaler974.cpp`). See [`driver`] for the `ScalerDriver` implementation,
//! [`connect`] for the octet-port lookup helper, [`registry`] for the
//! `initScaler974` -> scalerRecord driver hand-off, and [`wire`] for the
//! SHOW_COUNTS/SET_COUNT_PRESET wire helpers.

pub mod connect;
pub mod driver;
pub mod registry;
pub mod wire;
