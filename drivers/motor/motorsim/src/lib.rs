//! Simulated motor controller driver, ported from `epics-modules/motor`
//! motorMotorSim (`motorSimDriver.cpp` + `route.c`).
//!
//! [`MotorSimAxis`] implements [`epics_rs::asyn::interfaces::motor::AsynMotor`]
//! by integrating a [`route::Route`] trapezoidal trajectory forward in real
//! time; the [`ioc`] module provides the `motorSimCreateController` /
//! `motorSimConfigAxis` iocsh commands. No hardware and no vendor library —
//! builds and tests on any host.

pub mod ioc;
pub mod motorsim;
pub mod route;

pub use motorsim::MotorSimAxis;
