//! Vendor-independent support for the motor-controller driver crates ported
//! from `epics-modules/motor`.
//!
//! The per-vendor crates (`motor-newport`, `motor-micronix`, …) all share the
//! same shape: a `*CreateController` iocsh command connects an octet port,
//! builds one [`epics_rs::asyn::interfaces::motor::AsynMotor`] per axis, and
//! registers each behind a `DTYP`-keyed motor device support that
//! `dbLoadRecords` binds. This crate factors out everything that is not
//! protocol-specific:
//!
//! - [`MotorHolder`] — stores the device supports + poll-loop senders, wires
//!   each motor through motor-rs's [`MotorBuilder`], and hands the device
//!   support back to the record layer by `DTYP` name ([`take`
//!   semantics][MotorHolder::device_support_factory]).
//! - [`iocsh`] — argument parsing helpers for the create commands.
//! - [`connect`] — octet-port connect helpers ([`connect_serial`],
//!   [`connect_ip`]).
//! - [`util`] — the C-runtime numeric helpers (`atof`/`atoi`/`NINT`/…) the
//!   protocols share.
//!
//! [`MotorBuilder`]: epics_rs::motor::builder::MotorBuilder
//! [`connect_serial`]: connect::connect_serial
//! [`connect_ip`]: connect::connect_ip

pub mod connect;
pub mod iocsh;
pub mod util;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::CommandContext;
use epics_rs::motor::builder::{MotorBuilder, MotorSetup};
use epics_rs::motor::device_support::MotorDeviceSupport;
use epics_rs::motor::poll_loop::PollCommand;

/// Holds every motor device support created by a vendor's `*CreateController`
/// commands and the poll-loop command senders, keyed for the record layer to
/// claim by `DTYP` name.
///
/// One holder is shared (via `Arc`) across all of a controller's axes and
/// registered once as the IOC's dynamic device-support factory. It is
/// vendor-independent; a vendor that needs extra per-controller bookkeeping
/// (e.g. a two-step create-controller/create-axis API) keeps that state in
/// its own holder wrapper and delegates motor installation here.
#[derive(Default)]
pub struct MotorHolder {
    motors: Mutex<HashMap<String, Option<MotorDeviceSupport>>>,
    poll_senders: Mutex<Vec<tokio::sync::mpsc::Sender<PollCommand>>>,
}

impl MotorHolder {
    /// Create an empty holder.
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Wire one motor into a record + poll loop: build the device support and
    /// poll loop for `motor`, spawn the poll loop (starts idle), and store the
    /// device support under `dtyp_key` for later `dbLoadRecords` binding.
    pub fn install(
        &self,
        ctx: &CommandContext,
        dtyp_key: String,
        motor: Arc<Mutex<dyn AsynMotor>>,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
    ) {
        let MotorSetup {
            record: _,
            device_support,
            poll_loop,
            poll_cmd_tx,
        } = MotorBuilder::new(motor)
            .moving_poll_interval(Duration::from_millis(moving_poll_ms))
            .idle_poll_interval(Duration::from_millis(idle_poll_ms))
            .build();

        let device_support = device_support.with_dtyp_name(dtyp_key.clone());

        ctx.runtime_handle().spawn(poll_loop.run());

        self.poll_senders
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(poll_cmd_tx);
        self.motors
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(dtyp_key, Some(device_support));
    }

    /// Start polling on every registered controller. Call after PINI
    /// processing to avoid queue buildup.
    pub fn start_all_polling(&self) {
        for tx in self
            .poll_senders
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
        {
            let _ = tx.try_send(PollCommand::StartPolling);
        }
    }

    /// Dynamic device-support factory: hand the record layer the device
    /// support registered under the record's `DTYP` name. Each support is
    /// consumed once (take semantics), so a second record with the same `DTYP`
    /// gets `None`.
    pub fn device_support_factory(
        self: &Arc<Self>,
    ) -> impl Fn(
        &epics_rs::ca::server::ioc_app::DeviceSupportContext,
    ) -> Option<Box<dyn epics_rs::base::server::device_support::DeviceSupport>>
    + Send
    + Sync
    + 'static {
        let holder = self.clone();
        move |ctx: &epics_rs::ca::server::ioc_app::DeviceSupportContext| {
            let mut motors = holder.motors.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(slot) = motors.get_mut(ctx.dtyp)
                && let Some(ds) = slot.take()
            {
                return Some(
                    Box::new(ds) as Box<dyn epics_rs::base::server::device_support::DeviceSupport>
                );
            }
            None
        }
    }
}
