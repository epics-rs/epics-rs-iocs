//! The mar345dtb server execution context: batched parameter access, the
//! `writeServer` / `readServer` / `waitForCompletion` trio and the `setShutter`
//! override.
//!
//! `mar345.cpp` runs server I/O from two contexts — the asyn port thread
//! (`writeInt32`, which only sets `mode` and signals an event) and `mar345Task`
//! (which does all the socket work) — serialised by the driver lock. Since a
//! Rust `PortDriver` method cannot block on another port, all socket I/O lives
//! on the single `mar345Task` worker thread, which owns this [`Server`]
//! exclusively (no cross-thread lock is needed because it is the only writer).

use std::time::Instant;

use epics_rs::ad_core::driver::ShutterMode;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::user::AsynUser;

use crate::protocol::{CMD_SHUTTER_CLOSE, CMD_SHUTTER_OPEN, response_done};
use crate::types::{CamStatus, MAR345_POLL_DELAY, MAR345_SOCKET_TIMEOUT, MAX_MESSAGE_SIZE, secs};

/// asyn subaddress of the single-device marServer octet port.
const SERVER_ADDR: i32 = 0;

fn classify(err: &AsynError) -> CamStatus {
    match err {
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        } => CamStatus::Timeout,
        _ => CamStatus::Error,
    }
}

/// Worker-thread server context.
pub struct Server {
    /// This driver's own port (parameter library).
    handle: PortHandle,
    /// The marServer octet port created by `drvAsynIPPortConfigure`.
    server_port: PortHandle,
    pub ad: ADBaseParams,
    /// C `fromServer`.
    from: String,
    /// Pending parameter writes, flushed at each C `callParamCallbacks()`.
    batch: Vec<ParamSetValue>,
}

impl Server {
    pub fn new(handle: PortHandle, server_port: PortHandle, ad: ADBaseParams) -> Self {
        Self {
            handle,
            server_port,
            ad,
            from: String::new(),
            batch: Vec::new(),
        }
    }

    /// The last server reply (C `fromServer`).
    pub fn from(&self) -> &str {
        &self.from
    }

    // --- parameter library -------------------------------------------------

    /// C `setIntegerParam`.
    pub fn set_i32(&mut self, reason: usize, value: i32) {
        self.batch
            .push(ParamSetValue::new(reason, 0, ParamValue::Int32(value)));
    }

    /// C `setDoubleParam`.
    pub fn set_f64(&mut self, reason: usize, value: f64) {
        self.batch
            .push(ParamSetValue::new(reason, 0, ParamValue::Float64(value)));
    }

    /// C `setStringParam`.
    pub fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.batch.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(value.into()),
        ));
    }

    /// C `callParamCallbacks()` — apply the pending writes and post monitors.
    pub async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.batch);
        if let Err(e) = self.handle.set_params_and_notify(0, updates).await {
            log::error!("mar345: callParamCallbacks failed: {e}");
        }
    }

    /// Flush pending writes so a following read observes them.
    async fn flush(&mut self) {
        if !self.batch.is_empty() {
            self.callbacks().await;
        }
    }

    /// C `getIntegerParam`.
    pub async fn get_i32(&mut self, reason: usize) -> i32 {
        self.flush().await;
        self.handle.read_int32(reason, 0).await.unwrap_or(0)
    }

    /// C `getDoubleParam`.
    pub async fn get_f64(&mut self, reason: usize) -> f64 {
        self.flush().await;
        self.handle.read_float64(reason, 0).await.unwrap_or(0.0)
    }

    /// C `getStringParam` truncated to `max_chars`.
    pub async fn get_str(&mut self, reason: usize, max_chars: usize) -> String {
        self.flush().await;
        match self.handle.read_octet(reason, 0, max_chars).await {
            Ok(bytes) => {
                let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
                String::from_utf8_lossy(&bytes[..end]).into_owned()
            }
            Err(e) => {
                log::error!("mar345: getStringParam({reason}) failed: {e}");
                String::new()
            }
        }
    }

    // --- server ------------------------------------------------------------

    /// C `writeServer`: flush stale input, write with `MAR345_SOCKET_TIMEOUT`,
    /// then publish the command on `ADStringToServer`.
    pub async fn write_server(&mut self, output: &str) -> CamStatus {
        let _ = self
            .server_port
            .submit_async(RequestOp::Flush, AsynUser::default().with_addr(SERVER_ADDR))
            .await;

        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(MAR345_SOCKET_TIMEOUT));
        let status = match self
            .server_port
            .submit_async(
                RequestOp::OctetWrite {
                    data: output.as_bytes().to_vec(),
                },
                user,
            )
            .await
        {
            Ok(_) => CamStatus::Success,
            Err(e) => {
                log::error!("mar345: writeServer failed, sent {output:?}: {e}");
                classify(&e)
            }
        };

        self.set_str(self.ad.string_to_server, output);
        self.callbacks().await;
        status
    }

    /// C `readServer`: a single read with the given timeout. Only a non-empty
    /// read publishes `ADStringFromServer` (C returns before `setStringParam`
    /// when `nread == 0`).
    pub async fn read_server(&mut self, timeout: f64) -> CamStatus {
        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(timeout));
        match self
            .server_port
            .submit_async(
                RequestOp::OctetRead {
                    buf_size: MAX_MESSAGE_SIZE,
                },
                user,
            )
            .await
        {
            Ok(result) => {
                let data = result.data.unwrap_or_default();
                if data.is_empty() {
                    // C: `if (nread == 0) return status` without publishing.
                    self.from.clear();
                    return CamStatus::Success;
                }
                self.from = String::from_utf8_lossy(&data).into_owned();
                let from = self.from.clone();
                self.set_str(self.ad.string_from_server, from);
                self.callbacks().await;
                CamStatus::Success
            }
            Err(e) => {
                self.from.clear();
                classify(&e)
            }
        }
    }

    /// C `waitForCompletion`: poll `readServer` at `MAR345_POLL_DELAY` until the
    /// reply contains `done`, or `timeout` elapses.
    pub async fn wait_for_completion(&mut self, done: &str, timeout: f64) -> CamStatus {
        let start = Instant::now();
        loop {
            let status = self.read_server(MAR345_POLL_DELAY).await;
            if status == CamStatus::Success && response_done(self.from(), done) {
                return CamStatus::Success;
            }
            if start.elapsed().as_secs_f64() > timeout {
                log::error!("mar345: error waiting for response from marServer");
                return CamStatus::Error;
            }
        }
    }

    /// C `mar345::setShutter` — overrides the base class for detector-controlled
    /// shutters and delegates the EPICS-controlled case to the base behaviour.
    pub async fn set_shutter(&mut self, open: bool) {
        let mode = ShutterMode::from_i32(self.get_i32(self.ad.shutter_mode).await);
        let shutter_open_delay = self.get_f64(self.ad.shutter_open_delay).await;
        let shutter_close_delay = self.get_f64(self.ad.shutter_close_delay).await;

        match mode {
            Some(ShutterMode::DetectorOnly) => {
                if open {
                    self.write_server(CMD_SHUTTER_OPEN).await;
                    // Correct the exposure time: opening minus closing time,
                    // floored at 1 ms so the delay is never negative and the
                    // commands are not back-to-back.
                    let mut delay = shutter_open_delay - shutter_close_delay;
                    if delay < 0.001 {
                        delay = 0.001;
                    }
                    rt::sleep(secs(delay)).await;
                } else {
                    self.write_server(CMD_SHUTTER_CLOSE).await;
                    rt::sleep(secs(shutter_close_delay)).await;
                }
                // The mar345 does not report actual shutter status, so set it to
                // agree with the control value.
                self.set_i32(self.ad.shutter_status, i32::from(open));
                self.callbacks().await;
            }
            // Base ADDriver::setShutter: EPICS-controlled shutter.
            Some(ShutterMode::EpicsOnly) => {
                self.set_i32(self.ad.shutter_control_epics, i32::from(open));
                self.callbacks().await;
                let delay = shutter_open_delay - shutter_close_delay;
                if delay > 0.0 {
                    rt::sleep(secs(delay)).await;
                }
            }
            // ADShutterModeNone or unknown: nothing to do.
            Some(ShutterMode::None) | None => {}
        }
    }
}
