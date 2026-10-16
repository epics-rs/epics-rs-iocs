//! marServer I/O: the driver's `writeServer` and `setShutter`, and the
//! `mar345Task` worker's batched parameter access, `readServer` and
//! `waitForCompletion`.
//!
//! `mar345.cpp` writes to marServer under the driver lock, from `writeInt32`
//! (`setShutter`) and from `mar345Task`, and drops the lock for each read
//! (`readServer`, :260-263). Here the lock is the port actor: writes run on
//! [`Mar345Driver`], inline or through `PortHandle::with_driver`, and the
//! worker reads with no lock held.

use std::time::Instant;

use epics_rs::ad_core::driver::ADDriver;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::driver::Mar345Driver;
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
    pub fn set_str(&mut self, reason: usize, value: impl Into<Vec<u8>>) {
        self.batch.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(Into::<Vec<u8>>::into(value).into()),
        ));
    }

    /// C `callParamCallbacks()` — apply the pending writes and post monitors.
    pub async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.batch);
        let _ = self.handle.set_params_and_notify(0, updates).await;
    }

    /// C `asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, ...)`.
    pub fn trace_error(&self, message: &str) {
        self.handle
            .trace()
            .output(self.handle.port_name(), TraceMask::ERROR, message);
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
        match self.handle.get_string_param(reason, 0, max_chars).await {
            Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
            Err(_) => String::new(),
        }
    }

    // --- server ------------------------------------------------------------

    /// C `writeServer` from `mar345Task`, which holds the driver lock: the
    /// pending parameter writes go with it, posted by its `callParamCallbacks`.
    pub async fn write_server(&mut self, output: &str) -> CamStatus {
        let output = output.to_string();
        self.locked(move |d| d.write_server(&output))
            .await
            .unwrap_or(CamStatus::Error)
    }

    /// C `setShutter` from `mar345Task`.
    pub async fn set_shutter(&mut self, open: bool) {
        let _ = self
            .locked(move |d| {
                let _ = ADDriver::set_shutter(d, open);
            })
            .await;
    }

    /// Run `f` on the driver (C `lock()` … `unlock()`) after storing the
    /// pending parameter writes, which `f`'s `callParamCallbacks` posts.
    async fn locked<R: Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut Mar345Driver) -> R + Send + 'static,
    ) -> Option<R> {
        let batch = std::mem::take(&mut self.batch);
        self.handle
            .with_driver::<Mar345Driver, R>(move |d| {
                for update in batch {
                    if let ParamSetValue::Value {
                        reason,
                        addr,
                        value,
                    } = update
                    {
                        let _ = d.ad.port_base.set_param_value(reason, addr, value);
                    }
                }
                f(d)
            })
            .await
            .ok()
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
                self.trace_error(
                    "mar345:waitForCompletion: error waiting for response from marServer",
                );
                return CamStatus::Error;
            }
        }
    }
}

impl Mar345Driver {
    /// C `writeServer`: flush stale input, write with `MAR345_SOCKET_TIMEOUT`,
    /// then publish the command on `ADStringToServer`.
    pub fn write_server(&mut self, output: &str) -> CamStatus {
        let _ = self
            .server
            .submit_blocking(RequestOp::Flush, AsynUser::default().with_addr(SERVER_ADDR));

        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(MAR345_SOCKET_TIMEOUT));
        let status = match self.server.submit_blocking(
            RequestOp::OctetWrite {
                data: output.as_bytes().to_vec(),
            },
            user,
        ) {
            Ok(_) => CamStatus::Success,
            Err(e) => {
                // On pasynUserServer, so the server port's trace mask gates it.
                self.server.trace().output(
                    self.server.port_name(),
                    TraceMask::ERROR,
                    &format!(
                        "mar345:writeServer, status={}, sent\n{output}",
                        e.status() as i32
                    ),
                );
                classify(&e)
            }
        };

        let base = &mut self.ad.port_base;
        let _ = base.set_string_param(self.ad.params.string_to_server, 0, output);
        let _ = base.call_param_callbacks(0);
        status
    }

    /// `mar345::setShutter`'s detector branch (mar345.cpp:372-396).
    pub(crate) fn set_detector_shutter(&mut self, open: bool) {
        let ad = self.ad.params;
        let base = &self.ad.port_base;
        let shutter_open_delay = base
            .get_float64_param(ad.shutter_open_delay, 0)
            .unwrap_or(0.0);
        let shutter_close_delay = base
            .get_float64_param(ad.shutter_close_delay, 0)
            .unwrap_or(0.0);
        let (command, delay) = if open {
            // Correct the exposure time: opening minus closing time, floored
            // at 1 ms so the delay is never negative and the commands are not
            // back-to-back.
            (
                CMD_SHUTTER_OPEN,
                (shutter_open_delay - shutter_close_delay).max(0.001),
            )
        } else {
            (CMD_SHUTTER_CLOSE, shutter_close_delay)
        };
        self.write_server(command);
        std::thread::sleep(secs(delay));
        // The mar345 does not report actual shutter status, so set it to agree
        // with the control value.
        let base = &mut self.ad.port_base;
        let _ = base.set_int32_param(ad.shutter_status, 0, i32::from(open));
        let _ = base.call_param_callbacks(0);
    }
}
