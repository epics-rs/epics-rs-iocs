//! The worker-thread execution context: batched parameter access plus the
//! camserver `writeCamserver` / `readCamserver` / `writeReadCamserver` trio.
//!
//! `pilatusDetector.cpp` runs camserver I/O from two contexts — the asyn port
//! thread (`writeInt32` / `writeFloat64` / `writeOctet`) and `pilatusTask` —
//! serialised by the driver lock, which is released around every socket poll so
//! an abort can preempt a long read. A Rust `PortDriver` method runs *inside*
//! the port actor and cannot block on another port, so both contexts here are
//! worker threads that reach the driver's own parameters through a
//! [`PortHandle`]. Everything else about the sequence is unchanged.

use std::sync::Arc;
use std::time::Instant;

use epics_rs::ad_core::driver::{ADStatus, ShutterMode};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::user::AsynUser;

use crate::params::PilatusParams;
use crate::protocol::reply_is_ok;
use crate::types::{ASYN_POLL_TIME, CamStatus, Event, MAX_MESSAGE_SIZE, secs};

/// asyn subaddress of the single-device camserver octet port.
const CAM_ADDR: i32 = 0;

/// C `readCamserver`: `epicsEventWaitWithTimeout(stopEventId, 0.001)` right
/// after the socket read.
const ABORT_POLL_TIME: f64 = 0.001;

/// Pending parameter writes, flushed by [`Ctx::callbacks`] at exactly the
/// points where C calls `callParamCallbacks()`.
#[derive(Default)]
struct Batch {
    updates: Vec<ParamSetValue>,
}

/// Worker-thread context.
pub struct Ctx {
    /// This driver's own port (parameter library).
    handle: PortHandle,
    /// The camserver octet port created by `drvAsynIPPortConfigure`.
    cam_port: PortHandle,
    pub ad: ADBaseParams,
    pub p: PilatusParams,
    pub stop: Arc<Event>,
    /// C `fromCamserver`.
    from: String,
    /// C `toCamserver`. Kept because `pilatusTask`'s trigger-mode `switch` has
    /// no `default`: an out-of-range `ADTriggerMode` re-sends whatever command
    /// the buffer still holds.
    to: String,
    batch: Batch,
}

fn classify(err: &AsynError) -> CamStatus {
    match err {
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        } => CamStatus::Timeout,
        _ => CamStatus::Error,
    }
}

impl Ctx {
    pub fn new(
        handle: PortHandle,
        cam_port: PortHandle,
        ad: ADBaseParams,
        p: PilatusParams,
        stop: Arc<Event>,
    ) -> Self {
        Self {
            handle,
            cam_port,
            ad,
            p,
            stop,
            from: String::new(),
            to: String::new(),
            batch: Batch::default(),
        }
    }

    /// The last camserver reply (C `fromCamserver`).
    pub fn from(&self) -> &str {
        &self.from
    }

    /// The last camserver command (C `toCamserver`).
    pub fn to_camserver(&self) -> &str {
        &self.to
    }

    // --- parameter library -------------------------------------------------

    /// C `setIntegerParam`.
    pub fn set_i32(&mut self, reason: usize, value: i32) {
        self.batch
            .updates
            .push(ParamSetValue::new(reason, 0, ParamValue::Int32(value)));
    }

    /// C `setDoubleParam`.
    pub fn set_f64(&mut self, reason: usize, value: f64) {
        self.batch
            .updates
            .push(ParamSetValue::new(reason, 0, ParamValue::Float64(value)));
    }

    /// C `setStringParam`.
    pub fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.batch.updates.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(value.into()),
        ));
    }

    /// C `callParamCallbacks()` — applies the pending writes and posts monitors
    /// in one actor message.
    pub async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.batch.updates);
        if let Err(e) = self.handle.set_params_and_notify(0, updates).await {
            log::error!("pilatus: callParamCallbacks failed: {e}");
        }
    }

    /// Flush pending writes so a following read observes them. The actor
    /// processes its inbox in FIFO order, so enqueueing is enough.
    async fn flush(&mut self) {
        if !self.batch.updates.is_empty() {
            self.callbacks().await;
        }
    }

    /// C `getIntegerParam`.
    pub async fn get_i32(&mut self, reason: usize) -> Option<i32> {
        self.flush().await;
        self.handle.read_int32(reason, 0).await.ok()
    }

    /// C `getDoubleParam`.
    pub async fn get_f64(&mut self, reason: usize) -> Option<f64> {
        self.flush().await;
        self.handle.read_float64(reason, 0).await.ok()
    }

    /// C `getStringParam` with a `max_chars` capacity (the C call truncates to
    /// the caller's buffer).
    pub async fn get_str(&mut self, reason: usize, max_chars: usize) -> String {
        self.flush().await;
        match self.handle.read_octet(reason, 0, max_chars).await {
            Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
            Err(e) => {
                log::error!("pilatus: getStringParam({reason}) failed: {e}");
                String::new()
            }
        }
    }

    // --- camserver ---------------------------------------------------------

    /// C `writeCamserver`: flush stale input, write, then publish the command
    /// on `ADStringToServer` regardless of the write status.
    pub async fn cam_write(&mut self, cmd: &str, timeout: f64) -> CamStatus {
        self.to = cmd.to_string();
        let _ = self
            .cam_port
            .submit_async(RequestOp::Flush, AsynUser::default().with_addr(CAM_ADDR))
            .await;

        let user = AsynUser::default()
            .with_addr(CAM_ADDR)
            .with_timeout(secs(timeout));
        let status = match self
            .cam_port
            .submit_async(
                RequestOp::OctetWrite {
                    data: cmd.as_bytes().to_vec(),
                },
                user,
            )
            .await
        {
            Ok(_) => CamStatus::Success,
            Err(e) => {
                log::error!("pilatus: writeCamserver failed, sent {cmd:?}: {e}");
                classify(&e)
            }
        };

        self.set_str(self.ad.string_to_server, cmd);
        status
    }

    /// C `readCamserver`: poll the socket in `ASYN_POLL_TIME` slices so an
    /// abort can interrupt a long exposure.
    pub async fn cam_read(&mut self, timeout: f64) -> CamStatus {
        let t_start = Instant::now();
        let mut delta_time = 0.0f64;
        let mut status = CamStatus::Success;

        while delta_time <= timeout {
            let user = AsynUser::default()
                .with_addr(CAM_ADDR)
                .with_timeout(secs(ASYN_POLL_TIME));
            status = match self
                .cam_port
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
                    self.from = String::from_utf8_lossy(&data).into_owned();
                    CamStatus::Success
                }
                Err(e) => {
                    // C's asynOctetSyncIO null-terminates at `nbytesTransfered`,
                    // which is 0 on a failed read.
                    self.from.clear();
                    classify(&e)
                }
            };

            // C checks for an abort delivered during the read, so the next
            // acquisition does not inherit it.
            if self.stop.wait_timeout(secs(ABORT_POLL_TIME)) {
                self.set_str(self.ad.status_message, "Acquisition aborted");
                self.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return CamStatus::Error;
            }
            if status != CamStatus::Timeout {
                break;
            }

            if self.stop.wait_timeout(secs(ASYN_POLL_TIME)) {
                self.set_str(self.ad.status_message, "Acquisition aborted");
                self.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return CamStatus::Error;
            }
            delta_time = t_start.elapsed().as_secs_f64();
        }

        // A poll with timeout == 0 is only checking for a possible reply.
        if status == CamStatus::Timeout && timeout == 0.0 {
            return CamStatus::Success;
        }
        if status != CamStatus::Success {
            log::error!(
                "pilatus: readCamserver timeout={timeout}, status={:?}, received {:?}",
                status,
                self.from
            );
        } else if !reply_is_ok(&self.from) {
            log::error!(
                "pilatus: unexpected response from camserver, no OK, response={:?}",
                self.from
            );
            self.set_str(self.ad.status_message, "Error from camserver");
            status = CamStatus::Error;
        } else {
            self.set_str(self.ad.status_message, "Camserver returned OK");
        }

        let from = self.from.clone();
        self.set_str(self.ad.string_from_server, from);
        status
    }

    /// C `writeReadCamserver`.
    pub async fn cam_write_read(&mut self, cmd: &str, timeout: f64) -> CamStatus {
        let status = self.cam_write(cmd, timeout).await;
        if status.is_err() {
            return status;
        }
        self.cam_read(timeout).await
    }

    // --- shutter -----------------------------------------------------------

    /// C `ADDriver::setShutter`, executed from a worker thread.
    pub async fn set_shutter(&mut self, open: bool) {
        let mode = self
            .get_i32(self.ad.shutter_mode)
            .await
            .and_then(ShutterMode::from_i32);
        if mode != Some(ShutterMode::EpicsOnly) {
            return;
        }
        let open_delay = self
            .get_f64(self.ad.shutter_open_delay)
            .await
            .unwrap_or(0.0);
        let close_delay = self
            .get_f64(self.ad.shutter_close_delay)
            .await
            .unwrap_or(0.0);
        self.set_i32(self.ad.shutter_control_epics, i32::from(open));
        self.callbacks().await;
        let delay = open_delay - close_delay;
        if delay > 0.0 {
            epics_rs::ad_core::runtime::sleep(secs(delay)).await;
        }
    }
}
