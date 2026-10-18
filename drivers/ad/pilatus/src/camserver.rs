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

use epics_rs::ad_core::driver::ndarray_driver::check_path_str;
use epics_rs::ad_core::driver::{ADStatus, ShutterMode};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::{AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp, RequestResult};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::params::PilatusParams;
use crate::protocol::reply_is_ok;
use crate::types::{ASYN_POLL_TIME, Event, MAX_FILENAME_LEN, MAX_MESSAGE_SIZE, secs};

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
    /// C `pasynUserCamserver`: the user of the camserver octet port, which has
    /// no port when the startup script named one that does not exist.
    cam_port: Arc<SyncIOHandle>,
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

impl Ctx {
    pub fn new(
        handle: PortHandle,
        cam_port: Arc<SyncIOHandle>,
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
    pub fn set_str(&mut self, reason: usize, value: impl Into<Vec<u8>>) {
        self.batch.updates.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(Into::<Vec<u8>>::into(value).into()),
        ));
    }

    /// C `callParamCallbacks()` — applies the pending writes and posts monitors
    /// in one actor message.
    pub async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.batch.updates);
        let _ = self.handle.set_params_and_notify(0, updates).await;
    }

    /// C `updateTimeStamps`' stamp: the port's, moved now
    /// (asynNDArrayDriver.cpp:832-836).
    pub async fn update_time_stamp(&self) -> epics_rs::ad_core::timestamp::EpicsTimestamp {
        self.handle
            .update_time_stamp()
            .await
            .unwrap_or_else(|_| std::time::SystemTime::now())
            .into()
    }

    /// An `asynPrint` on `pasynUserSelf`.
    pub fn trace(&self, mask: TraceMask, msg: &str) {
        self.handle
            .trace()
            .output_device(self.handle.port_name(), Some(0), 0, mask, msg);
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
        match self.handle.get_string_param(reason, 0, max_chars).await {
            Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
            Err(_) => String::new(),
        }
    }

    /// C `asynNDArrayDriver::checkPath()` (asynNDArrayDriver.cpp:98-111): an
    /// empty FilePath leaves both parameters alone, any other is written back
    /// with its separator next to whether it exists.
    pub async fn check_path(&mut self) {
        let mut path = self.get_str(self.ad.base.file_path, MAX_FILENAME_LEN).await;
        if path.is_empty() {
            return;
        }
        let exists = check_path_str(&mut path);
        self.set_str(self.ad.base.file_path, path);
        self.set_i32(self.ad.base.file_path_exists, i32::from(exists));
    }

    // --- camserver ---------------------------------------------------------

    async fn cam_submit(&self, op: RequestOp, user: AsynUser) -> AsynResult<RequestResult> {
        self.cam_port.port()?.submit_async(op, user).await
    }

    /// C `writeCamserver`: flush stale input, write, then publish the command
    /// on `ADStringToServer` regardless of the write status.
    pub async fn cam_write(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        self.to = cmd.to_string();
        let _ = self
            .cam_submit(RequestOp::Flush, AsynUser::default().with_addr(CAM_ADDR))
            .await;

        let user = AsynUser::default()
            .with_addr(CAM_ADDR)
            .with_timeout(secs(timeout));
        let status = match self
            .cam_submit(
                RequestOp::OctetWrite {
                    data: cmd.as_bytes().to_vec(),
                },
                user,
            )
            .await
        {
            Ok(_) => AsynStatus::Success,
            Err(e) => {
                let status = e.status();
                self.trace(
                    TraceMask::ERROR,
                    &format!(
                        "pilatusDetector:writeCamserver, status={}, sent\n{cmd}\n",
                        status as i32
                    ),
                );
                status
            }
        };

        self.set_str(self.ad.string_to_server, cmd);
        status
    }

    /// C `readCamserver`: poll the socket in `ASYN_POLL_TIME` slices so an
    /// abort can interrupt a long exposure.
    pub async fn cam_read(&mut self, timeout: f64) -> AsynStatus {
        let t_start = Instant::now();
        let mut delta_time = 0.0f64;
        let mut status = AsynStatus::Success;

        while delta_time <= timeout {
            let user = AsynUser::default()
                .with_addr(CAM_ADDR)
                .with_timeout(secs(ASYN_POLL_TIME));
            status = match self
                .cam_submit(
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
                    AsynStatus::Success
                }
                Err(e) => {
                    // C's asynOctetSyncIO null-terminates at `nbytesTransfered`,
                    // which is 0 on a failed read.
                    self.from.clear();
                    e.status()
                }
            };

            // C checks for an abort delivered during the read, so the next
            // acquisition does not inherit it.
            if self.stop.wait_timeout(secs(ABORT_POLL_TIME)) {
                self.set_str(self.ad.status_message, "Acquisition aborted");
                self.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return AsynStatus::Error;
            }
            if status != AsynStatus::Timeout {
                break;
            }

            if self.stop.wait_timeout(secs(ASYN_POLL_TIME)) {
                self.set_str(self.ad.status_message, "Acquisition aborted");
                self.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return AsynStatus::Error;
            }
            delta_time = t_start.elapsed().as_secs_f64();
        }

        // A poll with timeout == 0 is only checking for a possible reply.
        if status == AsynStatus::Timeout && timeout == 0.0 {
            return AsynStatus::Success;
        }
        // C prints these on pasynUserCamserver.
        let cam_trace = |msg: &str| {
            self.cam_port
                .print(0, TraceMask::ERROR, format_args!("{msg}"));
        };
        if status != AsynStatus::Success {
            cam_trace(&format!(
                "pilatusDetector:readCamserver, timeout={timeout:.6}, status={} received {} bytes\n{}\n",
                status as i32,
                self.from.len(),
                self.from
            ));
        } else if !reply_is_ok(&self.from) {
            cam_trace(&format!(
                "pilatusDetector:readCamserver unexpected response from camserver, no OK, response={}\n",
                self.from
            ));
            self.set_str(self.ad.status_message, "Error from camserver");
            status = AsynStatus::Error;
        } else {
            self.set_str(self.ad.status_message, "Camserver returned OK");
        }

        let from = self.from.clone();
        self.set_str(self.ad.string_from_server, from);
        status
    }

    /// C `writeReadCamserver`.
    pub async fn cam_write_read(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        let status = self.cam_write(cmd, timeout).await;
        if status != AsynStatus::Success {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::{DriverLinks, PilatusDriver};
    use crate::task::{Cmd, Shared};
    use epics_rs::ad_core::runtime as rt;
    use epics_rs::asyn::error::AsynError;
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;

    struct RefusingPort {
        base: PortDriverBase,
        status: AsynStatus,
    }

    impl PortDriver for RefusingPort {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, _data: &[u8]) -> AsynResult<usize> {
            Err(AsynError::Status {
                status: self.status,
                message: "refused".into(),
            })
        }
    }

    /// C `writeCamserver` returns and prints `pasynOctetSyncIO->write`'s own
    /// status (pilatusDetector.cpp:1338-1345): a camserver port with no
    /// connection prints `status=4`, not asynError's 3.
    #[test]
    fn a_camserver_write_keeps_the_octet_ports_status() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Cmd>(4);
        let stop = Arc::new(Event::new());
        let det = PilatusDriver::new(
            "PILATUS_WRITE_STATUS",
            2,
            1,
            0,
            DriverLinks {
                cmd_tx,
                start: Arc::new(Event::new()),
                stop: stop.clone(),
                shared: Arc::new(parking_lot::Mutex::new(Shared::default())),
            },
        )
        .unwrap();
        let (ad, p) = (det.ad.params, det.p);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("PILATUS_WRITE_STATUS").unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let h = runtime.port_handle().clone();
        for status in [
            AsynStatus::Timeout,
            AsynStatus::Overflow,
            AsynStatus::Error,
            AsynStatus::Disconnected,
            AsynStatus::Disabled,
        ] {
            let name = format!("PILATUS_REFUSING_{status:?}");
            let (cam, _) = create_port_runtime(
                &epics_rs::asyn::registry::claim_port(&name).unwrap(),
                RefusingPort {
                    base: PortDriverBase::new(
                        &name,
                        1,
                        PortFlags::default(),
                        InterfaceMask::UNMIGRATED_ALL,
                        InterfaceMask::UNMIGRATED_ALL,
                    ),
                    status,
                },
                RuntimeConfig::default(),
            )
            .unwrap();
            let mut ctx = Ctx::new(
                h.clone(),
                Arc::new(SyncIOHandle::from_handle(
                    cam.port_handle().clone(),
                    0,
                    std::time::Duration::ZERO,
                )),
                ad,
                p,
                stop.clone(),
            );
            let (tx, rx) = std::sync::mpsc::channel();
            rt::run_thread_named("PilatusWriteStatusTest", move || async move {
                tx.send(ctx.cam_write("version", 1.0).await as i32).unwrap();
            })
            .join()
            .unwrap();
            assert_eq!(rx.recv().unwrap(), status as i32, "{status:?}");
            std::mem::forget(cam);
        }
        std::mem::forget(runtime);
    }
}
