//! The camserver `writeCamserver` / `readCamserver` / `writeReadCamserver`
//! trio, and the acquisition thread's batched parameter context.
//!
//! `pilatusDetector.cpp` runs camserver I/O from two contexts: the asyn port
//! thread (`writeInt32` / `writeFloat64` / `writeOctet`) and `pilatusTask`. The
//! first is the driver here, inline on the port's own thread; the second is
//! `PilatusDetTask`, which reaches the driver's parameters through a
//! [`PortHandle`]. Both run the one [`Camserver`] trio, blocking on the
//! camserver octet port, which is a different port.

use std::sync::Arc;
use std::time::Instant;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::driver::ndarray_driver::check_path_str;
use epics_rs::ad_core::driver::{ADStatus, ShutterMode};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

use crate::params::PilatusParams;
use crate::protocol::reply_is_ok;
use crate::types::{ASYN_POLL_TIME, Event, MAX_FILENAME_LEN, MAX_MESSAGE_SIZE, secs};

/// C `readCamserver`: `epicsEventWaitWithTimeout(stopEventId, 0.001)` right
/// after the socket read.
const ABORT_POLL_TIME: f64 = 0.001;

/// The parameter sets and `pasynUserSelf` prints a camserver exchange makes:
/// on the driver's own library inline, batched on a worker thread.
pub(crate) trait CamParams {
    fn set_i32(&mut self, reason: usize, value: i32);
    fn set_str(&mut self, reason: usize, value: String);
    fn trace(&self, mask: TraceMask, msg: &str);
}

impl CamParams for PortDriverBase {
    fn set_i32(&mut self, reason: usize, value: i32) {
        let _ = self.set_int32_param(reason, 0, value);
    }
    fn set_str(&mut self, reason: usize, value: String) {
        let _ = self.set_string_param(reason, 0, value);
    }
    fn trace(&self, mask: TraceMask, msg: &str) {
        self.trace_print(mask, format_args!("{msg}"));
    }
}

/// C `pasynUserCamserver` and the `toCamserver` / `fromCamserver` buffers.
pub(crate) struct Camserver {
    /// No port when the startup script named one that does not exist.
    port: Arc<SyncIOHandle>,
    ad: ADBaseParams,
    stop: Arc<Event>,
    from: String,
    /// Kept because `pilatusTask`'s trigger-mode `switch` has no `default`:
    /// an out-of-range `ADTriggerMode` re-sends whatever command the buffer
    /// still holds.
    to: String,
}

impl Camserver {
    pub(crate) fn new(port: Arc<SyncIOHandle>, ad: ADBaseParams, stop: Arc<Event>) -> Self {
        Self {
            port,
            ad,
            stop,
            from: String::new(),
            to: String::new(),
        }
    }

    /// C `fromCamserver`.
    pub(crate) fn from(&self) -> &str {
        &self.from
    }

    /// C `toCamserver`.
    pub(crate) fn to_camserver(&self) -> &str {
        &self.to
    }

    /// C `writeCamserver`: flush stale input, write, then publish the command
    /// on `ADStringToServer` regardless of the write status.
    pub(crate) fn write(
        &mut self,
        side: &mut impl CamParams,
        cmd: &str,
        timeout: f64,
    ) -> AsynStatus {
        self.to = cmd.to_string();
        let _ = self.port.flush(0);

        let status = match self
            .port
            .with_timeout(secs(timeout))
            .write_octet(0, cmd.as_bytes())
        {
            Ok(_) => AsynStatus::Success,
            Err(e) => {
                let status = e.status();
                side.trace(
                    TraceMask::ERROR,
                    &format!(
                        "pilatusDetector:writeCamserver, status={}, sent\n{cmd}\n",
                        status as i32
                    ),
                );
                status
            }
        };

        side.set_str(self.ad.string_to_server, cmd.to_string());
        status
    }

    /// C `readCamserver`: poll the socket in `ASYN_POLL_TIME` slices so an
    /// abort can interrupt a long exposure.
    pub(crate) fn read(&mut self, side: &mut impl CamParams, timeout: f64) -> AsynStatus {
        let t_start = Instant::now();
        let mut delta_time = 0.0f64;
        let mut status = AsynStatus::Success;
        let mut nread = 0;

        while delta_time <= timeout {
            let read = self
                .port
                .with_timeout(secs(ASYN_POLL_TIME))
                .read_octet(0, MAX_MESSAGE_SIZE);
            // C's fromCamserver ends at `nbytesTransfered`, which counts what a
            // timed-out read still delivered.
            let (data, result) = read.into_parts();
            nread = data.len();
            self.from = String::from_utf8_lossy(&data).into_owned();
            status = result.map_or_else(|e| e.status(), |()| AsynStatus::Success);

            // C checks for an abort delivered during the read, so the next
            // acquisition does not inherit it.
            if self.stop.wait_timeout(secs(ABORT_POLL_TIME)) {
                side.set_str(self.ad.status_message, "Acquisition aborted".into());
                side.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return AsynStatus::Error;
            }
            if status != AsynStatus::Timeout {
                break;
            }

            if self.stop.wait_timeout(secs(ASYN_POLL_TIME)) {
                side.set_str(self.ad.status_message, "Acquisition aborted".into());
                side.set_i32(self.ad.status, ADStatus::Aborted as i32);
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
            self.port.print(0, TraceMask::ERROR, format_args!("{msg}"));
        };
        if status != AsynStatus::Success {
            cam_trace(&format!(
                "pilatusDetector:readCamserver, timeout={timeout:.6}, status={} received {} bytes\n{}\n",
                status as i32,
                nread,
                self.from,
                timeout = CFloat(timeout)
            ));
        } else if !reply_is_ok(&self.from) {
            cam_trace(&format!(
                "pilatusDetector:readCamserver unexpected response from camserver, no OK, response={}\n",
                self.from
            ));
            side.set_str(self.ad.status_message, "Error from camserver".into());
            status = AsynStatus::Error;
        } else {
            side.set_str(self.ad.status_message, "Camserver returned OK".into());
        }

        side.set_str(self.ad.string_from_server, self.from.clone());
        status
    }

    /// C `writeReadCamserver`.
    pub(crate) fn write_read(
        &mut self,
        side: &mut impl CamParams,
        cmd: &str,
        timeout: f64,
    ) -> AsynStatus {
        let status = self.write(side, cmd, timeout);
        if status != AsynStatus::Success {
            return status;
        }
        self.read(side, timeout)
    }
}

/// Pending parameter writes, flushed by [`Ctx::callbacks`] at exactly the
/// points where C calls `callParamCallbacks()`.
pub(crate) struct Side {
    /// This driver's own port (parameter library).
    handle: PortHandle,
    updates: Vec<ParamSetValue>,
}

impl CamParams for Side {
    fn set_i32(&mut self, reason: usize, value: i32) {
        self.updates
            .push(ParamSetValue::new(reason, 0, ParamValue::Int32(value)));
    }
    fn set_str(&mut self, reason: usize, value: String) {
        self.updates.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(Into::<Vec<u8>>::into(value).into()),
        ));
    }
    /// An `asynPrint` on `pasynUserSelf`.
    fn trace(&self, mask: TraceMask, msg: &str) {
        self.handle
            .trace()
            .output_device(self.handle.port_name(), Some(0), 0, mask, msg);
    }
}

/// `pilatusTask`'s context.
pub struct Ctx {
    side: Side,
    cam: Camserver,
    pub ad: ADBaseParams,
    pub p: PilatusParams,
    pub stop: Arc<Event>,
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
            side: Side {
                handle,
                updates: Vec::new(),
            },
            cam: Camserver::new(cam_port, ad, stop.clone()),
            ad,
            p,
            stop,
        }
    }

    /// The last camserver reply (C `fromCamserver`).
    pub fn from(&self) -> &str {
        self.cam.from()
    }

    /// The last camserver command (C `toCamserver`).
    pub fn to_camserver(&self) -> &str {
        self.cam.to_camserver()
    }

    // --- parameter library -------------------------------------------------

    /// C `setIntegerParam`.
    pub fn set_i32(&mut self, reason: usize, value: i32) {
        self.side.set_i32(reason, value);
    }

    /// The image-file readers, posting through this context's batch.
    pub(crate) fn image_reader(&mut self) -> crate::task::ImageReader<'_, Side> {
        crate::task::ImageReader {
            cx: &mut self.side,
            ad: self.ad,
            stop: &self.stop,
        }
    }

    /// C `setDoubleParam`.
    pub fn set_f64(&mut self, reason: usize, value: f64) {
        self.side
            .updates
            .push(ParamSetValue::new(reason, 0, ParamValue::Float64(value)));
    }

    /// C `setStringParam`.
    pub fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.side.set_str(reason, value.into());
    }

    /// C `callParamCallbacks()` — applies the pending writes and posts monitors
    /// in one actor message.
    pub async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.side.updates);
        let _ = self.side.handle.set_params_and_notify(0, updates).await;
    }

    /// C `updateTimeStamps`' stamp: the port's, moved now
    /// (asynNDArrayDriver.cpp:832-836).
    pub async fn update_time_stamp(&self) -> epics_rs::ad_core::timestamp::EpicsTimestamp {
        self.side
            .handle
            .update_time_stamp()
            .await
            .unwrap_or_else(|_| std::time::SystemTime::now())
            .into()
    }

    /// An `asynPrint` on `pasynUserSelf`.
    pub fn trace(&self, mask: TraceMask, msg: &str) {
        self.side.trace(mask, msg);
    }

    /// Flush pending writes so a following read observes them. The actor
    /// processes its inbox in FIFO order, so enqueueing is enough.
    async fn flush(&mut self) {
        if !self.side.updates.is_empty() {
            self.callbacks().await;
        }
    }

    /// C `getIntegerParam`.
    pub async fn get_i32(&mut self, reason: usize) -> Option<i32> {
        self.flush().await;
        self.side.handle.read_int32(reason, 0).await.ok()
    }

    /// C `getDoubleParam`.
    pub async fn get_f64(&mut self, reason: usize) -> Option<f64> {
        self.flush().await;
        self.side.handle.read_float64(reason, 0).await.ok()
    }

    /// C `getStringParam` with a `max_chars` capacity (the C call truncates to
    /// the caller's buffer).
    pub async fn get_str(&mut self, reason: usize, max_chars: usize) -> String {
        self.flush().await;
        match self
            .side
            .handle
            .get_string_param(reason, 0, max_chars)
            .await
        {
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

    /// C `writeCamserver`.
    pub fn cam_write(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        self.cam.write(&mut self.side, cmd, timeout)
    }

    /// C `readCamserver`.
    pub fn cam_read(&mut self, timeout: f64) -> AsynStatus {
        self.cam.read(&mut self.side, timeout)
    }

    /// C `writeReadCamserver`.
    pub fn cam_write_read(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        self.cam.write_read(&mut self.side, cmd, timeout)
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
    use crate::task::FlatFieldJob;
    use crate::task::Shared;
    use epics_rs::ad_core::runtime as rt;
    use epics_rs::asyn::error::{AsynError, AsynResult};
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use epics_rs::asyn::user::AsynUser;

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

    /// A camserver that answers every read `15 OK`.
    struct Answers(PortDriverBase);

    impl PortDriver for Answers {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            buf[..5].copy_from_slice(b"15 OK");
            OctetReadResult::ok(5, EomReason::empty())
        }
    }

    /// `writeCamserver` and `readCamserver` are `pasynOctetSyncIO` flush,
    /// write and read (pilatusDetector.cpp:922-923, :956), which print their
    /// TRACEIO_DEVICE lines (asynOctetSyncIO.c:194-200, :219-223, :291-293).
    #[test]
    fn a_camserver_exchange_prints_its_sync_io_lines() {
        use epics_rs::asyn::trace::{TraceFile, TraceIoMask};
        let name = "PILATUS_LINES";
        let new_base = |name: &str| {
            PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            )
        };
        let (cam, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            Answers(new_base(name)),
            RuntimeConfig::default(),
        )
        .unwrap();
        let trace = cam.port_handle().trace();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        trace.set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&path).unwrap(),
            ))),
        );
        trace.set_trace_mask(Some(name), TraceMask::IO_DEVICE);
        trace.set_trace_io_mask(Some(name), TraceIoMask::ASCII);
        let mut side = new_base("PILATUS_LINES_DET");
        let (ad, _) = ADBaseParams::create(&mut side).unwrap();
        let mut camserver = Camserver::new(
            Arc::new(SyncIOHandle::from_handle(
                cam.port_handle().clone(),
                0,
                std::time::Duration::ZERO,
            )),
            ad,
            Arc::new(Event::new()),
        );
        assert_eq!(
            camserver.write_read(&mut side, "Exposure x", 1.0),
            AsynStatus::Success
        );
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.contains("asynOctetSyncIO flush\n"), "{text}");
        assert!(
            text.contains("asynOctetSyncIO wrote:\nExposure x\n"),
            "{text}"
        );
        assert!(text.contains("asynOctetSyncIO read:\n15 OK\n"), "{text}");
        std::mem::forget(cam);
    }

    /// C `writeCamserver` returns and prints `pasynOctetSyncIO->write`'s own
    /// status (pilatusDetector.cpp:1338-1345): a camserver port with no
    /// connection prints `status=4`, not asynError's 3.
    #[test]
    fn a_camserver_write_keeps_the_octet_ports_status() {
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let stop = Arc::new(Event::new());
        let det = PilatusDriver::new(
            "PILATUS_WRITE_STATUS",
            2,
            1,
            0,
            DriverLinks {
                flat_field_tx,
                start: Arc::new(Event::new()),
                stop: stop.clone(),
                shared: Arc::new(parking_lot::Mutex::new(Shared::default())),
                cam: Arc::new(
                    epics_rs::asyn::sync_io::SyncIOHandle::connect_port(
                        "PILATUS_NO_CAM",
                        0,
                        std::time::Duration::ZERO,
                        epics_rs::asyn::interfaces::InterfaceType::Octet,
                    )
                    .0,
                ),
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
                        InterfaceMask::UNMIGRATED_INTERRUPTS,
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
                tx.send(ctx.cam_write("version", 1.0) as i32).unwrap();
            })
            .join()
            .unwrap();
            assert_eq!(rx.recv().unwrap(), status as i32, "{status:?}");
            std::mem::forget(cam);
        }
        std::mem::forget(runtime);
    }

    /// Replays one read per entry, `(bytes, timed_out)`, repeating the last.
    struct ScriptedPort {
        base: PortDriverBase,
        reads: std::collections::VecDeque<(&'static [u8], bool)>,
    }

    impl PortDriver for ScriptedPort {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            let (data, timed_out) = if self.reads.len() > 1 {
                self.reads.pop_front().unwrap()
            } else {
                self.reads[0]
            };
            buf[..data.len()].copy_from_slice(data);
            if !timed_out {
                return OctetReadResult::ok(data.len(), EomReason::EOS);
            }
            OctetReadResult {
                nbytes_transferred: data.len(),
                status: Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "read timeout".into(),
                }),
                ..OctetReadResult::ok(0, EomReason::empty())
            }
        }
    }

    /// `cam_read(timeout)` over a camserver port replaying `reads`: the
    /// status and the `fromCamserver` it ends with.
    fn cam_read_over(name: &str, reads: &[(&'static [u8], bool)], timeout: f64) -> (i32, String) {
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let stop = Arc::new(Event::new());
        let det = PilatusDriver::new(
            name,
            2,
            1,
            0,
            DriverLinks {
                flat_field_tx,
                start: Arc::new(Event::new()),
                stop: stop.clone(),
                shared: Arc::new(parking_lot::Mutex::new(Shared::default())),
                cam: Arc::new(
                    epics_rs::asyn::sync_io::SyncIOHandle::connect_port(
                        "PILATUS_NO_CAM",
                        0,
                        std::time::Duration::ZERO,
                        epics_rs::asyn::interfaces::InterfaceType::Octet,
                    )
                    .0,
                ),
            },
        )
        .unwrap();
        let (ad, p) = (det.ad.params, det.p);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let cam_name = format!("{name}_CAM");
        let (cam, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(&cam_name).unwrap(),
            ScriptedPort {
                base: PortDriverBase::new(
                    &cam_name,
                    1,
                    PortFlags::default(),
                    InterfaceMask::UNMIGRATED_ALL,
                    InterfaceMask::UNMIGRATED_INTERRUPTS,
                ),
                reads: reads.iter().copied().collect(),
            },
            RuntimeConfig::default(),
        )
        .unwrap();
        let mut ctx = Ctx::new(
            runtime.port_handle().clone(),
            Arc::new(SyncIOHandle::from_handle(
                cam.port_handle().clone(),
                0,
                std::time::Duration::ZERO,
            )),
            ad,
            p,
            stop,
        );
        let (tx, rx) = std::sync::mpsc::channel();
        rt::run_thread_named("PilatusCamReadTest", move || async move {
            let status = ctx.cam_read(timeout) as i32;
            tx.send((status, ctx.from().to_string())).unwrap();
        })
        .join()
        .unwrap();
        std::mem::forget(cam);
        std::mem::forget(runtime);
        rx.recv().unwrap()
    }

    /// C `readCamserver` (pilatusDetector.cpp:953-976) breaks only on a
    /// status other than asynTimeout, so a timeout after part of a reply
    /// polls again and the next read completes it.
    #[test]
    fn a_timeout_after_part_of_a_reply_keeps_polling() {
        let (status, from) = cam_read_over(
            "PILATUS_PARTIAL_POLL",
            &[(b"15 OK", true), (b"15 OK exptime 1.0", false)],
            1.0,
        );
        assert_eq!(status, AsynStatus::Success as i32);
        assert_eq!(from, "15 OK exptime 1.0");
    }

    /// When the polling runs out, C's `fromCamserver` holds what the last
    /// read delivered, terminated at its `nbytesTransfered`.
    #[test]
    fn a_read_that_ends_on_a_partial_timeout_keeps_the_partial_reply() {
        let (status, from) = cam_read_over(
            "PILATUS_PARTIAL_END",
            &[(b"15 OK", true), (b"15 OK exp", true)],
            0.05,
        );
        assert_eq!(status, AsynStatus::Timeout as i32);
        assert_eq!(from, "15 OK exp");
    }

    /// C prints the last read's nread (pilatusDetector.cpp:985-987): the bytes
    /// the camserver sent, not the length of their lossy text.
    #[test]
    fn a_failed_read_prints_the_bytes_the_last_read_delivered() {
        use epics_rs::asyn::services::PortServices;
        use epics_rs::asyn::trace::TraceFile;

        let name = "PILATUS_NREAD";
        let log = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&log).unwrap();
        PortServices::global().trace().set_trace_file(
            Some(&format!("{name}_CAM")),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let (status, _) = cam_read_over(name, &[(b"15 \xff", true)], 0.05);
        let text = std::fs::read_to_string(&log).unwrap();
        let _ = std::fs::remove_file(&log);
        assert_eq!(status, AsynStatus::Timeout as i32);
        assert!(text.contains(" received 4 bytes\n"), "{text}");
    }
}
