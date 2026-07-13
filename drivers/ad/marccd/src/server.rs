//! The marccd-server execution context: batched parameter access plus the
//! `writeServer` / `readServer` / `writeReadServer` trio and the higher-level
//! `getState` / `getServerMode` / `getConfig` / `writeHeader` / `setShutter`
//! helpers.
//!
//! `marCCD.cpp` runs server I/O from three contexts — the asyn port thread
//! (`writeInt32` / `writeFloat64`), `marCCDTask` and `getImageDataTask` —
//! serialised by the driver lock. A Rust `PortDriver` method runs *inside* the
//! port actor and cannot block on another port, so every context here is a
//! worker thread and the three share one [`Server`] behind a
//! `tokio::sync::Mutex` (the driver-lock analog). `getState` holds that mutex
//! across its write, read and parameter updates, so a concurrent `getState`
//! from the image task cannot interleave a half-updated state word.

use epics_rs::ad_core::driver::{ADStatus, ShutterMode};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::user::AsynUser;

use crate::params::MarccdParams;
use crate::protocol::{
    cmd_header_1, cmd_header_2, cmd_shutter, parse_f64, parse_int, parse_pair, parse_state,
};
use crate::types::{
    CamStatus, MARCCD_SERVER_TIMEOUT, MAX_MESSAGE_SIZE, TASK_ACQUIRE, TASK_CORRECT, TASK_DEZINGER,
    TASK_READ, TASK_SERIES, TASK_STATUS_ERROR, TASK_STATUS_EXECUTING, TASK_WRITE, secs, task_state,
    task_status,
};

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

/// Worker-thread server context, shared behind a `tokio::sync::Mutex`.
pub struct Server {
    /// This driver's own port (parameter library).
    handle: PortHandle,
    /// The marServer octet port created by `drvAsynIPPortConfigure`.
    server_port: PortHandle,
    pub ad: ADBaseParams,
    pub p: MarccdParams,
    /// C `serverMode` (1 = marCCD, 2 = high speed).
    pub server_mode: i32,
    /// C `fromServer`.
    from: String,
    /// Pending parameter writes, flushed at each C `callParamCallbacks()`.
    batch: Vec<ParamSetValue>,
}

impl Server {
    pub fn new(
        handle: PortHandle,
        server_port: PortHandle,
        ad: ADBaseParams,
        p: MarccdParams,
    ) -> Self {
        Self {
            handle,
            server_port,
            ad,
            p,
            server_mode: 1,
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
        self.batch.push(ParamSetValue::Int32 {
            reason,
            addr: 0,
            value,
        });
    }

    /// C `setDoubleParam`.
    pub fn set_f64(&mut self, reason: usize, value: f64) {
        self.batch.push(ParamSetValue::Float64 {
            reason,
            addr: 0,
            value,
        });
    }

    /// C `setStringParam`.
    pub fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.batch.push(ParamSetValue::Octet {
            reason,
            addr: 0,
            value: value.into(),
        });
    }

    /// C `callParamCallbacks()` — apply the pending writes and post monitors.
    pub async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.batch);
        if let Err(e) = self.handle.set_params_and_notify(0, updates).await {
            log::error!("marccd: callParamCallbacks failed: {e}");
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
                log::error!("marccd: getStringParam({reason}) failed: {e}");
                String::new()
            }
        }
    }

    // --- server ------------------------------------------------------------

    /// C `writeServer`: flush stale input, write with `MARCCD_SERVER_TIMEOUT`,
    /// then publish the command on `ADStringToServer`.
    pub async fn write_server(&mut self, output: &str) -> CamStatus {
        let _ = self
            .server_port
            .submit_async(RequestOp::Flush, AsynUser::default().with_addr(SERVER_ADDR))
            .await;

        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(MARCCD_SERVER_TIMEOUT));
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
                log::error!("marccd: writeServer failed, sent {output:?}: {e}");
                classify(&e)
            }
        };

        self.set_str(self.ad.string_to_server, output);
        self.callbacks().await;
        status
    }

    /// C `readServer`: a single read with the given timeout, then publish the
    /// reply on `ADStringFromServer`.
    pub async fn read_server(&mut self, timeout: f64) -> CamStatus {
        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(timeout));
        let status = match self
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
                self.from = String::from_utf8_lossy(&data).into_owned();
                CamStatus::Success
            }
            Err(e) => {
                // C's asynOctetSyncIO null-terminates at nbytesTransfered,
                // which is 0 on a failed read.
                self.from.clear();
                log::error!("marccd: readServer timeout={timeout}, status error: {e}");
                classify(&e)
            }
        };

        let from = self.from.clone();
        self.set_str(self.ad.string_from_server, from);
        self.callbacks().await;
        status
    }

    /// C `writeReadServer`.
    pub async fn write_read_server(&mut self, output: &str, timeout: f64) -> CamStatus {
        let status = self.write_server(output).await;
        if status.is_err() {
            return status;
        }
        self.read_server(timeout).await
    }

    /// C `getState`. Returns the raw `marState` word (0 on a server error,
    /// matching C returning the default `ADStatusIdle` ordinal).
    pub async fn get_state(&mut self) -> i32 {
        let status = self
            .write_read_server("get_state", MARCCD_SERVER_TIMEOUT)
            .await;
        if status.is_err() {
            return ADStatus::Idle as i32;
        }
        let mar_state = parse_state(&self.from);
        let mar_status = task_state(mar_state);
        let acquire_status = task_status(mar_state, TASK_ACQUIRE);
        let readout_status = task_status(mar_state, TASK_READ);
        let correct_status = task_status(mar_state, TASK_CORRECT);
        let writing_status = task_status(mar_state, TASK_WRITE);
        let dezinger_status = task_status(mar_state, TASK_DEZINGER);
        let series_status = task_status(mar_state, TASK_SERIES);

        self.set_i32(self.p.state, mar_state);
        self.set_i32(self.p.status, mar_status);
        self.set_i32(self.p.task_acquire_status, acquire_status);
        self.set_i32(self.p.task_readout_status, readout_status);
        self.set_i32(self.p.task_correct_status, correct_status);
        self.set_i32(self.p.task_writing_status, writing_status);
        self.set_i32(self.p.task_dezinger_status, dezinger_status);
        self.set_i32(self.p.task_series_status, series_status);

        let mut ad_status = ADStatus::Idle;
        if mar_state == 0 {
            ad_status = ADStatus::Idle;
        } else if mar_state == 7 {
            ad_status = ADStatus::Error;
        } else if mar_state == 8 {
            // Really busy interpreting a command, but there is no status for
            // that yet.
            ad_status = ADStatus::Idle;
        } else if acquire_status & TASK_STATUS_EXECUTING != 0 {
            ad_status = ADStatus::Acquire;
        } else if readout_status & TASK_STATUS_EXECUTING != 0 {
            ad_status = ADStatus::Readout;
        } else if correct_status & TASK_STATUS_EXECUTING != 0 {
            ad_status = ADStatus::Correct;
        } else if writing_status & TASK_STATUS_EXECUTING != 0 {
            ad_status = ADStatus::Saving;
        }
        if (acquire_status | readout_status | correct_status | writing_status | dezinger_status)
            & TASK_STATUS_ERROR
            != 0
        {
            ad_status = ADStatus::Error;
        }
        self.set_i32(self.ad.status, ad_status as i32);
        self.callbacks().await;
        mar_state
    }

    /// C `getServerMode`.
    pub async fn get_server_mode(&mut self) -> CamStatus {
        let status = self
            .write_read_server("get_mode", MARCCD_SERVER_TIMEOUT)
            .await;
        if status.is_err() {
            return status;
        }
        if let Some(v) = parse_int(&self.from) {
            self.server_mode = v;
        }
        if !(1..=2).contains(&self.server_mode) {
            log::error!(
                "marccd: error serverMode must be 1 or 2, actual={}",
                self.server_mode
            );
            self.server_mode = 1;
            return CamStatus::Error;
        }
        self.set_i32(self.p.server_mode, self.server_mode);
        CamStatus::Success
    }

    /// C `getConfig`.
    pub async fn get_config(&mut self) -> CamStatus {
        if self.server_mode == 2 {
            let status = self
                .write_read_server("get_readout_mode", MARCCD_SERVER_TIMEOUT)
                .await;
            if status.is_err() {
                return status;
            }
            if let Some(v) = parse_int(&self.from) {
                self.set_i32(self.p.readout_mode, v);
            }
        }

        let status = self
            .write_read_server("get_size", MARCCD_SERVER_TIMEOUT)
            .await;
        if status.is_err() {
            return status;
        }
        let (size_x, size_y) = parse_pair(&self.from).unwrap_or((0, 0));
        self.set_i32(self.ad.base.array_size_x, size_x);
        self.set_i32(self.ad.base.array_size_y, size_y);

        let status = self
            .write_read_server("get_bin", MARCCD_SERVER_TIMEOUT)
            .await;
        if status.is_err() {
            return status;
        }
        let (bin_x, bin_y) = parse_pair(&self.from).unwrap_or((0, 0));
        self.set_i32(self.ad.bin_x, bin_x);
        self.set_i32(self.ad.bin_y, bin_y);
        self.set_i32(self.ad.max_size_x, size_x * bin_x);
        self.set_i32(self.ad.max_size_y, size_y * bin_y);
        let image_size = size_x * size_y * std::mem::size_of::<i16>() as i32;
        self.set_i32(self.ad.base.array_size, image_size);

        // C does not check the status of these last two reads.
        self.write_read_server("get_frameshift", MARCCD_SERVER_TIMEOUT)
            .await;
        if let Some(v) = parse_int(&self.from) {
            self.set_i32(self.p.frame_shift, v);
        }
        self.write_read_server("get_stability", MARCCD_SERVER_TIMEOUT)
            .await;
        if let Some(v) = parse_f64(&self.from) {
            self.set_f64(self.p.stability, v);
        }
        self.callbacks().await;
        CamStatus::Success
    }

    /// C `writeHeader` — two `header,...` writes, no reads.
    pub async fn write_header(&mut self) -> CamStatus {
        let detector_distance = self.get_f64(self.p.detector_distance).await;
        let beam_x = self.get_f64(self.p.beam_x).await;
        let beam_y = self.get_f64(self.p.beam_y).await;
        let exposure_time = self.get_f64(self.ad.acquire_time).await;
        let start_phi = self.get_f64(self.p.start_phi).await;
        let rotation_axis = self.get_str(self.p.rotation_axis, MAX_MESSAGE_SIZE).await;
        let rotation_range = self.get_f64(self.p.rotation_range).await;
        let two_theta = self.get_f64(self.p.two_theta).await;
        let wavelength = self.get_f64(self.p.wavelength).await;
        let file_comments = self.get_str(self.p.file_comments, MAX_MESSAGE_SIZE).await;
        let dataset_comments = self
            .get_str(self.p.dataset_comments, MAX_MESSAGE_SIZE)
            .await;

        self.write_server(&cmd_header_1(
            detector_distance,
            beam_x,
            beam_y,
            exposure_time,
        ))
        .await;
        self.write_server(&cmd_header_2(
            start_phi,
            &rotation_axis,
            rotation_range,
            two_theta,
            wavelength,
            &file_comments,
            &dataset_comments,
        ))
        .await
    }

    /// C `marCCD::setShutter` — overrides the base class for detector-controlled
    /// shutters and delegates the EPICS-controlled case to the base behaviour.
    pub async fn set_shutter(&mut self, open: bool) {
        let mode = ShutterMode::from_i32(self.get_i32(self.ad.shutter_mode).await);
        let shutter_open_delay = self.get_f64(self.ad.shutter_open_delay).await;
        let shutter_close_delay = self.get_f64(self.ad.shutter_close_delay).await;

        match mode {
            Some(ShutterMode::DetectorOnly) => {
                if open {
                    self.write_server(cmd_shutter(true)).await;
                    // Correct the exposure time: opening minus closing time,
                    // floored at 1 ms so the delay is never negative and the
                    // commands are not back-to-back.
                    let mut delay = shutter_open_delay - shutter_close_delay;
                    if delay < 0.001 {
                        delay = 0.001;
                    }
                    epics_rs::ad_core::runtime::sleep(secs(delay)).await;
                } else {
                    self.write_server(cmd_shutter(false)).await;
                    epics_rs::ad_core::runtime::sleep(secs(shutter_close_delay)).await;
                }
                // The marCCD does not report actual shutter status, so set it to
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
                    epics_rs::ad_core::runtime::sleep(secs(delay)).await;
                }
            }
            // ADShutterModeNone or unknown: nothing to do.
            Some(ShutterMode::None) | None => {}
        }
    }
}
