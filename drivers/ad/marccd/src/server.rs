//! marServer I/O on the driver: the `writeServer` / `readServer` /
//! `writeReadServer` trio and the `getState` / `getServerMode` / `getConfig` /
//! `writeHeader` / `saveFile` / `setShutter` helpers built on them.
//!
//! As in `marCCD.cpp` they run under the driver lock, which here is the port
//! actor: `writeInt32` / `writeFloat64` call them inline, and the worker
//! threads reach them through `task::Held`. Each call blocks the
//! actor on the marServer port, which is a different port, so the round trip
//! cannot interleave with another context's.

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::error::{AsynResult, AsynStatus};
use epics_rs::asyn::request::{OctetRead, OctetReadOp, RequestOp, RequestResult};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::driver::MarccdDriver;
use crate::file_name::format_file_template;
use crate::protocol::{
    cmd_header_1, cmd_header_2, cmd_readout, cmd_shutter, cmd_writefile, parse_f64, parse_int,
    parse_pair, parse_state,
};
use crate::types::{
    MARCCD_POLL_DELAY, MARCCD_SERVER_TIMEOUT, MAX_FILENAME_LEN, MAX_MESSAGE_SIZE, TASK_ACQUIRE,
    TASK_CORRECT, TASK_DEZINGER, TASK_READ, TASK_SERIES, TASK_STATE_BUSY, TASK_STATE_ERROR,
    TASK_STATUS_ERROR, TASK_STATUS_EXECUTING, TASK_STATUS_QUEUED, TASK_WRITE, secs, task_state,
    task_status, test_task_status,
};

/// asyn subaddress of the single-device marServer octet port.
const SERVER_ADDR: i32 = 0;

impl MarccdDriver {
    // --- parameter library -------------------------------------------------

    /// C `getIntegerParam`.
    pub fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    /// C `getDoubleParam`.
    pub fn get_f64(&self, reason: usize) -> f64 {
        self.ad
            .port_base
            .get_float64_param(reason, 0)
            .unwrap_or(0.0)
    }

    /// C `getStringParam`: the text up to its first NUL and at most
    /// `max_chars - 1` bytes.
    pub fn get_str(&self, reason: usize, max_chars: usize) -> String {
        let bytes = self
            .ad
            .port_base
            .get_string_param(reason, 0)
            .unwrap_or_default();
        let text = &bytes[..bytes.iter().position(|&c| c == 0).unwrap_or(bytes.len())];
        let len = text.len().min(max_chars.saturating_sub(1));
        String::from_utf8_lossy(&text[..len]).into_owned()
    }

    /// C `setIntegerParam`.
    pub fn set_i32(&mut self, reason: usize, value: i32) {
        let _ = self.ad.port_base.set_int32_param(reason, 0, value);
    }

    /// C `setDoubleParam`.
    pub fn set_f64(&mut self, reason: usize, value: f64) {
        let _ = self.ad.port_base.set_float64_param(reason, 0, value);
    }

    /// C `setStringParam`.
    pub fn set_str(&mut self, reason: usize, value: &str) {
        let _ = self.ad.port_base.set_string_param(reason, 0, value);
    }

    /// C `callParamCallbacks()`.
    pub fn callbacks(&mut self) {
        let _ = self.ad.port_base.call_param_callbacks(0);
    }

    /// C `asynPrint(pasynUserSelf, mask, ...)`.
    pub fn trace(&self, mask: TraceMask, message: &str) {
        self.ad
            .port_base
            .trace_print(mask, format_args!("{message}\n"));
    }

    /// C `asynPrint(pasynUserServer, ASYN_TRACE_ERROR, ...)`: the server
    /// port's trace mask gates it.
    fn trace_server_error(&self, message: &str) {
        self.server
            .print(0, TraceMask::ERROR, format_args!("{message}"));
    }

    /// One request on C `pasynUserServer`, which fails when it has no port.
    fn server_submit(&self, op: RequestOp, user: AsynUser) -> AsynResult<RequestResult> {
        self.server.port()?.submit_blocking(op, user)
    }

    // --- server ------------------------------------------------------------

    /// C `writeServer`: flush stale input, write with `MARCCD_SERVER_TIMEOUT`,
    /// then publish the command on `ADStringToServer`.
    pub fn write_server(&mut self, output: &str) -> AsynStatus {
        let _ = self.server_submit(RequestOp::Flush, AsynUser::default().with_addr(SERVER_ADDR));

        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(MARCCD_SERVER_TIMEOUT));
        let status = match self.server_submit(
            RequestOp::OctetWrite {
                data: output.as_bytes().to_vec(),
            },
            user,
        ) {
            Ok(_) => AsynStatus::Success,
            Err(e) => {
                self.trace_server_error(&format!(
                    "marCCD:writeServer, status={}, sent\n{output}\n",
                    e.status() as i32
                ));
                e.status()
            }
        };

        self.set_str(self.ad.params.string_to_server, output);
        self.callbacks();
        status
    }

    /// C `readServer`: a single read with the given timeout, then publish the
    /// reply on `ADStringFromServer`.
    pub fn read_server(&mut self, timeout: f64) -> AsynStatus {
        let user = AsynUser::default()
            .with_addr(SERVER_ADDR)
            .with_timeout(secs(timeout));
        let read = match self.server.port() {
            Ok(port) => port.submit_octet_read_blocking(
                OctetReadOp::Read {
                    buf_size: MAX_MESSAGE_SIZE,
                },
                user,
            ),
            Err(e) => OctetRead::failed(e),
        };
        // C's asynOctetSyncIO null-terminates at nbytesTransfered, which
        // counts what a failed read did transfer.
        let (data, status) = read.into_parts();
        self.from = String::from_utf8_lossy(&data).into_owned();
        let status = status.map_or_else(|e| e.status(), |()| AsynStatus::Success);
        if status != AsynStatus::Success {
            self.trace_server_error(&format!(
                "marCCD:readServer, timeout={timeout:.6}, status={} received {} bytes\n{}\n",
                status as i32,
                data.len(),
                self.from
            ));
        }

        let from = std::mem::take(&mut self.from);
        self.set_str(self.ad.params.string_from_server, &from);
        self.from = from;
        self.callbacks();
        status
    }

    /// C `writeReadServer`.
    pub fn write_read_server(&mut self, output: &str, timeout: f64) -> AsynStatus {
        let status = self.write_server(output);
        if status != AsynStatus::Success {
            return status;
        }
        self.read_server(timeout)
    }

    /// C `getState`. Returns the raw `marState` word (0 on a server error,
    /// matching C returning the default `ADStatusIdle` ordinal).
    pub fn get_state(&mut self) -> i32 {
        let status = self.write_read_server("get_state", MARCCD_SERVER_TIMEOUT);
        if status != AsynStatus::Success {
            return ADStatus::Idle as i32;
        }
        let p = self.p;
        let mar_state = parse_state(&self.from);
        let mar_status = task_state(mar_state);
        let acquire_status = task_status(mar_state, TASK_ACQUIRE);
        let readout_status = task_status(mar_state, TASK_READ);
        let correct_status = task_status(mar_state, TASK_CORRECT);
        let writing_status = task_status(mar_state, TASK_WRITE);
        let dezinger_status = task_status(mar_state, TASK_DEZINGER);
        let series_status = task_status(mar_state, TASK_SERIES);

        self.set_i32(p.state, mar_state);
        self.set_i32(p.status, mar_status);
        self.set_i32(p.task_acquire_status, acquire_status);
        self.set_i32(p.task_readout_status, readout_status);
        self.set_i32(p.task_correct_status, correct_status);
        self.set_i32(p.task_writing_status, writing_status);
        self.set_i32(p.task_dezinger_status, dezinger_status);
        self.set_i32(p.task_series_status, series_status);

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
        self.set_i32(self.ad.params.status, ad_status as i32);
        self.callbacks();
        mar_state
    }

    /// C `getServerMode`.
    pub fn get_server_mode(&mut self) -> AsynStatus {
        let status = self.write_read_server("get_mode", MARCCD_SERVER_TIMEOUT);
        if status != AsynStatus::Success {
            return status;
        }
        if let Some(v) = parse_int(&self.from) {
            self.server_mode = v;
        }
        if !(1..=2).contains(&self.server_mode) {
            self.trace(
                TraceMask::ERROR,
                &format!(
                    "marCCD:getServerMode: error serverMode must be 1 or 2, actual={}",
                    self.server_mode
                ),
            );
            self.server_mode = 1;
            return AsynStatus::Error;
        }
        self.set_i32(self.p.server_mode, self.server_mode);
        AsynStatus::Success
    }

    /// C `getConfig`.
    pub fn get_config(&mut self) -> AsynStatus {
        let ad = self.ad.params;
        let p = self.p;
        if self.server_mode == 2 {
            let status = self.write_read_server("get_readout_mode", MARCCD_SERVER_TIMEOUT);
            if status != AsynStatus::Success {
                return status;
            }
            if let Some(v) = parse_int(&self.from) {
                self.set_i32(p.readout_mode, v);
            }
        }

        let status = self.write_read_server("get_size", MARCCD_SERVER_TIMEOUT);
        if status != AsynStatus::Success {
            return status;
        }
        let (size_x, size_y) = parse_pair(&self.from).unwrap_or((0, 0));
        self.set_i32(ad.base.array_size_x, size_x);
        self.set_i32(ad.base.array_size_y, size_y);

        let status = self.write_read_server("get_bin", MARCCD_SERVER_TIMEOUT);
        if status != AsynStatus::Success {
            return status;
        }
        let (bin_x, bin_y) = parse_pair(&self.from).unwrap_or((0, 0));
        self.set_i32(ad.bin_x, bin_x);
        self.set_i32(ad.bin_y, bin_y);
        self.set_i32(ad.max_size_x, size_x * bin_x);
        self.set_i32(ad.max_size_y, size_y * bin_y);
        let image_size = size_x * size_y * std::mem::size_of::<i16>() as i32;
        self.set_i32(ad.base.array_size, image_size);

        // C does not check the status of these last two reads.
        self.write_read_server("get_frameshift", MARCCD_SERVER_TIMEOUT);
        if let Some(v) = parse_int(&self.from) {
            self.set_i32(p.frame_shift, v);
        }
        self.write_read_server("get_stability", MARCCD_SERVER_TIMEOUT);
        if let Some(v) = parse_f64(&self.from) {
            self.set_f64(p.stability, v);
        }
        self.callbacks();
        AsynStatus::Success
    }

    /// C `writeHeader` — two `header,...` writes, no reads.
    pub fn write_header(&mut self) -> AsynStatus {
        let p = self.p;
        let detector_distance = self.get_f64(p.detector_distance);
        let beam_x = self.get_f64(p.beam_x);
        let beam_y = self.get_f64(p.beam_y);
        let exposure_time = self.get_f64(self.ad.params.acquire_time);
        let start_phi = self.get_f64(p.start_phi);
        let rotation_axis = self.get_str(p.rotation_axis, MAX_MESSAGE_SIZE);
        let rotation_range = self.get_f64(p.rotation_range);
        let two_theta = self.get_f64(p.two_theta);
        let wavelength = self.get_f64(p.wavelength);
        let file_comments = self.get_str(p.file_comments, MAX_MESSAGE_SIZE);
        let dataset_comments = self.get_str(p.dataset_comments, MAX_MESSAGE_SIZE);

        self.write_server(&cmd_header_1(
            detector_distance,
            beam_x,
            beam_y,
            exposure_time,
        ));
        self.write_server(&cmd_header_2(
            start_phi,
            &rotation_axis,
            rotation_range,
            two_theta,
            wavelength,
            &file_comments,
            &dataset_comments,
        ))
    }

    /// C `asynNDArrayDriver::createFileName`.
    pub fn create_file_name(&mut self) -> String {
        let base = self.ad.params.base;
        let file_path = self.get_str(base.file_path, MAX_FILENAME_LEN);
        let file_name = self.get_str(base.file_name, MAX_FILENAME_LEN);
        let template = self.get_str(base.file_template, MAX_FILENAME_LEN);
        let file_number = self.get_i32(base.file_number);
        let auto_increment = self.get_i32(base.auto_increment);

        let full = format_file_template(&template, &file_path, &file_name, file_number)
            .unwrap_or_default();
        if auto_increment != 0 {
            self.set_i32(base.file_number, file_number + 1);
        }
        full
    }

    /// Poll `getState` every `MARCCD_POLL_DELAY` while `busy` holds, as C's
    /// task loops do — sleeping with the driver lock held.
    pub(crate) fn wait_state(&mut self, busy: impl Fn(i32) -> bool) {
        let mut s = self.get_state();
        while busy(s) {
            std::thread::sleep(secs(MARCCD_POLL_DELAY));
            s = self.get_state();
        }
    }

    /// [`Self::wait_state`] as `readoutFrame` polls: an error once marServer
    /// reports `TASK_STATE_ERROR`.
    fn wait_state_or_error(&mut self, busy: impl Fn(i32) -> bool) -> AsynStatus {
        let mut s = self.get_state();
        while busy(s) {
            std::thread::sleep(secs(MARCCD_POLL_DELAY));
            s = self.get_state();
            if task_state(s) == TASK_STATE_ERROR {
                return AsynStatus::Error;
            }
        }
        AsynStatus::Success
    }

    /// C `readoutFrame`: all of it under the driver lock.
    pub(crate) fn readout_frame(
        &mut self,
        buffer_number: i32,
        file_name: Option<&str>,
        wait: bool,
    ) -> AsynStatus {
        let reading =
            |s| test_task_status(s, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0;
        // Wait for the readout task to be done with the previous frame.
        let status = self.wait_state_or_error(|s| reading(s) || task_state(s) >= TASK_STATE_BUSY);
        if status != AsynStatus::Success {
            return status;
        }
        let file_name = file_name.filter(|n| !n.is_empty());
        if let Some(name) = file_name {
            self.set_str(self.ad.params.base.full_file_name, name);
            self.callbacks();
        }
        self.write_server(&cmd_readout(buffer_number, file_name));

        // Wait for the readout to start, then to complete.
        let status = self.wait_state_or_error(|s| !reading(s));
        if status != AsynStatus::Success {
            return status;
        }
        let status = self.wait_state_or_error(reading);
        if status != AsynStatus::Success || !wait {
            return status;
        }

        // Wait for the correction to complete.
        let status = self.wait_state_or_error(|s| {
            test_task_status(s, TASK_CORRECT, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
        });
        if status != AsynStatus::Success || file_name.is_none() {
            return status;
        }
        // A file was named: wait for the write to complete.
        self.wait_state_or_error(|s| {
            test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
                || task_state(s) >= TASK_STATE_BUSY
        })
    }

    /// C `saveFile`.
    pub fn save_file(&mut self, corrected_flag: i32, wait: bool) {
        let writing = |s| {
            test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
                || task_state(s) >= TASK_STATE_BUSY
        };
        // Wait for any previous write to complete.
        self.wait_state(writing);
        self.write_header();
        let full_file_name = self.create_file_name();
        self.write_server(&cmd_writefile(&full_file_name, corrected_flag));
        self.set_str(self.ad.params.base.full_file_name, &full_file_name);
        self.callbacks();
        if wait {
            self.wait_state(writing);
        }
    }

    /// `marCCD::setShutter`'s detector branch (marCCD.cpp:723-743).
    pub(crate) fn set_detector_shutter(&mut self, open: bool) {
        let ad = self.ad.params;
        let shutter_open_delay = self.get_f64(ad.shutter_open_delay);
        let shutter_close_delay = self.get_f64(ad.shutter_close_delay);
        let delay = if open {
            // Correct the exposure time: opening minus closing time, floored
            // at 1 ms so the delay is never negative and the commands are not
            // back-to-back.
            (shutter_open_delay - shutter_close_delay).max(0.001)
        } else {
            shutter_close_delay
        };
        self.write_server(cmd_shutter(open));
        std::thread::sleep(secs(delay));
        // The marCCD does not report actual shutter status, so set it to agree
        // with the control value.
        self.set_i32(ad.shutter_status, i32::from(open));
        self.callbacks();
    }
}
