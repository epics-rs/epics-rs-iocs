//! marServer I/O and the parameter library, on the driver.
//!
//! `mar345.cpp` writes to marServer under the driver lock, from `writeInt32`
//! (`setShutter`) and from `mar345Task`, and drops the lock for each read
//! (`readServer`, :260-263). Here the lock is the port actor: the writes and
//! parameter access run on [`Mar345Driver`], inline or through the worker's
//! `task::Held`, and the worker reads with the hold released.

use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::trace::TraceMask;

use crate::driver::Mar345Driver;
use crate::file_name::format_file_template;
use crate::protocol::{CMD_SHUTTER_CLOSE, CMD_SHUTTER_OPEN};
use crate::types::{MAR345_SOCKET_TIMEOUT, MAX_FILENAME_LEN, secs};

impl Mar345Driver {
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

    // --- server ------------------------------------------------------------

    /// C `writeServer`: flush stale input, write with `MAR345_SOCKET_TIMEOUT`,
    /// then publish the command on `ADStringToServer`.
    pub fn write_server(&mut self, output: &str) -> AsynStatus {
        let _ = self.server.flush(0);
        let status = match self
            .server
            .with_timeout(secs(MAR345_SOCKET_TIMEOUT))
            .write_octet(0, output.as_bytes())
        {
            Ok(_) => AsynStatus::Success,
            Err(e) => {
                // On pasynUserServer, so the server port's trace mask gates it.
                self.server.print(
                    0,
                    TraceMask::ERROR,
                    format_args!(
                        "mar345:writeServer, status={}, sent\n{output}\n",
                        e.status() as i32
                    ),
                );
                e.status()
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
