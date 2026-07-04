use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

/// Scalar arguments of C `ulAInScan`, grouped for
/// [`DaqDevice::analog_in_scan`]. The in/out `rate` and the data buffer stay
/// separate parameters.
#[derive(Clone, Copy, Debug)]
pub struct AInScanConfig {
    pub low_chan: i32,
    pub high_chan: i32,
    pub input_mode: i32,
    pub range: i32,
    pub samples_per_chan: i32,
    pub options: i32,
    pub flags: i32,
}

impl DaqDevice {
    /// Read a single analog input channel (voltage).
    pub fn analog_in(&self, channel: i32, input_mode: i32, range: i32, flags: i32) -> Result<f64> {
        let mut data: f64 = 0.0;
        error::check(unsafe {
            ulAIn(self.handle(), channel, input_mode, range, flags, &mut data)
        })?;
        Ok(data)
    }

    /// Read a single temperature input channel (thermocouple/RTD).
    pub fn temperature_in(&self, channel: i32, scale: i32, flags: i32) -> Result<f64> {
        let mut data: f64 = 0.0;
        error::check(unsafe { ulTIn(self.handle(), channel, scale, flags, &mut data) })?;
        Ok(data)
    }

    /// Set an analog input configuration item (integer value).
    pub fn ai_set_config(&self, config_item: i32, index: u32, value: i64) -> Result<()> {
        error::check(unsafe { ulAISetConfig(self.handle(), config_item, index, value) })
    }

    /// Set an analog input configuration item (double value, e.g. data rate).
    pub fn ai_set_config_dbl(&self, config_item: i32, index: u32, value: f64) -> Result<()> {
        error::check(unsafe { ulAISetConfigDbl(self.handle(), config_item, index, value) })
    }

    /// Load an analog input queue for scanning.
    pub fn analog_in_load_queue(&self, queue: &[AiQueueElement]) -> Result<()> {
        error::check(unsafe { ulAInLoadQueue(self.handle(), queue.as_ptr(), queue.len() as u32) })
    }

    /// Start an analog input scan. `rate` is in/out: the driver may adjust it
    /// to the nearest achievable value.
    pub fn analog_in_scan(
        &self,
        config: &AInScanConfig,
        rate: &mut f64,
        data: &mut [f64],
    ) -> Result<()> {
        error::check(unsafe {
            ulAInScan(
                self.handle(),
                config.low_chan,
                config.high_chan,
                config.input_mode,
                config.range,
                config.samples_per_chan,
                rate,
                config.options,
                config.flags,
                data.as_mut_ptr(),
            )
        })
    }

    /// Get analog input scan status.
    pub fn analog_in_scan_status(&self) -> Result<(i32, TransferStatus)> {
        let mut status: i32 = 0;
        let mut xfer = TransferStatus::default();
        error::check(unsafe { ulAInScanStatus(self.handle(), &mut status, &mut xfer) })?;
        Ok((status, xfer))
    }

    /// Stop an analog input scan.
    pub fn analog_in_scan_stop(&self) -> Result<()> {
        error::check(unsafe { ulAInScanStop(self.handle()) })
    }

    /// Set analog input trigger.
    pub fn analog_in_set_trigger(
        &self,
        trig_type: i32,
        trig_chan: i32,
        level: f64,
        variance: f64,
        retrigger_count: u32,
    ) -> Result<()> {
        error::check(unsafe {
            ulAInSetTrigger(
                self.handle(),
                trig_type,
                trig_chan,
                level,
                variance,
                retrigger_count,
            )
        })
    }

    /// Query AI info (e.g. number of channels, resolution).
    pub fn ai_get_info(&self, info_item: i32, index: u32) -> Result<i64> {
        let mut value: i64 = 0;
        error::check(unsafe { ulAIGetInfo(self.handle(), info_item, index, &mut value) })?;
        Ok(value)
    }
}
