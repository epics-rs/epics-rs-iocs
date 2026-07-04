use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

/// Scalar arguments of C `ulAOutScan`, grouped for
/// [`DaqDevice::analog_out_scan`]. The in/out `rate` and the data buffer stay
/// separate parameters.
#[derive(Clone, Copy, Debug)]
pub struct AOutScanConfig {
    pub low_chan: i32,
    pub high_chan: i32,
    pub range: i32,
    pub samples_per_chan: i32,
    pub options: i32,
    pub flags: i32,
}

impl DaqDevice {
    /// Write a single analog output channel.
    pub fn analog_out(&self, channel: i32, range: i32, flags: i32, value: f64) -> Result<()> {
        error::check(unsafe { ulAOut(self.handle(), channel, range, flags, value) })
    }

    /// Write multiple analog outputs simultaneously.
    pub fn analog_out_array(
        &self,
        low_chan: i32,
        high_chan: i32,
        ranges: &[i32],
        flags: i32,
        data: &mut [f64],
    ) -> Result<()> {
        error::check(unsafe {
            ulAOutArray(
                self.handle(),
                low_chan,
                high_chan,
                ranges.as_ptr(),
                flags,
                data.as_mut_ptr(),
            )
        })
    }

    /// Start an analog output scan (waveform generation). `rate` is in/out:
    /// the driver may adjust it to the nearest achievable value.
    pub fn analog_out_scan(
        &self,
        config: &AOutScanConfig,
        rate: &mut f64,
        data: &mut [f64],
    ) -> Result<()> {
        error::check(unsafe {
            ulAOutScan(
                self.handle(),
                config.low_chan,
                config.high_chan,
                config.range,
                config.samples_per_chan,
                rate,
                config.options,
                config.flags,
                data.as_mut_ptr(),
            )
        })
    }

    /// Get analog output scan status.
    pub fn analog_out_scan_status(&self) -> Result<(i32, TransferStatus)> {
        let mut status: i32 = 0;
        let mut xfer = TransferStatus::default();
        error::check(unsafe { ulAOutScanStatus(self.handle(), &mut status, &mut xfer) })?;
        Ok((status, xfer))
    }

    /// Stop an analog output scan.
    pub fn analog_out_scan_stop(&self) -> Result<()> {
        error::check(unsafe { ulAOutScanStop(self.handle()) })
    }

    /// Set analog output configuration (e.g. sync mode).
    pub fn ao_set_config(&self, config_item: i32, index: u32, value: i64) -> Result<()> {
        error::check(unsafe { ulAOSetConfig(self.handle(), config_item, index, value) })
    }

    /// Query AO info (e.g. number of channels, resolution).
    pub fn ao_get_info(&self, info_item: i32, index: u32) -> Result<i64> {
        let mut value: i64 = 0;
        error::check(unsafe { ulAOGetInfo(self.handle(), info_item, index, &mut value) })?;
        Ok(value)
    }
}
