use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

/// Per-counter arguments of C `ulCConfigScan`, grouped for
/// [`DaqDevice::counter_config_scan`] (everything but the counter number).
#[derive(Clone, Copy, Debug)]
pub struct CounterScanConfig {
    pub measurement_type: i32,
    pub measurement_mode: i32,
    pub edge_detection: i32,
    pub tick_size: i32,
    pub debounce_mode: i32,
    pub debounce_time: i32,
    pub flags: i32,
}

/// Scalar arguments of C `ulCInScan`, grouped for
/// [`DaqDevice::counter_in_scan`]. The in/out `rate` and the data buffer stay
/// separate parameters.
#[derive(Clone, Copy, Debug)]
pub struct CInScanConfig {
    pub low_counter: i32,
    pub high_counter: i32,
    pub samples_per_counter: i32,
    pub options: i32,
    pub flags: i32,
}

impl DaqDevice {
    /// Read a single counter value.
    pub fn counter_in(&self, counter: i32) -> Result<u64> {
        let mut data: u64 = 0;
        error::check(unsafe { ulCIn(self.handle(), counter, &mut data) })?;
        Ok(data)
    }

    /// Load a counter register (LOAD, OUTPUT_VAL0, OUTPUT_VAL1, MAX_LIMIT, etc.).
    pub fn counter_load(&self, counter: i32, register_type: i32, value: u64) -> Result<()> {
        error::check(unsafe { ulCLoad(self.handle(), counter, register_type, value) })
    }

    /// Clear a counter to zero.
    pub fn counter_clear(&self, counter: i32) -> Result<()> {
        error::check(unsafe { ulCClear(self.handle(), counter) })
    }

    /// Configure a counter for scanning.
    pub fn counter_config_scan(&self, counter: i32, config: &CounterScanConfig) -> Result<()> {
        error::check(unsafe {
            ulCConfigScan(
                self.handle(),
                counter,
                config.measurement_type,
                config.measurement_mode,
                config.edge_detection,
                config.tick_size,
                config.debounce_mode,
                config.debounce_time,
                config.flags,
            )
        })
    }

    /// Start a counter input scan. `rate` is in/out: the driver may adjust it
    /// to the nearest achievable value.
    pub fn counter_in_scan(
        &self,
        config: &CInScanConfig,
        rate: &mut f64,
        data: &mut [u64],
    ) -> Result<()> {
        error::check(unsafe {
            ulCInScan(
                self.handle(),
                config.low_counter,
                config.high_counter,
                config.samples_per_counter,
                rate,
                config.options,
                config.flags,
                data.as_mut_ptr(),
            )
        })
    }

    /// Get counter scan status.
    pub fn counter_in_scan_status(&self) -> Result<(i32, TransferStatus)> {
        let mut status: i32 = 0;
        let mut xfer = TransferStatus::default();
        error::check(unsafe { ulCInScanStatus(self.handle(), &mut status, &mut xfer) })?;
        Ok((status, xfer))
    }

    /// Stop a counter input scan.
    pub fn counter_in_scan_stop(&self) -> Result<()> {
        error::check(unsafe { ulCInScanStop(self.handle()) })
    }

    /// Start a DAQ input scan (multi-function, used for MCS).
    pub fn daq_in_scan(
        &self,
        chan_descriptors: &[DaqInChanDescriptor],
        samples_per_chan: i32,
        rate: &mut f64,
        options: i32,
        flags: i32,
        data: &mut [f64],
    ) -> Result<()> {
        error::check(unsafe {
            ulDaqInScan(
                self.handle(),
                chan_descriptors.as_ptr(),
                chan_descriptors.len() as i32,
                samples_per_chan,
                rate,
                options,
                flags,
                data.as_mut_ptr(),
            )
        })
    }

    /// Get DAQ input scan status.
    pub fn daq_in_scan_status(&self) -> Result<(i32, TransferStatus)> {
        let mut status: i32 = 0;
        let mut xfer = TransferStatus::default();
        error::check(unsafe { ulDaqInScanStatus(self.handle(), &mut status, &mut xfer) })?;
        Ok((status, xfer))
    }

    /// Stop a DAQ input scan.
    pub fn daq_in_scan_stop(&self) -> Result<()> {
        error::check(unsafe { ulDaqInScanStop(self.handle()) })
    }

    /// Set DAQ input trigger.
    pub fn daq_in_set_trigger(
        &self,
        trig_type: i32,
        trig_chan: DaqInChanDescriptor,
        level: f64,
        variance: f64,
        retrigger_count: u32,
    ) -> Result<()> {
        error::check(unsafe {
            ulDaqInSetTrigger(
                self.handle(),
                trig_type,
                trig_chan,
                level,
                variance,
                retrigger_count,
            )
        })
    }
}
