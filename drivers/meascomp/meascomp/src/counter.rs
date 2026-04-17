use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

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
    pub fn counter_config_scan(
        &self,
        counter: i32,
        measurement_type: i32,
        measurement_mode: i32,
        edge_detection: i32,
        tick_size: i32,
        debounce_mode: i32,
        debounce_time: i32,
        flags: i32,
    ) -> Result<()> {
        error::check(unsafe {
            ulCConfigScan(
                self.handle(),
                counter,
                measurement_type,
                measurement_mode,
                edge_detection,
                tick_size,
                debounce_mode,
                debounce_time,
                flags,
            )
        })
    }

    /// Start a counter input scan.
    pub fn counter_in_scan(
        &self,
        low_counter: i32,
        high_counter: i32,
        samples_per_counter: i32,
        rate: &mut f64,
        options: i32,
        flags: i32,
        data: &mut [u64],
    ) -> Result<()> {
        error::check(unsafe {
            ulCInScan(
                self.handle(),
                low_counter,
                high_counter,
                samples_per_counter,
                rate,
                options,
                flags,
                data.as_mut_ptr(),
            )
        })
    }

    /// Get counter scan status.
    pub fn counter_in_scan_status(&self) -> Result<(i32, TransferStatus)> {
        let mut status: i32 = 0;
        let mut xfer = TransferStatus::default();
        error::check(unsafe {
            ulCInScanStatus(self.handle(), &mut status, &mut xfer)
        })?;
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
        error::check(unsafe {
            ulDaqInScanStatus(self.handle(), &mut status, &mut xfer)
        })?;
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
