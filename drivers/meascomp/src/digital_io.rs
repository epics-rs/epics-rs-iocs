use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

impl DaqDevice {
    /// Read all bits of a digital port.
    pub fn digital_in(&self, port: i32) -> Result<u64> {
        let mut data: u64 = 0;
        error::check(unsafe { ulDIn(self.handle(), port, &mut data) })?;
        Ok(data)
    }

    /// Write all bits of a digital port.
    pub fn digital_out(&self, port: i32, data: u64) -> Result<()> {
        error::check(unsafe { ulDOut(self.handle(), port, data) })
    }

    /// Write a single digital bit.
    pub fn digital_bit_out(&self, port: i32, bit: i32, value: bool) -> Result<()> {
        error::check(unsafe { ulDBitOut(self.handle(), port, bit, value as u32) })
    }

    /// Configure an entire port direction.
    pub fn digital_config_port(&self, port: i32, direction: i32) -> Result<()> {
        error::check(unsafe { ulDConfigPort(self.handle(), port, direction) })
    }

    /// Configure a single bit direction.
    pub fn digital_config_bit(&self, port: i32, bit: i32, direction: i32) -> Result<()> {
        error::check(unsafe { ulDConfigBit(self.handle(), port, bit, direction) })
    }
}
