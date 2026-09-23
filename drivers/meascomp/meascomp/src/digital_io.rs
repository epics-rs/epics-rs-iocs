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

    /// `DPIOT_*` I/O type of port `index`.
    ///
    /// Only `DPIOT_IO` and `DPIOT_BITIO` ports accept `digital_config_port` /
    /// `digital_config_bit`; the rest reject them with `ERR_BAD_DEV_TYPE`.
    pub fn digital_port_io_type(&self, index: u32) -> Result<i64> {
        let mut value: i64 = 0;
        error::check(unsafe {
            ulDIOGetInfo(self.handle(), DIO_INFO_PORT_IO_TYPE, index, &mut value)
        })?;
        Ok(value)
    }

    /// Configure a single bit direction.
    pub fn digital_config_bit(&self, port: i32, bit: i32, direction: i32) -> Result<()> {
        error::check(unsafe { ulDConfigBit(self.handle(), port, bit, direction) })
    }
}

/// The bits a DIGITAL_OUTPUT write drives, with their levels.
///
/// C `writeUInt32Digital` writes a bit only if it is both in the record's
/// `mask` and an output in `direction` (1 = output): a bit configured as an
/// input is skipped, not driven -- libuldaq would reject it with
/// `ERR_WRONG_DIG_CONFIG`, and on an open-collector port driving it would
/// clamp whatever signal is wired to it.
pub fn output_bits(
    value: u32,
    mask: u32,
    direction: u32,
    num_bits: usize,
) -> impl Iterator<Item = (i32, bool)> {
    (0..num_bits)
        .filter(move |bit| mask & direction & (1 << bit) != 0)
        .map(move |bit| (bit as i32, value & (1 << bit) != 0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn input_bits_are_never_driven() {
        // Bits 0-3 In, 4-7 Out: a whole-port write touches only 4-7.
        let bits: Vec<_> = output_bits(0xA5, 0xFF, 0xF0, 8).collect();
        assert_eq!(bits, vec![(4, false), (5, true), (6, false), (7, true)]);
    }

    #[test]
    fn only_masked_output_bits_are_driven() {
        let bits: Vec<_> = output_bits(0xFF, 0x02, 0xFF, 8).collect();
        assert_eq!(bits, vec![(1, true)]);
    }

    #[test]
    fn a_port_with_no_outputs_drives_nothing() {
        assert_eq!(output_bits(0xFF, 0xFF, 0x00, 8).count(), 0);
    }
}
