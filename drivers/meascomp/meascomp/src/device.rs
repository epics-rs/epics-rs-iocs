use std::ffi::{CStr, c_char};
use std::fmt;

use uldaq_sys::*;

use crate::error::{self, MeasCompError, Result};

/// Maximum number of devices returned by discovery.
const MAX_DEVICES: usize = 64;

/// Why [`DaqDevice::connect`] failed: a uldaq call, or no device to open.
#[derive(Debug)]
pub enum ConnectError {
    Ul(MeasCompError),
    NoDevices,
    NotFound(String),
    NullHandle,
}

impl From<MeasCompError> for ConnectError {
    fn from(e: MeasCompError) -> Self {
        Self::Ul(e)
    }
}

impl fmt::Display for ConnectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ul(e) => e.fmt(f),
            Self::NoDevices => f.write_str("no MCC DAQ devices found"),
            Self::NotFound(id) => write!(f, "device with uniqueID '{id}' not found"),
            Self::NullHandle => f.write_str("ulCreateDaqDevice returned null handle"),
        }
    }
}

impl std::error::Error for ConnectError {}

/// RAII wrapper around a connected MCC DAQ device.
///
/// Disconnects and releases the device handle on drop.
pub struct DaqDevice {
    handle: DaqDeviceHandle,
    descriptor: DaqDeviceDescriptor,
}

impl DaqDevice {
    /// Discover all MCC DAQ devices and connect to the one matching `unique_id`.
    pub fn connect(unique_id: &str) -> std::result::Result<Self, ConnectError> {
        let mut descriptors = vec![DaqDeviceDescriptor::default(); MAX_DEVICES];
        let mut num_devs = MAX_DEVICES as u32;

        error::check(unsafe {
            ulGetDaqDeviceInventory(ANY_IFC, descriptors.as_mut_ptr(), &mut num_devs)
        })?;

        if num_devs == 0 {
            return Err(ConnectError::NoDevices);
        }

        let descriptor = find_descriptor(&descriptors[..num_devs as usize], unique_id)
            .cloned()
            .ok_or_else(|| ConnectError::NotFound(unique_id.to_owned()))?;

        let handle = unsafe { ulCreateDaqDevice(descriptor.clone()) };
        if handle == 0 {
            return Err(ConnectError::NullHandle);
        }

        error::check(unsafe { ulConnectDaqDevice(handle) })?;

        Ok(Self { handle, descriptor })
    }

    /// A device on uldaq's null handle, for tests that build a driver with
    /// no hardware: every call on it fails with ERR_BAD_DEV_HANDLE.
    #[doc(hidden)]
    pub fn unconnected() -> Self {
        Self {
            handle: 0,
            descriptor: DaqDeviceDescriptor::default(),
        }
    }

    /// Raw device handle for direct FFI calls.
    #[inline]
    pub fn handle(&self) -> DaqDeviceHandle {
        self.handle
    }

    pub fn product_name(&self) -> String {
        unsafe { CStr::from_ptr(self.descriptor.product_name.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    }

    pub fn product_id(&self) -> u32 {
        self.descriptor.product_id
    }

    pub fn unique_id(&self) -> String {
        unsafe { CStr::from_ptr(self.descriptor.unique_id.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    }

    pub fn firmware_version(&self) -> Result<String> {
        let mut buf = [0 as c_char; 256];
        let mut len = buf.len() as u32;
        error::check(unsafe {
            ulDevGetConfigStr(
                self.handle,
                DEV_CFG_VER_STR,
                DEV_VER_FW_MAIN,
                buf.as_mut_ptr(),
                &mut len,
            )
        })?;
        Ok(unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned())
    }

    pub fn ul_version() -> Result<String> {
        let mut buf = [0 as c_char; 256];
        let mut len = buf.len() as u32;
        error::check(unsafe { ulGetInfoStr(UL_INFO_VER_STR, 0, buf.as_mut_ptr(), &mut len) })?;
        Ok(unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned())
    }
}

impl Drop for DaqDevice {
    fn drop(&mut self) {
        unsafe {
            let _ = ulDisconnectDaqDevice(self.handle);
            let _ = ulReleaseDaqDevice(self.handle);
        }
    }
}

/// The inventory entry whose uniqueID is exactly `unique_id`. C
/// `measCompCreateDevice` (measCompDiscover.cpp:171-181) accepts only an
/// exact match, so an empty ID matches no board.
fn find_descriptor<'a>(
    descriptors: &'a [DaqDeviceDescriptor],
    unique_id: &str,
) -> Option<&'a DaqDeviceDescriptor> {
    descriptors.iter().find(|d| {
        let id = unsafe { CStr::from_ptr(d.unique_id.as_ptr()) }.to_string_lossy();
        id == unique_id
    })
}

/// Discover all connected MCC DAQ devices without connecting.
pub fn discover_devices() -> Result<Vec<(String, String, u32)>> {
    let mut descriptors = vec![DaqDeviceDescriptor::default(); MAX_DEVICES];
    let mut num_devs = MAX_DEVICES as u32;

    error::check(unsafe {
        ulGetDaqDeviceInventory(ANY_IFC, descriptors.as_mut_ptr(), &mut num_devs)
    })?;

    Ok(descriptors[..num_devs as usize]
        .iter()
        .map(|d| {
            let name = unsafe { CStr::from_ptr(d.product_name.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            let id = unsafe { CStr::from_ptr(d.unique_id.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            (name, id, d.product_id)
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor(id: &str) -> DaqDeviceDescriptor {
        let mut d = DaqDeviceDescriptor::default();
        for (slot, b) in d.unique_id.iter_mut().zip(id.bytes()) {
            *slot = b as c_char;
        }
        d
    }

    #[test]
    fn an_exact_unique_id_selects_its_device() {
        let devs = [descriptor("01DAB0FB"), descriptor("01DA523D")];
        let found = find_descriptor(&devs, "01DA523D").expect("present");
        assert!(std::ptr::eq(found, &devs[1]));
    }

    #[test]
    fn an_empty_unique_id_selects_no_device() {
        let devs = [descriptor("01DAB0FB"), descriptor("01DA523D")];
        assert!(find_descriptor(&devs, "").is_none());
    }

    #[test]
    fn a_prefix_is_not_a_match() {
        let devs = [descriptor("01DAB0FB")];
        assert!(find_descriptor(&devs, "01DA").is_none());
    }
}
