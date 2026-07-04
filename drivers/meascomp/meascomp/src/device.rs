use std::ffi::CStr;

use uldaq_sys::*;

use crate::error::{self, Result};

/// Maximum number of devices returned by discovery.
const MAX_DEVICES: usize = 64;

/// RAII wrapper around a connected MCC DAQ device.
///
/// Disconnects and releases the device handle on drop.
pub struct DaqDevice {
    handle: DaqDeviceHandle,
    descriptor: DaqDeviceDescriptor,
}

impl DaqDevice {
    /// Discover all MCC DAQ devices and connect to the one matching `unique_id`.
    ///
    /// If `unique_id` is empty, connects to the first device found.
    pub fn connect(unique_id: &str) -> Result<Self> {
        let mut descriptors = vec![DaqDeviceDescriptor::default(); MAX_DEVICES];
        let mut num_devs = MAX_DEVICES as u32;

        error::check(unsafe {
            ulGetDaqDeviceInventory(ANY_IFC, descriptors.as_mut_ptr(), &mut num_devs)
        })?;

        if num_devs == 0 {
            return Err(error::MeasCompError {
                code: ERR_DEV_NOT_FOUND,
                message: "no MCC DAQ devices found".into(),
            });
        }

        let descriptor = if unique_id.is_empty() {
            descriptors[0].clone()
        } else {
            descriptors[..num_devs as usize]
                .iter()
                .find(|d| {
                    let id = unsafe { CStr::from_ptr(d.unique_id.as_ptr()) }.to_string_lossy();
                    id == unique_id
                })
                .cloned()
                .ok_or_else(|| error::MeasCompError {
                    code: ERR_DEV_NOT_FOUND,
                    message: format!("device with uniqueID '{unique_id}' not found"),
                })?
        };

        let handle = unsafe { ulCreateDaqDevice(descriptor.clone()) };
        if handle == 0 {
            return Err(error::MeasCompError {
                code: ERR_DEV_NOT_FOUND,
                message: "ulCreateDaqDevice returned null handle".into(),
            });
        }

        error::check(unsafe { ulConnectDaqDevice(handle) })?;

        Ok(Self { handle, descriptor })
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
        let mut buf = [0i8; 256];
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
        let mut buf = [0i8; 256];
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
