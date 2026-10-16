//! Look up the octet port a device hangs off and wrap it in a blocking
//! [`SyncIOHandle`].
//!
//! The C device support did this per record with
//! `pasynManager->connectDevice(pasynUser, port, 0)` +
//! `findInterface(asynOctetType)`; here the port driver does it once, at
//! configure time.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

use crate::worker::{OCTET_ADDR, Transport};

/// Connect to an octet port created earlier in `st.cmd` by
/// `drvAsynSerialPortConfigure` or `drvAsynIPPortConfigure`.
pub fn connect_octet(port_name: &str, timeout: Duration) -> Result<Transport, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!(
            "octet port '{port_name}' not found (call drvAsynSerialPortConfigure or \
             drvAsynIPPortConfigure first)"
        )
    })?;
    Ok(Transport::new(
        SyncIOHandle::from_handle(port.handle.clone(), OCTET_ADDR, timeout),
        port.handle.trace().clone(),
        port_name,
    ))
}
