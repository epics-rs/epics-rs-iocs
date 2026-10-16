//! Octet-port connect helper for the `initScaler974` iocsh command. Mirrors
//! `pasynOctetSyncIO->connect(serialPort, serialAddr, &this->pasynUserScaler,
//! NULL)` (`drvScaler974.cpp:72`) — the C driver never calls
//! `pasynOctetSetInputEos`/`pasynOctetSetOutputEos` itself, so this doesn't
//! either. EOS is expected to already be configured on the underlying
//! octet port (e.g. via `asynOctetSetInputEos`/`OutputEos` in `st.cmd`,
//! consulting the Ortec 974 hardware manual for the actual terminator)
//! before `initScaler974` runs.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::services::PortServices;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::{TraceManager, TraceMask};

/// C `pasynUserScaler`, plus the trace manager `Scaler974`'s own
/// `pasynUserSelf` prints through.
pub struct OctetLink {
    pub(crate) sync: SyncIOHandle,
    trace: Arc<TraceManager>,
}

impl OctetLink {
    /// C `asynPrint(this->pasynUserSelf, mask, ...)`. The scaler port is no
    /// asyn port here, so the line takes the global trace settings.
    pub fn print_self(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        self.trace.print(None, None, 0, mask, args);
    }
}

/// Connect an [`OctetLink`] to a pre-configured octet port (serial or
/// GPIB) by name and address. `addr` is C `drvScaler974`'s `serialAddr`
/// argument, passed straight through. Like C's `pasynOctetSyncIO->connect`
/// it hands back a user even when the port is missing, beside the
/// `connectDevice` refusal. Called twice per driver (see
/// `driver::Scaler974Driver::create`) — once for the synchronous command
/// path, once for the background `SHOW_COUNTS` poll thread — since
/// `SyncIOHandle` is not `Clone`; both handles wrap the same underlying
/// `PortHandle`, whose requests are serialized by the port's actor.
pub fn connect_octet(port_name: &str, addr: i32, timeout: Duration) -> (OctetLink, AsynResult<()>) {
    let (sync, status) = SyncIOHandle::connect_port(port_name, addr, timeout);
    let trace = match sync.port() {
        Ok(port) => port.trace().clone(),
        Err(_) => PortServices::global().trace().clone(),
    };
    (OctetLink { sync, trace }, status)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_missing_port_hands_back_a_user_that_refuses_transfers() {
        let (link, status) = connect_octet("SCALER974_NONE", 0, Duration::ZERO);
        assert!(
            status
                .unwrap_err()
                .message()
                .contains("port SCALER974_NONE not found")
        );
        assert!(link.sync.write_octet(0, b"SHOW_COUNTS").is_err());
    }
}
