//! In-process connection to a modbus-rs port's register space — the
//! analogue of `pasynInt32SyncIO`/`pasynInt32ArraySyncIO`
//! `connect(port, addr)` + `read`/`write`, used because the AMCI controllers
//! are Modbus register devices rather than asyn-octet ones.
//!
//! A modbus-rs port is just another asyn port: `drvModbusAsynConfigure`
//! registers it in the same global port registry every asyn port uses
//! (`asyn_rs::asyn_record::register_port`), keyed by the port name given in
//! `st.cmd`. [`ModbusRegs::connect`] looks it up there and resolves the two
//! drvInfo strings the driver needs against it: `MODBUS_DATA` (plain register
//! access — C's `pasynInt32SyncIO->connect(port, addr, &user, NULL)`, which
//! skips `drvUserCreate` for a `NULL` drvInfo and so falls on the port's
//! default reason; explicit resolution is used here instead of relying on
//! that default, since modbus-rs's parameter creation order does not
//! reproduce it) and `MODBUS_READ` (the force-read trigger — C's
//! `pasynUserForceRead_`, connected with drvInfo `"MODBUS_READ"`).

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;

/// C `MODBUS_DATA_STRING` — the modbus-rs param every register read/write
/// resolves through.
const MODBUS_DATA: &str = "MODBUS_DATA";

/// C `"MODBUS_READ"` — writing 1 triggers an immediate synchronous poll cycle.
const MODBUS_READ: &str = "MODBUS_READ";

/// A connected handle onto one modbus-rs port's register space.
pub struct ModbusRegs {
    handle: PortHandle,
    data_reason: usize,
    timeout: Duration,
}

impl ModbusRegs {
    /// Connect to a modbus-rs port previously created by
    /// `drvModbusAsynConfigure`, by name.
    pub fn connect(port_name: &str, timeout: Duration) -> Result<Self, String> {
        let entry = get_port(port_name).ok_or_else(|| {
            format!("modbus port '{port_name}' not found (call drvModbusAsynConfigure first)")
        })?;
        let handle = entry.handle;
        let data_reason = handle
            .drv_user_create_blocking(&DrvUserRequest::new(MODBUS_DATA, 0))
            .map_err(|e| e.to_string())?
            .reason;
        Ok(Self {
            handle,
            data_reason,
            timeout,
        })
    }

    fn user(&self, reason: usize, addr: i32) -> AsynUser {
        AsynUser::new(reason)
            .with_addr(addr)
            .with_timeout(self.timeout)
    }

    /// Read one register (C `readReg16` / `pasynInt32SyncIO->read`).
    pub fn read16(&self, addr: i32) -> AsynResult<i32> {
        let result = self
            .handle
            .submit_blocking(RequestOp::Int32Read, self.user(self.data_reason, addr))?;
        result.int_val.ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "int32 read returned no value".into(),
        })
    }

    /// Write one register (C `writeReg16` / `pasynInt32SyncIO->write`).
    pub fn write16(&self, addr: i32, value: i32) -> AsynResult<()> {
        self.handle.submit_blocking(
            RequestOp::Int32Write { value },
            self.user(self.data_reason, addr),
        )?;
        Ok(())
    }

    /// Write an array of registers atomically (C `writeReg32Array` /
    /// `pasynInt32ArraySyncIO->write`); the port must be configured with a
    /// write-multiple-registers Modbus function.
    pub fn write_array(&self, addr: i32, data: Vec<i32>) -> AsynResult<()> {
        self.handle.submit_blocking(
            RequestOp::Int32ArrayWrite { data },
            self.user(self.data_reason, addr),
        )?;
        Ok(())
    }
}

/// C `pasynUserForceRead_`: one per axis, a `pasynInt32SyncIO` user on the
/// input port with drvInfo `"MODBUS_READ"`. As C's `connect`, a missing port
/// leaves it unconnected and a refused drvInfo leaves it on reason 0.
pub struct ForceRead {
    handle: Option<PortHandle>,
    reason: usize,
    addr: i32,
    timeout: Duration,
    connected: bool,
}

impl ForceRead {
    pub fn connect(port_name: &str, addr: i32, timeout: Duration) -> Self {
        let handle = get_port(port_name).map(|entry| entry.handle);
        let reason = handle.as_ref().and_then(|h| {
            h.drv_user_create_blocking(&DrvUserRequest::new(MODBUS_READ, addr))
                .ok()
                .map(|r| r.reason)
        });
        Self {
            connected: reason.is_some(),
            handle,
            reason: reason.unwrap_or(0),
            addr,
            timeout,
        }
    }

    /// Whether C's `pasynInt32SyncIO->connect` would have returned success.
    pub fn connected(&self) -> bool {
        self.connected
    }

    /// C `pasynUserForceRead_->reason`.
    pub fn reason(&self) -> usize {
        self.reason
    }

    /// Force an immediate synchronous poll cycle (C
    /// `pasynInt32SyncIO->write(pasynUserForceRead_, 1, ...)`).
    pub fn write(&self) -> AsynResult<()> {
        let handle = self.handle.as_ref().ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "asynManager::queueLockPort not connected".into(),
        })?;
        handle.submit_blocking(
            RequestOp::Int32Write { value: 1 },
            AsynUser::new(self.reason)
                .with_addr(self.addr)
                .with_timeout(self.timeout),
        )?;
        Ok(())
    }
}

/// A stand-in for a modbus-rs port that records every force-read it takes.
#[cfg(test)]
pub(crate) mod fake {
    use std::sync::{Arc, Mutex};

    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::param::ParamType;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;

    use super::{MODBUS_DATA, MODBUS_READ};

    /// `(reason, addr)` of each `MODBUS_READ` write.
    pub type Taken = Arc<Mutex<Vec<(usize, i32)>>>;

    struct Modbus {
        base: PortDriverBase,
        read_reason: usize,
        taken: Taken,
    }

    impl PortDriver for Modbus {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn read_int32(&mut self, _user: &AsynUser) -> AsynResult<i32> {
            Ok(0)
        }
        fn write_int32(&mut self, user: &mut AsynUser, _value: i32) -> AsynResult<()> {
            if user.reason == self.read_reason {
                self.taken.lock().unwrap().push((user.reason, user.addr));
            }
            Ok(())
        }
        fn write_int32_array(&mut self, _user: &AsynUser, _data: &[i32]) -> AsynResult<()> {
            Ok(())
        }
    }

    /// Publish port `name`; returns its `MODBUS_READ` reason and the record.
    pub fn publish(name: &str) -> (usize, Taken) {
        let flags = PortFlags {
            multi_device: true,
            ..PortFlags::default()
        };
        let mut base = PortDriverBase::new(name, 100, flags);
        base.create_param(MODBUS_DATA, ParamType::Int32).unwrap();
        let read_reason = base.create_param(MODBUS_READ, ParamType::Int32).unwrap();
        let taken = Taken::default();
        let driver = Modbus {
            base,
            read_reason,
            taken: taken.clone(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        std::mem::forget(rt);
        (read_reason, taken)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_force_read_writes_modbus_read_at_its_address() {
        let (reason, taken) = fake::publish("amciForceReadIn");
        let fr = ForceRead::connect("amciForceReadIn", 0, Duration::from_secs(1));
        assert!(fr.connected());
        assert_eq!(fr.reason(), reason);
        fr.write().unwrap();
        assert_eq!(*taken.lock().unwrap(), [(reason, 0)]);
    }

    #[test]
    fn a_missing_port_leaves_the_force_read_unconnected_on_reason_0() {
        let fr = ForceRead::connect("amciForceReadMissing", 0, Duration::from_secs(1));
        assert!(!fr.connected());
        assert_eq!(fr.reason(), 0);
        assert!(fr.write().is_err());
    }
}
