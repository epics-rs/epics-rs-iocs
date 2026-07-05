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
    read_reason: usize,
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
            .drv_user_create_blocking(MODBUS_DATA, 0)
            .map_err(|e| e.to_string())?
            .reason;
        let read_reason = handle
            .drv_user_create_blocking(MODBUS_READ, 0)
            .map_err(|e| e.to_string())?
            .reason;
        Ok(Self {
            handle,
            data_reason,
            read_reason,
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

    /// Force an immediate synchronous poll cycle on this (read) port (C
    /// `pasynInt32SyncIO->write(pasynUserForceRead_, 1, ...)`).
    pub fn force_read(&self) -> AsynResult<()> {
        self.handle.submit_blocking(
            RequestOp::Int32Write { value: 1 },
            self.user(self.read_reason, 0),
        )?;
        Ok(())
    }
}
