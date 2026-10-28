//! In-process connection to a modbus-rs port's register space — the
//! analogue of `pasynInt32SyncIO`/`pasynInt32ArraySyncIO`
//! `connect(port, addr)` + `read`/`write`, used because the AMCI controllers
//! are Modbus register devices rather than asyn-octet ones.
//!
//! A modbus-rs port is just another asyn port: `drvModbusAsynConfigure`
//! registers it in the same global port registry every asyn port uses
//! (`asyn_rs::asyn_record::register_port`), keyed by the port name given in
//! `st.cmd`. [`ModbusRegs::connect`] connects to it there and resolves the two
//! drvInfo strings the driver needs against it: `MODBUS_DATA` (plain register
//! access — C's `pasynInt32SyncIO->connect(port, addr, &user, NULL)`, which
//! skips `drvUserCreate` for a `NULL` drvInfo and so falls on the port's
//! default reason; explicit resolution is used here instead of relying on
//! that default, since modbus-rs's parameter creation order does not
//! reproduce it) and `MODBUS_READ` (the force-read trigger — C's
//! `pasynUserForceRead_`, connected with drvInfo `"MODBUS_READ"`). A missing
//! port keeps C's unconnected user, which refuses every transfer.

use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// C `MODBUS_DATA_STRING` — the modbus-rs param every register read/write
/// resolves through.
const MODBUS_DATA: &str = "MODBUS_DATA";

/// C `"MODBUS_READ"` — writing 1 triggers an immediate synchronous poll cycle.
const MODBUS_READ: &str = "MODBUS_READ";

/// A connected handle onto one modbus-rs port's register space.
pub struct ModbusRegs {
    handle: SyncIOHandle,
    data_reason: usize,
    timeout: Duration,
}

impl ModbusRegs {
    /// Connect to a modbus-rs port previously created by
    /// `drvModbusAsynConfigure`, by name. As C's SyncIO connect, the user is
    /// kept when the port is missing and the connect status is returned beside
    /// it; C's NULL drvInfo cannot be refused, so a refused `MODBUS_DATA`
    /// leaves reason 0 as C's default.
    pub fn connect(port_name: &str, timeout: Duration) -> (Self, AsynResult<()>) {
        let (handle, connected) =
            SyncIOHandle::connect_port(port_name, 0, timeout, InterfaceType::Int32);
        let data_reason = handle
            .port()
            .and_then(|port| port.drv_user_create_blocking(&DrvUserRequest::new(MODBUS_DATA, 0)))
            .map_or(0, |r| r.reason);
        let regs = Self {
            handle,
            data_reason,
            timeout,
        };
        (regs, connected)
    }

    /// C's `pasynInt32SyncIO` user of register `addr`
    /// (`pasynUserInReg_`/`pasynUserOutReg_`).
    fn reg(&self, addr: i32) -> AsynResult<SyncIOHandle> {
        let port = self.handle.port()?.clone();
        Ok(SyncIOHandle::from_handle(port, addr, self.timeout))
    }

    /// Read one register (C `readReg16` / `pasynInt32SyncIO->read`).
    pub fn read16(&self, addr: i32) -> AsynResult<i32> {
        self.reg(addr)?.read_int32(self.data_reason)
    }

    /// Write one register (C `writeReg16` / `pasynInt32SyncIO->write`).
    pub fn write16(&self, addr: i32, value: i32) -> AsynResult<()> {
        self.reg(addr)?.write_int32(self.data_reason, value)
    }

    /// Write an array of registers atomically (C `writeReg32Array` /
    /// `pasynInt32ArraySyncIO->write`); the port must be configured with a
    /// write-multiple-registers Modbus function.
    pub fn write_array(&self, addr: i32, data: Vec<i32>) -> AsynResult<()> {
        self.reg(addr)?.write_int32_array(self.data_reason, &data)
    }
}

/// C `pasynUserForceRead_`: one per axis, a `pasynInt32SyncIO` user on the
/// input port with drvInfo `"MODBUS_READ"`. As C's `connect`, a missing port
/// leaves it unconnected and a refused drvInfo leaves it on reason 0.
pub struct ForceRead {
    handle: SyncIOHandle,
    reason: usize,
    connected: bool,
}

impl ForceRead {
    pub fn connect(port_name: &str, addr: i32, timeout: Duration) -> Self {
        let (handle, connected) =
            SyncIOHandle::connect_port(port_name, addr, timeout, InterfaceType::Int32);
        let reason = connected.and_then(|()| handle.drv_user_create(MODBUS_READ));
        Self {
            connected: reason.is_ok(),
            reason: reason.unwrap_or(0),
            handle,
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
        self.handle.write_int32(self.reason, 1)
    }
}

/// A stand-in for a modbus-rs port that records every force-read it takes.
#[cfg(test)]
pub(crate) mod fake {
    use std::sync::{Arc, Mutex};

    use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
    use epics_rs::asyn::param::ParamType;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;

    use super::{MODBUS_DATA, MODBUS_READ};

    /// `(reason, addr)` of each `MODBUS_READ` write.
    pub type Taken = Arc<Mutex<Vec<(usize, i32)>>>;

    /// `(addr, registers)` of each `MODBUS_DATA` write, by port.
    static WRITTEN: Mutex<Vec<(String, i32, Vec<i32>)>> = Mutex::new(Vec::new());

    /// The `MODBUS_DATA` writes to port `name` since the last call.
    pub fn written(name: &str) -> Vec<(i32, Vec<i32>)> {
        let mut all = WRITTEN.lock().unwrap();
        let (mine, rest) = std::mem::take(&mut *all)
            .into_iter()
            .partition(|(port, ..)| port == name);
        *all = rest;
        mine.into_iter()
            .map(|(_, addr, data)| (addr, data))
            .collect()
    }

    /// `(port, registers)` of each `MODBUS_DATA` array write [`refuse`]
    /// makes the port refuse once it is recorded.
    static REFUSED: Mutex<Vec<(String, Vec<i32>)>> = Mutex::new(Vec::new());

    /// Make port `name` refuse an array write of exactly `data`.
    pub fn refuse(name: &str, data: &[i32]) {
        REFUSED
            .lock()
            .unwrap()
            .push((name.to_string(), data.to_vec()));
    }

    /// `addr` of each `MODBUS_READ` write, by port.
    static FORCED: Mutex<Vec<(String, i32)>> = Mutex::new(Vec::new());

    /// The `MODBUS_READ` writes to port `name` since the last call.
    pub fn forced(name: &str) -> Vec<i32> {
        let mut all = FORCED.lock().unwrap();
        let (mine, rest) = std::mem::take(&mut *all)
            .into_iter()
            .partition(|(port, _)| port == name);
        *all = rest;
        mine.into_iter().map(|(_, addr)| addr).collect()
    }

    /// `(port, register, value)`: what [`set_read`] makes a register read;
    /// `None` refuses the read. The latest entry wins.
    static READS: Mutex<Vec<(String, i32, Option<i32>)>> = Mutex::new(Vec::new());

    /// Make register `addr` of port `name` read `value`, or refuse on `None`.
    pub fn set_read(name: &str, addr: i32, value: Option<i32>) {
        READS.lock().unwrap().push((name.to_string(), addr, value));
    }

    struct Modbus {
        base: PortDriverBase,
        read_reason: usize,
        taken: Taken,
    }

    impl Modbus {
        fn record(&self, addr: i32, data: Vec<i32>) {
            let port = self.base.port_name.to_string();
            WRITTEN.lock().unwrap().push((port, addr, data));
        }
    }

    impl PortDriver for Modbus {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        /// Register `addr` reads `addr` unless [`set_read`] says otherwise.
        fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
            let port = &*self.base.port_name;
            let set = READS
                .lock()
                .unwrap()
                .iter()
                .rev()
                .find(|(p, addr, _)| p == port && *addr == user.addr)
                .map(|(.., value)| *value);
            match set {
                None => Ok(user.addr),
                Some(Some(value)) => Ok(value),
                Some(None) => Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: "refused".into(),
                }),
            }
        }
        fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
            if user.reason == self.read_reason {
                self.taken.lock().unwrap().push((user.reason, user.addr));
                let port = self.base.port_name.to_string();
                FORCED.lock().unwrap().push((port, user.addr));
            } else {
                self.record(user.addr, vec![value]);
            }
            Ok(())
        }
        fn write_int32_array(&mut self, user: &AsynUser, data: &[i32]) -> AsynResult<()> {
            self.record(user.addr, data.to_vec());
            let port = &*self.base.port_name;
            if REFUSED
                .lock()
                .unwrap()
                .iter()
                .any(|(p, d)| p == port && d == data)
            {
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: "refused".into(),
                });
            }
            Ok(())
        }
    }

    /// Publish port `name`; returns its `MODBUS_READ` reason and the record.
    pub fn publish(name: &str) -> (usize, Taken) {
        let flags = PortFlags {
            multi_device: true,
            ..PortFlags::default()
        };
        let mut base = PortDriverBase::new(
            name,
            100,
            flags,
            InterfaceMask::INT32
                | InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::INT64
                | InterfaceMask::FLOAT64
                | InterfaceMask::INT32_ARRAY
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::OCTET
                | InterfaceMask::DRV_USER,
            InterfaceMask::INT32
                | InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::INT64
                | InterfaceMask::FLOAT64
                | InterfaceMask::INT32_ARRAY
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::OCTET,
        );
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
    use epics_rs::asyn::asyn_record::get_port;

    #[test]
    fn a_force_read_writes_modbus_read_at_its_address() {
        let (reason, taken) = fake::publish("amciForceReadIn");
        let fr = ForceRead::connect("amciForceReadIn", 0, Duration::from_secs(1));
        assert!(fr.connected());
        assert_eq!(fr.reason(), reason);
        fr.write().unwrap();
        assert_eq!(*taken.lock().unwrap(), [(reason, 0)]);
    }

    /// ANF2Driver.cpp:271, :282, ANG1Driver.cpp:204, :547: register reads
    /// and writes and the force read are `pasynInt32SyncIO`'s and the array
    /// write `pasynInt32ArraySyncIO`'s, each printing its TRACEIO_DEVICE line.
    #[test]
    fn register_transfers_are_asyn_int32_sync_io_s() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};

        let name = "amciSyncIo";
        fake::publish(name);
        let trace = get_port(name).unwrap().handle.trace().clone();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        trace.set_trace_file(
            Some(name),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&path).unwrap(),
            ))),
        );
        trace.set_trace_mask(Some(name), TraceMask::IO_DEVICE);
        let (regs, status) = ModbusRegs::connect(name, Duration::from_secs(1));
        status.unwrap();
        assert_eq!(regs.read16(7).unwrap(), 7);
        regs.write16(3, 5).unwrap();
        regs.write_array(4, vec![9, 8]).unwrap();
        ForceRead::connect(name, 10, Duration::from_secs(1))
            .write()
            .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.split_once(" asynInt32").map(|(_, call)| call))
            .collect();
        let want = [
            "SyncIO read: 7",
            "SyncIO wrote: 5",
            "ArraySyncIO wrote: 9",
            "SyncIO wrote: 1",
        ];
        assert_eq!(lines, want, "{text}");
    }

    #[test]
    fn a_missing_port_keeps_registers_that_refuse_transfers() {
        let (regs, status) = ModbusRegs::connect("amciRegsMissing", Duration::from_millis(10));
        assert!(status.is_err());
        assert!(regs.read16(0).is_err());
        assert!(regs.write16(0, 1).is_err());
        assert!(regs.write_array(0, vec![0]).is_err());
    }

    #[test]
    fn a_missing_port_leaves_the_force_read_unconnected_on_reason_0() {
        let fr = ForceRead::connect("amciForceReadMissing", 0, Duration::from_secs(1));
        assert!(!fr.connected());
        assert_eq!(fr.reason(), 0);
        assert!(fr.write().is_err());
    }
}
