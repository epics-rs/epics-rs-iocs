//! A scripted XPS for unit tests: every call succeeds with a canned reply,
//! the call named in `fail` returns -3, and every command is logged.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::{PortRuntimeHandle, RuntimeConfig, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use super::rpc::{SocketMode, XpsSocket};

pub type Fail = Arc<Mutex<&'static str>>;
pub type Log = Arc<Mutex<Vec<String>>>;
pub type Digital = Arc<Mutex<u16>>;
pub type Firmware = Arc<Mutex<&'static str>>;
pub type GroupStatus = Arc<Mutex<i32>>;

struct FakeXps {
    base: PortDriverBase,
    reply: Option<String>,
    fail: Fail,
    log: Log,
    digital: Digital,
    firmware: Firmware,
    group_status: GroupStatus,
}

impl PortDriver for FakeXps {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let cmd = String::from_utf8_lossy(data);
        let name = cmd.split('(').next().unwrap_or("").trim();
        self.log.lock().unwrap().push(cmd.trim().to_string());
        let digital;
        let firmware;
        let group_status;
        let values = match name {
            _ if name == *self.fail.lock().unwrap() => "-3",
            "GPIOAnalogGet" => "0,1.5,2.5,3.5,4.5",
            "GPIOAnalogGainGet" => "0,4",
            "GPIODigitalGet" => {
                digital = format!("0,{}", self.digital.lock().unwrap());
                &digital
            }
            "ErrorStringGet" if cmd.contains("(-3,") => {
                "0,Error in the command parameters of a hexapod call"
            }
            "ErrorStringGet" => "0,Not allowed action",
            "FirmwareVersionGet" => {
                firmware = format!("0,{}", self.firmware.lock().unwrap());
                &firmware
            }
            "HexapodCoordinateSystemGet" => "0,1,2,3,4,5,6",
            "GroupStatusGet" => {
                group_status = format!("0,{}", self.group_status.lock().unwrap());
                &group_status
            }
            "GroupPositionCurrentGet" if cmd.contains(".P2,") => "0,2.5",
            "GroupPositionCurrentGet" => "0,1.5",
            "GroupPositionSetpointGet" => "0,1.25",
            "GroupVelocityCurrentGet" if cmd.contains(".P2,") => "0,3",
            "PositionerSGammaParametersGet" => "0,10,40,0.02,0.02",
            "GatheringCurrentNumberGet" => "0,2,100",
            "GatheringDataMultipleLinesGet" => "0,1;1.5\n2;2.5",
            "SingleAxisSlaveParametersGet" if cmd.contains("(NORATIO,") => "0,G1.P1,",
            "SingleAxisSlaveParametersGet" => "0,G1.P1,2.5",
            _ => "0,0",
        };
        self.reply = Some(format!("{values},EndOfAPI"));
        Ok(data.len())
    }
    fn io_read_octet(
        &mut self,
        _user: &AsynUser,
        buf: &mut [u8],
    ) -> epics_rs::asyn::interpose::OctetReadResult {
        let Some(reply) = self.reply.take() else {
            return epics_rs::asyn::interpose::OctetReadResult::failed(AsynError::Status {
                status: AsynStatus::Timeout,
                message: "no reply".into(),
            });
        };
        buf[..reply.len()].copy_from_slice(reply.as_bytes());
        epics_rs::asyn::interpose::OctetReadResult::ok(
            reply.len(),
            epics_rs::asyn::interpose::EomReason::empty(),
        )
    }
}

/// One fake XPS on asyn port `port`.
pub struct FakeLine {
    port: String,
    rt: PortRuntimeHandle,
    pub fail: Fail,
    pub log: Log,
    /// What `GPIODigitalGet` reads.
    pub digital: Digital,
    /// What `FirmwareVersionGet` reads.
    pub firmware: Firmware,
    /// What `GroupStatusGet` reads.
    pub group_status: GroupStatus,
}

impl FakeLine {
    pub fn new(port: &str, fail: &'static str) -> Self {
        let fail = Arc::new(Mutex::new(fail));
        let log = Log::default();
        let digital = Digital::default();
        let firmware = Arc::new(Mutex::new("0"));
        let group_status = Arc::new(Mutex::new(12));
        let line = FakeXps {
            base: PortDriverBase::new(
                port,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            reply: None,
            fail: fail.clone(),
            log: log.clone(),
            digital: digital.clone(),
            firmware: firmware.clone(),
            group_status: group_status.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(port).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        Self {
            port: port.into(),
            rt,
            fail,
            log,
            digital,
            firmware,
            group_status,
        }
    }

    pub fn socket(&self, mode: SocketMode) -> XpsSocket {
        XpsSocket::new(
            self.rt.port_handle().clone(),
            0,
            &self.port,
            mode,
            Duration::from_secs(1),
        )
    }

    /// The commands sent since the last call.
    pub fn take_log(&self) -> Vec<String> {
        self.log.lock().unwrap().drain(..).collect()
    }
}
