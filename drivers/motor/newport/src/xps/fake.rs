//! A scripted XPS for unit tests: every call succeeds with a canned reply,
//! the call named in `fail` returns -3, and every command is logged.

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex};
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
/// Replies by command name, over the canned ones.
pub type Replies = Arc<Mutex<HashMap<&'static str, String>>>;
pub type Hold = Arc<(Mutex<HoldState>, Condvar)>;

/// A held `MultipleAxesPVTExecution`: while `armed` it blocks until
/// `GroupMoveAbort`, then answers -27 as the XPS does.
#[derive(Default)]
pub struct HoldState {
    pub armed: bool,
    pub entered: bool,
    pub aborted: bool,
}

struct FakeXps {
    base: PortDriverBase,
    reply: Option<String>,
    fail: Fail,
    log: Log,
    digital: Digital,
    firmware: Firmware,
    group_status: GroupStatus,
    replies: Replies,
    hold: Hold,
}

impl FakeXps {
    /// Whether a held `MultipleAxesPVTExecution` was aborted.
    fn hold(&self, name: &str) -> bool {
        let (state, changed) = &*self.hold;
        let mut state = state.lock().unwrap();
        match name {
            "MultipleAxesPVTExecution" => {
                state.entered = true;
                changed.notify_all();
                while state.armed && !state.aborted {
                    state = changed.wait(state).unwrap();
                }
                state.armed && state.aborted
            }
            "GroupMoveAbort" => {
                state.aborted = true;
                changed.notify_all();
                false
            }
            _ => false,
        }
    }
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
        let aborted = self.hold(name);
        let replied = self.replies.lock().unwrap().get(name).cloned();
        let digital;
        let firmware;
        let group_status;
        let values = match name {
            _ if aborted => "-27",
            _ if replied.is_some() => replied.as_deref().unwrap(),
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
            "GroupPositionCurrentGet" if cmd.contains("(G1,") => "0,1.5,x,3.5",
            "GroupPositionCurrentGet" if cmd.contains(".P2,") => "0,2.5",
            "GroupPositionCurrentGet" => "0,1.5",
            "GroupPositionSetpointGet" => "0,1.25",
            "GroupVelocityCurrentGet" if cmd.contains(".P2,") => "0,3",
            "PositionerSGammaParametersGet" => "0,10,40,0.02,0.02",
            "PositionerUserTravelLimitsGet" => "0,-100,100",
            "MultipleAxesPVTVerificationResultGet" => "0,traj.trj,-0.5,2.5,1.2,3.4",
            "MultipleAxesPVTParametersGet" if cmd.contains("(NOELEMENT,") => "0,traj.trj,x",
            "MultipleAxesPVTParametersGet" => "0,traj.trj,42",
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

/// What a [`FakeLine`] and its twins share.
struct Script {
    fail: Fail,
    log: Log,
    digital: Digital,
    firmware: Firmware,
    group_status: GroupStatus,
    replies: Replies,
    hold: Hold,
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
    pub replies: Replies,
    pub hold: Hold,
}

impl FakeLine {
    pub fn new(port: &str, fail: &'static str) -> Self {
        Self::serve(
            port,
            Script {
                fail: Arc::new(Mutex::new(fail)),
                log: Log::default(),
                digital: Digital::default(),
                firmware: Arc::new(Mutex::new("0")),
                group_status: Arc::new(Mutex::new(12)),
                replies: Replies::default(),
                hold: Hold::default(),
            },
        )
    }

    /// The same XPS on a second asyn port, as a second socket to it is:
    /// a call blocked on one does not hold up the other.
    pub fn twin(&self, port: &str) -> Self {
        Self::serve(
            port,
            Script {
                fail: self.fail.clone(),
                log: self.log.clone(),
                digital: self.digital.clone(),
                firmware: self.firmware.clone(),
                group_status: self.group_status.clone(),
                replies: self.replies.clone(),
                hold: self.hold.clone(),
            },
        )
    }

    fn serve(port: &str, script: Script) -> Self {
        let Script {
            fail,
            log,
            digital,
            firmware,
            group_status,
            replies,
            hold,
        } = script;
        let line = FakeXps {
            base: PortDriverBase::new(
                port,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            reply: None,
            fail: fail.clone(),
            log: log.clone(),
            digital: digital.clone(),
            firmware: firmware.clone(),
            group_status: group_status.clone(),
            replies: replies.clone(),
            hold: hold.clone(),
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
            replies,
            hold,
        }
    }

    pub fn socket(&self, mode: SocketMode) -> XpsSocket {
        self.socket_with(mode, Duration::from_secs(1))
    }

    pub fn socket_with(&self, mode: SocketMode, timeout: Duration) -> XpsSocket {
        XpsSocket::new(self.rt.port_handle().clone(), 0, &self.port, mode, timeout)
    }

    /// The commands sent since the last call.
    pub fn take_log(&self) -> Vec<String> {
        self.log.lock().unwrap().drain(..).collect()
    }
}
