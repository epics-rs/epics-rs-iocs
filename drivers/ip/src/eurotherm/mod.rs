//! Eurotherm 800/2000 series temperature controller (`ipApp/src/devXxEurotherm.c`).
//!
//! Write-only, like the C: it sends the frame and never reads the reply. asyn
//! address = the controller's local address on the line (`LAD=` in the C link);
//! the group address (`GAD=`, 0 on every database in the module) belongs to the
//! line and is given to `EurothermConfig`.
//!
//! The C took the payload format from the record link (`FMT=SL%4.0lf`), so the
//! parameter mnemonic lives in the format. The port keeps that: the link's
//! parameter name *is* the format, and the driver creates the parameter for it
//! on the first record that binds (`drv_user_create`):
//!
//! ```text
//! field(OUT, "@asyn($(PORT),$(LADDR))SL%4.0lf")   # ao      -> value write
//! field(OUT, "@asyn($(PORT),$(LADDR))EURO_READ")  # stringout -> read request
//! ```
//!
//! Reading a value back is the `stringout` path in the C, and it is only half a
//! transaction: the C writes the request frame and never reads the answer — the
//! module's database pairs it with a `devXxStrParm` record that reads the line
//! afterwards. `devXxStrParm` is not ported, so `EURO_READ` sends the request
//! and nothing collects the reply. It is here because the C has it; a database
//! that uses it has nowhere to put the answer yet.

pub mod protocol;

use std::collections::HashMap;
use std::sync::mpsc::{Sender, channel};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::user::AsynUser;

use crate::connect::connect_octet;
use crate::fmt::format_c_double;
use crate::runtime::IpPortRuntime;
use crate::worker::{self, DeviceWorker, Transport};
use protocol::MAX_ADDRESS;

/// Write timeout (`pPvt->timeout`, `devXxEurotherm.c:118`).
pub const EUROTHERM_TIMEOUT: Duration = Duration::from_secs(3);

/// The port is write-only: the worker never polls, it only sends what the record
/// write handlers hand it. The tick just bounds how long it sleeps between them.
const IDLE_TICK: Duration = Duration::from_secs(60);

/// Parameter a `stringout` binds to: its value is the mnemonic of the parameter
/// to read (`Eurotherm.db`: `field(VAL, "SP")`).
pub const READ_REQUEST: &str = "EURO_READ";

/// A ready-made frame for the device.
pub struct EurothermCommand(Vec<u8>);

pub struct EurothermDriver {
    base: PortDriverBase,
    group_address: u8,
    read_request: usize,
    /// The payload format of every parameter created from a record link, keyed
    /// by its reason. This is the C's per-record `FMT=`.
    formats: HashMap<usize, String>,
    commands: Sender<EurothermCommand>,
}

impl EurothermDriver {
    pub fn new(
        port_name: &str,
        group_address: u8,
        commands: Sender<EurothermCommand>,
    ) -> AsynResult<Self> {
        if group_address > MAX_ADDRESS {
            return Err(crate::asyn_error(format!(
                "Eurotherm: the group address is a single digit, got {group_address}"
            )));
        }
        let mut base = PortDriverBase::new(
            port_name,
            usize::from(MAX_ADDRESS) + 1,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let read_request = base.create_param(READ_REQUEST, ParamType::Octet)?;
        Ok(Self {
            base,
            group_address,
            read_request,
            formats: HashMap::new(),
            commands,
        })
    }

    fn local_address(&self, addr: i32) -> AsynResult<u8> {
        u8::try_from(addr)
            .ok()
            .filter(|addr| *addr <= MAX_ADDRESS)
            .ok_or_else(|| {
                crate::asyn_error(format!(
                    "Eurotherm: the local address is a single digit, got {addr}"
                ))
            })
    }

    fn send(&self, frame: Vec<u8>) -> AsynResult<()> {
        self.commands
            .send(EurothermCommand(frame))
            .map_err(|_| crate::asyn_error("Eurotherm: the worker thread is gone"))
    }
}

impl PortDriver for EurothermDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// Bind a record link. A link that names an existing parameter resolves to
    /// it; any other link is a payload format (the C's `FMT=`) and gets a
    /// parameter of its own.
    fn drv_user_create(&mut self, drv_info: &str, _addr: i32) -> AsynResult<DrvUserInfo> {
        if let Some(reason) = self.base().params.find_param(drv_info) {
            return Ok(DrvUserInfo::from_reason(reason));
        }
        // Reject a format the driver could not send *now*, at bind time, rather
        // than on every write.
        format_c_double(drv_info, 0.0).map_err(|e| {
            AsynError::ParamNotFound(format!(
                "{drv_info:?} is neither a parameter of this port nor a value format: {e}"
            ))
        })?;
        let reason = self.base_mut().create_param(drv_info, ParamType::Float64)?;
        self.formats.insert(reason, drv_info.to_string());
        Ok(DrvUserInfo::from_reason(reason))
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let format = self
            .formats
            .get(&user.reason)
            .ok_or_else(|| crate::asyn_error("Eurotherm: this parameter is not a value format"))?
            .clone();
        let local = self.local_address(user.addr)?;
        let frame = protocol::write_value(self.group_address, local, &format, value)
            .map_err(crate::asyn_error)?;
        self.send(frame)?;

        self.base_mut()
            .params
            .set_float64(user.reason, user.addr, value)?;
        self.base_mut().call_param_callbacks(user.addr)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        if user.reason != self.read_request {
            return Err(crate::asyn_error(
                "Eurotherm: the only octet parameter is EURO_READ",
            ));
        }
        let mnemonic = String::from_utf8(data.to_vec())
            .map_err(|e| crate::asyn_error(format!("Eurotherm: the mnemonic is not ASCII: {e}")))?;
        let local = self.local_address(user.addr)?;
        let frame = protocol::read_request(self.group_address, local, &mnemonic)
            .map_err(crate::asyn_error)?;
        self.send(frame)?;

        self.base_mut()
            .params
            .set_string(user.reason, user.addr, mnemonic)?;
        self.base_mut().call_param_callbacks(user.addr)?;
        Ok(data.len())
    }
}

pub struct EurothermWorker {
    transport: Transport,
}

impl DeviceWorker for EurothermWorker {
    type Command = EurothermCommand;

    fn poll(&mut self) {}

    fn handle(&mut self, command: Self::Command) {
        // The C wrote the frame and never read a reply (devXxEurotherm.c:300-311).
        if let Err(e) = self.transport.write(&command.0) {
            log::error!("Eurotherm: write failed: {e}");
        }
    }
}

/// `EurothermConfig(port, octetPort, groupAddress)`.
pub fn create_eurotherm(
    port_name: &str,
    octet_port: &str,
    group_address: u8,
) -> AsynResult<IpPortRuntime> {
    let io = connect_octet(octet_port, EUROTHERM_TIMEOUT).map_err(crate::asyn_error)?;
    let (commands, rx) = channel::<EurothermCommand>();
    let driver = EurothermDriver::new(port_name, group_address, commands)?;

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let worker = EurothermWorker {
        transport: Transport::new(io),
    };
    let thread = worker::spawn(port_name, worker, rx, IDLE_TICK);
    Ok(IpPortRuntime::new(runtime_handle, thread))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::Receiver;

    /// The driver alone, with the worker's end of the channel in hand: every
    /// frame a record write produces can be read straight off it.
    fn driver(group: u8) -> (EurothermDriver, Receiver<EurothermCommand>) {
        let (tx, rx) = channel();
        (EurothermDriver::new("EURO_TEST", group, tx).unwrap(), rx)
    }

    #[test]
    fn a_record_link_that_is_a_format_creates_its_own_parameter() {
        let (mut driver, rx) = driver(0);

        // field(OUT, "@asyn(EURO1,1)SL%4.0lf")
        let reason = driver.drv_user_create("SL%4.0lf", 1).unwrap().reason;
        // The same link on a second record resolves to the same parameter.
        assert_eq!(
            driver.drv_user_create("SL%4.0lf", 1).unwrap().reason,
            reason
        );

        let mut user = AsynUser::new(reason).with_addr(1);
        driver.write_float64(&mut user, 123.4).unwrap();

        let frame = rx.try_recv().unwrap().0;
        assert_eq!(
            frame,
            protocol::write_value(0, 1, "SL%4.0lf", 123.4).unwrap()
        );
    }

    #[test]
    fn a_link_that_is_neither_a_parameter_nor_a_format_is_rejected() {
        let (mut driver, _rx) = driver(0);
        assert!(driver.drv_user_create("NOT_A_FORMAT", 0).is_err());
        assert!(driver.drv_user_create("SL%d", 0).is_err());
    }

    #[test]
    fn a_stringout_on_euro_read_sends_a_read_request() {
        let (mut driver, rx) = driver(2);
        let reason = driver.drv_user_create(READ_REQUEST, 0).unwrap().reason;

        let mut user = AsynUser::new(reason).with_addr(3);
        driver.write_octet(&mut user, b"SP").unwrap();

        let frame = rx.try_recv().unwrap().0;
        assert_eq!(frame, protocol::read_request(2, 3, "SP").unwrap());
    }

    #[test]
    fn an_address_that_is_not_a_single_digit_is_rejected() {
        let (mut driver, _rx) = driver(0);
        let reason = driver.drv_user_create("SL%.0f", 0).unwrap().reason;
        let mut user = AsynUser::new(reason).with_addr(10);
        assert!(driver.write_float64(&mut user, 1.0).is_err());

        let (tx, _rx) = channel();
        assert!(EurothermDriver::new("EURO_BAD", 10, tx).is_err());
    }
}
