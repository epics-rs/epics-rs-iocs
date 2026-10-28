//! C `drvXPSAsynAux.c` (`XPSAuxConfig`): the XPS analog and digital I/O as an
//! asyn port whose addr is the channel, with a poller that posts the inputs.
//!
//! The reasons are C's command enum in order, created as params so the poller
//! can post through them. Reads and writes go to the controller, as C's do;
//! the params hold only what the poller last read.

use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use super::rpc::XpsSocket;

/// C `drvXPSAsynAuxCommands`, in enum order: the index is the reason.
const COMMANDS: [(&str, ParamType); 5] = [
    ("ANALOG_INPUT", ParamType::Float64),
    ("ANALOG_OUTPUT", ParamType::Float64),
    ("ANALOG_GAIN", ParamType::UInt32Digital),
    ("BINARY_INPUT", ParamType::UInt32Digital),
    ("BINARY_OUTPUT", ParamType::UInt32Digital),
];
pub const ANALOG_INPUT: usize = 0;
pub const ANALOG_OUTPUT: usize = 1;
pub const ANALOG_GAIN: usize = 2;
pub const BINARY_INPUT: usize = 3;
pub const BINARY_OUTPUT: usize = 4;

/// C `TCP_TIMEOUT`.
pub const TCP_TIMEOUT: Duration = Duration::from_secs(1);

const ANALOG_INPUTS: [&str; 4] = ["GPIO2.ADC1", "GPIO2.ADC2", "GPIO2.ADC3", "GPIO2.ADC4"];
const ANALOG_OUTPUTS: [&str; 4] = ["GPIO2.DAC1", "GPIO2.DAC2", "GPIO2.DAC3", "GPIO2.DAC4"];
const DIGITAL_INPUTS: [&str; 4] = ["GPIO1.DI", "GPIO2.DI", "GPIO3.DI", "GPIO4.DI"];
const DIGITAL_OUTPUTS: [&str; 3] = ["GPIO1.DO", "GPIO3.DO", "GPIO4.DO"];

fn refuse(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

/// The GPIO name for `channel`, or C's "`what` channel out of range" error.
fn channel<const N: usize>(
    names: &[&'static str; N],
    channel: i32,
    what: &str,
) -> AsynResult<&'static str> {
    usize::try_from(channel)
        .ok()
        .and_then(|c| names.get(c).copied())
        .ok_or_else(|| {
            refuse(format!(
                "drvXPSAsynAux::{what} channel out of range={channel}"
            ))
        })
}

/// The XPSAuxConfig port.
pub struct XpsGpio {
    base: PortDriverBase,
    socket: XpsSocket,
}

impl XpsGpio {
    /// C registers the port multi-device and synchronous; it can block here
    /// because the socket is an asyn IP port, which a record must not wait on
    /// under its hold.
    pub fn new(port_name: &str, socket: XpsSocket) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            ANALOG_INPUTS.len(),
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
            InterfaceMask::COMMON
                | InterfaceMask::FLOAT64
                | InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::DRV_USER,
            InterfaceMask::FLOAT64 | InterfaceMask::UINT32_DIGITAL,
        );
        for (name, kind) in COMMANDS {
            base.create_param(name, kind)?;
        }
        Ok(Self { base, socket })
    }

    /// One pass of C `XPSAuxPoller`: read every input, then post them.
    ///
    /// C posts the inputs of its first pass whether or not the reads worked,
    /// so a failed first read posts uninitialized memory; here a channel is
    /// first posted, with every bit forced, once a read of it has worked.
    pub fn poll(&mut self, mut state: PollState) -> PollState {
        match self.socket.gpio_analog_get(&ANALOG_INPUTS) {
            Ok(values) => state.analog = values.try_into().ok(),
            Err(e) => self.trace_error(format_args!(
                "drvXPSAsynAux::XPSAuxPoller error calling GPIOAnalogGet={}\n",
                e.code()
            )),
        }
        for (addr, name) in DIGITAL_INPUTS.iter().enumerate() {
            match self.socket.gpio_digital_get(name) {
                Ok(value) => {
                    let forced = if state.posted[addr] { 0 } else { 0xffff };
                    let _ = self.base.set_uint32_param(
                        BINARY_INPUT,
                        addr as i32,
                        value.into(),
                        0xffff,
                        forced,
                    );
                    state.posted[addr] = true;
                }
                Err(e) => self.trace_error(format_args!(
                    "drvXPSAsynAux::XPSAuxPoller error calling GPIODigitalGet={}\n",
                    e.code()
                )),
            }
        }
        // C calls the analog inputs back every pass, changed or not.
        if let Some(values) = state.analog {
            for (addr, value) in values.into_iter().enumerate() {
                let _ = self
                    .base
                    .set_float64_param(ANALOG_INPUT, addr as i32, value);
                let _ = self.base.mark_param_changed(ANALOG_INPUT, addr as i32);
            }
        }
        for addr in 0..ANALOG_INPUTS.len() as i32 {
            let _ = self.base.call_param_callbacks(addr);
        }
        state
    }

    /// C `asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, …)`, a user on addr -1.
    fn trace_error(&self, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.base.port_name),
            Some(-1),
            0,
            TraceMask::ERROR,
            args,
        );
    }
}

/// What C's poller keeps between passes.
#[derive(Default)]
pub struct PollState {
    /// The last analog inputs read, which C posts again when a read fails.
    analog: Option<[f64; 4]>,
    /// Which digital inputs have been posted once.
    posted: [bool; 4],
}

/// C `XPSAuxPoller`'s loop, until the port goes away.
pub fn run_poller(handle: PortHandle, period: Duration) {
    let mut state = PollState::default();
    loop {
        std::thread::sleep(period);
        match handle.with_driver_blocking::<XpsGpio, _>(move |gpio| gpio.poll(state)) {
            Ok(next) => state = next,
            Err(_) => return,
        }
    }
}

impl PortDriver for XpsGpio {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `drvUserCreate`: the command names, case-insensitive.
    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        COMMANDS
            .iter()
            .position(|(name, _)| name.eq_ignore_ascii_case(&req.drv_info))
            .map(DrvUserInfo::from_reason)
            .ok_or_else(|| {
                refuse(format!(
                    "drvXPSAsynAux::drvUserCreate, unknown command={}",
                    req.drv_info
                ))
            })
    }

    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let name = match user.reason {
            ANALOG_INPUT => channel(&ANALOG_INPUTS, user.addr, "readFloat64")?,
            ANALOG_OUTPUT => channel(&ANALOG_OUTPUTS, user.addr, "readFloat64")?,
            command => {
                return Err(refuse(format!(
                    "drvXPSAsynAux::readFloat64 invalid command={command}"
                )));
            }
        };
        let value = self.socket.gpio_analog_get(&[name]).map_err(|e| {
            refuse(format!(
                "drvXPSAsynAux::readFloat64 error calling GPIOAnalogGet={}",
                e.code()
            ))
        })?[0];
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "drvXPSAsynAux::readFloat64, value={value:.6}\n",
                value = CFloat(value)
            ),
        );
        Ok(value)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let name = match user.reason {
            ANALOG_OUTPUT => channel(&ANALOG_OUTPUTS, user.addr, "writeFloat64")?,
            command => {
                return Err(refuse(format!(
                    "drvXPSAsynAux::writeFloat64 invalid command={command}"
                )));
            }
        };
        self.socket.gpio_analog_set(name, value).map_err(|e| {
            refuse(format!(
                "drvXPSAsynAux::writeFloat64 error calling GPIOAnalogSet={}",
                e.code()
            ))
        })?;
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "drvXPSAsynAux::writeFloat64, value={value:.6}\n",
                value = CFloat(value)
            ),
        );
        Ok(())
    }

    fn read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        let digital = |name| {
            self.socket
                .gpio_digital_get(name)
                .map(|raw| u32::from(raw) & mask)
                .map_err(|e| {
                    refuse(format!(
                        "drvXPSAsynAux::readUInt32D error calling GPIODigitalGet={}",
                        e.code()
                    ))
                })
        };
        let value = match user.reason {
            BINARY_INPUT => digital(channel(&DIGITAL_INPUTS, user.addr, "readUInt32D readBi")?)?,
            BINARY_OUTPUT => digital(channel(&DIGITAL_OUTPUTS, user.addr, "readUInt32D readBo")?)?,
            // C reads the gain unmasked, and names the call in its error
            // `writeUInt32D … GainSet`, a copy of the write's text; this
            // names the read.
            ANALOG_GAIN => {
                let name = channel(&ANALOG_INPUTS, user.addr, "readUInt32D")?;
                self.socket.gpio_analog_gain_get(name).map_err(|e| {
                    refuse(format!(
                        "drvXPSAsynAux::readUInt32D error calling GPIOAnalogGainGet={}",
                        e.code()
                    ))
                })? as u32
            }
            command => {
                return Err(refuse(format!(
                    "drvXPSAsynAux::readUInt32D invalid command={command}"
                )));
            }
        };
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("drvXPSAsynAux::readUInt32D, value={value:x}\n"),
        );
        Ok(value)
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        match user.reason {
            BINARY_OUTPUT => {
                let name = channel(&DIGITAL_OUTPUTS, user.addr, "writeUInt32D")?;
                // C passes both through `unsigned short` parameters.
                self.socket
                    .gpio_digital_set(name, mask as u16, value as u16)
                    .map_err(|e| {
                        refuse(format!(
                            "drvXPSAsynAux::writeUInt32D error calling GPIODigitalSet={}",
                            e.code()
                        ))
                    })?;
                user.print(
                    TraceMask::IO_DRIVER,
                    file!(),
                    line!(),
                    format_args!(
                        "drvXPSAsynAux::writeUInt32D, set binary output value={}\n",
                        value as i32
                    ),
                );
            }
            ANALOG_GAIN => {
                let name = channel(&ANALOG_INPUTS, user.addr, "writeUInt32D")?;
                self.socket
                    .gpio_analog_gain_set(name, value as i32)
                    .map_err(|e| {
                        refuse(format!(
                            "drvXPSAsynAux::writeUInt32D error calling GPIOAnalogGainSet={}",
                            e.code()
                        ))
                    })?;
                user.print(
                    TraceMask::IO_DRIVER,
                    file!(),
                    line!(),
                    format_args!(
                        "drvXPSAsynAux::writeUInt32D, set gain value={}\n",
                        value as i32
                    ),
                );
            }
            command => {
                return Err(refuse(format!(
                    "drvXPSAsynAux::writeUInt32D invalid command={command}"
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use epics_rs::asyn::interrupt::InterruptFilter;
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::request::RequestOp;
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

    use super::*;
    use crate::xps::SocketMode;
    use crate::xps::fake::FakeLine;

    fn gpio(name: &str) -> (FakeLine, PortHandle) {
        let line = FakeLine::new(&format!("{name}_line"), "");
        let driver = XpsGpio::new(name, line.socket(SocketMode::Query)).unwrap();
        let claim = claim_port(name).unwrap();
        let (rt, _jh) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        let handle = rt.port_handle().clone();
        claim.publish(handle.clone(), &open_owner()).unwrap();
        (line, handle)
    }

    fn message(e: AsynError) -> String {
        e.message().to_string()
    }

    fn uint32(handle: &PortHandle, op: RequestOp, reason: usize, addr: i32) -> AsynResult<u32> {
        let r = handle.submit_blocking(op, AsynUser::new(reason).with_addr(addr))?;
        Ok(r.uint_val.unwrap_or(0))
    }

    #[test]
    fn the_commands_reach_the_controller_by_channel() {
        let (line, port) = gpio("xps_gpio_io");
        let reason = |info: &str| port.drv_user_create_blocking(&DrvUserRequest::new(info, 0));
        assert_eq!(reason("binary_input").unwrap().reason, BINARY_INPUT);
        assert_eq!(
            message(reason("SPEED").unwrap_err()),
            "drvXPSAsynAux::drvUserCreate, unknown command=SPEED"
        );

        assert_eq!(port.read_float64_blocking(ANALOG_INPUT, 3).unwrap(), 1.5);
        port.write_float64_blocking(ANALOG_OUTPUT, 1, 2.5).unwrap();
        let write = RequestOp::UInt32DigitalWrite { value: 5, mask: 7 };
        uint32(&port, write.clone(), BINARY_OUTPUT, 2).unwrap();
        let gain = uint32(
            &port,
            RequestOp::UInt32DigitalRead { mask: 1 },
            ANALOG_GAIN,
            0,
        );
        assert_eq!(gain.unwrap(), 4, "the gain is read unmasked");
        assert_eq!(
            line.take_log(),
            [
                "GPIOAnalogGet (GPIO2.ADC4,double *)",
                "GPIOAnalogSet (GPIO2.DAC2,2.5)",
                "GPIODigitalSet (GPIO4.DO,7,5)",
                "GPIOAnalogGainGet (GPIO2.ADC1,int *)",
            ]
        );

        assert_eq!(
            message(uint32(&port, write, BINARY_OUTPUT, 3).unwrap_err()),
            "drvXPSAsynAux::writeUInt32D channel out of range=3"
        );
        assert_eq!(
            message(
                port.write_float64_blocking(ANALOG_INPUT, 0, 1.0)
                    .unwrap_err()
            ),
            "drvXPSAsynAux::writeFloat64 invalid command=0"
        );
        *line.fail.lock().unwrap() = "GPIODigitalGet";
        let read = RequestOp::UInt32DigitalRead { mask: 0xff };
        assert_eq!(
            message(uint32(&port, read, BINARY_INPUT, 0).unwrap_err()),
            "drvXPSAsynAux::readUInt32D error calling GPIODigitalGet=-3"
        );
    }

    #[test]
    fn the_poller_posts_changed_bits_and_every_analog_pass() {
        let (line, port) = gpio("xps_gpio_poll");
        let seen: Arc<Mutex<Vec<String>>> = Arc::default();
        let on = |reason, addr, mask| {
            let seen = seen.clone();
            port.interrupts()
                .register_sync_callback(
                    InterruptFilter {
                        reason: Some(reason),
                        addr: Some(addr),
                        uint32_mask: mask,
                        ..Default::default()
                    },
                    move |v| seen.lock().unwrap().push(format!("{:?}", v.value)),
                )
                .unwrap()
        };
        let _bits = on(BINARY_INPUT, 1, Some(0x3));
        let _analog = on(ANALOG_INPUT, 2, None);
        let mut state = Some(PollState::default());
        let mut poll = |digital: u16| {
            *line.digital.lock().unwrap() = digital;
            let s = state.take().unwrap();
            state = Some(
                port.with_driver_blocking::<XpsGpio, _>(move |g| g.poll(s))
                    .unwrap(),
            );
            std::mem::take(&mut *seen.lock().unwrap())
        };
        // The first pass posts every input, its bits unchanged or not.
        assert_eq!(poll(0), ["UInt32Digital(0)", "Float64(3.5)"]);
        // Unchanged bits are not posted; the analog inputs always are.
        assert_eq!(poll(0), ["Float64(3.5)"]);
        // A change outside the record's mask is not posted.
        assert_eq!(poll(4), ["Float64(3.5)"]);
        assert_eq!(poll(5), ["UInt32Digital(5)", "Float64(3.5)"]);
        // A failed analog read posts the last values again.
        *line.fail.lock().unwrap() = "GPIOAnalogGet";
        assert_eq!(poll(5), ["Float64(3.5)"]);
    }
}
