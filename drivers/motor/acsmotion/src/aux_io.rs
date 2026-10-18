//! SPiiPlusAuxDriver.cpp: `SPiiPlusAuxIO`, the controller's digital ports
//! (`IN`/`OUT`, 32 bits each) and analog signals (`AIN`/`AOUT`, percent),
//! one asyn address per index, polled in bulk over the binary protocol.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::exit::at_exit;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;

use crate::bincomm::Span;
use crate::comm::SpiiPlusComm;

/// `MAX_BITS`: the bits of one `OUT` port.
const MAX_BITS: u32 = 32;

#[derive(Clone, Copy)]
struct AuxParams {
    digital_input: usize,
    digital_output: usize,
    analog_input: usize,
    analog_output: usize,
}

struct SpiiPlusAuxIo {
    base: PortDriverBase,
    params: AuxParams,
    comm: Arc<SpiiPlusComm>,
}

impl SpiiPlusAuxIo {
    /// `writeBits`: all 32 bits with `OUT(chan)`, else each masked bit with
    /// `OUT(chan).bit`. C keeps only the last bit's status; this reports
    /// the first failure and still writes the other bits.
    fn write_bits(&self, chan: i32, mask: u32, value: u32) -> AsynResult<()> {
        if mask == u32::MAX {
            return self.comm.write_read_ack(&format!("OUT({chan}) = {value}"));
        }
        let mut result = Ok(());
        for bit in (0..MAX_BITS).filter(|bit| mask & 1 << bit != 0) {
            let on = u32::from(value & 1 << bit != 0);
            let status = self
                .comm
                .write_read_ack(&format!("OUT({chan}).{bit} = {on}"));
            result = result.and(status);
        }
        result
    }

    /// `writeAnalog`: `AOUT(chan)`, held to the controller's ±100 percent.
    fn write_analog(&self, chan: i32, value: f64) -> AsynResult<()> {
        let value = value.clamp(-100.0, 100.0);
        self.comm
            .write_read_ack(&format!("AOUT({chan}) = {}", CFloat(value).g()))
    }
}

impl PortDriver for SpiiPlusAuxIo {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        // C posts the callbacks of address 0 rather than of the channel
        // written; the base posts the channel's.
        self.base.write_uint32_digital_param(user, value, mask)?;
        if user.reason == self.params.digital_output {
            let chan = self.base.get_address(user.addr, user.reason)?;
            self.write_bits(chan, mask, value)?;
        }
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.base.write_float64_param(user, value)?;
        if user.reason == self.params.analog_output {
            let chan = self.base.get_address(user.addr, user.reason)?;
            self.write_analog(chan, value)?;
        }
        Ok(())
    }
}

/// The last values the poller read, kept for an array whose read fails.
struct Readings {
    ain: Vec<f64>,
    aout: Vec<f64>,
    input: Vec<i32>,
    output: Vec<i32>,
}

fn keep<T: Copy>(last: &mut [T], read: AsynResult<Vec<T>>, failed: &mut Option<AsynError>) {
    match read {
        Ok(values) => last
            .iter_mut()
            .zip(values)
            .for_each(|(slot, value)| *slot = value),
        Err(e) => {
            failed.get_or_insert(e);
        }
    }
}

/// `pollerThread`: read every channel of the four arrays, post them, sleep.
///
/// C asks for indices `0..=numChannels`, one past the port's addresses, so
/// an MP4U configured with its 256 channels has every read refused; and it
/// reads all four arrays through one buffer, so a failed read posts what
/// the previous array left there. This asks for `0..numChannels` and keeps
/// an array's last values when its read fails.
fn poll(
    comm: &SpiiPlusComm,
    handle: &PortHandle,
    params: AuxParams,
    port: &str,
    channels: i32,
    period: Duration,
    shutting_down: &AtomicBool,
) {
    let n = channels as usize;
    let span = Span::one(0, channels - 1);
    let mut last = Readings {
        ain: vec![0.0; n],
        aout: vec![0.0; n],
        input: vec![0; n],
        output: vec![0; n],
    };
    while !shutting_down.load(Ordering::Acquire) {
        let mut failed = None;
        keep(
            &mut last.ain,
            comm.get_double_array("AIN", span),
            &mut failed,
        );
        keep(
            &mut last.aout,
            comm.get_double_array("AOUT", span),
            &mut failed,
        );
        keep(
            &mut last.input,
            comm.get_integer_array("IN", span),
            &mut failed,
        );
        keep(
            &mut last.output,
            comm.get_integer_array("OUT", span),
            &mut failed,
        );
        if let Some(e) = failed {
            PortManager::global().trace_manager().print(
                Some(port),
                Some(0),
                0,
                TraceMask::ERROR,
                format_args!("SPiiPlusAuxIO:pollerThread: ERROR reading I/O, status={e}\n"),
            );
        }
        for i in 0..n {
            let addr = i as i32;
            let _ = handle.set_params_and_notify_blocking(
                addr,
                vec![
                    ParamSetValue::new(params.analog_input, addr, ParamValue::Float64(last.ain[i])),
                    ParamSetValue::new(
                        params.analog_output,
                        addr,
                        ParamValue::Float64(last.aout[i]),
                    ),
                    ParamSetValue::uint32_digital(
                        params.digital_input,
                        addr,
                        last.input[i] as u32,
                        u32::MAX,
                        0,
                    ),
                    ParamSetValue::uint32_digital(
                        params.digital_output,
                        addr,
                        last.output[i] as u32,
                        u32::MAX,
                        0,
                    ),
                ],
            );
        }
        std::thread::sleep(period);
    }
}

/// `AcsMotionAuxIOConfig`: aux port `port` with `channels` addresses on
/// octet port `asyn_port`, through a comm C names `<port>Comm`, polled
/// every `poll_period` seconds.
pub fn config(
    owner: &Arc<InterruptGate>,
    port: &str,
    asyn_port: &str,
    channels: i32,
    poll_period: f64,
) -> Result<(), String> {
    if channels < 1 {
        return Err(format!("AcsMotionAuxIOConfig: {port}: no I/O channels"));
    }
    let comm_port = format!("{port}Comm");
    let comm = Arc::new(SpiiPlusComm::connect(&comm_port, asyn_port)?);
    crate::comm::register_comm_port(owner, &comm_port, channels)?;

    let mut base = PortDriverBase::new(
        port,
        channels as usize,
        PortFlags {
            multi_device: true,
            can_block: true,
            destructible: true,
        },
        InterfaceMask::UNMIGRATED_ALL,
        InterfaceMask::UNMIGRATED_INTERRUPTS,
    );
    let created = |e: AsynError| format!("{port}: {e}");
    let params = AuxParams {
        digital_input: base
            .create_param("DIGITAL_INPUT", ParamType::UInt32Digital)
            .map_err(created)?,
        digital_output: base
            .create_param("DIGITAL_OUTPUT", ParamType::UInt32Digital)
            .map_err(created)?,
        analog_input: base
            .create_param("ANALOG_INPUT", ParamType::Float64)
            .map_err(created)?,
        analog_output: base
            .create_param("ANALOG_OUTPUT", ParamType::Float64)
            .map_err(created)?,
    };
    let driver = SpiiPlusAuxIo {
        base,
        params,
        comm: comm.clone(),
    };
    let claim = claim_port(port).map_err(created)?;
    let (runtime, _actor) =
        create_port_runtime(&claim, driver, RuntimeConfig::default()).map_err(created)?;
    let handle = runtime.port_handle().clone();
    if let Err(e) = claim.publish(handle.clone(), owner) {
        runtime.shutdown();
        return Err(created(e));
    }

    // C stops the poller at exit, before asyn closes the socket.
    let shutting_down = Arc::new(AtomicBool::new(false));
    let flag = shutting_down.clone();
    at_exit(format!("SPiiPlusAuxIO {port}"), move || {
        flag.store(true, Ordering::Release)
    });
    let period = Duration::try_from_secs_f64(poll_period).unwrap_or_default();
    let name = port.to_string();
    std::thread::Builder::new()
        .name("SPiiPlusAuxIOPoller".into())
        .spawn(move || {
            poll(
                &comm,
                &handle,
                params,
                &name,
                channels,
                period,
                &shutting_down,
            )
        })
        .map_err(|e| format!("{port}: {e}"))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::port::DrvUserRequest;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::request::RequestOp;

    use super::*;

    /// A write at address -1 goes to channel 0, getAddress's channel for it
    /// (SPiiPlusAuxDriver.cpp:135, :186).
    #[test]
    fn writes_at_address_minus_one_reach_channel_0() {
        let line = crate::fake::line("spiiAuxM1");
        config(&open_owner(), "spiiAuxM1Io", "spiiAuxM1", 2, 1000.0).unwrap();
        let port = PortManager::global()
            .find_port_handle("spiiAuxM1Io")
            .unwrap();
        let reason = |name: &str| {
            port.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
                .unwrap()
                .reason
        };
        port.write_float64_blocking(reason("ANALOG_OUTPUT"), -1, 5.0)
            .unwrap();
        let digital = AsynUser::new(reason("DIGITAL_OUTPUT")).with_addr(-1);
        let write = RequestOp::UInt32DigitalWrite {
            value: 1,
            mask: u32::MAX,
        };
        port.submit_blocking(write, digital).unwrap();
        let sent: Vec<String> = line
            .lock()
            .unwrap()
            .sent
            .iter()
            .filter(|c| c.contains("OUT(") && !c.starts_with("BIN"))
            .cloned()
            .collect();
        assert_eq!(sent, ["AOUT(0) = 5", "OUT(0) = 1"]);
    }
}
