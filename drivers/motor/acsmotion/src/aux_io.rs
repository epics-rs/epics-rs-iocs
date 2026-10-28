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
    /// `OUT(chan).bit`, returning the last bit's status as C
    /// (SPiiPlusAuxDriver.cpp:101-110). The traced `outValue` is what C's
    /// loop leaves in it: 0 for a full write, else bit 31 of `value`.
    fn write_bits(&self, chan: i32, mask: u32, value: u32) -> AsynResult<()> {
        let (out_value, result) = if mask == u32::MAX {
            let result = self.comm.write_read_ack(&format!("OUT({chan}) = {value}"));
            (0, result)
        } else {
            let mut result = Ok(());
            for bit in (0..MAX_BITS).filter(|bit| mask & 1 << bit != 0) {
                let on = u32::from(value & 1 << bit != 0);
                result = self
                    .comm
                    .write_read_ack(&format!("OUT({chan}).{bit} = {on}"));
            }
            (value >> (MAX_BITS - 1), result)
        };
        let port = &self.base.port_name;
        match &result {
            Ok(()) => self.base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "SPiiPlusAuxIO:writeBits, port {port}, wrote outValue=0x{out_value:x}, value=0x{value:x}, mask=0x{mask:x}, chan={chan}\n"
                ),
            ),
            Err(e) => self.base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "SPiiPlusAuxIO:writeBits, port {port}, ERROR writing outValue=0x{out_value:x}, value=0x{value:x}, mask=0x{mask:x}, chan={chan}, status={}\n",
                    e.status() as i32
                ),
            ),
        }
        result
    }

    /// `writeAnalog`: `AOUT(chan)`, held to the controller's ±100 percent.
    fn write_analog(&self, chan: i32, value: f64) -> AsynResult<()> {
        let value = value.clamp(-100.0, 100.0);
        let result = self
            .comm
            .write_read_ack(&format!("AOUT({chan}) = {}", CFloat(value).g()));
        let (port, value) = (&self.base.port_name, CFloat(value));
        match &result {
            Ok(()) => self.base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "SPiiPlusAuxIO:writeAnalog, port {port}, wrote value={value}, chan={chan}\n"
                ),
            ),
            Err(e) => self.base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "SPiiPlusAuxIO:writeAnalog, port {port}, ERROR writing value={value}, chan={chan}, status={}\n",
                    e.status() as i32
                ),
            ),
        }
        result
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
        // C posts address 0's callbacks (SPiiPlusAuxDriver.cpp:143) where
        // writeFloat64 and the poller post the channel's (:193, :257): a C
        // defect, not copied; the base posts the channel's.
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

/// Take a read's values, or OR its failure into `status` as C ORs the four
/// reads' `asynStatus` (SPiiPlusAuxDriver.cpp:221-231).
fn keep<T: Copy>(last: &mut [T], read: AsynResult<Vec<T>>, status: &mut i32) {
    match read {
        Ok(values) => last
            .iter_mut()
            .zip(values)
            .for_each(|(slot, value)| *slot = value),
        Err(e) => *status |= e.status() as i32,
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
        let mut status = 0;
        keep(
            &mut last.ain,
            comm.get_double_array("AIN", span),
            &mut status,
        );
        keep(
            &mut last.aout,
            comm.get_double_array("AOUT", span),
            &mut status,
        );
        keep(
            &mut last.input,
            comm.get_integer_array("IN", span),
            &mut status,
        );
        keep(
            &mut last.output,
            comm.get_integer_array("OUT", span),
            &mut status,
        );
        if status != 0 {
            PortManager::global().trace_manager().print(
                Some(port),
                Some(0),
                0,
                TraceMask::ERROR,
                format_args!("SPiiPlusAuxIO:pollerThread: ERROR reading I/O, status={status}\n"),
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
        InterfaceMask::INT32
            | InterfaceMask::FLOAT64
            | InterfaceMask::UINT32_DIGITAL
            | InterfaceMask::DRV_USER,
        InterfaceMask::INT32 | InterfaceMask::FLOAT64 | InterfaceMask::UINT32_DIGITAL,
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
    use std::sync::Mutex;
    use std::time::Instant;

    use epics_rs::asyn::port::DrvUserRequest;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::request::RequestOp;
    use epics_rs::asyn::trace::TraceFile;

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

    /// C `writeBits` returns the last bit's status (SPiiPlusAuxDriver.cpp:
    /// 101-110), which `writeUInt32Digital` returns.
    #[test]
    fn a_masked_write_returns_the_last_bit_s_status() {
        let line = crate::fake::line("spiiAuxBits");
        config(&open_owner(), "spiiAuxBitsIo", "spiiAuxBits", 1, 1000.0).unwrap();
        let port = PortManager::global()
            .find_port_handle("spiiAuxBitsIo")
            .unwrap();
        let reason = port
            .drv_user_create_blocking(&DrvUserRequest::new("DIGITAL_OUTPUT", 0))
            .unwrap()
            .reason;
        let write = || {
            let op = RequestOp::UInt32DigitalWrite {
                value: 0b01,
                mask: 0b11,
            };
            port.submit_blocking(op, AsynUser::new(reason))
        };
        line.lock().unwrap().refused.insert("OUT(0).0 = 1".into());
        write().unwrap();
        let sent = line.lock().unwrap().sent.clone();
        assert!(sent.iter().any(|c| c == "OUT(0).1 = 0"), "{sent:?}");
        line.lock().unwrap().refused.insert("OUT(0).1 = 0".into());
        assert!(write().is_err());
    }

    /// `port`'s trace at `IO_DRIVER` and `ERROR`, written to a file.
    fn trace_to_file(port: &str, channels: i32) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
        for addr in 0..channels {
            trace.set_device_trace_file(port, addr, TraceFile::File(file.clone()));
        }
        path
    }

    /// The aux driver's own lines in the trace file, without their stamps.
    fn aux_lines(path: &std::path::Path) -> Vec<String> {
        std::fs::read_to_string(path)
            .unwrap()
            .lines()
            .filter_map(|l| l.find("SPiiPlusAuxIO:").map(|at| l[at..].to_string()))
            .collect()
    }

    /// `writeBits` and `writeAnalog` trace a write at `ASYN_TRACEIO_DRIVER`,
    /// a failed one at `ASYN_TRACE_ERROR` with its status
    /// (SPiiPlusAuxDriver.cpp:113-121, 165-173).
    #[test]
    fn writes_are_traced_as_c_traces_them() {
        let line = crate::fake::line("spiiAuxTrc");
        config(&open_owner(), "spiiAuxTrcIo", "spiiAuxTrc", 2, 1000.0).unwrap();
        let port = PortManager::global()
            .find_port_handle("spiiAuxTrcIo")
            .unwrap();
        let path = trace_to_file("spiiAuxTrcIo", 2);
        let reason = |name: &str| {
            port.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
                .unwrap()
                .reason
        };
        let (analog, digital) = (reason("ANALOG_OUTPUT"), reason("DIGITAL_OUTPUT"));
        let bits = |addr, value, mask| {
            let op = RequestOp::UInt32DigitalWrite { value, mask };
            port.submit_blocking(op, AsynUser::new(digital).with_addr(addr))
        };
        bits(1, 5, u32::MAX).unwrap();
        bits(1, 0x8000_0001, 0b11).unwrap();
        port.write_float64_blocking(analog, 1, 150.0).unwrap();
        line.lock()
            .unwrap()
            .refused
            .extend(["OUT(0).0 = 1".to_string(), "AOUT(0) = -2.5".to_string()]);
        assert!(bits(0, 1, 1).is_err());
        assert!(port.write_float64_blocking(analog, 0, -2.5).is_err());
        let lines = aux_lines(&path);
        let _ = std::fs::remove_file(&path);
        // outValue is what C's loop left in it: 0 for a full write, else
        // bit 31 of the value.
        assert_eq!(
            lines,
            [
                "SPiiPlusAuxIO:writeBits, port spiiAuxTrcIo, wrote outValue=0x0, value=0x5, mask=0xffffffff, chan=1",
                "SPiiPlusAuxIO:writeBits, port spiiAuxTrcIo, wrote outValue=0x1, value=0x80000001, mask=0x3, chan=1",
                "SPiiPlusAuxIO:writeAnalog, port spiiAuxTrcIo, wrote value=100.000000, chan=1",
                "SPiiPlusAuxIO:writeBits, port spiiAuxTrcIo, ERROR writing outValue=0x0, value=0x1, mask=0x1, chan=0, status=3",
                "SPiiPlusAuxIO:writeAnalog, port spiiAuxTrcIo, ERROR writing value=-2.500000, chan=0, status=3",
            ]
        );
    }

    /// The poller traces the OR of its four reads' statuses
    /// (SPiiPlusAuxDriver.cpp:221-235): timeouts (1) on `AIN` and `IN`
    /// around a controller error (3) on `AOUT` print 3, which neither the
    /// first nor the last failure is.
    #[test]
    fn a_failed_poll_traces_the_reads_or_ed_status() {
        let line = crate::fake::line("spiiAuxPoll");
        {
            let mut ctrl = line.lock().unwrap();
            ctrl.dropped.insert("BIN ??AIN(0,1)(0,0)".into());
            ctrl.refused.insert("BIN ??AOUT(0,1)(0,0)".into());
            ctrl.dropped.insert("BIN ??IN(0,1)(0,0)".into());
        }
        config(&open_owner(), "spiiAuxPollIo", "spiiAuxPoll", 2, 0.05).unwrap();
        let path = trace_to_file("spiiAuxPollIo", 2);
        let deadline = Instant::now() + Duration::from_secs(10);
        let lines = loop {
            let lines = aux_lines(&path);
            if !lines.is_empty() || Instant::now() > deadline {
                break lines;
            }
            std::thread::sleep(Duration::from_millis(20));
        };
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            lines.first().map(String::as_str),
            Some("SPiiPlusAuxIO:pollerThread: ERROR reading I/O, status=3"),
            "{lines:?}"
        );
    }
}
