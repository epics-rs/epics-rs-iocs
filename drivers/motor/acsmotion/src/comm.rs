//! SPiiPlusCommDriver.cpp: `SPiiPlusComm`, the ASCII and binary exchanges
//! with a SPiiPlus controller on an octet port, and the asyn port C
//! registers for it.
//!
//! An ASCII reply starting with `?` is an error number; the comm asks the
//! controller for its text (`??<errno>`) and prints it. A binary exchange
//! holds the octet port for its flush, write and reads, and suppresses the
//! EOS for those calls only. C instead clears the port's EOS and restores
//! `\r` under the lock of its own asyn port, which the controller's and the
//! aux driver's comms do not share, so their exchanges on one octet port
//! interleave.

use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::request::{OctetReadOp, PortCalls, RequestOp};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;

use motor_common::DEFAULT_CONTROLLER_TIMEOUT;
use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::util::{atof, atoi};

use crate::bincomm::{
    ACKNOWLEDGE, Element, MAX_MESSAGE_LEN, REPLY_END, REPLY_START, SLICE_AVAILABLE, Span,
    read_array_cmd, read_slice_cmd, write_float64_array_cmd,
};
use crate::spiiplus::{INPUT_EOS, OUTPUT_EOS};

/// `SPIIPLUS_CMD_TIMEOUT`: the write of a binary query.
const CMD_TIMEOUT: Duration = Duration::from_millis(50);
/// `SPIIPLUS_ACK_TIMEOUT`: the reply to a binary write.
const ACK_TIMEOUT: Duration = Duration::from_millis(200);
/// `SPIIPLUS_ARRAY_TIMEOUT`: an array reply, and the write of an array.
const ARRAY_TIMEOUT: Duration = Duration::from_secs(10);
/// `MAX_CONTROLLER_STRING_SIZE`.
const READ_BUF: usize = 256;

/// One controller's comm: C `SPiiPlusComm`.
pub struct SpiiPlusComm {
    io: SyncIOHandle,
    /// The comm's own asyn port, which C's messages are printed under.
    port: String,
}

impl SpiiPlusComm {
    /// Connect to octet port `asyn_port` with the `\r` EOS C restores after
    /// every binary exchange. C carries on with an unset asynUser when the
    /// connect fails and crashes on the first exchange; this refuses.
    pub fn connect(comm_port: &str, asyn_port: &str) -> Result<Self, String> {
        let io = connect_with_eos(
            asyn_port,
            0,
            // An ASCII exchange. C passes -1, which waits forever, so one
            // lost reply wedges the comm; asynMotorController's default is
            // what C's `writeReadController` was copied from.
            DEFAULT_CONTROLLER_TIMEOUT,
            INPUT_EOS,
            OUTPUT_EOS,
            EosRefusal::Ignored,
        )
        .map_err(|e| {
            format!("SPiiPlusComm::SPiiPlusComm: cannot connect to SPii+ controller: {e}")
        })?;
        Ok(Self {
            io,
            port: comm_port.to_string(),
        })
    }

    fn print(&self, args: std::fmt::Arguments<'_>) {
        self.trace(TraceMask::ERROR, args);
    }

    fn trace(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        PortManager::global()
            .trace_manager()
            .print(Some(&self.port), Some(0), 0, mask, args);
    }

    /// `writeReadController` between `func`'s `output`, `input` and `status`
    /// lines: the reply whatever the status, as C's zero-filled `inString`
    /// holds it.
    fn exchange(&self, func: &str, cmd: &str) -> (String, AsynResult<()>) {
        self.trace(
            TraceMask::IO_DRIVER,
            format_args!("SPiiPlusComm:{func}: output = {cmd}\n"),
        );
        let (raw, status) = self.io.write_read(0, cmd.as_bytes(), READ_BUF).into_parts();
        let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
        let reply = String::from_utf8_lossy(&raw[..end]).into_owned();
        let code = status.as_ref().err().map_or(0, AsynError::c_status_code);
        self.trace(
            TraceMask::IO_DRIVER,
            format_args!("SPiiPlusComm:{func}:  input = {reply}\n"),
        );
        self.trace(
            TraceMask::IO_DRIVER,
            format_args!("SPiiPlusComm:{func}: status = {code}\n"),
        );
        (reply, status)
    }

    /// The exchange of `writeReadInt`/`Double`/`Str`: a `?` reply is
    /// printed as `func`'s failure, with the error's text, and refused.
    fn query(&self, func: &str, cmd: &str) -> AsynResult<String> {
        let (reply, status) = self.exchange(func, cmd);
        status?;
        self.refuse_error(func, cmd, &reply)?;
        Ok(reply)
    }

    fn refuse_error(&self, func: &str, cmd: &str, reply: &str) -> AsynResult<()> {
        if reply.starts_with('?') {
            self.print(format_args!("SPiiPlusComm:{func}: Command failed: {cmd}\n"));
            self.error_message(reply);
            return Err(comm_error(format!("{cmd}: {reply}")));
        }
        Ok(())
    }

    fn print_val(&self, func: &str, val: impl std::fmt::Display) {
        self.trace(
            TraceMask::IO_DRIVER,
            format_args!("SPiiPlusComm:{func}:    val = {val}\n"),
        );
    }

    /// `writeReadInt`.
    pub fn write_read_int(&self, cmd: &str) -> AsynResult<i32> {
        let val = atoi(&self.query("writeReadInt", cmd)?);
        self.print_val("writeReadInt", val);
        Ok(val)
    }

    /// `writeReadDouble`.
    pub fn write_read_double(&self, cmd: &str) -> AsynResult<f64> {
        let val = atof(&self.query("writeReadDouble", cmd)?);
        self.print_val("writeReadDouble", CFloat(val));
        Ok(val)
    }

    /// `writeReadStr`: the reply up to its first `\r`.
    pub fn write_read_str(&self, cmd: &str) -> AsynResult<String> {
        let reply = self.query("writeReadStr", cmd)?;
        let val = reply.split(['\r', '\0']).next().unwrap_or("");
        self.print_val("writeReadStr", val);
        Ok(val.to_string())
    }

    /// `writeReadAck`: a `?` reply is an error whatever the status, so a
    /// read cut short after one still asks for its text
    /// (SPiiPlusCommDriver.cpp:241-249).
    pub fn write_read_ack(&self, cmd: &str) -> AsynResult<()> {
        let (reply, status) = self.exchange("writeReadAck", cmd);
        self.refuse_error("writeReadAck", cmd, &reply)?;
        status
    }

    /// `writeReadErrorMessage`: `?` + the `?<errno>` reply asks for the
    /// error's text.
    fn error_message(&self, reply: &str) {
        self.print_error(
            atoi(&reply[1..]),
            &format!("?{reply}"),
            "writeReadErrorMessage",
        );
    }

    /// `writeReadBinaryErrorMessage`.
    fn binary_error_message(&self, errno: i32) {
        self.print_error(errno, &format!("??{errno}"), "writeReadBinaryErrorMessage");
    }

    /// C prints the text whatever the status: what a cut read delivered,
    /// nothing after a failed write (SPiiPlusCommDriver.cpp:288-295).
    fn print_error(&self, errno: i32, cmd: &str, func: &str) {
        let (text, _) = self.exchange(func, cmd);
        if text.starts_with('?') {
            self.print(format_args!("SPiiPlusComm:{func}: ERROR #{errno}\n"));
        } else {
            self.print(format_args!(
                "SPiiPlusComm:{func}: ERROR #{errno}: {text}\n"
            ));
        }
    }

    /// `isVariableDefined`: whether `?var` is answered without an error.
    /// C reads the reply whatever the status, so an exchange that failed
    /// before a `?` arrived reads as defined (SPiiPlusCommDriver.cpp:541):
    /// a C defect, not copied.
    pub fn is_variable_defined(&self, var: &str) -> AsynResult<bool> {
        let (reply, status) = self.exchange("isVariableDefined", &format!("?{var}"));
        status?;
        Ok(!reply.starts_with('?'))
    }

    /// `writeReadBinary`: send a query frame, read its `in_bytes` reply,
    /// and return the data with whether another slice is waiting.
    fn write_read_binary(&self, frame: Vec<u8>, in_bytes: usize) -> AsynResult<(Vec<u8>, bool)> {
        let port = self.port()?;
        let (reply, status) = port.lock_port_blocking(user(CMD_TIMEOUT), move |calls| {
            calls.call(&RequestOp::Flush, user(CMD_TIMEOUT))?;
            calls.call(
                &RequestOp::OctetWriteBinary { data: frame },
                user(CMD_TIMEOUT),
            )?;
            Ok::<_, AsynError>(read(calls, in_bytes, ARRAY_TIMEOUT))
        })??;
        let errno = self.binary_error_check(&reply);
        if errno != 0 {
            self.print(format_args!(
                "SPiiPlusComm:writeReadBinary: Binary read failed (controller)\n"
            ));
            self.binary_error_message(errno);
            return Err(comm_error(format!("binary error {errno}")));
        }
        if let Err(e) = status {
            self.print(format_args!(
                "SPiiPlusComm:writeReadBinary: Binary read failed (asyn): {e}, nread={}\n",
                reply.len()
            ));
            return Err(e);
        }
        if reply.len() < 5 {
            return Err(comm_error(format!(
                "short binary reply of {} bytes",
                reply.len()
            )));
        }
        Ok((
            reply[4..reply.len() - 1].to_vec(),
            reply[3] & SLICE_AVAILABLE != 0,
        ))
    }

    /// `writeReadAckBinary`: send one packet of an array write; the
    /// controller answers [`ACKNOWLEDGE`] and the command id, or an error.
    fn write_read_ack_binary(&self, frame: Vec<u8>) -> AsynResult<()> {
        let id = frame[1];
        let port = self.port()?;
        let (reply, status) = port.lock_port_blocking(user(ARRAY_TIMEOUT), move |calls| {
            calls.call(&RequestOp::Flush, user(ARRAY_TIMEOUT))?;
            calls.call(
                &RequestOp::OctetWriteBinary { data: frame },
                user(ARRAY_TIMEOUT),
            )?;
            let (mut reply, status) = read(calls, 2, ACK_TIMEOUT);
            if status.is_err() || reply.first() == Some(&ACKNOWLEDGE) {
                return Ok::<_, AsynError>((reply, status));
            }
            // An error is longer than the two bytes read so far.
            let (rest, status) = read(calls, MAX_MESSAGE_LEN, ACK_TIMEOUT);
            reply.extend(rest);
            Ok((reply, status))
        })??;
        if reply.first() != Some(&ACKNOWLEDGE) {
            let errno = self.binary_error_check(&reply);
            if errno != 0 {
                self.print(format_args!(
                    "SPiiPlusComm:writeReadAckBinary: Binary read failed (controller)\n"
                ));
                self.binary_error_message(errno);
            }
            // C reports a reply that is neither an acknowledgement nor an
            // error it can read as success.
            return Err(status.err().unwrap_or_else(|| {
                comm_error(format!("write of command id {id} not acknowledged"))
            }));
        }
        if let Err(e) = status {
            self.print(format_args!(
                "SPiiPlusComm:writeReadAckBinary: Binary read failed (asyn): {e}, nread={}\n",
                reply.len()
            ));
            return Err(e);
        }
        if reply.get(1) != Some(&id) {
            self.print(format_args!(
                "SPiiPlusComm:writeReadAckBinary: Command ID mismatch: write ID = {id}, read ID = {:?}\n",
                reply.get(1)
            ));
        }
        Ok(())
    }

    /// `binaryErrorCheck`: the error number of an
    /// `[E3][id][06][00]?####[0D][E6]` reply, 0 for any other.
    fn binary_error_check(&self, reply: &[u8]) -> i32 {
        let errno = binary_error_number(reply);
        if let Some(errno) = errno.filter(|&n| n != 0) {
            self.print(format_args!(
                "SPiiPlusComm:binaryErrorCheck: Binary command error {errno} for command id {:x}\n",
                reply[1]
            ));
        }
        errno.unwrap_or(0)
    }

    fn port(&self) -> AsynResult<&PortHandle> {
        self.io.port()
    }

    /// `getDoubleArray`: `var` over `span`, slice by slice.
    pub fn get_double_array(&self, var: &str, span: Span) -> AsynResult<Vec<f64>> {
        let bytes = self.get_array(Element::Float64, var, span)?;
        Ok(bytes
            .as_chunks::<8>()
            .0
            .iter()
            .map(|b| f64::from_le_bytes(*b))
            .collect())
    }

    /// `getIntegerArray`.
    pub fn get_integer_array(&self, var: &str, span: Span) -> AsynResult<Vec<i32>> {
        let bytes = self.get_array(Element::Int32, var, span)?;
        Ok(bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| i32::from_le_bytes(*b))
            .collect())
    }

    fn get_array(&self, element: Element, var: &str, span: Span) -> AsynResult<Vec<u8>> {
        let cmd = read_array_cmd(element, var, span);
        let (mut data, mut more) = self.write_read_binary(cmd.frame, cmd.in_bytes)?;
        let mut slice = 1;
        while more {
            let cmd = read_slice_cmd(element, slice, var, span);
            let (next, again) = self.write_read_binary(cmd.frame, cmd.in_bytes)?;
            data.extend(next);
            more = again;
            slice += 1;
        }
        Ok(data)
    }

    /// `putDoubleArray`: write `data` to global REAL `var` over `span`,
    /// declaring it first when the controller does not have it. C sends
    /// every packet and returns the last one's status, so a failed packet
    /// in the middle reads as success; this stops at the first failure.
    pub fn put_double_array(&self, data: &[f64], var: &str, span: Span) -> AsynResult<()> {
        let count = span.elements();
        if data.len() < count {
            return Err(comm_error(format!(
                "putDoubleArray: {var} needs {count} values, got {}",
                data.len()
            )));
        }
        let (dimensions, missing) = self.global_var_check(var, span).inspect_err(|_| {
            self.print(format_args!(
                "SPiiPlusComm:putDoubleArray: Global variable exists but isn't large enough: {var}\n"
            ))
        })?;
        if missing {
            self.create_global_real_var(var, span).inspect_err(|_| {
                self.print(format_args!(
                    "SPiiPlusComm:putDoubleArray: Failed to create gobal variable: {var}\n"
                ))
            })?;
        }
        let (mut span, mut slice, mut sent, mut offset) = (span, 0, 0, 0);
        loop {
            let cmd = write_float64_array_cmd(var, span, &data[offset..], slice);
            self.write_read_ack_binary(cmd.frame)?;
            sent += cmd.packet_doubles;
            if cmd.remaining_slices == 0 {
                return Ok(());
            }
            slice += 1;
            // The slice number is one character: after the tenth packet
            // the rest is written as a new array from where it stopped.
            if slice == 10 {
                slice = 0;
                offset = sent;
                let sent = sent as i32;
                match dimensions {
                    1 => span.idx1.0 = sent,
                    // C notes this is broken past the tenth packet.
                    2 => {
                        span.idx2.0 = sent / (span.idx1.1 + 1);
                        span.idx1.0 = sent % (span.idx1.1 + 1);
                    }
                    _ => {}
                }
            }
        }
    }

    /// `globalVarCheck`: read the last element of `var`. Error 1064 means
    /// it is not declared yet, 1035 that it is too small. Returns the
    /// dimensions and whether it must be declared.
    fn global_var_check(&self, var: &str, span: Span) -> AsynResult<(u8, bool)> {
        let (dimensions, cmd) = if span.idx2.1 - span.idx2.0 > 0 {
            (2, format!("?{var}({})({})", span.idx1.1, span.idx2.1))
        } else if span.idx1.1 - span.idx1.0 > 0 {
            (1, format!("?{var}({})", span.idx1.1))
        } else {
            (0, format!("?{var}(0)"))
        };
        let (reply, status) = self.exchange("globalVarCheck", &cmd);
        status?;
        if !reply.starts_with('?') {
            return Ok((dimensions, false));
        }
        self.trace(
            TraceMask::IO_DRIVER,
            format_args!("SPiiPlusComm:globalVarCheck: Command failed: {cmd}\n"),
        );
        match atoi(&reply[1..]) {
            1064 => Ok((dimensions, true)),
            1035 => Err(comm_error(format!("{var} is too small"))),
            errno => {
                self.print(format_args!(
                    "SPiiPlusComm:globalVarCheck: Unexpected error! errNo = {errno}\n"
                ));
                Err(comm_error(format!("{cmd}: {reply}")))
            }
        }
    }

    /// `createGlobalRealVar`.
    fn create_global_real_var(&self, var: &str, span: Span) -> AsynResult<()> {
        let cmd = if span.idx2.1 - span.idx2.0 > 0 {
            format!(
                "global REAL {var}({})({})",
                span.idx1.1 + 1,
                span.idx2.1 + 1
            )
        } else if span.idx1.1 - span.idx1.0 > 0 {
            format!("global REAL {var}({})", span.idx1.1 + 1)
        } else {
            format!("global REAL {var}")
        };
        self.write_read_ack(&cmd)
    }
}

/// The error number of an `[E3][id][06][00]?####[0D][E6]` reply: `None`
/// when the reply is not one, else its four characters as C's stream
/// extraction reads them.
fn binary_error_number(reply: &[u8]) -> Option<i32> {
    let [start, _, lsb, msb, b'?', digits @ .., b'\r', end] = reply else {
        return None;
    };
    if *start != REPLY_START || *end != REPLY_END || (*lsb, *msb) != (6, 0) || digits.len() != 4 {
        return None;
    }
    Some(atoi(digits))
}

fn user(timeout: Duration) -> AsynUser {
    AsynUser::new(0).with_addr(0).with_timeout(timeout)
}

/// An EOS-free read of `n` bytes: what arrived, and the read's status.
fn read(calls: &mut dyn PortCalls, n: usize, timeout: Duration) -> (Vec<u8>, AsynResult<()>) {
    calls
        .octet_read(OctetReadOp::ReadBinary { buf_size: n }, user(timeout))
        .into_parts()
}

fn comm_error(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("SPiiPlusComm: {message}"),
    }
}

/// The asyn port C's `SPiiPlusComm` is: no parameters, `num_channels`
/// addresses.
struct CommPort(PortDriverBase);

impl PortDriver for CommPort {
    fn base(&self) -> &PortDriverBase {
        &self.0
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.0
    }
}

/// `SPiiPlusCommDriver.cpp:25-26`: the interfaces it implements, and the
/// ones that do callbacks.
pub const INTERFACES: InterfaceMask = InterfaceMask::INT32
    .or(InterfaceMask::UINT32_DIGITAL)
    .or(InterfaceMask::DRV_USER);
pub const INTERRUPTS: InterfaceMask = InterfaceMask::UINT32_DIGITAL;

/// Register the comm's asyn port `name`, as C's constructor does.
pub fn register_comm_port(
    owner: &std::sync::Arc<InterruptGate>,
    name: &str,
    num_channels: i32,
) -> Result<(), String> {
    let base = PortDriverBase::new(
        name,
        num_channels.max(1) as usize,
        PortFlags {
            multi_device: true,
            can_block: true,
            destructible: true,
        },
        INTERFACES,
        INTERRUPTS,
    );
    let claim = claim_port(name).map_err(|e| format!("{name}: {e}"))?;
    let (runtime, _actor) = create_port_runtime(&claim, CommPort(base), RuntimeConfig::default())
        .map_err(|e| format!("{name}: {e}"))?;
    if let Err(e) = claim.publish(runtime.port_handle().clone(), owner) {
        runtime.shutdown();
        return Err(format!("{name}: {e}"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};
    use motor_common::testing::LineRig;
    use std::sync::{Arc, Mutex};

    /// A comm on `io` printing under `port`, and that port's IO_DRIVER and
    /// ERROR lines without the prefix.
    fn traced_comm(port: &str, io: SyncIOHandle) -> (SpiiPlusComm, std::path::PathBuf) {
        let path = std::env::temp_dir().join(format!("acs-comm-{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_trace_info_mask(Some(port), TraceInfoMask::empty());
        trace.set_trace_file(Some(port), TraceFile::File(file));
        let comm = SpiiPlusComm {
            io,
            port: port.into(),
        };
        (comm, path)
    }

    /// `SPiiPlusCommDriver.cpp:25-26`: the port implements asynInt32,
    /// asynUInt32Digital and asynDrvUser, and posts asynUInt32Digital
    /// callbacks.
    #[test]
    fn the_comm_port_registers_c_interfaces_and_interrupts() {
        use epics_rs::asyn::interfaces::InterfaceType;
        use epics_rs::asyn::interrupt::InterruptFilter;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;

        let owner = Arc::new(InterruptGate::opened());
        register_comm_port(&owner, "acsCommMasks", 1).unwrap();
        let handle = epics_rs::asyn::asyn_record::get_port("acsCommMasks")
            .unwrap()
            .handle;
        for iface in [
            InterfaceType::Int32,
            InterfaceType::UInt32Digital,
            InterfaceType::DrvUser,
        ] {
            assert!(handle.has_interface(iface), "{iface}");
        }
        for iface in [
            InterfaceType::Int32,
            InterfaceType::UInt32Digital,
            InterfaceType::Float64,
        ] {
            let filter = InterruptFilter {
                iface: Some(iface),
                ..Default::default()
            };
            let accepted = handle
                .interrupts()
                .register_interrupt_user(filter, 0)
                .is_ok();
            assert_eq!(accepted, iface == InterfaceType::UInt32Digital, "{iface}");
        }
    }

    fn take(path: &std::path::Path) -> String {
        let text = std::fs::read_to_string(path).unwrap();
        std::fs::remove_file(path).unwrap();
        text
    }

    fn answer(cmd: &str) -> String {
        match cmd.trim_end_matches(['\r', '\n']) {
            "?X" => "12",
            "?Y" => "2.5",
            "ENABLE 9" => "?1021",
            "??1021" => "Bad axis",
            _ => "",
        }
        .into()
    }

    /// SPiiPlusCommDriver.cpp:82-100, 128-147, 231-249, 280-295: the output,
    /// input and status of every exchange, the value read, and a `?` reply
    /// a timeout cut short still refused with its text.
    #[test]
    fn exchanges_print_c_io_driver_lines() {
        let (_rig, io) = LineRig::cut("acsIoCut", answer, |cmd| cmd.starts_with("ENABLE"));
        let (comm, path) = traced_comm("acsIoCutComm", io);
        assert_eq!(comm.write_read_int("?X").unwrap(), 12);
        assert_eq!(comm.write_read_double("?Y").unwrap(), 2.5);
        assert!(comm.write_read_ack("ENABLE 9").is_err());
        assert_eq!(
            take(&path),
            "SPiiPlusComm:writeReadInt: output = ?X\n\
             SPiiPlusComm:writeReadInt:  input = 12\n\
             SPiiPlusComm:writeReadInt: status = 0\n\
             SPiiPlusComm:writeReadInt:    val = 12\n\
             SPiiPlusComm:writeReadDouble: output = ?Y\n\
             SPiiPlusComm:writeReadDouble:  input = 2.5\n\
             SPiiPlusComm:writeReadDouble: status = 0\n\
             SPiiPlusComm:writeReadDouble:    val = 2.500000\n\
             SPiiPlusComm:writeReadAck: output = ENABLE 9\n\
             SPiiPlusComm:writeReadAck:  input = ?1021\n\
             SPiiPlusComm:writeReadAck: status = 1\n\
             SPiiPlusComm:writeReadAck: Command failed: ENABLE 9\n\
             SPiiPlusComm:writeReadErrorMessage: output = ??1021\n\
             SPiiPlusComm:writeReadErrorMessage:  input = Bad axis\n\
             SPiiPlusComm:writeReadErrorMessage: status = 0\n\
             SPiiPlusComm:writeReadErrorMessage: ERROR #1021: Bad axis\n"
        );
    }

    /// SPiiPlusCommDriver.cpp:76-100, 288-291: a failed write reads as an
    /// empty reply, and the error's text is printed even then.
    #[test]
    fn a_failed_write_prints_an_empty_input() {
        let (_rig, io) = LineRig::failing("acsIoFail", answer, |cmd| {
            cmd.starts_with("?Z") || cmd.starts_with("??")
        });
        let (comm, path) = traced_comm("acsIoFailComm", io);
        assert!(comm.write_read_int("?Z").is_err());
        assert!(comm.write_read_ack("ENABLE 9").is_err());
        assert_eq!(
            take(&path),
            "SPiiPlusComm:writeReadInt: output = ?Z\n\
             SPiiPlusComm:writeReadInt:  input = \n\
             SPiiPlusComm:writeReadInt: status = 3\n\
             SPiiPlusComm:writeReadAck: output = ENABLE 9\n\
             SPiiPlusComm:writeReadAck:  input = ?1021\n\
             SPiiPlusComm:writeReadAck: status = 0\n\
             SPiiPlusComm:writeReadAck: Command failed: ENABLE 9\n\
             SPiiPlusComm:writeReadErrorMessage: output = ??1021\n\
             SPiiPlusComm:writeReadErrorMessage:  input = \n\
             SPiiPlusComm:writeReadErrorMessage: status = 3\n\
             SPiiPlusComm:writeReadErrorMessage: ERROR #1021: \n"
        );
    }

    #[test]
    fn error_reply_carries_its_number() {
        assert_eq!(
            binary_error_number(b"\xe3\x07\x06\x00?3260\r\xe6"),
            Some(3260)
        );
        assert_eq!(
            binary_error_number(b"\xe3\x07\x06\x00?12ab\r\xe6"),
            Some(12)
        );
    }

    #[test]
    fn other_replies_are_not_errors() {
        // The ten-byte form some manuals give, which C does not read either.
        assert_eq!(binary_error_number(b"\xe3\x076?3260\r\xe6"), None);
        assert_eq!(binary_error_number(b"\xe3\x07\x06\x00?3260\r\xe5"), None);
        assert_eq!(binary_error_number(b"\xe3\x07\x05\x00?3260\r\xe6"), None);
        assert_eq!(binary_error_number(b"\xe9\x07"), None);
    }
}
