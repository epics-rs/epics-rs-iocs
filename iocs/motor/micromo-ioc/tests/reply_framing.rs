//! The MVP 2001 answers a query "\r\n0001 <hex>\r\n", a prepended terminator
//! and then the reply; drvMVP2001.cc:597 reads it with input EOS "\n".
//! MVP2001Driver.cpp sets no EOS, so st.cmd's decides what its second read
//! hands `parseReply`.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::interpose::eos::EosInterpose;
use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use motor_micromo::{Mvp2001Axis, Mvp2001Controller};

/// The input and output EOS st.cmd sets.
fn st_cmd_eos() -> (Vec<u8>, Vec<u8>) {
    let eos = |cmd: &str| {
        let line = include_str!("../st.cmd")
            .lines()
            .find(|l| l.starts_with(cmd))
            .unwrap();
        let arg = line.rsplit('"').nth(1).unwrap();
        arg.replace("\\r", "\r").replace("\\n", "\n").into_bytes()
    };
    (eos("asynOctetSetInputEos("), eos("asynOctetSetOutputEos("))
}

/// Node 1 of a chain on a serial line behind the port's EOS layer.
struct Mvp {
    base: PortDriverBase,
    line: VecDeque<u8>,
}

impl PortDriver for Mvp {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let hex = match data {
            b"1 SR\r" => "01F4",
            b"1 POS\r" => "000001F4",
            b"1 ST\r" => "0000",
            _ => return Ok(data.len()),
        };
        self.line.extend(format!("\r\n0001 {hex}\r\n").bytes());
        Ok(data.len())
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
        if self.line.is_empty() {
            return OctetReadResult::failed(AsynError::Status {
                status: AsynStatus::Timeout,
                message: "no reply".into(),
            });
        }
        let n = self.line.len().min(buf.len());
        for (b, byte) in buf.iter_mut().zip(self.line.drain(..n)) {
            *b = byte;
        }
        OctetReadResult::ok(n, EomReason::empty())
    }
    fn io_flush(&mut self, _user: &mut AsynUser) -> AsynResult<()> {
        self.line.clear();
        Ok(())
    }
}

/// A handle on a fresh line to the device, with `input_eos` and st.cmd's
/// output EOS set.
fn line(name: &str, input_eos: &[u8]) -> SyncIOHandle {
    let mut base = PortDriverBase::new(
        name,
        1,
        PortFlags::default(),
        InterfaceMask::OCTET_TRANSPORT,
        InterfaceMask::OCTET,
    );
    base.install_octet_interpose(Box::new(EosInterpose::default()));
    let mvp = Mvp {
        base,
        line: VecDeque::new(),
    };
    let claim = claim_port(name).unwrap();
    let (rt, _jh) = create_port_runtime(&claim, mvp, RuntimeConfig::default()).unwrap();
    claim
        .publish(rt.port_handle().clone(), &open_owner())
        .unwrap();
    let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
    io.set_input_eos(0, input_eos).unwrap();
    io.set_output_eos(0, &st_cmd_eos().1).unwrap();
    io
}

fn polled_position(name: &str, input_eos: &[u8]) -> f64 {
    let controller = Arc::new(Mutex::new(Mvp2001Controller::new(line(name, input_eos), 1)));
    let mut axis = Mvp2001Axis::new(controller, name, 0, 512, 1000, 1);
    let mut status = MotorStatus::default();
    axis.poll(&AsynUser::default(), &mut status).unwrap();
    status.position
}

/// MVP2001Driver.cpp:129-137: the first read takes the prepended terminator,
/// the second the reply `parseReply` indexes from byte 5.
#[test]
fn st_cmd_input_eos_leaves_the_second_read_the_bare_reply() {
    let io = line("mvpFramingRaw", &st_cmd_eos().0);
    let (first, status) = io.write_read(0, b"1 POS", 32).into_parts();
    status.unwrap();
    assert_eq!(first, b"");
    let (reply, status) = io.read_octet(0, 32).into_parts();
    status.unwrap();
    assert_eq!(String::from_utf8_lossy(&reply), "0001 000001F4");
}

#[test]
fn the_axis_reads_the_position_under_st_cmd_eos() {
    assert_eq!(polled_position("mvpFramingSt", &st_cmd_eos().0), 500.0);
}

/// Under a CR input EOS the second read keeps the LF, and C's `parseReply`
/// reads " 000001F" from byte 5; the port reads the same.
#[test]
fn a_reply_left_framed_parses_as_c_parses_it() {
    assert_eq!(polled_position("mvpFramingCr", b"\r"), 31.0);
}
