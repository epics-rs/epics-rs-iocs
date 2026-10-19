//! C `phytronIO` (phytronIOctrl.cpp): a digital or analog I/O card channel
//! of a Phytron controller as an asyn port with `IN`, `OUT` and, on an
//! MCC-1, `DIR`. The controller poll reads every channel on its own line
//! (phytronAxisMotor.cpp:921-922, 976-978) and posts what it read.

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::runtime::epics_string::print_escaped;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;

use crate::phytron::{CtrlType, PhytronController, c_isspace, parse_int64};

/// C `phytronIO::IOTYPE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IoType {
    GuessD,
    GuessA,
    Di,
    Do,
    Dio,
    Ai,
    Ao,
    Aio,
}

impl IoType {
    fn name(self) -> &'static str {
        match self {
            IoType::GuessD => "guess digital",
            IoType::GuessA => "guess analog",
            IoType::Di => "DIN",
            IoType::Do => "DOUT",
            IoType::Dio => "DIO",
            IoType::Ai => "AIN",
            IoType::Ao => "AOUT",
            IoType::Aio => "AIO",
        }
    }

    fn has_output(self) -> bool {
        matches!(self, IoType::Do | IoType::Dio | IoType::Ao | IoType::Aio)
    }

    fn has_input(self) -> bool {
        matches!(self, IoType::Di | IoType::Dio | IoType::Ai | IoType::Aio)
    }

    /// The kinds read by an output and an input request.
    fn two_requests(self) -> bool {
        matches!(
            self,
            IoType::GuessD | IoType::GuessA | IoType::Dio | IoType::Aio
        )
    }
}

/// The MCC preamble of C static `pollIO` (phytronIOctrl.cpp:361-372): an
/// MCC has one card of fixed I/O, digital in and out and analog in, whose
/// digital values are bit strings. `None` for any other card; else whether
/// the values are bit strings.
fn preamble(kind: &mut IoType, mcc: bool, card: u8) -> Option<bool> {
    if !mcc {
        return Some(false);
    }
    if card > 1 {
        return None;
    }
    match *kind {
        IoType::GuessD | IoType::Dio => {
            *kind = IoType::Dio;
            Some(true)
        }
        IoType::GuessA => {
            *kind = IoType::Ai;
            Some(false)
        }
        _ => Some(false),
    }
}

/// C static `pollIO` with `bIsRequest` (phytronIOctrl.cpp:373-407): the
/// commands that read card `card` channel `channel`, output first.
fn requests(kind: &mut IoType, mcc: bool, card: u8, channel: u8) -> Vec<String> {
    if preamble(kind, mcc, card).is_none() {
        return Vec::new();
    }
    let (ag, eg) = (format!("AG{card}R"), format!("EG{card}R"));
    let (da, ad) = (format!("DA{card}.{channel}"), format!("AD{card}.{channel}"));
    match *kind {
        IoType::GuessD | IoType::Dio => vec![ag, eg],
        IoType::GuessA | IoType::Aio => vec![da, ad],
        IoType::Di => vec![eg],
        IoType::Do => vec![ag],
        IoType::Ai if mcc => vec![format!("AD{channel}R")],
        IoType::Ai => vec![ad],
        IoType::Ao => vec![da],
    }
}

/// C static `pollIO` without `bIsRequest` (phytronIOctrl.cpp:408-520):
/// `replies` start with the answers to [`requests`] of `kind`, which a
/// guessed kind resolves to by the requests the card ACKs. The values are
/// written as C writes them, so a reply that fails half way leaves the
/// first value read.
fn parse(
    kind: &mut IoType,
    mcc: bool,
    card: u8,
    replies: &[String],
    input: &mut i32,
    output: &mut i32,
) -> bool {
    let Some(binary) = preamble(kind, mcc, card) else {
        return false;
    };
    let two = kind.two_requests();
    if replies.len() < if two { 2 } else { 1 } {
        return false;
    }
    let ack = |i: usize| replies[i].starts_with('\x06');
    let acked = match *kind {
        IoType::GuessD | IoType::GuessA => {
            let digital = *kind == IoType::GuessD;
            *kind = match (ack(0), ack(1)) {
                (false, false) => return false,
                (true, true) if digital => IoType::Dio,
                (true, false) if digital => IoType::Do,
                (false, true) if digital => IoType::Di,
                (true, true) => IoType::Aio,
                (true, false) => IoType::Ao,
                (false, true) => IoType::Ai,
            };
            true
        }
        IoType::Dio | IoType::Aio => ack(0) && ack(1),
        _ => ack(0),
    };
    if !acked {
        return false;
    }
    // C reads the input after erasing the output's reply, so a guessed card
    // without outputs reads its input from the NAK to the output request
    // (:489-495) and can never be created. Each value is read from the
    // reply to its own request here.
    let (out_at, in_at) = if two { (0, 1) } else { (0, 0) };
    if kind.has_output() && !read_value(&replies[out_at], binary, output) {
        return false;
    }
    !kind.has_input() || read_value(&replies[in_at], binary, input)
}

/// One value of [`parse`]: the ACKed reply's text, a bit string (first
/// character bit 0) or a base-0 `epicsParseInt32`.
fn read_value(reply: &str, binary: bool, to: &mut i32) -> bool {
    let text = &reply[1..];
    if !binary {
        return match parse_int64(text).and_then(|v| i32::try_from(v).ok()) {
            Some(v) => {
                *to = v;
                true
            }
            None => false,
        };
    }
    *to = 0;
    let mut bit = 1i32;
    for c in text.chars() {
        match c {
            '0' => {}
            '1' => *to |= bit,
            _ => return false,
        }
        bit = bit.wrapping_shl(1);
    }
    true
}

/// C `epicsParseInt32(s, &v, 0, &units)`: the base-0 integer `s` starts
/// with, whatever follows it.
fn parse_int32_prefix(s: &str) -> Option<i32> {
    let t = s.trim_start_matches(c_isspace);
    let b = t.as_bytes();
    let sign = usize::from(matches!(b.first(), Some(b'+' | b'-')));
    let rest = &b[sign..];
    let digits = |radix: u32, from: usize| {
        from + rest[from..]
            .iter()
            .take_while(|&&c| char::from(c).is_digit(radix))
            .count()
    };
    let hex = rest.len() > 2
        && rest[0] == b'0'
        && (rest[1] | 0x20) == b'x'
        && rest[2].is_ascii_hexdigit();
    let end = if hex {
        digits(16, 2)
    } else if rest.first() == Some(&b'0') {
        digits(8, 1)
    } else {
        digits(10, 0)
    };
    parse_int64(&t[..sign + end]).and_then(|v| i32::try_from(v).ok())
}

/// C `iIn_`/`iOut_`: written under the controller lock by its poll and an
/// OUT write, read by the I/O port without it, as C reads them.
#[derive(Default)]
struct IoValues {
    input: AtomicI32,
    output: AtomicI32,
}

impl IoValues {
    fn get(&self) -> (i32, i32) {
        (
            self.input.load(Ordering::Relaxed),
            self.output.load(Ordering::Relaxed),
        )
    }

    fn set(&self, input: i32, output: i32) {
        self.input.store(input, Ordering::Relaxed);
        self.output.store(output, Ordering::Relaxed);
    }
}

/// A created phytronIO as the controller poll reads it, an entry of C's
/// `IOs_`.
pub(crate) struct IoChannel {
    kind: IoType,
    card: u8,
    channel: u8,
    values: Arc<IoValues>,
    port: PortHandle,
    in_reason: usize,
    out_reason: usize,
}

impl IoChannel {
    /// C member `pollIO` with `bIsRequest`.
    pub(crate) fn requests(&self, mcc: bool) -> Vec<String> {
        let mut kind = self.kind;
        requests(&mut kind, mcc, self.card, self.channel)
    }

    /// C member `pollIO` without `bIsRequest` (phytronIOctrl.cpp:334-344):
    /// `replies` are this channel's, which post IN and OUT if they parse.
    pub(crate) fn parse(&self, mcc: bool, replies: &[String]) -> bool {
        let (mut kind, (mut input, mut output)) = (self.kind, self.values.get());
        let ok = parse(&mut kind, mcc, self.card, replies, &mut input, &mut output);
        self.values.set(input, output);
        if ok {
            let value = |reason, v| ParamSetValue::Value {
                reason,
                addr: 0,
                value: ParamValue::Int32(v),
            };
            let posted = vec![value(self.in_reason, input), value(self.out_reason, output)];
            let _ = self.port.set_params_and_notify_blocking(0, posted);
        }
        ok
    }
}

/// What [`probe`] read of a card channel: its kind and values.
pub struct Probed {
    kind: IoType,
    input: i32,
    output: i32,
}

/// C `createIOcontroller`'s read of the channel (phytronIOctrl.cpp:121-130):
/// card `card` channel `channel` of `ctrl`, with the `init` commands on the
/// same line, resolving the kind guessed from `analog`.
pub fn probe(
    ctrl: &PhytronController,
    analog: bool,
    card: u8,
    channel: u8,
    init: &str,
) -> Option<Probed> {
    let mcc = ctrl.ctrl_type() == CtrlType::Mcc;
    let mut kind = if analog {
        IoType::GuessA
    } else {
        IoType::GuessD
    };
    let mut commands = requests(&mut kind, mcc, card, channel);
    if !init.is_empty() {
        commands.push(init.to_owned());
    }
    let replies = ctrl.multi_command(commands, true, mcc).ok()?;
    let (mut input, mut output) = (0, 0);
    parse(&mut kind, mcc, card, &replies, &mut input, &mut output).then_some(Probed {
        kind,
        input,
        output,
    })
}

/// C `phytronIO`'s constructor (phytronIOctrl.cpp:47-98): the port `claim`
/// names for the channel `probed` read, which joins `controller`'s poll.
pub fn register(
    claim: PortClaim,
    controller: &Arc<Mutex<PhytronController>>,
    probed: Probed,
    card: u8,
    channel: u8,
    owner: &Arc<InterruptGate>,
) -> AsynResult<()> {
    let (mcc, type_name) = {
        let ctrl = lock(controller);
        (
            ctrl.ctrl_type() == CtrlType::Mcc,
            ctrl.type_name().to_owned(),
        )
    };
    let mut base = PortDriverBase::new(
        claim.name(),
        1,
        PortFlags {
            can_block: true,
            ..PortFlags::default()
        },
        InterfaceMask::INT32.or(InterfaceMask::DRV_USER),
        InterfaceMask::INT32,
    );
    let in_reason = base.create_param("IN", ParamType::Int32)?;
    base.set_int32_param(in_reason, 0, probed.input)?;
    let out_reason = base.create_param("OUT", ParamType::Int32)?;
    base.set_int32_param(out_reason, 0, probed.output)?;
    let mcc1 = mcc
        && type_name
            .get(..5)
            .is_some_and(|t| t.eq_ignore_ascii_case("MCC-1"));
    let dir_reason = if mcc1 {
        Some(base.create_param("DIR", ParamType::Int32)?)
    } else {
        None
    };
    let values = Arc::new(IoValues::default());
    values.set(probed.input, probed.output);
    let driver = PhytronIo {
        base,
        controller: controller.clone(),
        mcc,
        type_name,
        kind: probed.kind,
        card,
        channel,
        values: values.clone(),
        in_reason,
        out_reason,
        dir_reason,
    };
    let runtime = PortManager::global().register_port(claim, driver, owner)?;
    lock(controller).add_io(IoChannel {
        kind: probed.kind,
        card,
        channel,
        values,
        port: runtime.port_handle().clone(),
        in_reason,
        out_reason,
    });
    Ok(())
}

fn lock(controller: &Mutex<PhytronController>) -> MutexGuard<'_, PhytronController> {
    controller.lock().unwrap_or_else(|e| e.into_inner())
}

fn io_err(message: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.to_owned(),
    }
}

/// The phytronIO asyn port.
struct PhytronIo {
    base: PortDriverBase,
    controller: Arc<Mutex<PhytronController>>,
    mcc: bool,
    /// C `pCtrl_->sCtrlType_`, set before any channel exists.
    type_name: String,
    kind: IoType,
    card: u8,
    channel: u8,
    values: Arc<IoValues>,
    in_reason: usize,
    out_reason: usize,
    dir_reason: Option<usize>,
}

impl PhytronIo {
    /// The command of an OUT or DIR write of `v` (phytronIOctrl.cpp:183-220).
    fn command(&self, dir: bool, v: i32) -> Option<String> {
        match (self.mcc, self.kind) {
            (true, IoType::Do | IoType::Dio) => {
                let bits: String = (0..8)
                    .map(|b| if v & (1 << b) != 0 { '1' } else { '0' })
                    .collect();
                let cmd = if dir { "EAS" } else { "AG" };
                Some(format!("{cmd}{}S{bits}", self.channel))
            }
            (false, IoType::Do | IoType::Dio) => Some(format!("AG{}={v}", self.card)),
            (false, IoType::Ao | IoType::Aio) => {
                Some(format!("DA{}.{}={v}", self.card, self.channel))
            }
            _ => None,
        }
    }
}

impl PortDriver for PhytronIo {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `phytronIO::readInt32` (phytronIOctrl.cpp:151-170).
    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let (input, output) = self.values.get();
        if user.reason == self.in_reason {
            Ok(input)
        } else if user.reason == self.out_reason {
            Ok(output)
        } else {
            self.base.read_int32_param(user)
        }
    }

    /// C `phytronIO::writeInt32` (phytronIOctrl.cpp:172-235). C never sets
    /// `iDirReason_`, so its DIR write sends `AG` and sets the outputs, and
    /// its DIR read returns them; DIR sends `EAS` here and reads what was
    /// written.
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        if user.reason == self.in_reason {
            return Err(io_err("phytronIO::writeInt32: read-only parameter"));
        }
        let dir = Some(user.reason) == self.dir_reason;
        if dir || user.reason == self.out_reason {
            let cmd = self
                .command(dir, value)
                .ok_or_else(|| io_err("phytronIO::writeInt32: no output on this channel"))?;
            lock(&self.controller).command(&cmd)?;
            if !dir {
                self.values.output.store(value, Ordering::Relaxed);
            }
        }
        self.base.write_int32_param(user, value)
    }

    /// C `phytronIO::report` (phytronIOctrl.cpp:237-327).
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        self.asyn_port_driver_report(out, level);
        let (input, output) = self.values.get();
        let _ = writeln!(
            out,
            "{} {}: card={} channel={} in={input} out={output}",
            self.type_name,
            self.kind.name(),
            self.card,
            self.channel
        );
        if level < 1 {
            return;
        }
        let ctrl = lock(&self.controller);
        let reply = |cmd: &str| ctrl.send(cmd, true).ok();
        let escaped = |s: &str| print_escaped(s.as_bytes());
        let max = reply(if self.mcc { "IAR" } else { "IV" })
            .and_then(|r| parse_int32_prefix(&r))
            .filter(|&m| m >= 1)
            .unwrap_or(if self.mcc { 8 } else { 21 });
        if level == 1 {
            let _ = writeln!(out, "  controller: {}", self.type_name);
            if !self.mcc {
                for i in 1..=max {
                    if let Some(t) = reply(&format!("IM{i}")) {
                        let _ = writeln!(out, "  module {i}: {}", escaped(&t));
                    }
                }
            }
        } else if self.mcc {
            // C prints a failed reply's buffer; nothing here, as read_type.
            let ivr = reply("IVR").unwrap_or_default();
            let _ = writeln!(out, "  controller: {}  {}", self.type_name, escaped(&ivr));
            for i in 1..=max {
                let mut versions = String::new();
                for v in [reply(&format!("{i}VI")), reply(&format!("{i}VL"))]
                    .into_iter()
                    .flatten()
                {
                    if !versions.is_empty() {
                        versions.push_str(", ");
                    }
                    versions.push_str(&v);
                }
                if versions.is_empty() {
                    versions = "no version info".to_owned();
                }
                let _ = writeln!(out, "  axis {i}: {}", escaped(&versions));
            }
        } else {
            for i in 0..=max {
                let t = reply(&format!("IV{i}"));
                if i == 0 {
                    let t = t.unwrap_or_default();
                    let _ = writeln!(out, "  controller: {}", escaped(&t));
                } else if let Some(t) = t {
                    let _ = writeln!(out, "  module {i}: {}", escaped(&t));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|&r| r.to_owned()).collect()
    }

    #[test]
    fn a_guessed_card_resolves_by_the_requests_it_acks() {
        let cases = [
            (IoType::GuessD, ["\x063", "\x065"], IoType::Dio, (5, 3)),
            (IoType::GuessD, ["\x063", "\x15"], IoType::Do, (0, 3)),
            (IoType::GuessD, ["\x15", "\x065"], IoType::Di, (5, 0)),
            (
                IoType::GuessA,
                ["\x06100", "\x06200"],
                IoType::Aio,
                (200, 100),
            ),
            (IoType::GuessA, ["\x06100", "\x15"], IoType::Ao, (0, 100)),
            (IoType::GuessA, ["\x15", "\x06200"], IoType::Ai, (200, 0)),
        ];
        for (guess, replies, want, values) in cases {
            let mut kind = guess;
            let (mut i, mut o) = (0, 0);
            assert!(parse(&mut kind, false, 1, &s(&replies), &mut i, &mut o));
            assert_eq!((kind, (i, o)), (want, values), "{guess:?} {replies:?}");
        }
        let mut kind = IoType::GuessD;
        assert!(!parse(
            &mut kind,
            false,
            1,
            &s(&["\x15", "\x15"]),
            &mut 0,
            &mut 0
        ));
    }

    #[test]
    fn an_mcc_reads_its_digital_card_as_bit_strings() {
        let mut kind = IoType::GuessD;
        assert_eq!(requests(&mut kind, true, 1, 1), ["AG1R", "EG1R"]);
        assert_eq!(kind, IoType::Dio);
        let (mut i, mut o) = (0, 0);
        let replies = s(&["\x0610000001", "\x0601100000"]);
        assert!(parse(&mut kind, true, 1, &replies, &mut i, &mut o));
        assert_eq!((i, o), (6, 0x81));
        let mut kind = IoType::GuessA;
        assert_eq!(requests(&mut kind, true, 1, 2), ["AD2R"]);
        assert_eq!(kind, IoType::Ai);
        assert!(requests(&mut IoType::GuessD, true, 2, 1).is_empty());
    }

    #[test]
    fn a_reply_that_fails_half_way_keeps_the_first_value() {
        let (mut i, mut o) = (7, 7);
        let replies = s(&["\x0612", "\x06x"]);
        assert!(!parse(&mut IoType::Dio, false, 1, &replies, &mut i, &mut o));
        assert_eq!((i, o), (7, 12));
    }

    #[test]
    fn the_report_s_module_count_is_the_integer_a_reply_starts_with() {
        assert_eq!(parse_int32_prefix(" 12 modules"), Some(12));
        assert_eq!(parse_int32_prefix("0x1fz"), Some(31));
        assert_eq!(parse_int32_prefix("0xz"), Some(0));
        assert_eq!(parse_int32_prefix("019"), Some(1));
        assert_eq!(parse_int32_prefix("-3"), Some(-3));
        assert_eq!(parse_int32_prefix("MCC"), None);
        assert_eq!(parse_int32_prefix("4294967296"), None);
    }
}
