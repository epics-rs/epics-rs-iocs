//! `devDigitelPump.c` wire protocol: MPC, Digitel 500/1500, QPC.
//!
//! # EOS ownership
//!
//! `devDigitelPump.c` never calls `setInputEos`/`setOutputEos` — see the
//! comment block in `devDigitelPumpProcess`, which states the terminators are
//! "set in startup file". Output EOS is `\r` for every device; input EOS is
//! `\r` for MPC/QPC over serial and `\n\r` for the Digitels. This port keeps
//! that split: nothing here appends a terminator.
//!
//! # Response assembly
//!
//! `devDigitelPumpCallback` walks eleven read slots and `strcpy`s each stripped
//! reply into a 330-byte buffer at offset `30*i`. `readWrite_dg` then decodes
//! that buffer by absolute offset. [`ResponseBuf`] is the buffer, [`decode`] the
//! decoder.

use super::scan::{scan_char, scan_float, scan_int};
use super::{CBuf, cstr};

/// `DigitelPump_BUFFER_SIZE` — the assembled response buffer, and `recBuf`.
pub const BUFFER_SIZE: usize = 330;
/// `DigitelPump_SIZE` — the per-reply read length, and `sendBuf`'s capacity.
pub const READ_SIZE: usize = 50;
/// `DigitelPump_TIMEOUT`, seconds.
pub const TIMEOUT_SECS: f64 = 1.0;
/// `MAX_CONSEC_ERRORS` — above this the record takes a READ_ALARM.
pub const MAX_CONSEC_ERRORS: i32 = 2;
/// The read loop visits eleven slots.
pub const READ_SLOTS: usize = 11;

pub type ResponseBuf = CBuf<BUFFER_SIZE>;

/// `flgs` bits from `choiceDigitel.h`.
pub const MOD_DSPL: u32 = 0x0001;
pub const MOD_KLCK: u32 = 0x0002;
pub const MOD_MODS: u32 = 0x0004;
pub const MOD_BAKE: u32 = 0x0008;
pub const MOD_SETP: u32 = 0x0010;

/// `spfg` bits from `choiceDigitel.h`. Indexed `[setpoint - 1]`; note that
/// setpoint 4's bits are *not* a continuation of the 4-bit stride, because
/// `MOD_S3BS`/`MOD_S3TS` occupy 0x1000/0x2000.
pub const MOD_SPNS: [u32; 4] = [0x0001, 0x0010, 0x0100, 0x4000];
pub const MOD_SNHS: [u32; 4] = [0x0002, 0x0020, 0x0200, 0x8000];
pub const MOD_SNMS: [u32; 4] = [0x0004, 0x0040, 0x0400, 0x1_0000];
pub const MOD_SNVS: [u32; 4] = [0x0008, 0x0080, 0x0800, 0x2_0000];
pub const MOD_S3BS: u32 = 0x1000;
pub const MOD_S3TS: u32 = 0x2000;

/// `menu(digitelTYPE)` from `digitelRecord.dbd`. The numeric value is the
/// record's `TYPE` field and drives every branch in the C device support.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DevType {
    Mpc = 0,
    D500 = 1,
    D1500 = 2,
    Qpc = 3,
}

impl DevType {
    pub fn from_index(i: u16) -> Option<Self> {
        Some(match i {
            0 => Self::Mpc,
            1 => Self::D500,
            2 => Self::D1500,
            3 => Self::Qpc,
            _ => return None,
        })
    }

    /// C's `devType && devType != 3` — the Digitel 500/1500 branch.
    pub fn is_digitel(self) -> bool {
        matches!(self, Self::D500 | Self::D1500)
    }
}

/// `readCmdString[]` from `devDigitelPump.h`. Slots 0..=10 are MPC/QPC; the
/// Digitel commands live at 11..=15 and are reached as `readCmdString[11 + i]`.
const READ_CMD_MPC: [&str; READ_SLOTS] = [
    "0D", "0B", "0A", "0C", "11", "3C", "3C", "3C", "3C", "01", "02",
];
const READ_CMD_DIGITEL: [&str; 5] = ["RD", "RC", "RS1", "RS2", "RS3"];

/// `ctlCmdString[]` from `devDigitelPump.h`.
const CTL_CMD: [&str; 6] = ["25", "38", "37", "45", "44", "3D"];

/// `displayStr[]` from `devDigitelPump.h`, indexed by `menu(digitelDSPL)`.
const DISPLAY_STR: [&str; 3] = ["VOLT", "CUR", "PRES"];

/// Per-record configuration derived from `INP` and `TYPE`, mirroring
/// `devDigitelPumpPvt`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    pub dev: DevType,
    /// `cmdPrefix` — `"~ AA"` for MPC/QPC, empty for the Digitels.
    pub prefix: Vec<u8>,
    /// MPC pump 1-2, QPC setpoint 1-4; unused by the Digitels.
    pub pump_no: i32,
    /// MPC 3, QPC 4, Digitel = the station (number of setpoints, 0-3).
    pub no_spt: i32,
}

/// `errCount`'s initial value. C primes the Digitels with 3 so the first
/// process cycle issues the `SL3`/`SL4` reset pair that disables the
/// controller's unsolicited exception and timed reporting.
pub fn initial_err_count(dev: DevType) -> i32 {
    if dev.is_digitel() { 3 } else { 0 }
}

/// Build the `cmdPrefix`, pump number and setpoint count. Errors are the
/// `errlogPrintf` + `goto bad` cases, which disable the record in C.
pub fn configure(dev: DevType, address: i32, user_param: &str) -> Result<Config, String> {
    // C: `sscanf(userParam, "%d", &station)` on an uninitialised `station`.
    // A userParam that does not convert leaves it indeterminate; we start at 0,
    // which is in range for the Digitels and rejected for MPC/QPC.
    let station = scan_int(user_param.as_bytes(), None).map_or(0, |(v, _)| v as i32);

    let mut cfg = Config {
        dev,
        prefix: Vec::new(),
        pump_no: 0,
        no_spt: if dev == DevType::Qpc { 4 } else { 3 },
    };

    match dev {
        DevType::Mpc | DevType::Qpc => {
            if !(0..=255).contains(&address) {
                return Err(format!("address out of range {address}"));
            }
            let limit = if dev == DevType::Mpc { 2 } else { 4 };
            if !(1..=limit).contains(&station) {
                return Err(format!("{dev:?} station out of range {station}"));
            }
            cfg.prefix = format!("~ {address:02X}").into_bytes();
            cfg.pump_no = station;
        }
        DevType::D500 | DevType::D1500 => {
            if !(0..=3).contains(&station) {
                return Err(format!("Digitel station out of range {station}"));
            }
            cfg.no_spt = station;
        }
    }
    Ok(cfg)
}

/// `buildCommand`: `cmdPrefix + pvalue`, and for MPC/QPC a trailing space plus
/// the two-hex-digit checksum of every byte after the leading `~`.
pub fn build_command(cfg: &Config, pvalue: &[u8]) -> Vec<u8> {
    let mut buf = cfg.prefix.clone();
    buf.extend_from_slice(pvalue);
    if matches!(cfg.dev, DevType::Mpc | DevType::Qpc) {
        buf.push(b' ');
        let sum: u32 = buf[1..].iter().map(|&b| b as u32).sum();
        buf.extend_from_slice(format!("{:02X}", sum & 0xff).as_bytes());
    }
    // C writes into a 50-byte `sendBuf`; anything longer would have been
    // truncated there, so the same bound holds here.
    buf.truncate(READ_SIZE - 1);
    buf
}

/// C `printf("%.0e", v)`: one mantissa digit, `e`, sign, at least two exponent
/// digits. Rust's `{:.0e}` rounds the same way but writes a bare exponent.
fn fmt_e0(v: f64) -> String {
    reformat_exponent(&format!("{v:.0e}"), 'e')
}

/// C `printf("%.1E", v)`.
fn fmt_e1_upper(v: f64) -> String {
    reformat_exponent(&format!("{v:.1E}"), 'E')
}

fn reformat_exponent(s: &str, marker: char) -> String {
    let (mantissa, exp) = s.split_once(marker).expect("float always has an exponent");
    let (sign, digits) = match exp.strip_prefix('-') {
        Some(d) => ('-', d),
        None => ('+', exp.strip_prefix('+').unwrap_or(exp)),
    };
    format!("{mantissa}{marker}{sign}{digits:0>2}")
}

/// The record fields the control-command builder reads. Arrays are indexed
/// `[setpoint - 1]`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ControlFields {
    pub dspl: u16,
    pub mods: u16,
    pub klck: u16,
    pub baks: u16,
    pub bkin: u16,
    pub spfg: u32,
    /// `SPnS` — on-pressure setting.
    pub sps: [f64; 4],
    /// `SPnR` — on-pressure readback.
    pub spr: [f64; 4],
    /// `SnHS` — off-pressure (hysteresis) setting.
    pub shs: [f64; 4],
    /// `SnHR` — off-pressure readback.
    pub shr: [f64; 4],
    /// `SnMS` / `SnMR` — mode setting and readback.
    pub sms: [u16; 4],
    pub smr: [u16; 4],
    /// `SnVS` / `SnVR` — enable setting and readback.
    pub svs: [u16; 4],
    pub svr: [u16; 4],
}

/// The payload `readWrite_dg` hands to `buildCommand`, plus whether that path
/// also clears `spfg`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlCommand {
    pub payload: Vec<u8>,
    pub clear_spfg: bool,
}

/// `readWrite_dg`'s `pact == 0` control branch. `flgs` is the record's change
/// mask; only the first matching bit produces a command, exactly as in C.
///
/// An empty payload is not an error: C still calls `buildCommand`, which emits
/// the bare prefix (plus checksum for MPC/QPC).
pub fn control_command(cfg: &Config, flgs: u32, f: &ControlFields) -> ControlCommand {
    let digitel = cfg.dev.is_digitel();
    let mut clear_spfg = false;

    let payload: Vec<u8> = if flgs & MOD_DSPL != 0 {
        if digitel {
            format!("M{}", 3 + f.dspl).into_bytes()
        } else if cfg.dev == DevType::Mpc {
            format!(
                " {} {}, {}",
                CTL_CMD[0],
                cfg.pump_no,
                DISPLAY_STR[(f.dspl as usize).min(DISPLAY_STR.len() - 1)]
            )
            .into_bytes()
        } else {
            // The QPC has no display command.
            Vec::new()
        }
    } else if flgs & MOD_MODS != 0 {
        if digitel {
            format!("M{}", 1 + f.mods).into_bytes()
        } else {
            format!(" {} {}", CTL_CMD[1 + f.mods as usize], cfg.pump_no).into_bytes()
        }
    } else if flgs & MOD_KLCK != 0 {
        if digitel {
            format!("M{}", 8 + f.klck).into_bytes()
        } else if cfg.dev == DevType::Mpc {
            // Never sent to the QPC: locking its keypad drops remote mode and
            // it cannot be re-established.
            format!(" {} {}", CTL_CMD[3 + f.klck as usize], cfg.pump_no).into_bytes()
        } else {
            Vec::new()
        }
    } else if flgs & MOD_BAKE != 0 && f.bkin != 0 {
        if digitel {
            format!("M{}", 7 - f.baks).into_bytes()
        } else {
            Vec::new()
        }
    } else if flgs & MOD_SETP != 0 {
        clear_spfg = true;
        if digitel {
            digitel_setpoint_command(cfg, f)
        } else {
            mpc_setpoint_command(cfg, f)
        }
    } else {
        Vec::new()
    };

    ControlCommand {
        payload,
        clear_spfg,
    }
}

/// C's `switch (noSPT)` fall-through chain (its author calls it Pigeon's
/// device). Entering at level `noSPT`, each level tries its four `spfg` bits in
/// order and, when none match, falls into the next level down. Level 3 is
/// gated on `bkin` throughout.
fn digitel_setpoint_command(cfg: &Config, f: &ControlFields) -> Vec<u8> {
    for level in (1..=cfg.no_spt.min(3)).rev() {
        let n = level as usize;
        let i = n - 1;
        if level == 3 && f.bkin == 0 {
            // C's four `&& pr->bkin` guards all fail, so the `else` block runs
            // and control falls straight into level 2.
            continue;
        }
        // Fixes doc/upstream-c-defects.md #20: C's `S32` read `sp3s` (the
        // on-pressure) instead of `s3hs`, where the `S22`/`S12` analogues
        // correctly read `s2hs`/`s1hs`. The off-pressure command is now
        // symmetric across all three setpoints: `SnHS` sends `shs[i]`.
        if f.spfg & MOD_SPNS[i] != 0 {
            return snm_pressure(n, 1, f.sps[i]);
        } else if f.spfg & MOD_SNHS[i] != 0 {
            return snm_pressure(n, 2, f.shs[i]);
        } else if f.spfg & MOD_SNMS[i] != 0 {
            return format!("S{n}3{}0{}0", f.sms[i], 1 - f.svr[i]).into_bytes();
        } else if f.spfg & MOD_SNVS[i] != 0 {
            return format!("S{n}3{}0{}0", f.smr[i], 1 - f.svs[i]).into_bytes();
        }
    }
    Vec::new()
}

/// C: `sprintf(pvalue, "S%d%d%.0e", ...); pvalue[4] = pvalue[7]; pvalue[5] = 0;`
/// — "Snmxe-0y" collapsed to "Snmxy", keeping only the last exponent digit.
fn snm_pressure(n: usize, m: usize, value: f64) -> Vec<u8> {
    let mut p = format!("S{n}{m}{}", fmt_e0(value)).into_bytes();
    p[4] = p[7];
    p.truncate(5);
    p
}

/// The MPC/QPC `3D` setpoint command. Odd pump numbers drive odd setpoint
/// numbers on the MPC; the QPC has one setpoint per pump.
///
/// C leaves `t1`, `val1` and `val2` uninitialised when no `spfg` bit matches
/// (or when a bit matches but the device type excludes it). We start them at
/// zero, so `val1 == 0` and no command is emitted — see the crate docs.
fn mpc_setpoint_command(cfg: &Config, f: &ControlFields) -> Vec<u8> {
    let qpc = cfg.dev == DevType::Qpc;
    let (mut t1, mut val1, mut val2) = (0i32, 0.0f32, 0.0f32);

    // Fixes doc/upstream-c-defects.md #21: C's guard `v < 1e-4 || v > 1e-11`
    // is true for every finite non-negative v (`||` where `&&` was meant), so
    // it never rejected an out-of-range pressure. The intended check accepts
    // only pressures within the device's [1e-11, 1e-4] Torr span; a value
    // outside it leaves `val1 == 0` and suppresses the command.
    let in_range = |v: f64| (1e-11..=1e-4).contains(&v);

    if f.spfg & MOD_SPNS[0] != 0 {
        t1 = cfg.pump_no;
        if in_range(f.sps[0]) {
            val1 = f.sps[0] as f32;
            val2 = f.shr[0] as f32;
        }
    } else if f.spfg & MOD_SNHS[0] != 0 {
        t1 = cfg.pump_no;
        if in_range(f.shs[0]) {
            val1 = f.spr[0] as f32;
            val2 = f.shs[0] as f32;
        }
    } else if f.spfg & MOD_SPNS[1] != 0 {
        t1 = if qpc { cfg.pump_no } else { 2 + cfg.pump_no };
        if in_range(f.sps[1]) {
            val1 = f.sps[1] as f32;
            val2 = f.shr[1] as f32;
        }
    } else if f.spfg & MOD_SNHS[1] != 0 {
        t1 = if qpc { cfg.pump_no } else { 2 + cfg.pump_no };
        if in_range(f.shs[1]) {
            val1 = f.spr[1] as f32;
            val2 = f.shs[1] as f32;
        }
    } else if qpc && f.spfg & MOD_SPNS[2] != 0 {
        t1 = cfg.pump_no;
        if in_range(f.sps[2]) {
            val1 = f.sps[2] as f32;
            val2 = f.shr[2] as f32;
        }
    } else if qpc && f.spfg & MOD_SNHS[2] != 0 {
        t1 = cfg.pump_no;
        if in_range(f.shs[2]) {
            val1 = f.spr[2] as f32;
            val2 = f.shs[2] as f32;
        }
    } else if qpc && f.spfg & MOD_SPNS[3] != 0 {
        t1 = cfg.pump_no;
        if in_range(f.sps[3]) {
            val1 = f.sps[3] as f32;
            val2 = f.shr[3] as f32;
        }
    } else if qpc && f.spfg & MOD_SNHS[3] != 0 {
        t1 = cfg.pump_no;
        if in_range(f.shs[3]) {
            val1 = f.spr[3] as f32;
            val2 = f.shs[3] as f32;
        }
    }

    if val1 == 0.0 {
        return Vec::new();
    }
    format!(
        " {} {},{},{:>7},{:>7}",
        CTL_CMD[5],
        t1,
        cfg.pump_no,
        fmt_e1_upper(val1 as f64),
        fmt_e1_upper(val2 as f64)
    )
    .into_bytes()
}

/// What the read loop does with slot `i`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadSlot {
    /// Build and send this payload, then store the stripped reply at `30*i`.
    Send(Vec<u8>),
    /// C performs no I/O for this slot but still stores `pstartdata`, which
    /// still points into the previous slot's `readBuffer` — the previous
    /// reply is duplicated into this slot. QPC slots 6, 7 and 8.
    ReuseLast,
    /// C `continue`s: the slot is not visited at all. Digitel slots beyond
    /// `noSPT + 1`.
    Skip,
}

/// The `i`th read slot, `0 <= i < READ_SLOTS`.
pub fn read_slot(cfg: &Config, i: usize) -> ReadSlot {
    if cfg.dev.is_digitel() {
        if i as i32 > cfg.no_spt + 1 {
            return ReadSlot::Skip;
        }
        return ReadSlot::Send(READ_CMD_DIGITEL[i].as_bytes().to_vec());
    }

    match (cfg.dev, i) {
        (_, 0..=4) => ReadSlot::Send(format!(" {} {}", READ_CMD_MPC[i], cfg.pump_no).into_bytes()),

        // The QPC has one setpoint per pump, so only slot 5 is issued. C's send
        // guard `(i < 6) || (i > 8)` then skips 6..=8 outright.
        (DevType::Qpc, 5) => {
            ReadSlot::Send(format!(" {} {}", READ_CMD_MPC[5], cfg.pump_no).into_bytes())
        }
        (DevType::Qpc, 6..=8) => ReadSlot::ReuseLast,

        // MPC: odd pumps read odd setpoints. C guards the `sprintf` with
        // `if (i < 8)` but not the send, so slot 8 re-sends slot 7's command
        // and `sp4r` ends up mirroring `sp3r`. Ported as written.
        (DevType::Mpc, 5..=7) => ReadSlot::Send(
            format!(" {} {}", READ_CMD_MPC[i], (i as i32 - 5) * 2 + cfg.pump_no).into_bytes(),
        ),
        (DevType::Mpc, 8) => {
            ReadSlot::Send(format!(" {} {}", READ_CMD_MPC[7], 4 + cfg.pump_no).into_bytes())
        }

        (_, 9..=10) => ReadSlot::Send(format!(" {}", READ_CMD_MPC[i]).into_bytes()),
        _ => ReadSlot::Skip,
    }
}

/// The Digitel reset pair, sent verbatim (no prefix, no checksum) before the
/// read loop whenever `errCount != 0`. They disable the controller's
/// unsolicited exception and timed reporting.
pub const RESET_COMMANDS: [&[u8]; 2] = [b"SL3", b"SL4"];

/// A reply the device flagged as an error, or one too short to be a reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplyError;

/// Validate and strip one read reply, returning the payload that gets `strcpy`d
/// into the response buffer.
///
/// * Digitel — `"ERROR"` shows up at index 0, 4 or 5 depending on whether the
///   command echo survived. Otherwise the echo is skipped: 4 bytes for `RD`/`RC`
///   (`"RD\r\n"`), 5 for `RSx`. A reply that does not start with `'R'` came from
///   a BITBUS bridge that already stripped the echo.
/// * MPC/QPC — `"OK"` must sit at index 3. A reply shorter than 12 bytes is the
///   bare acknowledgement and is recorded as `"OK"`; otherwise the nine-byte
///   `"~ AA OK X"` header is stripped.
pub fn strip_read_reply(dev: DevType, i: usize, reply: &[u8]) -> Result<Vec<u8>, ReplyError> {
    if reply.is_empty() {
        return Err(ReplyError);
    }
    // C indexes a stack `readBuffer` past `nread`, reading whatever the previous
    // reply left there. We read NUL past the end, which never matches 'E'/'O'/'K'.
    let at = |n: usize| reply.get(n).copied().unwrap_or(0);

    if dev.is_digitel() {
        if at(4) == b'E' || at(5) == b'E' || at(0) == b'E' {
            return Err(ReplyError);
        }
        let skip = if at(0) == b'R' {
            if i < 2 { 4 } else { 5 }
        } else {
            0
        };
        return Ok(reply.get(skip..).unwrap_or(&[]).to_vec());
    }

    if at(3) != b'O' || at(4) != b'K' {
        return Err(ReplyError);
    }
    if reply.len() < 12 {
        return Ok(b"OK".to_vec());
    }
    Ok(reply[9..].to_vec())
}

/// Validate a control-command reply. The Digitels answer `Mx`/`Sx`; MPC and QPC
/// answer with `"OK"` at index 3.
pub fn check_control_reply(dev: DevType, reply: &[u8]) -> Result<(), ReplyError> {
    if reply.is_empty() {
        return Err(ReplyError);
    }
    let at = |n: usize| reply.get(n).copied().unwrap_or(0);
    if dev.is_digitel() {
        if at(0) != b'M' && at(0) != b'S' {
            return Err(ReplyError);
        }
    } else if at(3) != b'O' || at(4) != b'K' {
        return Err(ReplyError);
    }
    Ok(())
}

/// C's `char pvalue[30]` decode scratch.
///
/// `strncpy(dst, src, n)` copies at most `n` bytes, stops at the source NUL,
/// and NUL-pads the rest of `dst[..n]` — but leaves `dst[n..]` alone. The
/// decoder relies on that: after `strncpy(pvalue, &recBuf[139], 2)` the bytes
/// past index 1 still hold the previous copy's tail, and the following
/// `sscanf(pvalue, "%lf", ...)` may run into them. Modelling the whole buffer
/// keeps that behaviour instead of guessing at it.
struct Scratch([u8; 30]);

impl Scratch {
    fn new() -> Self {
        Self([0; 30])
    }

    fn strncpy(&mut self, src: &[u8], n: usize) {
        let n = n.min(self.0.len());
        let src = cstr(src);
        let m = src.len().min(n);
        self.0[..m].copy_from_slice(&src[..m]);
        self.0[m..n].fill(0);
    }

    /// The buffer as C's string functions see it.
    fn s(&self) -> &[u8] {
        cstr(&self.0)
    }

    /// A raw byte, as `pvalue[14]` reads it — past the NUL if need be.
    fn at(&self, i: usize) -> u8 {
        self.0[i]
    }
}

/// Everything `readWrite_dg` writes into the record from one response buffer.
/// Arrays are indexed `[setpoint - 1]`.
#[derive(Debug, Clone, PartialEq)]
pub struct Readings {
    pub val: f64,
    pub lval: f64,
    pub volt: f64,
    pub crnt: f64,
    pub tonl: i64,
    pub modr: u16,
    pub cmor: u16,
    pub bakr: u16,
    pub set: [u16; 4],
    pub accw: f64,
    pub acci: f64,
    pub cool: i64,
    pub ptyp: u16,
    pub bkin: u16,
    /// `SPnR` / `SnHR` — on- and off-pressure readbacks.
    pub spr: [f64; 4],
    pub shr: [f64; 4],
    /// `SnMR` / `SnVR` — mode and enable readbacks.
    pub smr: [u16; 4],
    pub svr: [u16; 4],
    pub s3br: u16,
    pub s3tr: f64,
    /// The exact bytes `strncpy` would write into `MODL` / `VERS`, or `None`
    /// for the Digitels, which report neither.
    pub modl: Option<Vec<u8>>,
    pub vers: Option<Vec<u8>>,
}

impl Default for Readings {
    fn default() -> Self {
        Self {
            val: 9.9e9,
            lval: 0.0,
            volt: 0.0,
            crnt: 0.0,
            tonl: 0,
            modr: 0,
            cmor: 0,
            bakr: 0,
            set: [0; 4],
            accw: 0.0,
            acci: 0.0,
            cool: 0,
            ptyp: 0,
            bkin: 0,
            spr: [0.0; 4],
            shr: [0.0; 4],
            smr: [0; 4],
            svr: [1; 4],
            s3br: 0,
            s3tr: 0.0,
            modl: None,
            vers: None,
        }
    }
}

/// C `sscanf(s, "%d %d:%d %lfV%leI", ...)`. Writes only the targets that
/// converted, and returns how many did.
fn scan_status(s: &[u8], t: &mut [i64; 3], volt: &mut f64, crnt: &mut f64) -> usize {
    let mut p = s;
    macro_rules! int {
        ($slot:expr, $n:expr) => {
            match scan_int(p, None) {
                Some((v, rest)) => {
                    $slot = v;
                    p = rest;
                }
                None => return $n,
            }
        };
    }
    macro_rules! lit {
        ($c:expr, $n:expr) => {
            match scan_char(p) {
                Some(($c, rest)) => p = rest,
                _ => return $n,
            }
        };
    }
    int!(t[0], 0);
    int!(t[1], 1);
    lit!(b':', 2);
    int!(t[2], 2);
    match scan_float(p, None) {
        Some((v, rest)) => {
            *volt = v;
            p = rest;
        }
        None => return 3,
    }
    lit!(b'V', 4);
    match scan_float(p, None) {
        Some((v, _)) => *crnt = v,
        None => return 4,
    }
    5
}

/// C `sscanf(s, "%dP %dI %dC %dS", ...)`.
fn scan_accumulated(s: &[u8], out: &mut [i64; 4]) -> usize {
    let mut p = s;
    for (n, marker) in (*b"PICS").into_iter().enumerate() {
        match scan_int(p, None) {
            Some((v, rest)) => {
                out[n] = v;
                p = rest;
            }
            None => return n,
        }
        match scan_char(p) {
            Some((c, rest)) if c == marker => p = rest,
            _ => return n + 1,
        }
    }
    4
}

/// C `sscanf(s, "%le %le ", ...)`.
fn scan_two_floats(s: &[u8], a: &mut f64, b: &mut f64) -> usize {
    let mut p = s;
    match scan_float(p, None) {
        Some((v, rest)) => {
            *a = v;
            p = rest;
        }
        None => return 0,
    }
    match scan_float(p, None) {
        Some((v, _)) => *b = v,
        None => return 1,
    }
    2
}

/// C `sscanf(s, "%d,%d,%e,%e,%d", ...)` — note `%e` targets are `float`.
fn scan_setpoint(s: &[u8], t: &mut [i64; 3], val1: &mut f32, val2: &mut f32) -> usize {
    let mut p = s;
    macro_rules! comma {
        ($n:expr) => {
            match scan_char(p) {
                Some((b',', rest)) => p = rest,
                _ => return $n,
            }
        };
    }
    match scan_int(p, None) {
        Some((v, rest)) => {
            t[0] = v;
            p = rest;
        }
        None => return 0,
    }
    comma!(1);
    match scan_int(p, None) {
        Some((v, rest)) => {
            t[1] = v;
            p = rest;
        }
        None => return 1,
    }
    comma!(2);
    match scan_float(p, None) {
        Some((v, rest)) => {
            *val1 = v as f32;
            p = rest;
        }
        None => return 2,
    }
    comma!(3);
    match scan_float(p, None) {
        Some((v, rest)) => {
            *val2 = v as f32;
            p = rest;
        }
        None => return 3,
    }
    comma!(4);
    match scan_int(p, None) {
        Some((v, _)) => t[2] = v,
        None => return 4,
    }
    5
}

/// Decode a response buffer.
///
/// `prev` supplies the record's current values for every field C's `sscanf`
/// calls write directly (`volt`, `crnt`, `spNr`, `sNhr`, `s3tr`): a conversion
/// that fails leaves the record field untouched, so a short or missing reply
/// repeats the previous cycle's reading rather than zeroing it.
pub fn decode(cfg: &Config, buf: &ResponseBuf, prev: &Readings) -> Readings {
    // C's "set to safe value initially" block, plus the fields it never resets.
    let mut r = Readings {
        val: 9.9e9,
        volt: prev.volt,
        crnt: prev.crnt,
        tonl: prev.tonl,
        accw: prev.accw,
        acci: prev.acci,
        cool: prev.cool,
        bkin: prev.bkin,
        spr: prev.spr,
        shr: prev.shr,
        s3tr: prev.s3tr,
        ..Readings::default()
    };

    let pv = &mut Scratch::new();
    // C's function-scope conversion targets. `t1`/`t2`/`t3`/`pumpType` are
    // uninitialised there; we start at zero, which the pump-size clamp maps to
    // the smallest pump. See the crate docs.
    let mut t = [0i64; 3];
    let mut pump_type = 0i64;
    let (mut val1, mut val2) = (0.0f32, 0.0f32);

    if cfg.dev.is_digitel() {
        pv.strncpy(buf.slice(0, 22), 22);
        scan_status(pv.s(), &mut t, &mut r.volt, &mut r.crnt);
        r.tonl = t[0] * 1440 + t[1] * 60 + t[2];

        if buf.at(23) == b'H' {
            r.modr = 1;
        }
        if buf.at(24) == b'C' {
            r.cmor = 1;
        }
        if buf.at(25) == b'B' {
            r.bakr = 1;
        }
        if buf.at(26) == b'1' && cfg.no_spt >= 1 {
            r.set[0] = 1;
        }
        if buf.at(27) == b'2' && cfg.no_spt >= 2 {
            r.set[1] = 1;
        }
        if buf.at(28) == b'3' && cfg.no_spt == 3 {
            r.set[2] = 1;
        }

        pv.strncpy(buf.slice(30, 13), 13);
        let mut acc = [0i64; 4];
        acc[..3].copy_from_slice(&t);
        acc[3] = pump_type;
        scan_accumulated(pv.s(), &mut acc);
        t.copy_from_slice(&acc[..3]);
        pump_type = acc[3];
        r.accw = t[0] as f64 * 0.444;
        r.acci = t[1] as f64 * 1.11;
        r.cool = t[2];

        if !(1..=32).contains(&pump_type) {
            pump_type = 1;
        }
        if cfg.dev == DevType::D1500 {
            pump_type *= 4;
        }
        r.ptyp = match pump_type {
            1 => 0,
            2 => 1,
            4 => 2,
            8 => 3,
            16 => 4,
            32 => 5,
            // C's switch has no default: `ptyp` keeps the 0 it was reset to.
            _ => 0,
        };

        if r.modr == 1 && r.volt != 0.0 {
            r.val = 0.005 * r.crnt / pump_type as f64;
        }

        // C's `switch (noSPT)` falls through from 3 to 2 to 1.
        if cfg.no_spt >= 3 {
            pv.strncpy(buf.slice(120, 18), 18);
            if pv.at(0) == b'E' && pv.at(1) == b'R' {
                r.bkin = 0;
            } else {
                r.bkin = 1;
                scan_two_floats(pv.s(), &mut r.spr[2], &mut r.shr[2]);
                // C writes `s2mr`/`s2vr` here where it means `s3mr`/`s3vr`;
                // the case-2 fall-through then overwrites both. Ported as
                // written — see the crate docs.
                if pv.at(14) == b'1' {
                    r.smr[1] = 1;
                }
                if pv.at(16) == b'1' {
                    r.svr[1] = 0;
                }
                if pv.at(17) == b'1' {
                    r.s3br = 1;
                }
                pv.strncpy(buf.slice(139, 2), 2);
                if let Some((v, _)) = scan_float(pv.s(), None) {
                    r.s3tr = v;
                }
            }
        }
        if cfg.no_spt >= 2 {
            pv.strncpy(buf.slice(90, 18), 18);
            scan_two_floats(pv.s(), &mut r.spr[1], &mut r.shr[1]);
            if pv.at(14) == b'1' {
                r.smr[1] = 1;
            }
            if pv.at(16) == b'1' {
                r.svr[1] = 0;
            }
        }
        if cfg.no_spt >= 1 {
            pv.strncpy(buf.slice(60, 18), 18);
            scan_two_floats(pv.s(), &mut r.spr[0], &mut r.shr[0]);
            if pv.at(14) == b'1' {
                r.smr[0] = 1;
            }
            if pv.at(16) == b'1' {
                r.svr[0] = 0;
            }
        }
    } else {
        pv.strncpy(buf.slice(0, 20), 20);
        if pv.s().starts_with(b"RUNNING") {
            r.modr = 1;
        } else if pv.s().starts_with(b"COOL DOWN") {
            r.cmor = 1;
        }

        pv.strncpy(buf.slice(30, 8), 8);
        if let Some((v, _)) = scan_float(pv.s(), None) {
            val1 = v as f32;
        }
        r.val = val1 as f64;

        pv.strncpy(buf.slice(60, 7), 7);
        if let Some((v, _)) = scan_float(pv.s(), None) {
            val2 = v as f32;
        }
        r.crnt = val2 as f64;

        pv.strncpy(buf.slice(90, 4), 4);
        if let Some((v, _)) = scan_int(pv.s(), None) {
            t[0] = v;
        }
        r.volt = t[0] as f64;

        if r.volt < 1000.0 && r.crnt < 1e-6 {
            r.val = 9.9e9;
        }

        pv.strncpy(buf.slice(120, 4), 4);
        if let Some((v, _)) = scan_int(pv.s(), None) {
            pump_type = v;
        }
        r.ptyp = match pump_type {
            ..45 => 0,
            45..75 => 1,
            75..170 => 2,
            170..300 => 3,
            300..500 => 4,
            _ => 5,
        };

        for n in 0..4 {
            pv.strncpy(buf.slice(150 + 30 * n, 25), 25);
            scan_setpoint(pv.s(), &mut t, &mut val1, &mut val2);
            r.spr[n] = val1 as f64;
            r.shr[n] = val2 as f64;
            r.set[n] = t[2] as u16;
        }

        let mut modl = [0u8; 4];
        let src = cstr(buf.slice(278, 4));
        modl[..src.len()].copy_from_slice(src);
        r.modl = Some(modl.to_vec());

        let n = if cfg.dev == DevType::Qpc { 5 } else { 8 };
        let mut vers = vec![0u8; n];
        let src = cstr(buf.slice(318, n));
        vers[..src.len()].copy_from_slice(src);
        r.vers = Some(vers);
    }

    r.lval = r.val.log10();
    if r.val < 1e-12 {
        r.lval = -12.0;
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    fn buf_from(pairs: &[(usize, &[u8])]) -> ResponseBuf {
        let mut b = ResponseBuf::default();
        for (off, s) in pairs {
            b.strcpy_at(*off, s);
        }
        b
    }

    fn mpc(pump: i32) -> Config {
        configure(DevType::Mpc, 5, &pump.to_string()).unwrap()
    }

    fn qpc(pump: i32) -> Config {
        configure(DevType::Qpc, 5, &pump.to_string()).unwrap()
    }

    fn digitel(spt: i32) -> Config {
        configure(DevType::D500, 0, &spt.to_string()).unwrap()
    }

    // ---- configuration ----------------------------------------------------

    #[test]
    fn mpc_prefix_is_tilde_space_two_hex_digits() {
        assert_eq!(mpc(1).prefix, b"~ 05");
        assert_eq!(configure(DevType::Mpc, 255, "2").unwrap().prefix, b"~ FF");
        assert!(configure(DevType::Mpc, 256, "1").is_err());
    }

    #[test]
    fn mpc_allows_two_pumps_and_the_qpc_four() {
        assert_eq!(mpc(2).pump_no, 2);
        assert!(configure(DevType::Mpc, 5, "3").is_err());
        assert_eq!(qpc(4).pump_no, 4);
        assert!(configure(DevType::Qpc, 5, "5").is_err());
        assert!(configure(DevType::Qpc, 5, "0").is_err());
    }

    #[test]
    fn setpoint_count_defaults_to_three_for_mpc_and_four_for_qpc() {
        assert_eq!(mpc(1).no_spt, 3);
        assert_eq!(qpc(1).no_spt, 4);
    }

    #[test]
    fn digitel_has_no_prefix_and_takes_its_setpoint_count_from_the_station() {
        let c = digitel(3);
        assert_eq!(c.prefix, b"");
        assert_eq!(c.no_spt, 3);
        assert_eq!(digitel(0).no_spt, 0);
        assert!(configure(DevType::D1500, 0, "4").is_err());
    }

    #[test]
    fn only_the_digitels_start_with_a_primed_error_count() {
        assert_eq!(initial_err_count(DevType::D500), 3);
        assert_eq!(initial_err_count(DevType::D1500), 3);
        assert_eq!(initial_err_count(DevType::Mpc), 0);
        assert_eq!(initial_err_count(DevType::Qpc), 0);
    }

    // ---- command framing --------------------------------------------------

    #[test]
    fn mpc_command_gets_a_trailing_space_and_a_checksum() {
        // Checksum spans every byte after the '~', including the trailing space.
        let sent = build_command(&mpc(1), b" 0D 1");
        assert_eq!(sent, b"~ 05 0D 1 8A");
        let sum: u32 = b" 05 0D 1 ".iter().map(|&b| b as u32).sum();
        assert_eq!(
            format!("{:02X}", sum & 0xff).as_bytes(),
            &sent[sent.len() - 2..]
        );
    }

    #[test]
    fn qpc_commands_are_always_at_least_ten_characters() {
        // devDigitelPumpProcess refuses to transmit a shorter QPC command.
        for i in 0..READ_SLOTS {
            if let ReadSlot::Send(p) = read_slot(&qpc(1), i) {
                assert!(build_command(&qpc(1), &p).len() >= 10, "slot {i}");
            }
        }
    }

    #[test]
    fn digitel_command_carries_neither_prefix_nor_checksum() {
        assert_eq!(build_command(&digitel(2), b"M4"), b"M4");
    }

    #[test]
    fn checksum_masks_to_one_byte() {
        // A long payload pushes the sum past 0xff.
        let sent = build_command(&mpc(1), b" 3D 1,1,1.0E-06,1.0E-05");
        let sum: u32 = sent[1..sent.len() - 2].iter().map(|&b| b as u32).sum();
        assert_eq!(
            std::str::from_utf8(&sent[sent.len() - 2..]).unwrap(),
            format!("{:02X}", sum & 0xff)
        );
    }

    // ---- %.0e / %.1E ------------------------------------------------------

    #[test]
    fn exponent_formats_match_c_printf() {
        assert_eq!(fmt_e0(1e-6), "1e-06");
        assert_eq!(fmt_e0(1e-11), "1e-11");
        assert_eq!(fmt_e0(0.0), "0e+00");
        assert_eq!(fmt_e0(1e5), "1e+05");
        assert_eq!(fmt_e1_upper(1e-6), "1.0E-06");
        assert_eq!(fmt_e1_upper(0.0), "0.0E+00");
        assert_eq!(fmt_e1_upper(1.25e-5), "1.3E-05");
    }

    // ---- control commands: Digitel ----------------------------------------

    #[test]
    fn digitel_display_mode_keypad_and_bakeout_commands() {
        let c = digitel(2);
        let f = ControlFields {
            dspl: 2,
            mods: 1,
            klck: 1,
            baks: 1,
            bkin: 1,
            ..Default::default()
        };
        assert_eq!(control_command(&c, MOD_DSPL, &f).payload, b"M5");
        assert_eq!(control_command(&c, MOD_MODS, &f).payload, b"M2");
        assert_eq!(control_command(&c, MOD_KLCK, &f).payload, b"M9");
        assert_eq!(control_command(&c, MOD_BAKE, &f).payload, b"M6");
    }

    #[test]
    fn digitel_bakeout_command_needs_the_bakeout_option_installed() {
        let c = digitel(2);
        let f = ControlFields {
            bkin: 0,
            ..Default::default()
        };
        // MOD_BAKE without BKIN falls through to the (unset) MOD_SETP arm.
        let cmd = control_command(&c, MOD_BAKE, &f);
        assert_eq!(cmd.payload, b"");
        assert!(!cmd.clear_spfg);
    }

    #[test]
    fn digitel_display_precedes_mode_which_precedes_keypad() {
        let c = digitel(2);
        let f = ControlFields::default();
        let all = MOD_DSPL | MOD_MODS | MOD_KLCK;
        assert_eq!(control_command(&c, all, &f).payload, b"M3");
        assert_eq!(control_command(&c, MOD_MODS | MOD_KLCK, &f).payload, b"M1");
    }

    #[test]
    fn digitel_setpoint_pressure_collapses_the_exponent_to_one_digit() {
        let c = digitel(2);
        let mut f = ControlFields {
            spfg: MOD_SPNS[1],
            ..Default::default()
        };
        f.sps[1] = 1e-6;
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S2116");
        f.spfg = MOD_SNHS[1];
        f.shs[1] = 1e-11;
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S2211");
    }

    #[test]
    fn digitel_setpoint_mode_and_enable_commands() {
        let c = digitel(1);
        let mut f = ControlFields {
            spfg: MOD_SNMS[0],
            ..Default::default()
        };
        f.sms[0] = 1;
        f.svr[0] = 1;
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S131000");
        f.spfg = MOD_SNVS[0];
        f.smr[0] = 0;
        f.svs[0] = 0;
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S130010");
    }

    #[test]
    fn digitel_setpoint_three_needs_bakeout_and_otherwise_falls_to_setpoint_two() {
        let c = digitel(3);
        let mut f = ControlFields {
            spfg: MOD_SPNS[2] | MOD_SPNS[1],
            bkin: 0,
            ..Default::default()
        };
        f.sps[2] = 1e-6;
        f.sps[1] = 1e-8;
        // No bakeout option: level 3 is skipped entirely, level 2 answers.
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S2118");
        f.bkin = 1;
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S3116");
    }

    #[test]
    fn digitel_setpoint_level_falls_through_when_no_flag_matches() {
        let c = digitel(3);
        let mut f = ControlFields {
            spfg: MOD_SPNS[0],
            bkin: 1,
            ..Default::default()
        };
        f.sps[0] = 1e-9;
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S1119");
    }

    #[test]
    fn digitel_off_pressure_for_setpoint_three_reads_s3hs() {
        // Regression for doc/upstream-c-defects.md #20: `S32` sends the
        // setpoint-3 off-pressure `s3hs` (`shs[2]`), symmetric with S22/S12.
        // The on-pressure `sp3s` (`sps[2]`) is a distinct value here to prove
        // it is no longer read; pre-fix this produced "S3217".
        let c = digitel(3);
        let mut f = ControlFields {
            spfg: MOD_SNHS[2],
            bkin: 1,
            ..Default::default()
        };
        f.sps[2] = 1e-7; // on-pressure — must NOT be read
        f.shs[2] = 1e-4; // off-pressure — the value S32 carries
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S3214");
    }

    #[test]
    fn digitel_off_pressure_is_symmetric_across_all_three_setpoints() {
        // Regression for doc/upstream-c-defects.md #20: `SnHS` reads `shs[i]`
        // for every level. Each setpoint's on-pressure is set to a distinct,
        // never-read value to prove only the off-pressure drives the command.
        let c = digitel(3);
        let mut f = ControlFields {
            bkin: 1,
            ..Default::default()
        };
        f.sps = [1e-7, 1e-7, 1e-7, 0.0];
        f.shs = [1e-9, 1e-8, 1e-4, 0.0];

        f.spfg = MOD_SNHS[0];
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S1219");
        f.spfg = MOD_SNHS[1];
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S2218");
        f.spfg = MOD_SNHS[2];
        assert_eq!(control_command(&c, MOD_SETP, &f).payload, b"S3214");
    }

    #[test]
    fn digitel_with_no_setpoints_emits_nothing() {
        let c = digitel(0);
        let f = ControlFields {
            spfg: MOD_SPNS[0],
            ..Default::default()
        };
        let cmd = control_command(&c, MOD_SETP, &f);
        assert_eq!(cmd.payload, b"");
        assert!(cmd.clear_spfg);
    }

    // ---- control commands: MPC / QPC --------------------------------------

    #[test]
    fn mpc_display_command_names_the_pump_and_the_parameter() {
        let f = ControlFields {
            dspl: 2,
            ..Default::default()
        };
        assert_eq!(
            control_command(&mpc(2), MOD_DSPL, &f).payload,
            b" 25 2, PRES"
        );
    }

    #[test]
    fn mpc_mode_and_keypad_commands_pick_their_opcode_from_the_setting() {
        let mut f = ControlFields::default();
        assert_eq!(control_command(&mpc(1), MOD_MODS, &f).payload, b" 38 1");
        f.mods = 1;
        assert_eq!(control_command(&mpc(1), MOD_MODS, &f).payload, b" 37 1");
        assert_eq!(control_command(&mpc(1), MOD_KLCK, &f).payload, b" 45 1");
        f.klck = 1;
        assert_eq!(control_command(&mpc(1), MOD_KLCK, &f).payload, b" 44 1");
    }

    #[test]
    fn qpc_never_receives_display_keypad_or_bakeout_commands() {
        let f = ControlFields {
            dspl: 1,
            klck: 1,
            bkin: 1,
            ..Default::default()
        };
        assert_eq!(control_command(&qpc(1), MOD_DSPL, &f).payload, b"");
        assert_eq!(control_command(&qpc(1), MOD_KLCK, &f).payload, b"");
        assert_eq!(control_command(&qpc(1), MOD_BAKE, &f).payload, b"");
        // Mode selection is the one control the QPC does accept.
        assert_eq!(control_command(&qpc(3), MOD_MODS, &f).payload, b" 38 3");
    }

    #[test]
    fn mpc_setpoint_command_sends_both_pressures() {
        let mut f = ControlFields {
            spfg: MOD_SPNS[0],
            ..Default::default()
        };
        f.sps[0] = 1e-6;
        f.shr[0] = 1.2e-6;
        let cmd = control_command(&mpc(1), MOD_SETP, &f);
        assert_eq!(cmd.payload, b" 3D 1,1,1.0E-06,1.2E-06");
        assert!(cmd.clear_spfg);
    }

    #[test]
    fn mpc_off_pressure_change_resends_the_on_pressure_readback() {
        let mut f = ControlFields {
            spfg: MOD_SNHS[0],
            ..Default::default()
        };
        f.spr[0] = 1e-6;
        f.shs[0] = 2e-6;
        assert_eq!(
            control_command(&mpc(1), MOD_SETP, &f).payload,
            b" 3D 1,1,1.0E-06,2.0E-06"
        );
    }

    #[test]
    fn mpc_second_setpoint_offsets_the_setpoint_number_by_two() {
        let mut f = ControlFields {
            spfg: MOD_SPNS[1],
            ..Default::default()
        };
        f.sps[1] = 1e-7;
        f.shr[1] = 2e-7;
        assert_eq!(
            control_command(&mpc(2), MOD_SETP, &f).payload,
            b" 3D 4,2,1.0E-07,2.0E-07"
        );
    }

    #[test]
    fn qpc_uses_the_pump_number_as_the_setpoint_number_for_every_setpoint() {
        let mut f = ControlFields {
            spfg: MOD_SPNS[1],
            ..Default::default()
        };
        f.sps[1] = 1e-7;
        f.shr[1] = 2e-7;
        assert_eq!(
            control_command(&qpc(3), MOD_SETP, &f).payload,
            b" 3D 3,3,1.0E-07,2.0E-07"
        );
        f.spfg = MOD_SPNS[3];
        f.sps[3] = 1e-9;
        f.shr[3] = 2e-9;
        assert_eq!(
            control_command(&qpc(3), MOD_SETP, &f).payload,
            b" 3D 3,3,1.0E-09,2.0E-09"
        );
    }

    #[test]
    fn mpc_ignores_setpoints_three_and_four() {
        let mut f = ControlFields {
            spfg: MOD_SPNS[2] | MOD_SPNS[3],
            ..Default::default()
        };
        f.sps[2] = 1e-6;
        f.sps[3] = 1e-6;
        assert_eq!(control_command(&mpc(1), MOD_SETP, &f).payload, b"");
    }

    #[test]
    fn a_zero_on_pressure_suppresses_the_setpoint_command() {
        let f = ControlFields {
            spfg: MOD_SPNS[0],
            ..Default::default()
        };
        assert_eq!(control_command(&mpc(1), MOD_SETP, &f).payload, b"");
    }

    #[test]
    fn a_pressure_outside_the_device_span_suppresses_the_setpoint_command() {
        // Regression for doc/upstream-c-defects.md #21: pressures outside
        // [1e-11, 1e-4] Torr are rejected, leaving val1 == 0 so no command is
        // sent. Pre-fix the guard passed every value.
        let mut f = ControlFields {
            spfg: MOD_SPNS[0],
            ..Default::default()
        };
        f.sps[0] = 1e-3; // above the 1e-4 ceiling
        f.shr[0] = 1e-6;
        assert_eq!(control_command(&mpc(1), MOD_SETP, &f).payload, b"");

        f.sps[0] = 1e-13; // below the 1e-11 floor
        assert_eq!(control_command(&mpc(1), MOD_SETP, &f).payload, b"");

        // An off-pressure out of range likewise suppresses the command even
        // though val1 would come from the in-range on-pressure readback.
        let mut g = ControlFields {
            spfg: MOD_SNHS[0],
            ..Default::default()
        };
        g.spr[0] = 1e-6;
        g.shs[0] = 1e-3;
        assert_eq!(control_command(&mpc(1), MOD_SETP, &g).payload, b"");
    }

    // ---- read slots -------------------------------------------------------

    #[test]
    fn mpc_reads_status_pressure_current_voltage_and_pump_size() {
        let c = mpc(1);
        assert_eq!(read_slot(&c, 0), ReadSlot::Send(b" 0D 1".to_vec()));
        assert_eq!(read_slot(&c, 1), ReadSlot::Send(b" 0B 1".to_vec()));
        assert_eq!(read_slot(&c, 2), ReadSlot::Send(b" 0A 1".to_vec()));
        assert_eq!(read_slot(&c, 3), ReadSlot::Send(b" 0C 1".to_vec()));
        assert_eq!(read_slot(&c, 4), ReadSlot::Send(b" 11 1".to_vec()));
        assert_eq!(read_slot(&c, 9), ReadSlot::Send(b" 01".to_vec()));
        assert_eq!(read_slot(&c, 10), ReadSlot::Send(b" 02".to_vec()));
    }

    #[test]
    fn mpc_odd_pumps_read_odd_setpoints() {
        let c = mpc(1);
        assert_eq!(read_slot(&c, 5), ReadSlot::Send(b" 3C 1".to_vec()));
        assert_eq!(read_slot(&c, 6), ReadSlot::Send(b" 3C 3".to_vec()));
        assert_eq!(read_slot(&c, 7), ReadSlot::Send(b" 3C 5".to_vec()));
        let c = mpc(2);
        assert_eq!(read_slot(&c, 5), ReadSlot::Send(b" 3C 2".to_vec()));
        assert_eq!(read_slot(&c, 7), ReadSlot::Send(b" 3C 6".to_vec()));
    }

    #[test]
    fn mpc_slot_eight_repeats_slot_seven_because_c_leaves_pvalue_stale() {
        let c = mpc(1);
        assert_eq!(read_slot(&c, 8), read_slot(&c, 7));
    }

    #[test]
    fn qpc_issues_one_setpoint_read_and_skips_the_other_three() {
        let c = qpc(3);
        assert_eq!(read_slot(&c, 5), ReadSlot::Send(b" 3C 3".to_vec()));
        assert_eq!(read_slot(&c, 6), ReadSlot::ReuseLast);
        assert_eq!(read_slot(&c, 7), ReadSlot::ReuseLast);
        assert_eq!(read_slot(&c, 8), ReadSlot::ReuseLast);
        assert_eq!(read_slot(&c, 9), ReadSlot::Send(b" 01".to_vec()));
    }

    #[test]
    fn digitel_reads_stop_after_its_configured_setpoints() {
        let c = digitel(0);
        assert_eq!(read_slot(&c, 0), ReadSlot::Send(b"RD".to_vec()));
        assert_eq!(read_slot(&c, 1), ReadSlot::Send(b"RC".to_vec()));
        assert_eq!(read_slot(&c, 2), ReadSlot::Skip);

        let c = digitel(3);
        assert_eq!(read_slot(&c, 2), ReadSlot::Send(b"RS1".to_vec()));
        assert_eq!(read_slot(&c, 3), ReadSlot::Send(b"RS2".to_vec()));
        assert_eq!(read_slot(&c, 4), ReadSlot::Send(b"RS3".to_vec()));
        assert_eq!(read_slot(&c, 5), ReadSlot::Skip);
    }

    // ---- reply stripping --------------------------------------------------

    #[test]
    fn mpc_reply_must_carry_ok_at_index_three() {
        assert!(strip_read_reply(DevType::Mpc, 0, b"05 ER 00 3B").is_err());
        assert_eq!(
            strip_read_reply(DevType::Mpc, 0, b"05 OK 00 RUNNING 8F").unwrap(),
            b"RUNNING 8F"
        );
    }

    #[test]
    fn a_short_mpc_reply_is_recorded_as_a_bare_acknowledgement() {
        assert_eq!(
            strip_read_reply(DevType::Mpc, 0, b"05 OK 00").unwrap(),
            b"OK"
        );
    }

    #[test]
    fn digitel_reply_flags_error_at_index_zero_four_or_five() {
        assert!(strip_read_reply(DevType::D500, 0, b"ERROR").is_err());
        assert!(strip_read_reply(DevType::D500, 0, b"RD\r\nERROR").is_err());
        assert!(strip_read_reply(DevType::D500, 2, b"RS1\r\nERROR").is_err());
    }

    #[test]
    fn digitel_strips_the_command_echo_by_command_length() {
        assert_eq!(
            strip_read_reply(DevType::D500, 0, b"RD\r\n01 02:03 5600V 1.2-8I HCB123").unwrap(),
            b"01 02:03 5600V 1.2-8I HCB123"
        );
        assert_eq!(
            strip_read_reply(DevType::D500, 2, b"RS1\r\n1.0-6 2.0-6").unwrap(),
            b"1.0-6 2.0-6"
        );
    }

    #[test]
    fn a_bitbus_digitel_reply_has_no_echo_to_strip() {
        assert_eq!(
            strip_read_reply(DevType::D500, 2, b"1.0-6 2.0-6").unwrap(),
            b"1.0-6 2.0-6"
        );
    }

    #[test]
    fn control_reply_checks() {
        assert!(check_control_reply(DevType::D500, b"M4").is_ok());
        assert!(check_control_reply(DevType::D500, b"S2116*").is_ok());
        assert!(check_control_reply(DevType::D500, b"ERROR").is_err());
        assert!(check_control_reply(DevType::Mpc, b"05 OK 00 25").is_ok());
        assert!(check_control_reply(DevType::Qpc, b"05 ER 00").is_err());
        assert!(check_control_reply(DevType::Mpc, b"").is_err());
    }

    // ---- decoding: MPC / QPC ---------------------------------------------

    fn mpc_buf() -> ResponseBuf {
        buf_from(&[
            (0, b"RUNNING"),
            (30, b"1.2E-08"),
            (60, b"5.4E-06"),
            (90, b"5600"),
            (120, b"120 "),
            (150, b"1,1,1.0E-06,1.2E-06,1"),
            (180, b"3,1,2.0E-06,2.4E-06,0"),
            (210, b"5,1,3.0E-06,3.6E-06,1"),
            (240, b"7,1,4.0E-06,4.8E-06,0"),
            (278, b"MPC2"),
            (318, b"1.30.4"),
        ])
    }

    #[test]
    fn mpc_decodes_status_pressure_current_and_voltage() {
        let r = decode(&mpc(1), &mpc_buf(), &Readings::default());
        assert_eq!(r.modr, 1);
        assert_eq!(r.cmor, 0);
        assert!((r.val - 1.2e-8).abs() < 1e-14);
        assert!((r.crnt - 5.4e-6).abs() < 1e-12);
        assert_eq!(r.volt, 5600.0);
        assert!((r.lval - (1.2e-8f32 as f64).log10()).abs() < 1e-9);
    }

    #[test]
    fn mpc_cool_down_status_sets_the_cooldown_readback() {
        let mut b = mpc_buf();
        b.strcpy_at(0, b"COOL DOWN");
        let r = decode(&mpc(1), &b, &Readings::default());
        assert_eq!((r.modr, r.cmor), (0, 1));
    }

    #[test]
    fn mpc_maps_the_reported_pump_size_onto_the_records_size_menu() {
        for (size, ptyp) in [
            (b"30  ".as_slice(), 0),
            (b"60  ", 1),
            (b"120 ", 2),
            (b"220 ", 3),
            (b"400 ", 4),
            (b"700 ", 5),
            (b"1200", 5),
        ] {
            let mut b = mpc_buf();
            b.strcpy_at(120, size);
            assert_eq!(decode(&mpc(1), &b, &Readings::default()).ptyp, ptyp);
        }
    }

    #[test]
    fn mpc_parks_the_pressure_when_the_pump_is_off() {
        let mut b = mpc_buf();
        b.strcpy_at(90, b"0");
        b.strcpy_at(60, b"1.0E-09");
        let r = decode(&mpc(1), &b, &Readings::default());
        assert_eq!(r.val, 9.9e9);
        assert_eq!(r.volt, 0.0);
    }

    #[test]
    fn mpc_decodes_four_setpoint_replies() {
        let r = decode(&mpc(1), &mpc_buf(), &Readings::default());
        assert!((r.spr[0] - 1.0e-6f32 as f64).abs() < 1e-12);
        assert!((r.shr[0] - 1.2e-6f32 as f64).abs() < 1e-12);
        assert!((r.spr[3] - 4.0e-6f32 as f64).abs() < 1e-12);
        assert_eq!(r.set, [1, 0, 1, 0]);
    }

    #[test]
    fn mpc_reads_model_and_firmware_from_fixed_offsets() {
        let r = decode(&mpc(1), &mpc_buf(), &Readings::default());
        assert_eq!(r.modl.unwrap(), b"MPC2");
        // MPC copies eight bytes; the reply is six, so strncpy NUL-pads.
        assert_eq!(r.vers.unwrap(), b"1.30.4\0\0");
    }

    #[test]
    fn qpc_reads_a_five_character_firmware_version() {
        let mut b = mpc_buf();
        b.strcpy_at(318, b"1.35A");
        let r = decode(&qpc(1), &b, &Readings::default());
        assert_eq!(r.vers.unwrap(), b"1.35A");
    }

    #[test]
    fn an_empty_mpc_setpoint_slot_repeats_the_previous_slot() {
        // C's val1/val2/t3 are function-scope, so a slot that fails to convert
        // leaves the previous slot's values in place.
        let mut b = mpc_buf();
        b.strcpy_at(180, b"");
        b.strcpy_at(210, b"");
        b.strcpy_at(240, b"");
        let r = decode(&mpc(1), &b, &Readings::default());
        assert_eq!(r.spr[1], r.spr[0]);
        assert_eq!(r.spr[3], r.spr[0]);
        assert_eq!(r.set, [1, 1, 1, 1]);
    }

    #[test]
    fn an_mpc_pressure_below_the_floor_clamps_the_log() {
        let mut b = mpc_buf();
        b.strcpy_at(30, b"1.0E-13");
        b.strcpy_at(60, b"1.0E-05");
        let r = decode(&mpc(1), &b, &Readings::default());
        assert_eq!(r.lval, -12.0);
    }

    // ---- decoding: Digitel ------------------------------------------------

    /// The stripped replies, laid out exactly as `devDigitelPumpCallback` would
    /// `strcpy` them. Field positions inside each reply are load-bearing: the
    /// decoder indexes columns 14, 16 and 17 of the setpoint replies, and the
    /// status characters at 23..=28 of the `RD` reply.
    ///
    ///   RD  -> "DD HH:MM XXXXV x.xE-xI HCB123"
    ///   RC  -> "XXP XXI XC XS"
    ///   RSx -> "X.0E-X Y.0E-Y ZZZZ HH"
    fn digitel_buf() -> ResponseBuf {
        buf_from(&[
            (0, b"01 02:03 5600V 1.2E-8I HCB123"),
            (30, b"10P 20I 3C 2S"),
            (60, b"1.0E-6 1.2E-6 1010 12"),
            (90, b"2.0E-6 2.4E-6 1010 12"),
            (120, b"3.0E-6 3.6E-6 1011 12"),
        ])
    }

    #[test]
    fn digitel_decodes_time_online_voltage_and_current() {
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        // "1:02:03" days:hours:minutes -> minutes online.
        assert_eq!(r.tonl, 1440 + 2 * 60 + 3);
        assert_eq!(r.volt, 5600.0);
        assert!((r.crnt - 1.2e-8).abs() < 1e-16);
    }

    #[test]
    fn digitel_decodes_the_status_characters() {
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        assert_eq!((r.modr, r.cmor, r.bakr), (1, 1, 1));
        assert_eq!(&r.set[..3], &[1, 1, 1]);
    }

    #[test]
    fn digitel_setpoint_status_characters_are_gated_on_the_station_count() {
        let r = decode(&digitel(1), &digitel_buf(), &Readings::default());
        assert_eq!(&r.set[..3], &[1, 0, 0]);
        let r = decode(&digitel(2), &digitel_buf(), &Readings::default());
        assert_eq!(&r.set[..3], &[1, 1, 0]);
    }

    #[test]
    fn digitel_decodes_accumulated_power_current_and_cooldown_cycles() {
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        assert!((r.accw - 10.0 * 0.444).abs() < 1e-9);
        assert!((r.acci - 20.0 * 1.11).abs() < 1e-9);
        assert_eq!(r.cool, 3);
    }

    #[test]
    fn digitel_pressure_needs_high_voltage_and_a_nonzero_supply_voltage() {
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        // pumpType 2, HV on: 0.005 * 1.2e-8 / 2.
        assert!((r.val - 0.005 * 1.2e-8 / 2.0).abs() < 1e-18);

        let mut b = digitel_buf();
        b.strcpy_at(23, b"-CB123");
        let r = decode(&digitel(3), &b, &Readings::default());
        assert_eq!((r.modr, r.val), (0, 9.9e9));
    }

    #[test]
    fn a_digitel_1500_quadruples_the_reported_pump_size() {
        let c = configure(DevType::D1500, 0, "3").unwrap();
        let r = decode(&c, &digitel_buf(), &Readings::default());
        // Reported 2 -> 8 -> menu index 3.
        assert_eq!(r.ptyp, 3);
        assert!((r.val - 0.005 * 1.2e-8 / 8.0).abs() < 1e-18);
    }

    #[test]
    fn an_out_of_range_digitel_pump_size_falls_back_to_the_smallest() {
        let mut b = digitel_buf();
        b.strcpy_at(30, b"10P 20I 3C 99S");
        let r = decode(&digitel(3), &b, &Readings::default());
        assert_eq!(r.ptyp, 0);
    }

    #[test]
    fn digitel_decodes_all_three_setpoints_by_falling_through() {
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        assert!((r.spr[0] - 1.0e-6).abs() < 1e-12);
        assert!((r.shr[0] - 1.2e-6).abs() < 1e-12);
        assert!((r.spr[1] - 2.0e-6).abs() < 1e-12);
        assert!((r.spr[2] - 3.0e-6).abs() < 1e-12);
        assert_eq!(r.bkin, 1);
        assert_eq!(r.s3br, 1);
    }

    #[test]
    fn digitel_bakeout_time_reads_past_its_two_digits_into_the_scratch_tail() {
        // C: `strncpy(pvalue, &recBuf[139], 2); sscanf(pvalue, "%lf", &s3tr);`
        // `strncpy` writes exactly two bytes and no terminator, so `sscanf` runs
        // straight into what the previous 18-byte copy left at pvalue[2..]. For
        // the reply "3.0E-6 3.6E-6 1011 12" that tail is "0E-6 ...", so the two
        // hours "12" are read as "120E-6". Upstream defect, reproduced.
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        assert!((r.s3tr - 1.2e-4).abs() < 1e-10, "s3tr = {}", r.s3tr);
    }

    #[test]
    fn digitel_setpoint_mode_and_enable_readbacks_come_from_fixed_columns() {
        // Column 14 is the mode digit, column 16 the enable digit.
        let r = decode(&digitel(3), &digitel_buf(), &Readings::default());
        assert_eq!(r.smr[0], 1);
        assert_eq!(r.svr[0], 0);
        assert_eq!(r.smr[1], 1);
        assert_eq!(r.svr[1], 0);
        // Upstream never writes s3mr/s3vr: the case-3 block writes s2mr/s2vr
        // and the case-2 fall-through overwrites them.
        assert_eq!(r.smr[2], 0);
        assert_eq!(r.svr[2], 1);
    }

    #[test]
    fn a_digitel_without_the_bakeout_option_reports_an_error_setpoint_three() {
        let mut b = digitel_buf();
        b.strcpy_at(120, b"ERROR");
        let r = decode(&digitel(3), &b, &Readings::default());
        assert_eq!(r.bkin, 0);
        // The case-2 fall-through still runs.
        assert!((r.spr[1] - 2.0e-6).abs() < 1e-12);
    }

    #[test]
    fn a_digitel_with_one_setpoint_leaves_the_others_at_their_previous_values() {
        let prev = Readings {
            spr: [9.0, 8.0, 7.0, 6.0],
            ..Readings::default()
        };
        let r = decode(&digitel(1), &digitel_buf(), &prev);
        assert!((r.spr[0] - 1.0e-6).abs() < 1e-12);
        assert_eq!(r.spr[1], 8.0);
        assert_eq!(r.spr[2], 7.0);
    }

    #[test]
    fn a_digitel_status_reply_that_fails_to_convert_keeps_the_previous_voltage() {
        let prev = Readings {
            volt: 4200.0,
            crnt: 3.3e-9,
            ..Readings::default()
        };
        let mut b = digitel_buf();
        b.strcpy_at(0, b"");
        let r = decode(&digitel(0), &b, &prev);
        assert_eq!(r.volt, 4200.0);
        assert_eq!(r.crnt, 3.3e-9);
    }
}
