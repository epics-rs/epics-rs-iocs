//! `devVacSen.c` wire protocol: GP307, GP350, MM200, CC10, MX200.
//!
//! # EOS ownership
//!
//! `devVacSen.c` never calls `setInputEos`/`setOutputEos`. It writes the bare
//! command with `pasynOctet->write` and reads with `pasynOctet->read`, so both
//! terminators come from the startup script's `asynOctetSetInputEos` /
//! `asynOctetSetOutputEos`. This port keeps that split: nothing here appends a
//! terminator, and the shipped `st.cmd` sets the per-device EOS.
//!
//! # Response assembly
//!
//! `devVacSenCallback` issues up to eight reads and `strcpy`s each stripped
//! reply into an 80-byte buffer at offset `10*i` (degas for GP307/GP350 goes to
//! offset 15 instead). `readWrite_vs` then decodes the buffer by absolute
//! offset. [`ResponseBuf`] is that buffer; [`decode`] is that decoder.

use super::scan::{scan_char, scan_float, scan_hex, scan_int};
use super::{CBuf, cstr};

/// `vacSen_BUFFER_SIZE` — the assembled response buffer, and `recBuf`.
pub const BUFFER_SIZE: usize = 80;
/// `vacSen_READ_SIZE` — the per-reply read length.
pub const READ_SIZE: usize = 55;
/// `vacSen_TIMEOUT`, seconds.
pub const TIMEOUT_SECS: f64 = 3.0;

pub type ResponseBuf = CBuf<BUFFER_SIZE>;

/// `menu(vsTYPE)` from `vsRecord.dbd`, in declaration order. The numeric value
/// is the record's `TYPE` field and indexes the command tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DevType {
    Gp307 = 0,
    Gp350 = 1,
    Mm200 = 2,
    Cc10 = 3,
    Mx200 = 4,
}

impl DevType {
    pub fn from_index(i: u16) -> Option<Self> {
        Some(match i {
            0 => Self::Gp307,
            1 => Self::Gp350,
            2 => Self::Mm200,
            3 => Self::Cc10,
            4 => Self::Mx200,
            _ => return None,
        })
    }
}

/// `readCmdString[]` from `devVacSen.h`, ten slots per device type.
const READ_CMD: [[&str; 10]; 5] = [
    [
        "PCS", "DGS", "DS IG1", "DS IG2", "DS CG1", "DS CG2", "", "", "", "",
    ],
    [
        "PC S", "DGS", "RD 1", "RD 2", "RD A", "RD B", "", "", "", "",
    ],
    ["RY", "R", "R", "R", "SP", "SP", "SP", "SP", "", ""],
    ["S5", "S1", "R2", "R3", "R4", "", "", "", "", ""],
    ["S5", "S1", "S1", "S1", "R7", "R7", "R7", "R7", "", ""],
];

/// `ctlCmdString[]` from `devVacSen.h`. Only GP307 and GP350 have control
/// commands; the table is indexed `cmd + devType*10`, so it only spans two
/// device types.
const CTL_CMD: [[&str; 10]; 2] = [
    [
        "IG1 OFF", "IG1 ON", "IG2 OFF", "IG2 ON", "DG OFF", "DG ON", "", "", "", "",
    ],
    [
        "F1 0", "F1 1", "F2 0", "F2 1", "DG0 OFF", "DG1 ON", "", "", "", "",
    ],
];

/// STX, the CC10 RS-485 command prefix character.
const STX: u8 = 0x02;

/// Per-record configuration derived from `INP` (`@asyn(port addr)userParam`)
/// and the record's `TYPE` field, mirroring `devVacSenPvt`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    pub dev: DevType,
    /// `cmdPrefix` — prepended to every command.
    pub prefix: Vec<u8>,
    /// Cold-cathode station (MM200/MX200).
    pub cc: i32,
    /// Convectron 1 station (MM200/MX200).
    pub cv1: i32,
    /// Convectron 2 station; 0 means absent (MM200/MX200).
    pub cv2: i32,
    /// First setpoint station (MM200/MX200); 1 or 2.
    pub spt: i32,
}

/// Build the `cmdPrefix` and station set from the asyn address and userParam.
///
/// `devVacSen::init` validates the address range per device and formats the
/// prefix; the station parse is device-specific. Errors are the `errlogPrintf`
/// + `goto bad` cases, which disable the record in C.
pub fn configure(dev: DevType, address: i32, user_param: &str) -> Result<Config, String> {
    let mut cfg = Config {
        dev,
        prefix: Vec::new(),
        cc: 0,
        cv1: 0,
        cv2: 0,
        spt: 0,
    };

    match dev {
        // GP307: `pPvt` is calloc'd and no branch runs — prefix stays empty.
        DevType::Gp307 => {}

        DevType::Gp350 => {
            if !(0..=31).contains(&address) {
                return Err(format!("GP350 address out of range {address}"));
            }
            // C: `address` is the calloc'd empty string when address == 0, so
            // the prefix for RS-232 is a bare "#".
            let addr = if address > 0 {
                format!("{address:02X}")
            } else {
                String::new()
            };
            cfg.prefix = format!("#{addr}").into_bytes();
        }

        DevType::Mm200 => {
            if address != 0 {
                return Err("RS485 to MM200 not supported".into());
            }
            let (cc, cv1, cv2, spt) = parse_mm200_stations(user_param)?;
            cfg.cc = cc;
            cfg.cv1 = cv1;
            cfg.cv2 = cv2;
            cfg.spt = spt;
        }

        DevType::Cc10 => {
            if !(0..=15).contains(&address) {
                return Err(format!("CC10 address out of range {address}"));
            }
            cfg.prefix = vec![STX];
            cfg.prefix
                .extend_from_slice(format!("{address:X}").as_bytes());
        }

        DevType::Mx200 => {
            if !(0..=99).contains(&address) {
                return Err(format!("MX200 address out of range {address}"));
            }
            let (cc, cv1, cv2, spt) = parse_mx200_stations(user_param)?;
            // C formats the MX200 address with "%02X" (hex) even though the
            // manual calls it a two-digit decimal address. Ported as written.
            if address > 0 {
                cfg.prefix = format!("*{address:02X}").into_bytes();
            }
            cfg.cc = cc;
            cfg.spt = spt;
            cfg.cv1 = if (1..=6).contains(&cv1) { cv1 } else { 0 };
            cfg.cv2 = if (1..=6).contains(&cv2) { cv2 } else { 0 };
        }
    }
    Ok(cfg)
}

/// C `sscanf(userParam, "%1d %1d %1d %1d", ...)` — each field is one digit.
/// Returns how many of the four converted, plus the values.
fn scan_four_single_digits(s: &str) -> (usize, [i32; 4]) {
    let mut out = [0i32; 4];
    let mut rest = s.as_bytes();
    let mut n = 0;
    for slot in out.iter_mut() {
        match scan_int(rest, Some(1)) {
            Some((v, tail)) => {
                *slot = v as i32;
                rest = tail;
                n += 1;
            }
            None => break,
        }
    }
    (n, out)
}

fn parse_mm200_stations(user_param: &str) -> Result<(i32, i32, i32, i32), String> {
    let (n, [station, mut cv1, mut cv2, mut spt]) = scan_four_single_digits(user_param);
    match n {
        1 => {
            // Backward-compatible CC-only form.
            if !(3..=6).contains(&station) {
                return Err(format!("MM200 CC out of range: {station}"));
            }
            const CV1: [i32; 4] = [1, 2, 1, 2];
            const CV2: [i32; 4] = [0, 0, 3, 4];
            cv1 = CV1[(station - 3) as usize];
            cv2 = CV2[(station - 3) as usize];
            spt = cv1;
        }
        4 => {
            if !(3..=9).contains(&station) {
                return Err(format!("MM200 CC out of range: {station}"));
            }
            if !(1..=2).contains(&spt) {
                return Err(format!("MM200 spt out of range {spt}"));
            }
            if !(1..=6).contains(&cv1) {
                return Err(format!("MM200 CV1 out of range: {cv1}"));
            }
            if !(0..=6).contains(&cv2) {
                return Err(format!("MM200 CV2 out of range: {cv2}"));
            }
        }
        _ => return Err(format!("MM200 too few/many parameters: {user_param}")),
    }
    Ok((station, cv1, cv2, spt))
}

fn parse_mx200_stations(user_param: &str) -> Result<(i32, i32, i32, i32), String> {
    // C ignores `sscanf`'s return value here, so a short userParam leaves
    // `station`/`spt`/`stationC1`/`stationC2` uninitialised and the range
    // checks below read indeterminate values. This port requires all four —
    // see the UNFIXED note in the crate docs.
    let (n, [station, cv1, cv2, spt]) = scan_four_single_digits(user_param);
    if n != 4 {
        return Err(format!(
            "MX200 needs 4 station parameters \"CC CV1 CV2 SPT\", got: {user_param:?}"
        ));
    }
    if !(3..=9).contains(&station) {
        return Err(format!("MX200 station for CC out of range {station}"));
    }
    if !(1..=2).contains(&spt) {
        return Err(format!("MX200 setpoint out of range {spt}"));
    }
    Ok((station, cv1, cv2, spt))
}

/// The control command for a changed IG1/IG2/DGS field, or `None` when the
/// device has no control commands (MM200, CC10, MX200 — C downgrades those to
/// a plain read cycle).
///
/// `cmd` is `ig1s`, `2 + ig2s` or `4 + dgss`, exactly as `readWrite_vs` builds
/// it, and indexes `ctlCmdString[cmd + type*10]`.
pub fn control_command(cfg: &Config, cmd: usize) -> Option<Vec<u8>> {
    let table = match cfg.dev {
        DevType::Gp307 => &CTL_CMD[0],
        DevType::Gp350 => &CTL_CMD[1],
        DevType::Mm200 | DevType::Cc10 | DevType::Mx200 => return None,
    };
    let mut buf = cfg.prefix.clone();
    buf.extend_from_slice(table[cmd].as_bytes());
    Some(buf)
}

/// Which of the eight read slots this device actually issues.
///
/// * GP307/GP350 stop after slot 5;
/// * MM200 skips slot 3 when there is no second convectron;
/// * CC10 stops after slot 4;
/// * MX200 issues all eight — including slot 3 with a zero station, as in C.
pub fn skips_read(cfg: &Config, i: usize) -> bool {
    match cfg.dev {
        DevType::Gp307 | DevType::Gp350 => i > 5,
        DevType::Mm200 => i == 3 && cfg.cv2 == 0,
        DevType::Cc10 => i > 4,
        DevType::Mx200 => false,
    }
}

/// The `i`th read command: `cmdPrefix + readCmdString[i + type*10]`, plus the
/// station suffix that MM200 and MX200 append.
pub fn read_command(cfg: &Config, i: usize) -> Vec<u8> {
    let mut buf = cfg.prefix.clone();
    buf.extend_from_slice(READ_CMD[cfg.dev as usize][i].as_bytes());

    match cfg.dev {
        DevType::Mm200 => {
            let suffix = match i {
                0 => String::new(),
                1 => format!("{}", cfg.cc),
                2 => format!("{}", cfg.cv1),
                3 => format!("{}", cfg.cv2),
                // Must match the "RY" relay-bit decode in `decode`.
                _ => format!("{}N", cfg.spt + ((i - 4) as i32) * 2),
            };
            buf.extend_from_slice(suffix.as_bytes());
        }
        DevType::Mx200 => {
            let suffix = match i {
                0 => String::new(),
                1 => format!("{:02}", cfg.cc),
                2 => format!("{:02}", cfg.cv1),
                3 => format!("{:02}", cfg.cv2),
                _ => format!("{}", cfg.spt + ((i - 4) as i32) * 2),
            };
            buf.extend_from_slice(suffix.as_bytes());
        }
        _ => {}
    }
    buf
}

/// Where the `i`th stripped reply lands in the response buffer. GP307/GP350
/// push the degas reply to offset 15 so the six-setpoint status string at
/// offset 0 is not overwritten.
pub fn place_offset(dev: DevType, i: usize) -> usize {
    if matches!(dev, DevType::Gp307 | DevType::Gp350) && i == 1 {
        15
    } else {
        10 * i
    }
}

/// A read reply that the device flagged as an error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplyError;

/// Validate and strip one read reply, returning the payload that gets copied
/// into the response buffer.
///
/// * GP307 — trailing `"ERROR"`; payload is the whole reply.
/// * GP350 — leading `'?'`; payload skips the two-character `"* "` header.
/// * MM200 — `reply[1] == '?'`; payload is the whole reply.
/// * CC10  — `reply[2] == 'N'`; payload skips `<STX><addr><cmd>`.
/// * MX200 — `reply[1] == 'N'`. Slot 0 is recoded from the eight `ON`/`OF`/`00`
///   relay fields into the two hex digits the decoder expects; slots 4..7 drop
///   the two trailing station characters.
pub fn strip_read_reply(dev: DevType, i: usize, reply: &[u8]) -> Result<Vec<u8>, ReplyError> {
    let at = |n: usize| reply.get(n).copied().unwrap_or(0);
    match dev {
        DevType::Gp307 => {
            if reply.len() >= 5 && &reply[reply.len() - 5..] == b"ERROR" {
                return Err(ReplyError);
            }
            Ok(reply.to_vec())
        }
        DevType::Gp350 => {
            if at(0) == b'?' {
                return Err(ReplyError);
            }
            Ok(reply.get(2..).unwrap_or(&[]).to_vec())
        }
        DevType::Mm200 => {
            if at(1) == b'?' {
                return Err(ReplyError);
            }
            Ok(reply.to_vec())
        }
        DevType::Cc10 => {
            if at(2) == b'N' {
                return Err(ReplyError);
            }
            Ok(reply.get(3..).unwrap_or(&[]).to_vec())
        }
        DevType::Mx200 => {
            if at(1) == b'N' {
                return Err(ReplyError);
            }
            if i == 0 {
                // "01=ON 02=OF ... 08=00": bit j is set when the 'N' of "ON"
                // sits at index 4 + 6*j. C then rewrites the reply as "%2x".
                //
                // (Upstream writes `*readBuffer = sprintf("%2x", value)`, which
                // passes a string literal as sprintf's destination — undefined
                // behaviour. The evident intent, and the only form the decoder's
                // `sscanf(data, "%x")` can consume, is `sprintf(readBuffer,
                // "%2x", value)`. Ported as the intent.)
                let mut value = 0u32;
                for j in 0..8 {
                    if at(4 + j * 6) == b'N' {
                        value += 1 << j;
                    }
                }
                return Ok(format!("{value:2x}").into_bytes());
            }
            if i > 3 {
                // 12-character setpoint reply; the last two are the station.
                return Ok(reply.get(..10).unwrap_or(reply).to_vec());
            }
            Ok(reply.to_vec())
        }
    }
}

/// Validate a control-command reply. Only GP307 and GP350 issue one.
pub fn check_control_reply(dev: DevType, reply: &[u8]) -> Result<(), ReplyError> {
    match dev {
        DevType::Gp307 if cstr(reply) != b"OK" => Err(ReplyError),
        DevType::Gp350 if reply.first() == Some(&b'?') => Err(ReplyError),
        _ => Ok(()),
    }
}

/// Everything `readWrite_vs` writes into the record from one response buffer.
#[derive(Debug, Clone, PartialEq)]
pub struct Readings {
    pub val: f64,
    pub cgap: f64,
    pub cgbp: f64,
    pub ig1r: u16,
    pub ig2r: u16,
    pub dgsr: u16,
    pub sp: [u16; 6],
    pub spr: [f64; 4],
}

impl Default for Readings {
    fn default() -> Self {
        // C zeroes dgsr/ig1r/ig2r and parks VAL at 9.9e9 before decoding; the
        // other fields keep their previous record values unless the device
        // type writes them.
        Self {
            val: 9.9e9,
            cgap: 0.0,
            cgbp: 0.0,
            ig1r: 0,
            ig2r: 0,
            dgsr: 0,
            sp: [0; 6],
            spr: [0.0; 4],
        }
    }
}

/// `readWrite_vs`'s conversion targets, hoisted so they survive across
/// iterations.
///
/// C declares `int value; float fvalue; char sign; int exp;` once per call and
/// reuses them for every `sscanf`. A conversion that fails leaves its target
/// holding the *previous* field's value, and the record picks that up — a
/// short reply makes SP2..SP4 inherit SP1, not read as zero. Reproducing that
/// requires the same lifetime, hence one struct threaded through the decode.
///
/// `sign` and `exp` are genuinely uninitialised in C (`char sign; int exp;`
/// with no initialiser); the first device branch that reaches a failing
/// `sscanf` before any successful one reads indeterminate values. We seed them
/// with the benign `('+', 0)` — see the UNFIXED note in the crate docs.
struct ScanState {
    value: i64,
    fvalue: f32,
    sign: u8,
    exp: i64,
}

impl Default for ScanState {
    fn default() -> Self {
        Self {
            value: 0,
            fvalue: 0.0,
            sign: b'+',
            exp: 0,
        }
    }
}

impl ScanState {
    /// C `sscanf(data, "%d", &value)` on a one-character field.
    fn scan_digit(&mut self, data: &[u8]) -> u16 {
        if let Some((v, _)) = scan_int(data, None) {
            self.value = v;
        }
        self.value as u16
    }

    /// C `sscanf(data, "%e", &fvalue)`.
    fn scan_e(&mut self, data: &[u8]) -> f32 {
        if let Some((v, _)) = scan_float(data, None) {
            self.fvalue = v as f32;
        }
        self.fvalue
    }

    /// C `sscanf(data, "%f%c%x", &fvalue, &sign, &exp)`.
    fn scan_f_c_x(&mut self, data: &[u8]) {
        if let Some((f, rest)) = scan_float(data, None) {
            self.fvalue = f as f32;
            if let Some((c, rest)) = scan_char(rest) {
                self.sign = c;
                if let Some((e, _)) = scan_hex(rest, None) {
                    self.exp = e;
                }
            }
        }
    }

    /// C `sscanf(data, "%2d%c%x", &value, &sign, &exp)`.
    fn scan_2d_c_x(&mut self, data: &[u8]) {
        if let Some((v, rest)) = scan_int(data, Some(2)) {
            self.value = v;
            if let Some((c, rest)) = scan_char(rest) {
                self.sign = c;
                if let Some((e, _)) = scan_hex(rest, None) {
                    self.exp = e;
                }
            }
        }
    }

    /// C `sscanf(data, "%Nd%c%2d", &value, &sign, &exp)` for N in {2, 3}.
    fn scan_nd_c_2d(&mut self, data: &[u8], width: usize) {
        if let Some((v, rest)) = scan_int(data, Some(width)) {
            self.value = v;
            if let Some((c, rest)) = scan_char(rest) {
                self.sign = c;
                if let Some((e, _)) = scan_int(rest, Some(2)) {
                    self.exp = e;
                }
            }
        }
    }

    /// Apply the sign character and scale: C `if (sign == neg) exp = -exp;
    /// fvalue = fvalue * pow(10, exp)`. `pow` is `double`, and the product is
    /// narrowed back into the `float` fvalue — reproduced exactly.
    fn scaled(&self, neg: u8, mantissa: f64) -> f32 {
        let exp = if self.sign == neg {
            -self.exp
        } else {
            self.exp
        };
        (mantissa * 10f64.powi(exp as i32)) as f32
    }
}

/// Decode a response buffer for `dev`. `spt` is the first setpoint station,
/// used by the MM200/MX200 relay-bit mapping.
///
/// `prev` supplies the fields this device type does not write, matching C,
/// where those record fields simply keep their previous values.
pub fn decode(dev: DevType, spt: i32, buf: &ResponseBuf, prev: &Readings) -> Readings {
    let mut r = Readings {
        cgap: prev.cgap,
        cgbp: prev.cgbp,
        sp: prev.sp,
        spr: prev.spr,
        ..Readings::default()
    };
    let s = &mut ScanState::default();

    /// SP1..SP4 (or SP1..SP6) map onto relay bits `spt-1`, `spt+1`, `spt+3`,
    /// `spt+5`. Must match the setpoint command generation in `read_command`.
    fn relay_bits(sp: &mut [u16], value: i64, spt: i32) {
        for (n, slot) in sp.iter_mut().enumerate() {
            *slot = ((value >> (spt - 1 + 2 * n as i32)) & 1) as u16;
        }
    }

    match dev {
        DevType::Gp307 | DevType::Gp350 => {
            if dev == DevType::Gp350 {
                for n in 0..4 {
                    r.sp[n] = s.scan_digit(buf.slice(n, 1));
                }
            } else {
                for n in 0..6 {
                    r.sp[n] = s.scan_digit(buf.slice(2 * n, 1));
                }
            }

            if buf.at(15) == b'1' {
                r.dgsr = 1;
            }

            // Four pressures of the form "x.xxE-yy": C copies ten bytes then
            // terminates at index 8.
            for i in 2..6 {
                let fvalue = s.scan_e(cstr(buf.slice(10 * i, 8)));
                match i {
                    2 => {
                        if fvalue < 1.0 {
                            r.ig1r = 1;
                            r.val = fvalue as f64;
                        }
                    }
                    3 => {
                        if fvalue < 1.0 {
                            r.ig2r = 1;
                            r.val = fvalue as f64;
                        }
                    }
                    4 => r.cgap = fvalue as f64,
                    _ => r.cgbp = fvalue as f64,
                }
            }
        }

        DevType::Mm200 => {
            // "RY" relay status: two hex nibbles, with 'n' meaning "no relay".
            let mut relay = [buf.at(0), buf.at(1)];
            for b in relay.iter_mut() {
                if *b == b'n' {
                    *b = b'0';
                }
            }
            if let Some((v, _)) = scan_hex(cstr(&relay), None) {
                s.value = v;
            }
            relay_bits(&mut r.sp[..4], s.value, spt);

            for i in 1..8 {
                if i < 4 {
                    // "n=x.xx-(+)eT" with the "n=" already skipped.
                    let data = buf.slice(10 * i + 2, 8);
                    if data.get(6) == Some(&b'T') {
                        s.scan_f_c_x(&data[..6]);
                    } else {
                        s.fvalue = 9.9;
                        s.sign = b'+';
                        s.exp = 9;
                    }
                } else {
                    // Setpoints: "x.x-(+)e" or "x.xx-(+)e".
                    let data = buf.slice(10 * i, 10);
                    match (data.get(3), data.get(4)) {
                        (Some(b'-' | b'+'), _) => s.scan_f_c_x(&data[..5.min(data.len())]),
                        (_, Some(b'-' | b'+')) => s.scan_f_c_x(&data[..6.min(data.len())]),
                        _ => {
                            s.fvalue = 0.0;
                            s.sign = b'+';
                            s.exp = 0;
                        }
                    }
                }

                let fvalue = s.scaled(b'-', s.fvalue as f64);
                match i {
                    1 => {
                        r.val = fvalue as f64;
                        if fvalue < 1.0 {
                            r.ig1r = 1;
                        }
                    }
                    2 => r.cgap = fvalue as f64,
                    3 => r.cgbp = fvalue as f64,
                    _ => r.spr[i - 4] = fvalue as f64,
                }
            }
        }

        DevType::Cc10 => {
            for n in 0..4 {
                r.sp[n] = s.scan_digit(buf.slice(n, 1));
            }

            // "abcd" -> a.b * 10^(+/-)d, where c == '0' means a negative
            // exponent and c == '1' a positive one.
            for i in 1..5 {
                s.scan_2d_c_x(buf.slice(10 * i, 4));
                let fvalue = s.scaled(b'0', s.value as f64 / 10.0);
                match i {
                    1 => {
                        r.val = fvalue as f64;
                        if fvalue < 1.0 {
                            r.ig1r = 1;
                        }
                    }
                    _ => r.spr[i - 2] = fvalue as f64,
                }
            }
        }

        DevType::Mx200 => {
            if let Some((v, _)) = scan_hex(cstr(buf.slice(0, 2)), None) {
                s.value = v;
            }
            relay_bits(&mut r.sp[..4], s.value, spt);

            for i in 1..8 {
                // C copies ten bytes then terminates at index 9.
                let data = cstr(buf.slice(10 * i, 9));
                // Gauge slots may report either resolution; setpoint readbacks
                // (i >= 4) are always LO. HI is detected by the exponent-sign
                // character having moved from index 2 to index 3.
                let hi = i < 4 && data.len() >= 6 && matches!(data.get(3), Some(b'0' | b'1'));
                let (width, divisor, limit) = if hi { (3, 100.0, 7) } else { (2, 10.0, 6) };
                s.scan_nd_c_2d(&data[..limit.min(data.len())], width);
                let fvalue = s.scaled(b'0', s.value as f64 / divisor);
                match i {
                    1 => {
                        r.val = fvalue as f64;
                        if fvalue < 1.0 {
                            r.ig1r = 1;
                        }
                    }
                    2 => r.cgap = fvalue as f64,
                    3 => r.cgbp = fvalue as f64,
                    _ => r.spr[i - 4] = fvalue as f64,
                }
            }
        }
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

    // ---- configuration / addressing ------------------------------------

    #[test]
    fn gp307_has_no_prefix() {
        assert_eq!(configure(DevType::Gp307, 0, "0").unwrap().prefix, b"");
    }

    #[test]
    fn gp350_prefix_is_hash_plus_two_hex_digits() {
        assert_eq!(configure(DevType::Gp350, 17, "").unwrap().prefix, b"#11");
        // RS-232 (address 0) leaves the address string empty: a bare '#'.
        assert_eq!(configure(DevType::Gp350, 0, "").unwrap().prefix, b"#");
        assert!(configure(DevType::Gp350, 32, "").is_err());
    }

    #[test]
    fn cc10_prefix_is_stx_plus_one_hex_digit() {
        assert_eq!(configure(DevType::Cc10, 10, "").unwrap().prefix, b"\x02A");
        assert_eq!(configure(DevType::Cc10, 0, "").unwrap().prefix, b"\x020");
        assert!(configure(DevType::Cc10, 16, "").is_err());
    }

    #[test]
    fn mx200_prefix_is_star_plus_two_hex_digits_and_empty_at_address_zero() {
        assert_eq!(
            configure(DevType::Mx200, 16, "3 1 0 1").unwrap().prefix,
            b"*10"
        );
        assert_eq!(configure(DevType::Mx200, 0, "3 1 0 1").unwrap().prefix, b"");
        assert!(configure(DevType::Mx200, 100, "3 1 0 1").is_err());
    }

    #[test]
    fn mm200_single_station_expands_to_the_legacy_table() {
        let c = configure(DevType::Mm200, 0, "5").unwrap();
        assert_eq!((c.cc, c.cv1, c.cv2, c.spt), (5, 1, 3, 1));
        let c = configure(DevType::Mm200, 0, "6").unwrap();
        assert_eq!((c.cc, c.cv1, c.cv2, c.spt), (6, 2, 4, 2));
        let c = configure(DevType::Mm200, 0, "3").unwrap();
        assert_eq!((c.cc, c.cv1, c.cv2, c.spt), (3, 1, 0, 1));
    }

    #[test]
    fn mm200_four_station_form_is_taken_verbatim() {
        let c = configure(DevType::Mm200, 0, "7 2 4 2").unwrap();
        assert_eq!((c.cc, c.cv1, c.cv2, c.spt), (7, 2, 4, 2));
    }

    #[test]
    fn mm200_rejects_rs485_and_bad_stations() {
        assert!(configure(DevType::Mm200, 1, "5").is_err());
        assert!(configure(DevType::Mm200, 0, "2").is_err());
        assert!(configure(DevType::Mm200, 0, "3 1").is_err());
        assert!(configure(DevType::Mm200, 0, "9 7 0 1").is_err());
        assert!(configure(DevType::Mm200, 0, "9 1 0 3").is_err());
    }

    #[test]
    fn mx200_clamps_out_of_range_convectrons_to_absent() {
        let c = configure(DevType::Mx200, 0, "4 0 2 2").unwrap();
        assert_eq!((c.cc, c.cv1, c.cv2, c.spt), (4, 0, 2, 2));
    }

    // ---- command formatting ---------------------------------------------

    #[test]
    fn gp307_control_commands_cover_ig1_ig2_degas() {
        let c = configure(DevType::Gp307, 0, "").unwrap();
        assert_eq!(control_command(&c, 0).unwrap(), b"IG1 OFF");
        assert_eq!(control_command(&c, 1).unwrap(), b"IG1 ON");
        assert_eq!(control_command(&c, 3).unwrap(), b"IG2 ON");
        assert_eq!(control_command(&c, 5).unwrap(), b"DG ON");
    }

    #[test]
    fn gp350_control_commands_carry_the_address_prefix() {
        let c = configure(DevType::Gp350, 1, "").unwrap();
        assert_eq!(control_command(&c, 1).unwrap(), b"#01F1 1");
        assert_eq!(control_command(&c, 4).unwrap(), b"#01DG0 OFF");
    }

    #[test]
    fn televac_devices_have_no_control_commands() {
        for (dev, param) in [
            (DevType::Mm200, "5"),
            (DevType::Cc10, ""),
            (DevType::Mx200, "3 1 0 1"),
        ] {
            let c = configure(dev, 0, param).unwrap();
            assert!(control_command(&c, 1).is_none());
        }
    }

    #[test]
    fn gp307_read_commands() {
        let c = configure(DevType::Gp307, 0, "").unwrap();
        assert_eq!(read_command(&c, 0), b"PCS");
        assert_eq!(read_command(&c, 1), b"DGS");
        assert_eq!(read_command(&c, 2), b"DS IG1");
        assert_eq!(read_command(&c, 5), b"DS CG2");
        assert!(skips_read(&c, 6));
    }

    #[test]
    fn gp350_read_commands_concatenate_prefix_without_a_space() {
        let c = configure(DevType::Gp350, 1, "").unwrap();
        assert_eq!(read_command(&c, 0), b"#01PC S");
        assert_eq!(read_command(&c, 4), b"#01RD A");
    }

    #[test]
    fn mm200_read_commands_append_station_numbers() {
        let c = configure(DevType::Mm200, 0, "5").unwrap(); // cc=5 cv1=1 cv2=3 spt=1
        assert_eq!(read_command(&c, 0), b"RY");
        assert_eq!(read_command(&c, 1), b"R5");
        assert_eq!(read_command(&c, 2), b"R1");
        assert_eq!(read_command(&c, 3), b"R3");
        assert_eq!(read_command(&c, 4), b"SP1N");
        assert_eq!(read_command(&c, 5), b"SP3N");
        assert_eq!(read_command(&c, 6), b"SP5N");
        assert_eq!(read_command(&c, 7), b"SP7N");
    }

    #[test]
    fn mm200_skips_the_second_convectron_when_absent() {
        let c = configure(DevType::Mm200, 0, "3").unwrap(); // cv2 = 0
        assert!(skips_read(&c, 3));
        let c = configure(DevType::Mm200, 0, "5").unwrap();
        assert!(!skips_read(&c, 3));
    }

    #[test]
    fn cc10_read_commands_carry_the_stx_prefix_and_stop_after_slot_four() {
        let c = configure(DevType::Cc10, 3, "").unwrap();
        assert_eq!(read_command(&c, 0), b"\x023S5");
        assert_eq!(read_command(&c, 4), b"\x023R4");
        assert!(skips_read(&c, 5));
    }

    #[test]
    fn mx200_read_commands_pad_stations_to_two_digits_and_never_skip() {
        let c = configure(DevType::Mx200, 0, "3 1 0 1").unwrap();
        assert_eq!(read_command(&c, 0), b"S5");
        assert_eq!(read_command(&c, 1), b"S103");
        assert_eq!(read_command(&c, 2), b"S101");
        // Slot 3 is issued with a zero station even though CV2 is absent.
        assert_eq!(read_command(&c, 3), b"S100");
        assert!(!skips_read(&c, 3));
        assert_eq!(read_command(&c, 4), b"R71");
        assert_eq!(read_command(&c, 7), b"R77");
    }

    #[test]
    fn degas_reply_is_placed_after_the_setpoint_status_for_the_gp_family() {
        assert_eq!(place_offset(DevType::Gp307, 1), 15);
        assert_eq!(place_offset(DevType::Gp350, 1), 15);
        assert_eq!(place_offset(DevType::Gp307, 2), 20);
        assert_eq!(place_offset(DevType::Mm200, 1), 10);
    }

    // ---- reply stripping -------------------------------------------------

    #[test]
    fn gp307_flags_a_trailing_error_word() {
        assert!(strip_read_reply(DevType::Gp307, 0, b"SYNTX ERROR").is_err());
        assert_eq!(
            strip_read_reply(DevType::Gp307, 0, b"1,0,1,0,0,1").unwrap(),
            b"1,0,1,0,0,1"
        );
    }

    #[test]
    fn gp350_flags_a_leading_question_mark_and_strips_two_header_chars() {
        assert!(strip_read_reply(DevType::Gp350, 0, b"?01 SYNTX").is_err());
        assert_eq!(
            strip_read_reply(DevType::Gp350, 0, b"* 1010").unwrap(),
            b"1010"
        );
    }

    #[test]
    fn mm200_flags_a_question_mark_in_the_second_column() {
        assert!(strip_read_reply(DevType::Mm200, 1, b"R?").is_err());
        assert_eq!(
            strip_read_reply(DevType::Mm200, 1, b"5=1.23-7T").unwrap(),
            b"5=1.23-7T"
        );
    }

    #[test]
    fn cc10_flags_n_in_the_third_column_and_strips_stx_addr_cmd() {
        assert!(strip_read_reply(DevType::Cc10, 1, b"\x023N0001").is_err());
        assert_eq!(
            strip_read_reply(DevType::Cc10, 1, b"\x023S1005").unwrap(),
            b"1005"
        );
    }

    #[test]
    fn mx200_flags_n_in_the_second_column() {
        assert!(strip_read_reply(DevType::Mx200, 1, b"0N0001").is_err());
    }

    #[test]
    fn mx200_relay_status_recodes_into_two_hex_digits() {
        // Relays 1, 2 and 8 on -> bits 0,1,7 -> 0x83.
        let reply = b"01=ON 02=ON 03=OF 04=OF 05=00 06=00 07=OF 08=ON";
        assert_eq!(strip_read_reply(DevType::Mx200, 0, reply).unwrap(), b"83");
        // Only relay 1 -> 0x01, which "%2x" renders space-padded.
        let reply = b"01=ON 02=OF 03=OF 04=OF 05=OF 06=OF 07=OF 08=OF";
        assert_eq!(strip_read_reply(DevType::Mx200, 0, reply).unwrap(), b" 1");
    }

    #[test]
    fn mx200_setpoint_reply_drops_the_trailing_station_characters() {
        assert_eq!(
            strip_read_reply(DevType::Mx200, 4, b"1010512105ZZ").unwrap(),
            b"1010512105"
        );
    }

    #[test]
    fn control_reply_checks() {
        assert!(check_control_reply(DevType::Gp307, b"OK").is_ok());
        assert!(check_control_reply(DevType::Gp307, b"ERROR").is_err());
        assert!(check_control_reply(DevType::Gp350, b"* ").is_ok());
        assert!(check_control_reply(DevType::Gp350, b"?01").is_err());
        // The Televac family never issues one.
        assert!(check_control_reply(DevType::Mm200, b"anything").is_ok());
    }

    // ---- decoding --------------------------------------------------------

    #[test]
    fn gp307_decodes_six_setpoints_degas_two_ion_and_two_convectron_gauges() {
        let buf = buf_from(&[
            (0, b"1,0,1,0,0,1"),
            (15, b"1"),
            (20, b"1.23E-07"),
            (30, b"5.00E+00"), // IG2 off: >= 1.0, so ig2r stays 0
            (40, b"7.60E+02"),
            (50, b"1.00E-03"),
        ]);
        let r = decode(DevType::Gp307, 0, &buf, &Readings::default());
        assert_eq!(r.sp, [1, 0, 1, 0, 0, 1]);
        assert_eq!(r.dgsr, 1);
        assert_eq!(r.ig1r, 1);
        assert_eq!(r.ig2r, 0);
        assert!((r.val - 1.23e-7).abs() < 1e-13);
        assert!((r.cgap - 760.0).abs() < 1e-3);
        assert!((r.cgbp - 1.0e-3).abs() < 1e-9);
    }

    #[test]
    fn gp307_second_ion_gauge_wins_the_pressure_when_both_are_on() {
        let buf = buf_from(&[(20, b"1.00E-07"), (30, b"2.00E-08")]);
        let r = decode(DevType::Gp307, 0, &buf, &Readings::default());
        assert_eq!((r.ig1r, r.ig2r), (1, 1));
        assert!((r.val - 2.0e-8).abs() < 1e-14);
    }

    #[test]
    fn gp307_with_both_ion_gauges_off_parks_the_pressure_at_the_sentinel() {
        let buf = buf_from(&[(20, b"9.90E+09"), (30, b"9.90E+09")]);
        let r = decode(DevType::Gp307, 0, &buf, &Readings::default());
        assert_eq!((r.ig1r, r.ig2r), (0, 0));
        assert_eq!(r.val, 9.9e9);
    }

    #[test]
    fn gp350_decodes_four_setpoints_from_adjacent_characters() {
        let buf = buf_from(&[(0, b"1011"), (15, b"0"), (20, b"4.50E-09")]);
        let r = decode(DevType::Gp350, 0, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[1, 0, 1, 1]);
        assert_eq!(r.dgsr, 0);
        assert!((r.val - 4.5e-9).abs() < 1e-15);
    }

    #[test]
    fn mm200_decodes_relay_bits_from_the_first_setpoint_station() {
        // spt = 1 -> SP1..SP4 read bits 0,2,4,6. 0x55 = 0b01010101.
        let buf = buf_from(&[(0, b"55")]);
        let r = decode(DevType::Mm200, 1, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[1, 1, 1, 1]);
        // spt = 2 -> bits 1,3,5,7 -> all zero for 0x55.
        let r = decode(DevType::Mm200, 2, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[0, 0, 0, 0]);
    }

    #[test]
    fn mm200_treats_an_n_relay_nibble_as_zero() {
        let buf = buf_from(&[(0, b"n3")]);
        let r = decode(DevType::Mm200, 1, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[1, 0, 0, 0]); // 0x03 -> bits 0,2,4,6
    }

    #[test]
    fn mm200_decodes_gauge_and_setpoint_pressures() {
        let buf = buf_from(&[
            (0, b"00"),
            (10, b"5=1.23-7T"),
            (20, b"1=7.60+2T"),
            (30, b"3=1.00-3T"),
            (40, b"1.0-3"),
            (50, b"1.50-4"),
        ]);
        let r = decode(DevType::Mm200, 1, &buf, &Readings::default());
        assert!((r.val - 1.23e-7).abs() < 1e-12);
        assert_eq!(r.ig1r, 1);
        assert!((r.cgap - 760.0).abs() < 1e-2);
        assert!((r.cgbp - 1.0e-3).abs() < 1e-8);
        assert!((r.spr[0] - 1.0e-3).abs() < 1e-8);
        assert!((r.spr[1] - 1.5e-4).abs() < 1e-9);
    }

    #[test]
    fn mm200_missing_terminator_falls_back_to_the_overrange_sentinel() {
        // Slot 3 was skipped (no CV2), so its ten bytes are still zero and the
        // 'T' terminator is absent: C substitutes 9.9e+9.
        let buf = buf_from(&[(0, b"00"), (10, b"5=1.23-7T"), (20, b"1=7.60+2T")]);
        let r = decode(DevType::Mm200, 1, &buf, &Readings::default());
        // C's sentinel is built as `(float)9.9 * pow(10, 9)` stored back into a
        // float, so it is not the exact double 9.9e9.
        assert_eq!(r.cgbp, (9.9f32 as f64 * 1e9) as f32 as f64);
    }

    #[test]
    fn mm200_hex_exponent_letters_decode() {
        // "1.00-aT" -> 1.00 * 10^-10.
        let buf = buf_from(&[(0, b"00"), (10, b"5=1.00-aT")]);
        let r = decode(DevType::Mm200, 1, &buf, &Readings::default());
        assert!((r.val - 1.0e-10).abs() < 1e-16);
    }

    #[test]
    fn cc10_decodes_four_setpoint_flags_and_four_pressures() {
        let buf = buf_from(&[
            (0, b"1010"),
            (10, b"1005"), // 1.0 * 10^-5
            (20, b"5004"), // 5.0 * 10^-4
            (30, b"2113"), // 2.1 * 10^+3
            (40, b"1000"), // 1.0 * 10^-0
        ]);
        let r = decode(DevType::Cc10, 0, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[1, 0, 1, 0]);
        assert!((r.val - 1.0e-5).abs() < 1e-11);
        assert_eq!(r.ig1r, 1);
        assert!((r.spr[0] - 5.0e-4).abs() < 1e-10);
        assert!((r.spr[1] - 2.1e3).abs() < 1e-2);
        assert!((r.spr[2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn mx200_decodes_lo_resolution_gauges() {
        // "ppsee": 12 -> 1.2, s='0' -> negative, ee=07.
        let buf = buf_from(&[(0, b"03"), (10, b"12007"), (20, b"76102"), (30, b"10003")]);
        let r = decode(DevType::Mx200, 1, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[1, 0, 0, 0]);
        assert!((r.val - 1.2e-7).abs() < 1e-13);
        assert_eq!(r.ig1r, 1);
        assert!((r.cgap - 760.0).abs() < 1e-2);
        assert!((r.cgbp - 1.0e-3).abs() < 1e-9);
    }

    #[test]
    fn mx200_auto_detects_hi_resolution_on_gauge_slots() {
        // "pppsee": 123 -> 1.23, s='0' -> negative, ee=07.
        let buf = buf_from(&[(0, b"00"), (10, b"123007")]);
        let r = decode(DevType::Mx200, 1, &buf, &Readings::default());
        assert!((r.val - 1.23e-7).abs() < 1e-13);
    }

    #[test]
    fn mx200_setpoint_readbacks_are_always_lo_resolution() {
        // Slot 4 carries "ppseePPSEE"; only the leading LO group is decoded.
        let buf = buf_from(&[(0, b"00"), (40, b"1000512105")]);
        let r = decode(DevType::Mx200, 1, &buf, &Readings::default());
        assert!((r.spr[0] - 1.0e-5).abs() < 1e-11);
    }

    #[test]
    fn mx200_positive_exponent_sign_character_is_one() {
        let buf = buf_from(&[(0, b"00"), (20, b"20103")]);
        let r = decode(DevType::Mx200, 1, &buf, &Readings::default());
        assert!((r.cgap - 2.0e3).abs() < 1e-3);
    }

    // ---- sticky sscanf targets -------------------------------------------
    //
    // C declares `int value; float fvalue;` once per `readWrite_vs` call, so a
    // failed conversion leaves the previous field's value in place. These cases
    // pin that behaviour: a truncated reply must repeat the last good value,
    // not read as zero.

    #[test]
    fn gp350_short_setpoint_status_repeats_the_last_digit() {
        // The stripped reply is a single '1', so the sscanf for SP2..SP4 sees
        // NUL and fails: C leaves `value` at 1 for all four.
        let buf = buf_from(&[(0, b"1")]);
        let r = decode(DevType::Gp350, 0, &buf, &Readings::default());
        assert_eq!(&r.sp[..4], &[1, 1, 1, 1]);
    }

    #[test]
    fn gp307_empty_convectron_reply_repeats_the_last_pressure() {
        // CG1/CG2 never answered; C's `fvalue` still holds the IG2 reading.
        let buf = buf_from(&[(20, b"1.00E-07"), (30, b"2.00E-08")]);
        let r = decode(DevType::Gp307, 0, &buf, &Readings::default());
        assert!((r.cgap - 2.0e-8).abs() < 1e-14);
        assert!((r.cgbp - 2.0e-8).abs() < 1e-14);
    }

    #[test]
    fn cc10_empty_setpoint_reply_repeats_the_last_value_and_exponent() {
        let buf = buf_from(&[(0, b"1010"), (10, b"5004")]);
        let r = decode(DevType::Cc10, 0, &buf, &Readings::default());
        assert!((r.val - 5.0e-4).abs() < 1e-10);
        // Slots 2..4 fail to convert: value=50, sign='0', exp=4 all persist.
        assert!((r.spr[0] - 5.0e-4).abs() < 1e-10);
        assert!((r.spr[1] - 5.0e-4).abs() < 1e-10);
        assert!((r.spr[2] - 5.0e-4).abs() < 1e-10);
    }

    #[test]
    fn mx200_empty_gauge_reply_repeats_the_relay_status_as_a_mantissa() {
        // The relay `%x` and the gauge `%2d` share C's `value`. With no gauge
        // reply, the pressure is decoded from the leftover relay word.
        let buf = buf_from(&[(0, b"20")]);
        let r = decode(DevType::Mx200, 1, &buf, &Readings::default());
        // 0x20 = 32 -> 3.2, sign '+' (never overwritten), exp 0.
        assert!((r.val - 3.2).abs() < 1e-6);
        assert_eq!(r.ig1r, 0);
    }

    #[test]
    fn mm200_setpoint_scaling_narrows_through_double_not_float() {
        // C: `fvalue = fvalue * pow(10, exp)` — the product is formed in double
        // and only then stored back into the float. Computing 10^-7 in f32 and
        // multiplying would land on a different float.
        let buf = buf_from(&[(0, b"00"), (10, b"5=1.23-7T")]);
        let r = decode(DevType::Mm200, 1, &buf, &Readings::default());
        assert_eq!(r.val, (1.23f32 as f64 * 1e-7) as f32 as f64);
    }
}
