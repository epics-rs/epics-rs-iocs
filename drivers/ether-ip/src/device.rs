//! EPICS device support -- port of `devEtherIP.c`.
//!
//! One DTYP, `EtherIP`, for every record type, plus `EtherIPReset` for the
//! statistics-reset `bo`. The INST_IO link is
//!
//! ```text
//! @<PLC> <tag>[<element>] [flags...]
//! ```
//!
//! with the same flags as the C: `E`, `S <period>`, `B <bit>`, `FORCE`, and the
//! statistics selectors (`PLC_ERRORS`, `LIST_SCAN_TIME`, ...).

use std::sync::Arc;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::base::error::{CaError, CaResult, LinkRefusal};
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::base::server::database::{AsyncDbHandle, DeviceDoneSource, ProcessCallback};
use epics_rs::base::server::device_support::{
    DeviceAlarm, DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, IoIntrAction,
    WriteStart,
};
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::recgbl::{alarm_status, rec_gbl_record_error};
use epics_rs::base::server::record::{
    AlarmSeverity, CommonFields, Record, SCAN_1ST_PERIODIC, ScanType, menu_scan,
};
use epics_rs::base::types::EpicsValue;

use crate::cip::{self, CipType};
use crate::driver::{self, Listener, Plc, TagInfo};

pub const DTYP: &str = "EtherIP";
pub const DTYP_RESET: &str = "EtherIPReset";

/// The `stringin`/`stringout` value field is `DBR_STRING` -- 40 bytes.
const MAX_STRING_SIZE: usize = 40;

// ---------------------------------------------------------------------------
// Link flags (C `SpecialOptions`)
// ---------------------------------------------------------------------------

mod spco {
    pub const READ_SINGLE_ELEMENT: u32 = 1 << 0;
    pub const SCAN_PERIOD: u32 = 1 << 1;
    pub const BIT: u32 = 1 << 2;
    pub const FORCE: u32 = 1 << 3;
    pub const INDEX_INCLUDED: u32 = 1 << 4;
    pub const PLC_ERRORS: u32 = 1 << 6;
    pub const PLC_TASK_SLOW: u32 = 1 << 7;
    pub const LIST_ERRORS: u32 = 1 << 8;
    pub const LIST_TICKS: u32 = 1 << 9;
    pub const LIST_SCAN_TIME: u32 = 1 << 10;
    pub const LIST_MIN_SCAN_TIME: u32 = 1 << 11;
    pub const LIST_MAX_SCAN_TIME: u32 = 1 << 12;
    pub const TAG_TRANSFER_TIME: u32 = 1 << 13;
    pub const LIST_TIME: u32 = 1 << 14;

    /// Every flag at or above this bit selects a driver statistic instead of a
    /// PLC tag value (C: `pvt->special < SPCO_PLC_ERRORS`).
    pub const FIRST_STATISTIC: u32 = PLC_ERRORS;
}

/// A parsed INST_IO link.
#[derive(Clone, Debug, PartialEq)]
pub struct Link {
    pub plc_name: String,
    /// The tag with any trailing `[index]` removed.
    pub string_tag: String,
    /// Element index within the tag's array (already folded to a UDINT index
    /// for bit records).
    pub element: usize,
    /// First bit of interest, for the (multi-)bit records.
    pub mask: u32,
    pub special: u32,
    /// The scan period: the `S` flag's, else the record's SCAN, else the
    /// default rate.
    pub period: f64,
    /// How many elements the driver must fetch for this record.
    pub elements: usize,
}

impl Link {
    pub fn is_statistic(&self) -> bool {
        self.special >= spco::FIRST_STATISTIC
    }
    fn forced(&self) -> bool {
        self.special & spco::FORCE != 0
    }
}

/// C `special_options`, matched in this order as a prefix of the rest of the
/// link: `S ` and `B ` need the space, every other flag matches any token it
/// begins.
const SPECIAL_OPTIONS: [(&str, u32); 13] = [
    ("E", spco::READ_SINGLE_ELEMENT),
    ("S ", spco::SCAN_PERIOD),
    ("B ", spco::BIT),
    ("FORCE", spco::FORCE),
    ("PLC_ERRORS", spco::PLC_ERRORS),
    ("PLC_TASK_SLOW", spco::PLC_TASK_SLOW),
    ("LIST_ERRORS", spco::LIST_ERRORS),
    ("LIST_TICKS", spco::LIST_TICKS),
    ("LIST_SCAN_TIME", spco::LIST_SCAN_TIME),
    ("LIST_MIN_SCAN_TIME", spco::LIST_MIN_SCAN_TIME),
    ("LIST_MAX_SCAN_TIME", spco::LIST_MAX_SCAN_TIME),
    ("TAG_TRANSFER_TIME", spco::TAG_TRANSFER_TIME),
    ("LIST_TIME", spco::LIST_TIME),
];

/// C `find_token`: skip spaces, then the token runs to the next space.
fn find_token(text: &str, from: usize) -> Option<(usize, usize)> {
    let start = from + text[from..].len() - text[from..].trim_start_matches(' ').len();
    if start >= text.len() {
        return None;
    }
    let end = text[start..].find(' ').map_or(text.len(), |n| start + n);
    Some((start, end))
}

/// C `analyze_link` on `@PLC tag[element] flags...`, failing with the text C
/// prints after `devEtherIP (<record>): `.
///
/// `count` is the number of array elements the record wants (1, or NELM for a
/// waveform); `bits` is >0 for the (multi-)bit records and is then the number
/// of bits (NOBT, or 1 for bi/bo). C resolves the period and finds the PLC
/// between its parsing steps, so a link that fails later still printed those
/// steps' lines first: `period_of` runs when no positive `S` period was
/// given, `find_plc` once the tag is split.
pub fn analyze_link<P>(
    text: &str,
    count: usize,
    bits: usize,
    period_of: impl FnOnce() -> f64,
    find_plc: impl FnOnce(&str) -> Option<P>,
) -> Result<(Link, P), String> {
    let text = text.strip_prefix('@').unwrap_or(text);
    let (p, end) = find_token(text, 0).ok_or_else(|| format!("Missing PLC in link '{text}'"))?;
    let plc_name = text[p..end].to_string();
    let (p, mut end) =
        find_token(text, end).ok_or_else(|| format!("Missing tag in link '{text}'"))?;
    let tag_text = &text[p..end];

    let mut special = 0u32;
    let mut period = 0.0f64;
    let mut bit = 0usize;
    let mut single_element = false;
    while let Some((p, e)) = find_token(text, end) {
        end = e;
        let rest = &text[p..];
        let Some(&(_, mask)) = SPECIAL_OPTIONS.iter().find(|(o, _)| rest.starts_with(o)) else {
            return Err(format!("Invalid flag '{rest}' in link '{text}'"));
        };
        special |= mask;
        match mask {
            spco::READ_SINGLE_ELEMENT => {
                if count != 1 {
                    return Err(format!("Array record cannot use 'E' flag ('{text}')"));
                }
                single_element = true;
            }
            // strtod: no number leaves 0 and resumes the scan right after the
            // flag, so the unparsed text comes back as the next token.
            spco::SCAN_PERIOD => {
                let (v, n) = strtod(&text[p + 2..]);
                if v.is_infinite() {
                    return Err(format!("Error in scan flag in link '{text}'"));
                }
                period = v;
                end = p + 2 + n;
            }
            // C tests `period` here, not `bit` (devEtherIP.c:974), and an
            // infinite period already failed above: a bit flag never fails.
            spco::BIT => {
                let (v, n) = strtod(&text[p + 2..]);
                // Only the low five bits reach the mask.
                bit = size_t_from_double(v) as usize;
                end = p + 2 + n;
            }
            _ => {}
        }
    }

    // C keeps a NaN period (`period <= 0.0` is false); a NaN cannot schedule
    // a scan list here, so it falls through like any non-positive one.
    if period <= 0.0 || period.is_nan() {
        period = period_of();
    }

    // Split "array_tag[el]" into "array_tag" + el, unless 'E' asked for the
    // element to stay in the tag path (so the PLC does the indexing).
    let mut string_tag = tag_text.to_string();
    let mut element = 0usize;
    if tag_text.ends_with(']') && !single_element {
        let malformed = || format!("malformed array tag in '{text}'");
        let open = match tag_text[..tag_text.len() - 1].rfind('[') {
            Some(open) if open > 0 => open,
            _ => return Err(malformed()),
        };
        let (v, n) = strtol(&tag_text[open + 1..]);
        if n == 0 || v == i64::MAX || v == i64::MIN {
            return Err(malformed());
        }
        // A negative index wraps as C's size_t does.
        element = v as usize;
        special |= spco::INDEX_INCLUDED;
        string_tag.truncate(open);
    }

    let plc = find_plc(&plc_name).ok_or_else(|| format!("unknown PLC '{plc_name}'"))?;

    if count > 1 && (bits > 0 || special & spco::BIT != 0) {
        return Err("cannot access bits for array records".to_string());
    }

    let mask;
    let last_element;
    if bits > 0 && special & spco::BIT == 0 {
        // A binary record with no explicit bit number: an indexed tag is a
        // BOOL array, which Logix packs into UDINTs, so the bit index folds
        // into a UDINT index plus a mask; an unindexed one reads from bit 0.
        // C defect not copied: devEtherIP.c:1101 masks it 255, so bit 0 of a
        // bi read as the OR of bits 0-7 and a bo wrote 255.
        mask = if special & spco::INDEX_INCLUDED != 0 {
            1u32 << (element & 0x1F)
        } else {
            1
        };
        last_element = element.wrapping_add(bits - 1) >> 5;
        element >>= 5;
    } else {
        // No binary record, or an explicit bit number: the element is a real
        // array index and the bit is selected by `B <bit>` (x86 `shl` takes
        // the count mod 32).
        last_element = element;
        mask = 1u32.wrapping_shl(bit as u32);
    }

    let link = Link {
        plc_name,
        string_tag,
        element,
        mask,
        special,
        period,
        elements: last_element.wrapping_add(count),
    };
    Ok((link, plc))
}

/// C's `size_t bit = strtod(...)` as gcc compiles it for x86-64: a negative
/// value wraps (so `B -1` selects bit 31 through `1U << bit`), and NaN or
/// anything past 2^64 becomes 2^63.
fn size_t_from_double(v: f64) -> u64 {
    const TWO_63: f64 = 9_223_372_036_854_775_808.0;
    if v > -TWO_63 && v < TWO_63 {
        v as i64 as u64
    } else if (TWO_63..2.0 * TWO_63).contains(&v) {
        ((v - TWO_63) as i64 as u64) | (1 << 63)
    } else {
        1 << 63
    }
}

/// C `strtod`: the value and how many bytes it took, `(0.0, 0)` when it
/// takes nothing.
fn strtod(text: &str) -> (f64, usize) {
    let t = text.trim_start();
    let skipped = text.len() - t.len();
    (1..=t.len())
        .rev()
        .filter(|&n| t.is_char_boundary(n))
        .find_map(|n| t[..n].parse::<f64>().ok().map(|v| (v, skipped + n)))
        .unwrap_or((0.0, 0))
}

/// C `strtol(text, &end, 0)`: the value, saturated to `i64::MIN..=MAX` as
/// `LONG_MIN`/`LONG_MAX`, and how many bytes it took (0 for none).
fn strtol(text: &str) -> (i64, usize) {
    let b = text.as_bytes();
    let mut i = b.iter().take_while(|c| c.is_ascii_whitespace()).count();
    let negative = match b.get(i) {
        Some(b'-') => {
            i += 1;
            true
        }
        Some(b'+') => {
            i += 1;
            false
        }
        _ => false,
    };
    let (radix, digits_at) = match (b.get(i), b.get(i + 1)) {
        (Some(b'0'), Some(b'x' | b'X')) if b.get(i + 2).is_some_and(u8::is_ascii_hexdigit) => {
            (16, i + 2)
        }
        (Some(b'0'), _) => (8, i),
        _ => (10, i),
    };
    let n = b[digits_at..]
        .iter()
        .take_while(|c| (**c as char).is_digit(radix))
        .count();
    if n == 0 {
        return (0, 0);
    }
    let digits = &text[digits_at..digits_at + n];
    let v = i128::from_str_radix(digits, radix).unwrap_or(i128::MAX);
    let v = if negative { -v } else { v };
    (
        v.clamp(i64::MIN.into(), i64::MAX.into()) as i64,
        digits_at + n,
    )
}

// ---------------------------------------------------------------------------
// Bit packing (C `get_bits` / `put_bits`)
// ---------------------------------------------------------------------------

/// Gather `bits` consecutive bits out of the BOOL array starting at
/// `element`/`mask`, packing them into an RVAL.
fn get_bits(raw: &[u8], mut element: usize, mut mask: u32, bits: usize) -> Option<u32> {
    let mut value = cip::get_udint(raw, element)?;
    let mut rval = 0u32;
    // C defect not copied: devEtherIP.c:213-215 tests a multi-bit read's
    // first bit as `(value & 0x1) & mask`, bit 0 instead of the mask's bit.
    if value & mask != 0 {
        rval |= 1;
    }
    for i in 1..bits {
        mask <<= 1;
        if mask == 0 {
            // Ran off the end of this UDINT.
            mask = 1;
            element += 1;
            value = cip::get_udint(raw, element)?;
        }
        if value & mask != 0 {
            rval |= 1 << i;
        }
    }
    Some(rval)
}

/// The inverse: scatter an RVAL's low `bits` bits back into the BOOL array.
fn put_bits(raw: &mut [u8], mut element: usize, mut mask: u32, bits: usize, rval: u32) -> bool {
    let Some(mut value) = cip::get_udint(raw, element) else {
        return false;
    };
    let mut rval = rval;

    if rval & 1 != 0 {
        value |= mask;
    } else {
        value &= !mask;
    }
    for _ in 1..bits {
        rval >>= 1;
        mask <<= 1;
        if mask == 0 {
            if !cip::put_udint(raw, element, value) {
                return false;
            }
            mask = 1;
            element += 1;
            let Some(v) = cip::get_udint(raw, element) else {
                return false;
            };
            value = v;
        }
        if rval & 1 != 0 {
            value |= mask;
        } else {
            value &= !mask;
        }
    }
    cip::put_udint(raw, element, value)
}

// ---------------------------------------------------------------------------
// Device support
// ---------------------------------------------------------------------------

pub struct EtherIpDevice {
    /// The record name, for C's `rec->name` in the lines it prints.
    name: String,
    /// The record's SCAN, for C's `get_period`.
    scan: ScanType,
    link_text: String,
    is_output: bool,
    link: Option<Link>,
    plc: Option<Arc<Plc>>,
    tag: Option<Arc<TagInfo>>,
    listener: Option<Arc<Listener>>,
    io_intr_rx: Option<mpsc::Receiver<()>>,
    alarm: Option<(u16, u16)>,
    /// For the `scanOnce` an output's PLC callback ends with.
    db: Option<AsyncDbHandle>,
    /// C's `rset->process` from an output's PLC callback that finds no data.
    process: Option<ProcessCallback>,
}

impl EtherIpDevice {
    fn new(link_text: &str, is_output: bool) -> EtherIpDevice {
        EtherIpDevice {
            name: String::new(),
            scan: ScanType::Passive,
            link_text: link_text.to_string(),
            is_output,
            link: None,
            plc: None,
            tag: None,
            listener: None,
            io_intr_rx: None,
            alarm: None,
            db: None,
            process: None,
        }
    }

    fn read_alarm(&mut self) {
        self.alarm = Some((alarm_status::READ_ALARM, AlarmSeverity::Invalid as u16));
    }
    fn write_alarm(&mut self) {
        self.alarm = Some((alarm_status::WRITE_ALARM, AlarmSeverity::Invalid as u16));
    }
    fn no_alarm(&mut self) {
        self.alarm = Some((alarm_status::NO_ALARM, AlarmSeverity::NoAlarm as u16));
    }

    /// C `init_record(rec, cbtype, link, count, bits)` (`devEtherIP.c:1183-1205`):
    /// bind this object's link for `record`, after the errlog line C prints.
    fn bind_link(&mut self, record: &dyn Record) -> Result<(), String> {
        let (bits, count) = record_shape(record);
        let name = self.name.clone();
        let fail = |msg: String| {
            errlog_vprintf(format_args!("devEtherIP ({name}): {msg}\n"));
            Err(msg)
        };
        if !driver::initialized() {
            return fail("Failed to all drvEtherIP_init()".to_string());
        }
        let (link, plc) = match analyze_link(
            &self.link_text,
            count,
            bits,
            // devEtherIP.c:1021-1035: a non-positive period at each step
            // falls through to the next.
            || {
                get_period(&name, self.scan)
                    .filter(|p| *p > 0.0)
                    .or_else(|| Some(driver::default_rate()).filter(|p| *p > 0.0))
                    .unwrap_or_else(|| {
                        errlog_vprintf(format_args!(
                            "devEtherIP ({name}): cannot decode SCAN field, no scan flag given\n"
                        ));
                        errlog_vprintf(format_args!(
                            "Device support will use the default of 1 secs, "
                        ));
                        errlog_vprintf(format_args!("please complete the record configuration\n"));
                        1.0
                    })
            },
            driver::find_plc,
        ) {
            Ok(found) => found,
            Err(msg) => return fail(msg),
        };

        let Some(tag) = plc.add_tag(link.period, &link.string_tag, link.elements) else {
            return fail(format!(
                "cannot register tag '{}' with driver",
                link.string_tag
            ));
        };

        // One I/O Intr pulse per new tag value. Depth 1: a pending pulse
        // already says "re-read", and the record reads the current value.
        // Whatever the SCAN: C adds `scan_callback` only to a record that is
        // I/O Intr when the link binds (`devEtherIP.c:1125-1133`) yet hands
        // its IOSCANPVT to `get_ioint_info` (`:790-796`), so a record switched
        // to I/O Intr later joins a list nothing ever posts. Not copied.
        let (tx, rx) = mpsc::channel(1);
        self.listener = Some(tag.add_listener(tx));
        self.io_intr_rx = Some(rx);

        self.link = Some(link);
        self.plc = Some(plc);
        self.tag = Some(tag);
        Ok(())
    }

    /// C `lock_data`'s first half: do we have a tag at all? Returns owned
    /// handles so the caller can still take `&mut self` for the alarm.
    fn bound(&self) -> Option<(Arc<TagInfo>, Link)> {
        Some((self.tag.clone()?, self.link.clone()?))
    }

    /// Read one driver statistic (C `ai_read`'s `special >= SPCO_PLC_ERRORS`
    /// branch).
    fn statistic(&self) -> Option<f64> {
        let plc = self.plc.as_ref()?;
        let link = self.link.as_ref()?;
        let s = link.special;

        if s & spco::PLC_ERRORS != 0 {
            return Some(plc.stats.lock().plc_errors as f64);
        }
        if s & spco::PLC_TASK_SLOW != 0 {
            return Some(plc.stats.lock().slow_scans as f64);
        }

        let tag = self.tag.as_ref()?;
        if s & spco::TAG_TRANSFER_TIME != 0 {
            return Some(tag.data.lock().transfer_time);
        }
        let stats = tag.list_stats();
        let stats = stats.lock();
        if s & spco::LIST_ERRORS != 0 {
            return Some(stats.list_errors as f64);
        }
        if s & (spco::LIST_TICKS | spco::LIST_TIME) != 0 {
            return Some(stats.scan_time);
        }
        if s & spco::LIST_SCAN_TIME != 0 {
            return Some(stats.last_scan_time);
        }
        if s & spco::LIST_MIN_SCAN_TIME != 0 {
            return Some(stats.min_scan_time);
        }
        if s & spco::LIST_MAX_SCAN_TIME != 0 {
            return Some(stats.max_scan_time);
        }
        None
    }
}

/// How many bits a record type reads/writes through the BOOL-array path, and
/// how many array elements it wants. Both come from the record, so this runs in
/// `init()` where the record is available.
fn record_shape(record: &dyn Record) -> (usize, usize) {
    match record.record_type() {
        "bi" | "bo" => (1, 1),
        "mbbi" | "mbbo" | "mbbiDirect" | "mbboDirect" => (count_field(record, "NOBT"), 1),
        "waveform" => (0, count_field(record, "NELM")),
        _ => (0, 1),
    }
}

/// C `get_period`: a periodic SCAN's menu label, when it says "second", read
/// as `strtod` reads it. Anything else is no period.
fn get_period(name: &str, scan: ScanType) -> Option<f64> {
    let ScanType::Menu(index) = scan else {
        return None;
    };
    let label = menu_scan().label_at(index)?;
    let period = label
        .contains("second")
        .then(|| strtod(label))
        .filter(|&(_, n)| n > 0)
        .map(|(v, _)| v)
        .filter(|p| p.is_finite());
    crate::eip_printf!(
        8,
        "EIP record '{name}' scans at {:.1} secs\n",
        CFloat(period.unwrap_or(-1.0))
    );
    period
}

impl DeviceSupport for EtherIpDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn set_record_info(&mut self, name: &str, scan: ScanType) {
        self.name = name.to_string();
        self.scan = scan;
    }

    fn bind_database(&mut self, db: AsyncDbHandle) {
        self.process = Some(db.process_callback(&self.name));
        self.db = Some(db);
    }

    // Every C init_record/analyze_link failure returns an `S_*` status after
    // an errlogPrintf, never touching pact (`devEtherIP.c:884-1007,1183-1205`)
    // — the record scans on: the `Err` shape.
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        // C's mbbi/mbbo(Direct) `init_record` zero SHFT (devEtherIP.c:1331,
        // 1355,1563,1597): the tag's NOBT bits are the raw value, unshifted.
        if matches!(
            record.record_type(),
            "mbbi" | "mbbo" | "mbbiDirect" | "mbboDirect"
        ) {
            record.put_field("SHFT", EpicsValue::UShort(0))?;
        }
        self.bind_link(record).map_err(CaError::InvalidValue)?;
        // devEtherIP.c ao/bo/mbbo/mbboDirect_init_record: "return 2; /* don't
        // convert, we have no value, yet */".
        if matches!(record.record_type(), "ao" | "bo" | "mbbo" | "mbboDirect") {
            return Ok(DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched));
        }
        Ok(DeviceInitOutcome::ok())
    }

    /// The `<type>_add_record` every dset but devBoEtherIPReset registers with
    /// `devExtend` (`devEtherIP.c:1210-1230` and its twins): `init_record`
    /// on the new link.
    fn add_record(&mut self, record: &mut dyn Record) -> Result<(), LinkRefusal> {
        self.bind_link(record)
            .map_err(|_| LinkRefusal::add_record("devEtherIP"))
    }

    /// `del_scan_callback_record` / `<type>_del_record`
    /// (`devEtherIP.c:1215-1223`, `:1409-1418`): the record leaves its tag.
    fn del_record(&mut self) -> Result<(), LinkRefusal> {
        println!("Updating link for {}", self.name);
        if let (Some(tag), Some(listener)) = (&self.tag, self.listener.take()) {
            tag.remove_listener(&listener);
        }
        Ok(())
    }

    fn io_intr_receiver(&mut self) -> Option<mpsc::Receiver<()>> {
        self.io_intr_rx.take()
    }

    /// Output records follow the PLC regardless of their SCAN, exactly as the
    /// C's `check_ao_callback` / `check_bo_callback` / ... do: the driver
    /// pushes each new tag value into the record without writing it back.
    fn io_intr_scan_independent(&self) -> bool {
        self.is_output
    }

    /// C `check_ao_callback` & friends (devEtherIP.c:326-794): copy a new
    /// PLC value into the record under its lock, then `scanOnce` it unless it
    /// is periodic. Its own process converts that value and its write pass
    /// finds the PLC agreeing. A failed get, or an unchanged value, leaves
    /// the record alone.
    fn io_intr_callback(
        &mut self,
        record: &mut dyn Record,
        common: &mut CommonFields,
    ) -> IoIntrAction {
        // Inputs: C's scan_callback is a bare scanIoRequest.
        if !self.is_output {
            return IoIntrAction::Process;
        }
        match self.check_callback(record, common) {
            Callback::ScanOnce => {
                if let Some(db) = &self.db {
                    db.scan_once(&self.name);
                }
            }
            // C calls `rset->process` under the record's lock, past every
            // dbProcess gate (devEtherIP.c:347-354). This pulse holds the
            // lock, so the record's ProcessCallback runs it on the record's
            // band as an async completion re-enters: past the PACT and TPRO
            // gates, not the DISA one.
            Callback::Process => {
                if let Some(process) = &self.process {
                    let _ = process.request(common.callback_priority());
                }
            }
            Callback::Leave => {}
        }
        IoIntrAction::Skip
    }

    fn last_alarm(&self) -> Option<DeviceAlarm<'_>> {
        let (stat, sevr) = self.alarm?;
        Some(DeviceAlarm {
            stat,
            sevr,
            msg: "",
        })
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        // Driver statistics do not touch the tag's value, but C reads them
        // inside `lock_data` (devEtherIP.c:1642), so a tag without data fails
        // them like any read: READ_ALARM and status 0, which converts RVAL.
        if self.link.as_ref().is_some_and(Link::is_statistic) {
            let has_data = self.bound().is_some_and(|(tag, link)| {
                tag.data.lock().has_value() && *tag.elements.lock() > link.element
            });
            if !has_data {
                self.read_alarm();
                return Ok(DeviceReadOutcome::ok());
            }
            return match self.statistic() {
                Some(v) => {
                    self.no_alarm();
                    record.set_val(EpicsValue::Double(v))?;
                    // C ai_read's statistics arm: udf = FALSE on success.
                    Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
                }
                None => {
                    self.read_alarm();
                    // C ai_read raises the alarm but leaves udf alone.
                    Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched))
                }
            };
        }

        let Some((tag, link)) = self.bound() else {
            self.read_alarm();
            return Ok(DeviceReadOutcome::ok());
        };
        let (element, mask) = (link.element, link.mask);
        let rtype = record.record_type();

        let outcome = {
            let data = tag.data.lock();
            let elements = *tag.elements.lock();
            if !data.has_value() || elements <= element {
                Err(())
            } else {
                let raw = &data.buf;
                match rtype {
                    "ai" => match cip::typecode(raw) {
                        Some(CipType::Real) | Some(CipType::Lreal) => cip::get_double(raw, element)
                            .map(|v| (EpicsValue::Double(v), true))
                            .ok_or(()),
                        _ => cip::get_dint(raw, element)
                            .map(|v| (EpicsValue::Long(v), false))
                            .ok_or(()),
                    },
                    "int64in" => cip::get_lint(raw, element)
                        .map(|v| (EpicsValue::Int64(v), true))
                        .ok_or(()),
                    "bi" => get_bits(raw, element, mask, 1)
                        .map(|v| (EpicsValue::Long(v as i32), false))
                        .ok_or(()),
                    "mbbi" | "mbbiDirect" => get_bits(raw, element, mask, nobt(record))
                        .map(|v| (EpicsValue::Long(v as i32), false))
                        .ok_or(()),
                    "stringin" | "lsi" => cip::get_string(raw, element, string_size(record))
                        .map(|s| (EpicsValue::String(s.into()), true))
                        .ok_or(()),
                    "waveform" => match read_waveform(raw, elements, record, &self.name) {
                        Ok(Some(v)) => Ok((v, true)),
                        Ok(None) => {
                            drop(data);
                            self.no_alarm();
                            return Ok(DeviceReadOutcome::ok());
                        }
                        Err(()) => Err(()),
                    },
                    _ => Err(()),
                }
            }
        };

        match outcome {
            Ok((value, computed)) => {
                self.no_alarm();
                if computed {
                    record.set_val(value)?;
                    // C parity: ai_read, si_read and lsi_read write
                    // udf = FALSE on a good read; int64in_read and wf_read
                    // never touch udf.
                    let udf = match rtype {
                        "int64in" | "waveform" => DeviceUdf::Untouched,
                        _ => DeviceUdf::Defined,
                    };
                    Ok(DeviceReadOutcome::computed(udf))
                } else {
                    record.put_field("RVAL", value)?;
                    Ok(DeviceReadOutcome::ok())
                }
            }
            Err(()) => {
                self.read_alarm();
                Ok(DeviceReadOutcome::ok())
            }
        }
    }

    fn write(&mut self, record: &mut dyn Record) -> CaResult<()> {
        self.stage_write(record, None).map(|_| ())
    }

    /// C's `*_write` first pass: a staged write leaves the record PACT until
    /// the driver's next transfer ends it, without a readback.
    fn write_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<WriteStart> {
        Ok(if self.stage_write(record, Some(done))? {
            WriteStart::Pending
        } else {
            WriteStart::Completed
        })
    }
}

impl EtherIpDevice {
    /// Stage the record's value in the tag buffer and flag the write; true
    /// when a write was flagged and `done` now waits on the next transfer.
    fn stage_write(
        &mut self,
        record: &mut dyn Record,
        done: Option<&DeviceDoneSource<'_>>,
    ) -> CaResult<bool> {
        let Some((tag, link)) = self.bound() else {
            self.write_alarm();
            return Ok(false);
        };
        let (element, mask) = (link.element, link.mask);
        let rtype = record.record_type();

        let mut data = tag.data.lock();
        let elements = *tag.elements.lock();
        if !data.has_value() || elements <= element {
            drop(data);
            self.write_alarm();
            return Ok(false);
        }

        // Stage the record's value in the tag buffer, but only when it differs
        // from what the PLC last reported -- otherwise every process would
        // queue a redundant write (C ao_write/bo_write/... all compare first).
        let changed = match rtype {
            "ao" => match cip::typecode(&data.buf) {
                Some(CipType::Real) | Some(CipType::Lreal) => {
                    let val = f64_field(record, "VAL");
                    match cip::get_double(&data.buf, element) {
                        Some(cur) if cur == val => false,
                        Some(_) => cip::put_double(&mut data.buf, element, val),
                        None => false,
                    }
                }
                _ => {
                    let rval = i32_field(record, "RVAL");
                    match cip::get_dint(&data.buf, element) {
                        Some(cur) if cur == rval => false,
                        Some(_) => cip::put_dint(&mut data.buf, element, rval),
                        None => false,
                    }
                }
            },
            "int64out" => {
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::Int64(v)) => v,
                    _ => 0,
                };
                match cip::get_lint(&data.buf, element) {
                    Some(cur) if cur == val => false,
                    Some(_) => cip::put_lint(&mut data.buf, element, val),
                    None => false,
                }
            }
            "bo" | "mbbo" | "mbboDirect" => {
                let bits = if rtype == "bo" { 1 } else { nobt(record) };
                let rval = i32_field(record, "RVAL") as u32;
                match get_bits(&data.buf, element, mask, bits) {
                    Some(cur) if cur == rval => false,
                    Some(_) => put_bits(&mut data.buf, element, mask, bits, rval),
                    None => false,
                }
            }
            "stringout" | "lso" => {
                let val = string_val(record);
                match cip::get_string(&data.buf, element, string_size(record)) {
                    Some(cur) if cur == val => false,
                    Some(_) => cip::put_string(&mut data.buf, &val),
                    None => false,
                }
            }
            _ => {
                drop(data);
                self.write_alarm();
                return Ok(false);
            }
        };

        if changed {
            tag.request_write(&mut data, &self.name);
            if let (Some(done), Some(listener)) = (done, &self.listener) {
                *listener.write_done.lock() = Some((done.device_done(), done.priority()));
            }
        }
        drop(data);
        self.no_alarm();
        Ok(changed && done.is_some())
    }
}

/// What [`EtherIpDevice::follow_plc`] found.
#[derive(Debug, PartialEq, Eq)]
enum Follow {
    /// No tag, no value yet, or the element is past the tag's end.
    NoData,
    /// The tag holds data the record's type cannot be read from.
    GetFailed,
    Same,
    /// The PLC's value was copied into the record.
    Copied,
    /// The PLC differs and the record keeps its value: FORCE, or a DINT
    /// ao whose breakpoint table cannot back-convert it.
    Kept,
}

/// What an output's PLC callback does with its record.
#[derive(Debug, PartialEq, Eq)]
enum Callback {
    Leave,
    /// `scanOnce`: the record takes the PLC's value, or FORCE writes its own.
    ScanOnce,
    /// `rset->process`: no data, for the write's WRITE_ALARM.
    Process,
}

impl EtherIpDevice {
    /// An output's `check_*_callback` up to its `scanOnce`: follow the PLC,
    /// and say how the record processes.
    fn check_callback(&self, record: &mut dyn Record, common: &mut CommonFields) -> Callback {
        // C's `if (rec->pact)` pass: a write in flight ends on the driver's
        // transfer callback.
        if self
            .listener
            .as_ref()
            .is_some_and(|l| l.write_done.lock().is_some())
        {
            return Callback::Leave;
        }
        let refresh = common.udf != 0 || common.sevr == AlarmSeverity::Invalid;
        let once = if common.scan.to_u16() >= SCAN_1ST_PERIODIC {
            Callback::Leave
        } else {
            Callback::ScanOnce
        };
        match self.follow_plc(record, refresh, common.udf == 0) {
            // Whatever the SCAN.
            Ok(Follow::NoData) => Callback::Process,
            Ok(Follow::Copied) => {
                common.udf = 0;
                once
            }
            Ok(Follow::Kept) => once,
            Ok(Follow::GetFailed | Follow::Same) | Err(_) => Callback::Leave,
        }
    }

    /// Compare the PLC's value with the record's, `refresh` counting as a
    /// difference (C's `rec->udf || rec->sevr == INVALID_ALARM`). A
    /// difference is copied into the record unless the link's `FORCE`
    /// applies (`udf_clear`, C's `!rec->udf`), when the record's write pass
    /// puts its own value back.
    fn follow_plc(
        &self,
        record: &mut dyn Record,
        refresh: bool,
        udf_clear: bool,
    ) -> CaResult<Follow> {
        let Some((tag, link)) = self.bound() else {
            return Ok(Follow::NoData);
        };
        let (element, mask) = (link.element, link.mask);
        let force = link.forced() && udf_clear;
        let rtype = record.record_type();

        let data = tag.data.lock();
        let elements = *tag.elements.lock();
        if !data.has_value() || elements <= element {
            return Ok(Follow::NoData);
        }

        // Per type: the PLC's value, whether it differs, and how to copy it
        // in.
        macro_rules! follow {
            ($plc:expr, $differs:expr, $copy:expr) => {{
                let Some(plc) = $plc else {
                    return Ok(Follow::GetFailed);
                };
                if !(refresh || $differs(&plc)) {
                    return Ok(Follow::Same);
                }
                if force {
                    return Ok(Follow::Kept);
                }
                drop(data);
                return $copy(plc);
            }};
        }

        match rtype {
            "ao" => match cip::typecode(&data.buf) {
                Some(CipType::Real) | Some(CipType::Lreal) => {
                    let val = f64_field(record, "VAL");
                    follow!(
                        cip::get_double(&data.buf, element),
                        |plc: &f64| *plc != val,
                        |plc: f64| {
                            record.set_val(EpicsValue::Double(plc))?;
                            seed_pval(record, plc);
                            Ok(Follow::Copied)
                        }
                    )
                }
                _ => {
                    let rval = i32_field(record, "RVAL");
                    follow!(
                        cip::get_dint(&data.buf, element),
                        |plc: &i32| *plc != rval,
                        |plc: i32| back_convert_ao(record, plc)
                    )
                }
            },
            "int64out" => {
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::Int64(v)) => v,
                    _ => 0,
                };
                follow!(
                    cip::get_lint(&data.buf, element),
                    |plc: &i64| *plc != val,
                    |plc: i64| record
                        .set_val(EpicsValue::Int64(plc))
                        .map(|()| Follow::Copied)
                )
            }
            "bo" | "mbbo" | "mbboDirect" => {
                // C defect, not copied: an mbbo whose PLC value matches no
                // state gets VAL 65535 and keeps its stale RVAL
                // (devEtherIP.c:641), which mbbo_write (:2136) writes back to
                // the PLC against :316's "should NOT cause the record to
                // write". Here RVAL takes the PLC's value.
                // C defect, not copied: an mbboDirect's VAL is cut to 16 bits
                // (:710) and only B0..B15 follow (:720), so with NOBT > 16 its
                // process writes the cut value back. Here all NOBT bits follow.
                let bits = if rtype == "bo" { 1 } else { nobt(record) };
                let rval = i32_field(record, "RVAL") as u32;
                follow!(
                    get_bits(&data.buf, element, mask, bits),
                    |plc: &u32| *plc != rval,
                    |plc: u32| if record.apply_raw_readback(plc as i32) {
                        Ok(Follow::Copied)
                    } else {
                        record
                            .set_val(EpicsValue::Long(plc as i32))
                            .map(|()| Follow::Copied)
                    }
                )
            }
            "stringout" | "lso" => {
                let size = string_size(record);
                let val = string_val(record);
                // C `strcpy`s the whole string into VAL; lso's `set_val` is
                // the dbGetLinkLS read, which cuts a string to 39 bytes.
                follow!(
                    cip::get_string(&data.buf, element, size),
                    |plc: &String| *plc != val,
                    |plc: String| record
                        .put_field_internal("VAL", EpicsValue::String(plc.into()))
                        .map(|()| Follow::Copied)
                )
            }
            _ => Ok(Follow::GetFailed),
        }
    }
}

/// C `wf_read` (devEtherIP.c:1838-1925): elements `0..NELM`, whatever index
/// the link named, typed by the tag's CIP type against FTVL. `Ok(None)` is
/// C's silent pass when the tag holds fewer than NELM elements.
fn read_waveform(
    raw: &[u8],
    elements: usize,
    record: &dyn Record,
    name: &str,
) -> Result<Option<EpicsValue>, ()> {
    const CHAR: i16 = 1;
    const UCHAR: i16 = 2;
    const LONG: i16 = 5;
    const DOUBLE: i16 = 10;
    let nelm = count_field(record, "NELM");
    if elements < nelm {
        return Ok(None);
    }
    let ftvl = f64_field(record, "FTVL") as i16;
    let refuse = |message: &str| {
        rec_gbl_record_error("Illegal field value", name, message);
        Err(())
    };
    let value = match cip::typecode(raw) {
        Some(CipType::Real | CipType::Lreal) => {
            if ftvl != DOUBLE {
                return refuse(
                    "EtherIP: tag data type REAL or LREAL requires waveform FTVL==DOUBLE",
                );
            }
            let v: Option<Vec<f64>> = (0..nelm).map(|i| cip::get_double(raw, i)).collect();
            EpicsValue::DoubleArray(v.ok_or(())?.into())
        }
        Some(CipType::Sint) => {
            if ftvl != CHAR && ftvl != UCHAR {
                return refuse("EtherIP: tag data type requires waveform FTVL==CHAR/UCHAR");
            }
            let v: Option<Vec<u8>> = (0..nelm).map(|i| cip::get_usint(raw, i)).collect();
            EpicsValue::CharArray(v.ok_or(())?.into())
        }
        _ => {
            let v: Option<Vec<i32>> = (0..nelm).map(|i| cip::get_dint(raw, i)).collect();
            match ftvl {
                LONG => EpicsValue::LongArray(v.ok_or(())?.into()),
                // `*(s++) = dint_val`: each DINT truncated to its low byte.
                CHAR => EpicsValue::CharArray(
                    v.ok_or(())?
                        .into_iter()
                        .map(|d| d as u8)
                        .collect::<Vec<_>>()
                        .into(),
                ),
                _ => return refuse("EtherIP: tag type requires waveform FTVL==LONG or CHAR"),
            }
        }
    };
    Ok(Some(value))
}

/// check_ao_callback's DINT copy (devEtherIP.c:398-424), "copied from ao
/// init": the raw value through the record's init convert into VAL, RVAL
/// left as it was. A breakpoint table that cannot take it leaves VAL and
/// UDF alone and raises nothing, and the record still processes.
fn back_convert_ao(record: &mut dyn Record, raw: i32) -> CaResult<Follow> {
    let rval = record.get_field("RVAL");
    record.put_field("RVAL", EpicsValue::Long(raw))?;
    let converted = record.dset_init_convert().is_some();
    if let Some(rval) = rval {
        record.put_field("RVAL", rval)?;
    }
    if !converted {
        return Ok(Follow::Kept);
    }
    let val = f64_field(record, "VAL");
    seed_pval(record, val);
    Ok(Follow::Copied)
}

/// The `rec->pval` of check_ao_callback's `rec->val = rec->pval = dbl`: a
/// closed-loop ao's fetch_value starts from PVAL. PVAL is SPC_NOMOD, so no
/// put reaches it; it is stored through its slot, as C stores the member.
fn seed_pval(record: &mut dyn Record, value: f64) {
    if let Some(slot) = record.field_slot("PVAL") {
        record.put_slot_f64(slot, value);
    }
}

fn nobt(record: &dyn Record) -> usize {
    count_field(record, "NOBT")
}

// Numeric fields are read through `to_f64`, never by matching one
// `EpicsValue` variant: each record type picks its own (waveform NELM is
// ULong, mbbi NOBT UShort, mbbiDirect NOBT Short).
fn f64_field(record: &dyn Record, name: &str) -> f64 {
    record
        .get_field(name)
        .and_then(|v| v.to_f64())
        .unwrap_or(0.0)
}

/// An `epicsInt32`/`epicsUInt32` field as its 32 bits (RVAL is ULong on
/// bo/mbbo).
fn i32_field(record: &dyn Record, name: &str) -> i32 {
    f64_field(record, name) as i64 as i32
}

/// NELM / NOBT / SIZV, used as C uses them: unclamped.
fn count_field(record: &dyn Record, name: &str) -> usize {
    f64_field(record, name) as usize
}

/// The buffer, terminator included, C reads a PLC string into for VAL:
/// `MAX_STRING_SIZE`, or SIZV on the long-string records. The PLC's string
/// is compared cut to it, or a longer one never matches VAL.
fn string_size(record: &dyn Record) -> usize {
    match record.record_type() {
        "lsi" | "lso" => count_field(record, "SIZV"),
        _ => MAX_STRING_SIZE,
    }
}

/// VAL as the string C compares and writes: stringout serves a String, lso
/// its NUL-terminated char buffer.
fn string_val(record: &dyn Record) -> String {
    match record.get_field("VAL") {
        Some(EpicsValue::String(s)) => s.to_string(),
        Some(EpicsValue::CharArray(bytes)) => {
            let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
            String::from_utf8_lossy(&bytes[..end]).into_owned()
        }
        _ => String::new(),
    }
}

// ---------------------------------------------------------------------------
// The statistics-reset bo (DTYP "EtherIPReset")
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct EtherIpResetDevice {
    name: String,
}

impl DeviceSupport for EtherIpResetDevice {
    fn dtyp(&self) -> &str {
        DTYP_RESET
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_string();
    }

    fn write(&mut self, record: &mut dyn Record) -> CaResult<()> {
        if i32_field(record, "RVAL") != 0 {
            println!("'{}': resetting PLC statistics", self.name);
            driver::reset_statistics();
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/// The dynamic device-support factory: one DTYP for every record type, with the
/// link text taken verbatim from INP or OUT.
pub fn device_factory(ctx: &DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> {
    match (ctx.dtyp, ctx.record_type) {
        (DTYP_RESET, "bo") => Some(Box::new(EtherIpResetDevice::default())),
        (DTYP, rtype) if INPUT_TYPES.contains(&rtype) => {
            Some(Box::new(EtherIpDevice::new(ctx.inp, false)))
        }
        (DTYP, rtype) if OUTPUT_TYPES.contains(&rtype) => {
            Some(Box::new(EtherIpDevice::new(ctx.out, true)))
        }
        _ => None,
    }
}

/// The record types `ether_ip.dbd` declares an "EtherIP" `device()` for.
const INPUT_TYPES: &[&str] = &[
    "ai",
    "int64in",
    "bi",
    "lsi",
    "mbbi",
    "mbbiDirect",
    "stringin",
    "waveform",
];
const OUTPUT_TYPES: &[&str] = &[
    "ao",
    "int64out",
    "bo",
    "lso",
    "mbbo",
    "mbboDirect",
    "stringout",
];

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::base::server::database::{ProcStack, PvDatabase, RecordLoad};
    use epics_rs::base::server::records::bo::BoRecord;

    /// devEtherIP.c mbbo_write / check_mbbo_callback: a staged write leaves
    /// the record PACT, and the next transfer ends it without the readback
    /// that would put the PLC's value back into VAL.
    #[test]
    fn a_staged_write_waits_for_the_next_transfer_instead_of_reading_back() {
        use epics_rs::base::server::database::{DeviceDone, PvDatabase};
        use epics_rs::base::server::records::mbbo::MbboRecord;

        let rt = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        let db = PvDatabase::new();
        rt.block_on(db.add_record("STAGED", Box::new(MbboRecord::default())))
            .unwrap();
        let done = DeviceDone::new(&db, &db.get_record("STAGED").unwrap().read());
        let source = DeviceDoneSource::new(&done);

        define_plc("staged", "127.0.0.1", 0);
        let mut record = MbboRecord::default();
        record.put_field("NOBT", EpicsValue::Short(2)).unwrap();
        let mut dev = EtherIpDevice::new("@staged BOOLs[300]", true);
        dev.init(&mut record).unwrap();
        let mut rx = dev.io_intr_receiver().unwrap();
        let tag = dev.tag.clone().unwrap();
        {
            let mut d = tag.data.lock();
            d.buf = bits_buf(&[0; 10]);
            d.valid_data_size = d.buf.len();
        }

        record.put_field("RVAL", EpicsValue::Long(3)).unwrap();
        assert!(matches!(
            dev.write_begin(&mut record, &source).unwrap(),
            WriteStart::Pending
        ));
        assert!(tag.data.lock().do_write);
        tag.notify();
        assert!(
            rx.try_recv().is_err(),
            "the write's transfer is not a readback"
        );
        tag.notify();
        assert!(rx.try_recv().is_ok(), "later transfers read back again");

        // The 3 just written reads back as 3, so the next pass has nothing
        // to write.
        assert!(matches!(
            dev.write_begin(&mut record, &source).unwrap(),
            WriteStart::Completed
        ));
    }

    /// devEtherIP's dsxt: the new link's object binds its tag in
    /// `add_record` (`ai_add_record` → `init_record`), and the old one's
    /// `del_scan_callback_record` takes its callback off the old tag.
    #[test]
    fn add_record_binds_the_new_link_and_del_record_leaves_the_old_tag() {
        use epics_rs::base::server::records::ai::AiRecord;

        define_plc("dsxt", "127.0.0.1", 0);
        let mut record = AiRecord::default();
        let mut old = EtherIpDevice::new("@dsxt DSXT_A", false);
        old.init(&mut record).unwrap();
        let mut old_rx = old.io_intr_receiver().unwrap();
        let a = old.tag.clone().unwrap();

        let mut new = EtherIpDevice::new("@dsxt DSXT_B", false);
        new.add_record(&mut record).unwrap();
        let mut new_rx = new.io_intr_receiver().unwrap();
        let b = new.tag.clone().unwrap();
        assert_eq!(b.string_tag, "DSXT_B");

        old.del_record().unwrap();
        a.notify();
        b.notify();
        assert!(old_rx.try_recv().is_err(), "the old tag still calls back");
        assert!(new_rx.try_recv().is_ok());
    }

    /// A record bound while Passive still hears its tag, so a later switch
    /// to I/O Intr is served (C's `devEtherIP.c:1125-1133` gate is not
    /// copied).
    #[test]
    fn a_record_bound_while_passive_still_hears_its_tag() {
        use epics_rs::base::server::records::ai::AiRecord;

        define_plc("passive", "127.0.0.1", 0);
        let mut record = AiRecord::default();
        let mut dev = EtherIpDevice::new("@passive PASSIVE_A", false);
        dev.set_record_info("AI", ScanType::Passive);
        dev.init(&mut record).unwrap();
        let mut rx = dev.io_intr_receiver().unwrap();
        dev.tag.clone().unwrap().notify();
        assert!(rx.try_recv().is_ok());
    }

    /// A link `analyze_link` refuses refuses the put through `add_record`.
    #[test]
    fn add_record_refuses_a_link_with_no_plc() {
        use epics_rs::base::server::records::ai::AiRecord;

        define_plc("dsxt2", "127.0.0.1", 0);
        let mut record = AiRecord::default();
        let mut new = EtherIpDevice::new("@nosuchplc DSXT_C", false);
        assert!(new.add_record(&mut record).is_err());
        assert!(new.tag.is_none());
    }

    /// A bo on bit 12 of the tag's UDINT 9, bound to `plc`, with its common
    /// fields defined and passive.
    fn bo_on_bit_300(plc: &str) -> (EtherIpDevice, BoRecord, CommonFields, Arc<TagInfo>) {
        define_plc(plc, "127.0.0.1", 0);
        let mut record = BoRecord::default();
        let mut dev = EtherIpDevice::new(&format!("@{plc} BOOLs[300]"), true);
        dev.init(&mut record).unwrap();
        let tag = dev.tag.clone().unwrap();
        set_words(&tag, &[0; 10]);
        let mut common = CommonFields::default();
        common.udf = 0;
        (dev, record, common, tag)
    }

    fn set_words(tag: &TagInfo, words: &[u32]) {
        let mut d = tag.data.lock();
        d.buf = bits_buf(words);
        d.valid_data_size = d.buf.len();
    }

    fn rval(record: &BoRecord) -> i32 {
        i32_field(record, "RVAL")
    }

    /// devEtherIP.c:326-437: a pulse whose value the record already holds
    /// neither copies nor processes; a new one is copied and processed.
    #[test]
    fn an_output_processes_only_on_a_new_plc_value() {
        let (dev, mut record, mut common, tag) = bo_on_bit_300("cbsame");
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::Leave
        );
        let mut words = [0; 10];
        words[9] = 1 << 12;
        set_words(&tag, &words);
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::ScanOnce
        );
        assert_eq!(rval(&record), 1);
    }

    /// `rec->udf || rec->sevr == INVALID_ALARM` counts as a change.
    #[test]
    fn an_undefined_or_invalid_output_takes_an_unchanged_value() {
        let (dev, mut record, mut common, _tag) = bo_on_bit_300("cbudf");
        common.udf = 1;
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::ScanOnce
        );
        assert_eq!(common.udf, 0);
        common.sevr = AlarmSeverity::Invalid;
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::ScanOnce
        );
    }

    /// C copies the value but leaves a periodic record to its own scan.
    #[test]
    fn a_periodic_output_takes_the_value_without_processing() {
        let (dev, mut record, mut common, tag) = bo_on_bit_300("cbperiodic");
        common.scan = ScanType::from_u16(SCAN_1ST_PERIODIC);
        let mut words = [0; 10];
        words[9] = 1 << 12;
        set_words(&tag, &words);
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::Leave
        );
        assert_eq!(rval(&record), 1);
    }

    /// No data processes the record (to raise WRITE_ALARM), periodic or
    /// not; a failed get does nothing.
    #[test]
    fn no_data_processes_and_a_failed_get_does_not() {
        let (dev, mut record, mut common, tag) = bo_on_bit_300("cbnodata");
        common.scan = ScanType::from_u16(SCAN_1ST_PERIODIC);
        tag.data.lock().valid_data_size = 0;
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::Process
        );
        // The BOOL typecode with no word behind it.
        set_words(&tag, &[]);
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::Leave
        );
    }

    /// A database whose records bind EtherIP at creation, as iocInit binds
    /// them, with `plc` defined.
    fn eip_db(plc: &str) -> PvDatabase {
        use epics_rs::base::server::record::{DbLinkType, DevSup, Dset, register_device_menu};
        // ether-ip-ioc's `DEVICES` rows for the outputs these tests load.
        const fn row(name: &'static str, number: u8) -> DevSup {
            DevSup {
                name,
                choice: DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(number, &[2, 4], true),
            }
        }
        static AO: [DevSup; 1] = [row("devAoEtherIP", 6)];
        static BO: [DevSup; 1] = [row("devBoEtherIP", 6)];
        static MBBO: [DevSup; 1] = [row("devMbboEtherIP", 5)];
        register_device_menu("ao", &AO);
        register_device_menu("bo", &BO);
        register_device_menu("mbbo", &MBBO);
        define_plc(plc, "127.0.0.1", 0);
        let db = PvDatabase::new();
        db.install_device_support_resolver(Arc::new(device_factory));
        db
    }

    /// `record` loaded as `name` with `OUT` `@<plc> <tag> S 1`, and the tag
    /// its link registered.
    fn load_output(
        rt: &tokio::runtime::Runtime,
        db: &PvDatabase,
        name: &str,
        record: Box<dyn Record>,
        plc: &str,
        tag: &str,
    ) -> Arc<TagInfo> {
        let load = RecordLoad::from_common_fields(vec![
            ("DTYP".into(), EpicsValue::String(DTYP.into())),
            (
                "OUT".into(),
                EpicsValue::String(format!("@{plc} {tag} S 1").into()),
            ),
        ]);
        rt.block_on(db.add_loaded_record(name, record, load))
            .unwrap();
        let string_tag = tag.split('[').next().unwrap();
        driver::find_plc(plc)
            .unwrap()
            .add_tag(1.0, string_tag, 1)
            .unwrap()
    }

    /// One driver callback as the I/O Intr pump runs it, then whatever it
    /// queued on scanOnce, whose ring runs in order on one thread.
    fn pulse(rt: &tokio::runtime::Runtime, db: &PvDatabase, name: &str) {
        let rec = db.get_record(name).unwrap();
        let process = {
            let mut guard = rec.write();
            let inst = &mut *guard;
            inst.device
                .as_mut()
                .unwrap()
                .io_intr_callback(&mut *inst.record, &mut inst.common)
                == IoIntrAction::Process
        };
        if process {
            rt.block_on(db.process_record_readback(name, &mut ProcStack::new()))
                .unwrap();
        }
        let (tx, rx) = std::sync::mpsc::channel();
        epics_rs::base::runtime::task::background_scan_once(
            "flush",
            Box::new(move || {
                let _ = tx.send(());
            }),
        )
        .unwrap();
        rx.recv_timeout(std::time::Duration::from_secs(10)).unwrap();
    }

    fn field(db: &PvDatabase, name: &str, f: &str) -> f64 {
        f64_field(&*db.get_record(name).unwrap().read().record, f)
    }

    fn current_thread() -> tokio::runtime::Runtime {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
    }

    /// devEtherIP.c:326-437: check_ao_callback copies the PLC's value and
    /// scanOnces the record, whose own process converts it, so OVAL, PVAL,
    /// RVAL and ORAW follow, and ao_write finds nothing to write.
    #[test]
    fn an_ao_follow_runs_the_records_convert() {
        use epics_rs::base::server::records::ao::AoRecord;
        let rt = current_thread();
        let db = eip_db("aoconv");
        let real = load_output(
            &rt,
            &db,
            "AOR",
            Box::new(AoRecord::default()),
            "aoconv",
            "R",
        );
        set_tag(&real, CipType::Real, &3.25f32.to_le_bytes());
        let dint = load_output(
            &rt,
            &db,
            "AOD",
            Box::new(AoRecord::default()),
            "aoconv",
            "D",
        );
        set_tag(&dint, CipType::Dint, &42i32.to_le_bytes());
        pulse(&rt, &db, "AOR");
        pulse(&rt, &db, "AOD");
        for f in ["VAL", "OVAL", "PVAL"] {
            assert_eq!(field(&db, "AOR", f), 3.25, "AOR.{f}");
            assert_eq!(field(&db, "AOD", f), 42.0, "AOD.{f}");
        }
        for f in ["RVAL", "ORAW"] {
            assert_eq!(field(&db, "AOR", f), 3.0, "AOR.{f}");
            assert_eq!(field(&db, "AOD", f), 42.0, "AOD.{f}");
        }
        assert!(!real.data.lock().do_write);
        assert!(!dint.data.lock().do_write);
    }

    /// check_ao_callback's `rec->val = rec->pval = dbl` (devEtherIP.c:371,
    /// :404-413): PVAL follows the PLC before the record processes.
    #[test]
    fn an_ao_follow_seeds_pval() {
        use epics_rs::base::server::records::ao::AoRecord;
        define_plc("aopval", "127.0.0.1", 0);
        for (tag, ty, bytes) in [
            ("R", CipType::Real, 3.25f32.to_le_bytes()),
            ("D", CipType::Dint, 42i32.to_le_bytes()),
        ] {
            let mut record = AoRecord::default();
            let mut dev = EtherIpDevice::new(&format!("@aopval {tag} S 1"), true);
            dev.init(&mut record).unwrap();
            set_tag(dev.tag.as_ref().unwrap(), ty, &bytes);
            assert_eq!(
                dev.check_callback(&mut record, &mut CommonFields::default()),
                Callback::ScanOnce
            );
            let want = if tag == "R" { 3.25 } else { 42.0 };
            assert_eq!(f64_field(&record, "VAL"), want, "{tag}");
            assert_eq!(f64_field(&record, "PVAL"), want, "{tag}");
        }
    }

    /// check_ao_callback's DINT back-convert breaks on a breakpoint table
    /// that cannot take the PLC's value (devEtherIP.c:416-420): VAL, RVAL
    /// and UDF stay as they were, nothing is raised, and the record still
    /// processes.
    #[test]
    fn an_ao_whose_table_cannot_back_convert_keeps_its_value() {
        use epics_rs::base::server::records::ao::AoRecord;
        define_plc("aobpt", "127.0.0.1", 0);
        let mut record = AoRecord::default();
        record.put_field("LINR", EpicsValue::Short(3)).unwrap();
        let mut dev = EtherIpDevice::new("@aobpt D S 1", true);
        dev.init(&mut record).unwrap();
        let tag = dev.tag.clone().unwrap();
        set_tag(&tag, CipType::Dint, &7i32.to_le_bytes());

        let mut common = CommonFields::default();
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::ScanOnce
        );
        assert_eq!(common.udf, 1);
        assert_eq!(record.get_field("RVAL"), Some(EpicsValue::Long(0)));
        assert_eq!(record.get_field("VAL"), Some(EpicsValue::Double(0.0)));
        assert_eq!(common.sevr, AlarmSeverity::NoAlarm);
    }

    /// check_bo_callback: the record's process forms RVAL from VAL and
    /// MASK (boRecord.c), after the callback's copy.
    #[test]
    fn a_bo_follow_runs_the_records_mask() {
        let rt = current_thread();
        let db = eip_db("bomask");
        let mut record = BoRecord::default();
        record.put_field("MASK", EpicsValue::ULong(0x10)).unwrap();
        let tag = load_output(&rt, &db, "BOM", Box::new(record), "bomask", "BOOLs[300]");
        let mut words = [0; 10];
        words[9] = 1 << 12;
        set_words(&tag, &words);
        pulse(&rt, &db, "BOM");
        assert_eq!(field(&db, "BOM", "VAL"), 1.0);
        assert_eq!(field(&db, "BOM", "RVAL"), 16.0);
    }

    /// check_mbbo_callback: a PLC value no state matches is VAL 65535, and
    /// the record's convert raises SOFT_ALARM/INVALID for it.
    #[test]
    fn an_mbbo_follow_runs_the_records_convert() {
        use epics_rs::base::server::records::mbbo::MbboRecord;
        let rt = current_thread();
        let db = eip_db("mbbostate");
        let mut record = MbboRecord::default();
        record.put_field("NOBT", EpicsValue::Short(2)).unwrap();
        record.put_field("ONVL", EpicsValue::ULong(1)).unwrap();
        record.put_field("TWVL", EpicsValue::ULong(2)).unwrap();
        let tag = load_output(
            &rt,
            &db,
            "MBBO",
            Box::new(record),
            "mbbostate",
            "BOOLs[300]",
        );
        let mut words = [0; 10];
        words[9] = 3 << 12;
        set_words(&tag, &words);
        pulse(&rt, &db, "MBBO");
        assert_eq!(field(&db, "MBBO", "VAL"), 65535.0);
        let rec = db.get_record("MBBO").unwrap();
        let common = &rec.read().common;
        assert_eq!(
            (common.stat, common.sevr),
            (alarm_status::SOFT_ALARM, AlarmSeverity::Invalid)
        );
    }

    /// A tag holding one Logix STRING, `text`.
    fn set_string_tag(tag: &TagInfo, text: &str) {
        let mut chars = [0u8; cip::STRUCT_STRING_BUF];
        chars[..text.len()].copy_from_slice(text.as_bytes());
        let mut bytes = cip::STRUCT_STRING_HANDLE.to_le_bytes().to_vec();
        bytes.extend_from_slice(&(text.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&chars);
        set_tag(tag, CipType::Struct, &bytes);
    }

    /// check_so_callback and so_write compare the PLC's string cut to
    /// MAX_STRING_SIZE, so a longer one is taken once, cut to fit, and not
    /// written back.
    #[test]
    fn a_long_plc_string_is_taken_cut_and_not_written_back() {
        use epics_rs::base::server::records::stringout::StringoutRecord;
        define_plc("strlong", "127.0.0.1", 0);
        let mut record = StringoutRecord::default();
        let mut dev = EtherIpDevice::new("@strlong S S 1", true);
        dev.init(&mut record).unwrap();
        let tag = dev.tag.clone().unwrap();
        let long = "x".repeat(60);
        set_string_tag(&tag, &long);

        let mut common = CommonFields::default();
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::ScanOnce
        );
        assert_eq!(
            record.get_field("VAL"),
            Some(EpicsValue::String(long[..MAX_STRING_SIZE - 1].into()))
        );
        dev.write(&mut record).unwrap();
        assert!(!tag.data.lock().do_write);
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::Leave
        );
    }

    /// check_lso_callback copies the PLC's string whole up to SIZV, and
    /// lso_write compares VAL's chars against it, so a string longer than a
    /// DBR_STRING is taken once and not written back.
    #[test]
    fn an_lso_follows_and_compares_its_char_val() {
        use epics_rs::base::server::records::lso::LsoRecord;
        define_plc("lsolong", "127.0.0.1", 0);
        let mut record = LsoRecord::default();
        let mut dev = EtherIpDevice::new("@lsolong S S 1", true);
        dev.init(&mut record).unwrap();
        let tag = dev.tag.clone().unwrap();
        let long = "y".repeat(60);
        set_string_tag(&tag, &long);

        let mut common = CommonFields::default();
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::ScanOnce
        );
        assert_eq!(string_val(&record), long);
        dev.write(&mut record).unwrap();
        assert!(!tag.data.lock().do_write, "lso wrote back");
        assert_eq!(
            dev.check_callback(&mut record, &mut common),
            Callback::Leave
        );
    }

    /// A tag holding one `ty` value of `bytes`.
    fn set_tag(tag: &TagInfo, ty: CipType, bytes: &[u8]) {
        let mut d = tag.data.lock();
        d.buf = ty.code().to_le_bytes().to_vec();
        d.buf.extend_from_slice(bytes);
        d.valid_data_size = d.buf.len();
    }

    /// `drvEtherIP_init` then `drvEtherIP_define_PLC`, as a st.cmd does.
    fn define_plc(name: &str, ip_addr: &str, slot: u8) {
        if !driver::initialized() {
            driver::init();
        }
        driver::define_plc(name, ip_addr, slot);
    }

    /// A raw BOOL-array (CIP BITS) buffer holding `words` UDINTs.
    fn bits_buf(words: &[u32]) -> Vec<u8> {
        let mut v = CipType::Bits.code().to_le_bytes().to_vec();
        for w in words {
            v.extend_from_slice(&w.to_le_bytes());
        }
        v
    }

    #[test]
    fn multi_bit_records_zero_shft_at_init() {
        use epics_rs::base::server::records::{
            mbbi::MbbiRecord, mbbi_direct::MbbiDirectRecord, mbbo::MbboRecord,
            mbbo_direct::MbboDirectRecord,
        };
        let records: [Box<dyn Record>; 4] = [
            Box::new(MbbiRecord::default()),
            Box::new(MbbiDirectRecord::default()),
            Box::new(MbboRecord::default()),
            Box::new(MbboDirectRecord::default()),
        ];
        for mut record in records {
            record.put_field("SHFT", EpicsValue::UShort(4)).unwrap();
            // The link names no PLC, so init refuses after the SHFT reset.
            let mut dev =
                EtherIpDevice::new("@nosuchplc tag", record.record_type().starts_with("mbbo"));
            let _ = dev.init(&mut *record);
            assert_eq!(
                record.get_field("SHFT"),
                Some(EpicsValue::UShort(0)),
                "{}",
                record.record_type()
            );
        }
    }

    #[test]
    fn record_shape_reads_each_record_types_own_field_variant() {
        use epics_rs::base::server::records::{
            mbbi::MbbiRecord, mbbi_direct::MbbiDirectRecord, mbbo::MbboRecord,
            waveform::WaveformRecord,
        };
        let mut wf = WaveformRecord::default();
        wf.put_field("NELM", EpicsValue::ULong(40)).unwrap();
        assert_eq!(record_shape(&wf), (0, 40));
        let multi: [Box<dyn Record>; 3] = [
            Box::new(MbbiRecord::default()),
            Box::new(MbboRecord::default()),
            Box::new(MbbiDirectRecord::default()),
        ];
        for mut record in multi {
            record.put_field("NOBT", EpicsValue::Short(2)).unwrap();
            assert_eq!(record_shape(&*record), (2, 1), "{}", record.record_type());
        }
    }

    #[test]
    fn get_period_reads_the_scan_menu_label() {
        assert_eq!(get_period("r", ScanType::SEC1), Some(1.0));
        assert_eq!(get_period("r", ScanType::Menu(9)), Some(0.1));
        assert_eq!(get_period("r", ScanType::SEC10), Some(10.0));
        assert_eq!(get_period("r", ScanType::Passive), None);
        assert_eq!(get_period("r", ScanType::IoIntr), None);
    }

    /// devEtherIP.c:361-375: FORCE re-writes the record's value only once
    /// the record is defined; an undefined record takes the PLC's.
    #[test]
    fn force_yields_to_the_plc_while_the_record_is_undefined() {
        use epics_rs::base::server::record::CommonFields;
        use epics_rs::base::server::records::ao::AoRecord;

        define_plc("forceudf", "127.0.0.1", 0);
        for (udf, want_val, want_write) in [(1u8, 5.0, false), (0, 0.0, true)] {
            let mut record = AoRecord::default();
            let mut dev = EtherIpDevice::new("@forceudf REAL S 1 FORCE", true);
            dev.init(&mut record).unwrap();
            let tag = dev.tag.clone().unwrap();
            {
                let mut d = tag.data.lock();
                d.buf = CipType::Real.code().to_le_bytes().to_vec();
                d.buf.extend_from_slice(&5.0f32.to_le_bytes());
                d.valid_data_size = d.buf.len();
                d.do_write = false;
            }
            let mut common = CommonFields::default();
            common.udf = udf;
            assert_eq!(
                dev.check_callback(&mut record, &mut common),
                Callback::ScanOnce,
                "udf={udf}"
            );
            assert_eq!(f64_field(&record, "VAL"), want_val, "udf={udf}");
            // ao_write, in the process that follows.
            dev.write(&mut record).unwrap();
            assert_eq!(tag.data.lock().do_write, want_write, "udf={udf}");
        }
    }

    /// devEtherIP.c:1853-1915: elements 0..NELM, nothing below NELM
    /// registered elements, and FTVL checked against the CIP type.
    #[test]
    fn waveform_reads_from_element_zero_as_c_wf_read_does() {
        use epics_rs::base::server::records::waveform::WaveformRecord;
        let mut raw = CipType::Real.code().to_le_bytes().to_vec();
        for v in [1.5f32, 2.5, 3.5, 4.5] {
            raw.extend_from_slice(&v.to_le_bytes());
        }
        let mut wf = WaveformRecord::default();
        wf.put_field("NELM", EpicsValue::ULong(3)).unwrap();
        wf.put_field("FTVL", EpicsValue::Short(10)).unwrap();
        assert_eq!(
            read_waveform(&raw, 4, &wf, "wf"),
            Ok(Some(EpicsValue::DoubleArray(vec![1.5, 2.5, 3.5].into())))
        );
        assert_eq!(read_waveform(&raw, 2, &wf, "wf"), Ok(None));
        wf.put_field("FTVL", EpicsValue::Short(5)).unwrap();
        assert_eq!(read_waveform(&raw, 4, &wf, "wf"), Err(()));
    }

    /// ether_ip.dbd declares no longin/longout dset and EtherIPReset only on bo.
    #[test]
    fn factory_serves_only_the_dbd_declared_record_types() {
        let ctx = |record_type, dtyp| DeviceSupportContext {
            record_type,
            dtyp,
            inp: "@plc1 DINT",
            out: "@plc1 DINT",
        };
        assert!(device_factory(&ctx("ai", DTYP)).is_some());
        assert!(device_factory(&ctx("stringout", DTYP)).is_some());
        assert!(device_factory(&ctx("bo", DTYP_RESET)).is_some());
        assert!(device_factory(&ctx("longin", DTYP)).is_none());
        assert!(device_factory(&ctx("longout", DTYP)).is_none());
        assert!(device_factory(&ctx("ao", DTYP_RESET)).is_none());
    }

    /// check_ao_callback's no-data branch calls `rset->process` itself
    /// (devEtherIP.c:347-354): a TPRO record prints no dbProcess line for
    /// it, and still ends in WRITE_ALARM. Run in a child, whose stdout the
    /// trace goes to.
    #[test]
    fn a_no_data_pulse_processes_past_dbprocess() {
        use epics_rs::base::server::records::ao::AoRecord;
        use std::time::{Duration, Instant};
        const CHILD: &str = "EIP_NO_DATA_CHILD";
        if std::env::var_os(CHILD).is_none() {
            let out = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "device::tests::a_no_data_pulse_processes_past_dbprocess",
                    "--exact",
                    "--nocapture",
                ])
                .env(CHILD, "1")
                .output()
                .unwrap();
            let stdout = String::from_utf8_lossy(&out.stdout);
            let stderr = String::from_utf8_lossy(&out.stderr);
            assert!(out.status.success(), "{stdout}{stderr}");
            assert!(stdout.contains("1 passed"), "{stdout}");
            assert!(!stdout.contains("dbProcess of 'AONODATA'"), "{stdout}");
            return;
        }
        epics_rs::base::runtime::task::background_arm();
        let rt = current_thread();
        let db = eip_db("nodatapulse");
        load_output(
            &rt,
            &db,
            "AONODATA",
            Box::new(AoRecord::default()),
            "nodatapulse",
            "R",
        );
        rt.block_on(db.put_pv_no_process("AONODATA.TPRO", EpicsValue::Char(1)))
            .unwrap();
        let stat = || db.get_record("AONODATA").unwrap().read().common.stat;
        assert_ne!(stat(), alarm_status::WRITE_ALARM);
        pulse(&rt, &db, "AONODATA");
        let deadline = Instant::now() + Duration::from_secs(10);
        while stat() != alarm_status::WRITE_ALARM {
            assert!(Instant::now() < deadline, "no WRITE_ALARM");
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// A disconnected tag leaves an output in WRITE_ALARM, as C's ao_write.
    #[test]
    fn an_output_without_data_raises_write_alarm() {
        use epics_rs::base::server::records::ao::AoRecord;
        define_plc("nodata", "127.0.0.1", 0);
        let mut record = AoRecord::default();
        let mut dev = EtherIpDevice::new("@nodata REAL S 1", true);
        dev.init(&mut record).unwrap();
        assert_eq!(
            dev.check_callback(&mut record, &mut CommonFields::default()),
            Callback::Process
        );
        dev.write(&mut record).unwrap();
        let alarm = dev.last_alarm().unwrap();
        assert_eq!(alarm.stat, alarm_status::WRITE_ALARM);
    }

    /// A get that fails on data the tag does hold: C's check_ao_callback
    /// does not process the record.
    #[test]
    fn an_output_whose_get_fails_does_not_process() {
        use epics_rs::base::server::records::ao::AoRecord;
        define_plc("shortdata", "127.0.0.1", 0);
        let mut record = AoRecord::default();
        let mut dev = EtherIpDevice::new("@shortdata DINT S 1", true);
        dev.init(&mut record).unwrap();
        let tag = dev.tag.clone().unwrap();
        {
            let mut d = tag.data.lock();
            // The DINT typecode and no value behind it.
            d.buf = CipType::Dint.code().to_le_bytes().to_vec();
            d.valid_data_size = d.buf.len();
        }
        assert_eq!(
            dev.check_callback(&mut record, &mut CommonFields::default()),
            Callback::Leave
        );
    }

    /// devEtherIP.c:1642: statistics are read under lock_data, so a tag
    /// without data fails them.
    #[test]
    fn a_statistic_without_tag_data_raises_read_alarm() {
        use epics_rs::base::server::records::ai::AiRecord;
        define_plc("statnodata", "127.0.0.1", 0);
        let mut record = AiRecord::default();
        let mut dev = EtherIpDevice::new("@statnodata REAL PLC_ERRORS", false);
        dev.init(&mut record).unwrap();
        dev.read(&mut record).unwrap();
        assert_eq!(dev.last_alarm().unwrap().stat, alarm_status::READ_ALARM);
    }

    // -- link parsing --------------------------------------------------------

    /// `analyze_link` with every PLC known and no SCAN period.
    fn parse_link(text: &str, count: usize, bits: usize) -> Result<Link, String> {
        analyze_link(text, count, bits, || 0.0, |_| Some(())).map(|(l, ())| l)
    }

    /// devEtherIP.c:884-1007: C's texts, and the steps that ran before a
    /// late failure have printed.
    #[test]
    fn analyze_link_fails_in_cs_order_with_cs_texts() {
        let err = |text, count, bits| {
            let mut asked = false;
            let e = analyze_link(
                text,
                count,
                bits,
                || {
                    asked = true;
                    0.0
                },
                |_| None::<()>,
            )
            .unwrap_err();
            (e, asked)
        };
        assert_eq!(err("", 1, 0).0, "Missing PLC in link ''");
        assert_eq!(err("@plc1", 1, 0).0, "Missing tag in link 'plc1'");
        assert_eq!(
            err("@plc1 Cnt NOPE x", 1, 0),
            (
                "Invalid flag 'NOPE x' in link 'plc1 Cnt NOPE x'".into(),
                false
            )
        );
        assert_eq!(
            err("@plc1 A[1] E", 2, 0).0,
            "Array record cannot use 'E' flag ('plc1 A[1] E')"
        );
        assert_eq!(
            err("@plc1 Cnt S 1e999", 1, 0).0,
            "Error in scan flag in link 'plc1 Cnt S 1e999'"
        );
        // The period was resolved before the tag failed.
        assert_eq!(
            err("@plc1 A[x]", 1, 0),
            ("malformed array tag in 'plc1 A[x]'".into(), true)
        );
        assert_eq!(err("@plc1 A", 1, 0).0, "unknown PLC 'plc1'");
        // The PLC is looked up before the bits check.
        assert_eq!(err("@plc1 A", 10, 1).0, "unknown PLC 'plc1'");
        assert_eq!(
            parse_link("@plc1 A", 10, 1).unwrap_err(),
            "cannot access bits for array records"
        );
    }

    /// strtod/strtol/prefix matching as C reads them.
    #[test]
    fn analyze_link_reads_flags_and_index_as_c_does() {
        // A bad period leaves 0 and its text becomes the next token.
        assert_eq!(
            parse_link("@plc1 Cnt S x", 1, 0).unwrap_err(),
            "Invalid flag 'x' in link 'plc1 Cnt S x'"
        );
        // A bad bit never fails: bit 0, then the text is the next token.
        assert_eq!(parse_link("@plc1 Cnt B FORCE", 1, 0).unwrap().mask, 1);
        // Flags match as prefixes; 'S' without its space does not.
        assert!(parse_link("@plc1 Cnt FORCED", 1, 0).unwrap().forced());
        assert!(parse_link("@plc1 Cnt S", 1, 0).is_err());
        // strtol base 0.
        assert_eq!(parse_link("@plc1 A[0x10]", 1, 0).unwrap().element, 16);
        assert_eq!(parse_link("@plc1 A[010]", 1, 0).unwrap().element, 8);
        assert_eq!(parse_link("@plc1 A[3x]", 1, 0).unwrap().element, 3);
        assert_eq!(parse_link("@plc1 Cnt S 0.5", 1, 0).unwrap().period, 0.5);
    }

    #[test]
    fn plain_scalar_link() {
        let l = parse_link("@plc1 REALS", 1, 0).unwrap();
        assert_eq!(l.plc_name, "plc1");
        assert_eq!(l.string_tag, "REALS");
        assert_eq!(l.element, 0);
        assert_eq!(l.elements, 1);
        assert_eq!(l.period, 0.0);
        assert_eq!(l.special, 0);
    }

    #[test]
    fn indexed_scalar_link_splits_the_element_off() {
        let l = parse_link("@plc1 REALS[7]", 1, 0).unwrap();
        assert_eq!(l.string_tag, "REALS");
        assert_eq!(l.element, 7);
        // The driver must fetch REALS[0..8] so element 7 is in the buffer.
        assert_eq!(l.elements, 8);
        assert!(l.special & spco::INDEX_INCLUDED != 0);
    }

    #[test]
    fn e_flag_keeps_the_index_in_the_tag_path() {
        // With 'E' the PLC does the indexing, so the tag keeps its "[7]" and a
        // single element comes back.
        let l = parse_link("@plc1 REALS[7] E", 1, 0).unwrap();
        assert_eq!(l.string_tag, "REALS[7]");
        assert_eq!(l.element, 0);
        assert_eq!(l.elements, 1);
    }

    #[test]
    fn e_flag_is_rejected_on_an_array_record() {
        assert!(parse_link("@plc1 REALS[7] E", 10, 0).is_err());
    }

    #[test]
    fn a_negative_bit_selects_bit_31_as_cs_size_t_does() {
        let l = parse_link("@plc1 Cnt B -1", 1, 0).unwrap();
        assert_eq!(l.mask, 1 << 31);
        let l = parse_link("@plc1 Cnt B -30", 1, 0).unwrap();
        assert_eq!(l.mask, 1 << 2);
    }

    #[test]
    fn scan_and_bit_flags_take_a_value() {
        let l = parse_link("@plc1 Cnt S 0.5 B 3", 1, 0).unwrap();
        assert_eq!(l.period, 0.5);
        assert_eq!(l.mask, 1 << 3);
        assert!(l.special & spco::BIT != 0);
        assert!(l.special & spco::SCAN_PERIOD != 0);
    }

    #[test]
    fn force_and_statistics_flags() {
        assert!(parse_link("@plc1 Cnt FORCE", 1, 0).unwrap().forced());
        let l = parse_link("@plc1 Cnt LIST_SCAN_TIME", 1, 0).unwrap();
        assert!(l.is_statistic());
        assert!(!parse_link("@plc1 Cnt", 1, 0).unwrap().is_statistic());
    }

    #[test]
    fn unknown_flag_is_an_error() {
        assert!(parse_link("@plc1 Cnt NOPE", 1, 0).is_err());
        assert!(parse_link("@plc1", 1, 0).is_err());
        assert!(parse_link("", 1, 0).is_err());
    }

    #[test]
    fn bool_array_index_folds_into_a_udint_index_and_mask() {
        // bi on BOOLS[37]: bit 37 lives in UDINT 1, bit 5.
        let l = parse_link("@plc1 BOOLS[37]", 1, 1).unwrap();
        assert_eq!(l.string_tag, "BOOLS");
        assert_eq!(l.element, 1);
        assert_eq!(l.mask, 1 << 5);
        assert_eq!(l.elements, 2); // UDINTs 0 and 1
    }

    #[test]
    fn multi_bit_record_spanning_a_udint_boundary() {
        // mbbi NOBT=4 on BOOLS[30]: bits 30..33 straddle UDINT 0 and 1.
        let l = parse_link("@plc1 BOOLS[30]", 1, 4).unwrap();
        assert_eq!(l.element, 0);
        assert_eq!(l.mask, 1 << 30);
        assert_eq!(l.elements, 2);
    }

    /// An unindexed binary link reads from bit 0, as test.db's biDINT says;
    /// C's mask of 255 made a bi on a DINT of 2 read 1.
    #[test]
    fn unindexed_binary_link_reads_from_bit_zero() {
        let bi = parse_link("@plc1 aBool", 1, 1).unwrap();
        let mbbi = parse_link("@plc1 aWord", 1, 4).unwrap();
        let raw = bits_buf(&[0b0110]);
        assert_eq!(get_bits(&raw, bi.element, bi.mask, 1), Some(0));
        assert_eq!(get_bits(&raw, mbbi.element, mbbi.mask, 4), Some(0b0110));
    }

    #[test]
    fn bits_are_rejected_on_an_array_record() {
        assert!(parse_link("@plc1 arr", 10, 1).is_err());
        assert!(parse_link("@plc1 arr B 2", 10, 0).is_err());
    }

    // -- bit packing ---------------------------------------------------------

    #[test]
    fn get_one_bit() {
        let raw = bits_buf(&[0b1010_0000]);
        assert_eq!(get_bits(&raw, 0, 1 << 5, 1), Some(1));
        assert_eq!(get_bits(&raw, 0, 1 << 6, 1), Some(0));
        assert_eq!(get_bits(&raw, 0, 1 << 7, 1), Some(1));
    }

    #[test]
    fn get_four_bits() {
        // bits 4..7 of 0xA5 = 0b1010_0101 -> 0b1010 = 10
        let raw = bits_buf(&[0xA5]);
        assert_eq!(get_bits(&raw, 0, 1 << 4, 4), Some(0b1010));
    }

    #[test]
    fn get_bits_across_a_udint_boundary() {
        // Bits 30,31 of word 0 are 1,0; bits 0,1 of word 1 are 1,1.
        let raw = bits_buf(&[1 << 30, 0b11]);
        assert_eq!(get_bits(&raw, 0, 1 << 30, 4), Some(0b1101));
    }

    /// mbbi NOBT=2 on BOOLs[300] with bits 300 and 301 set reads 3; C reads
    /// 2, taking the first bit from bit 0.
    #[test]
    fn multi_bit_read_takes_its_first_bit_from_the_mask() {
        let raw = bits_buf(&[0x3000]);
        assert_eq!(get_bits(&raw, 0, 1 << 12, 2), Some(3));
        assert_eq!(get_bits(&bits_buf(&[0x1]), 0, 1 << 12, 2), Some(0));
        assert_eq!(get_bits(&raw, 0, 1 << 12, 1), Some(1));
        assert_eq!(get_bits(&bits_buf(&[0b11]), 0, 1, 2), Some(3));
    }

    #[test]
    fn get_bits_past_the_end_fails() {
        let raw = bits_buf(&[0]);
        assert_eq!(get_bits(&raw, 0, 1 << 31, 2), None);
        assert_eq!(get_bits(&raw, 1, 1, 1), None);
    }

    #[test]
    fn put_bits_round_trips() {
        let mut raw = bits_buf(&[0xFFFF_FFFF]);
        assert!(put_bits(&mut raw, 0, 1 << 4, 4, 0b0101));
        assert_eq!(get_bits(&raw, 0, 1 << 4, 4), Some(0b0101));
        // Neighbouring bits are untouched.
        assert_eq!(cip::get_udint(&raw, 0), Some(0xFFFF_FF5F));
    }

    #[test]
    fn put_bits_across_a_udint_boundary() {
        let mut raw = bits_buf(&[0, 0]);
        assert!(put_bits(&mut raw, 0, 1 << 30, 4, 0b1101));
        assert_eq!(cip::get_udint(&raw, 0), Some(1 << 30));
        assert_eq!(cip::get_udint(&raw, 1), Some(0b11));
        assert_eq!(get_bits(&raw, 0, 1 << 30, 4), Some(0b1101));
    }

    #[test]
    fn put_bits_past_the_end_fails() {
        let mut raw = bits_buf(&[0]);
        assert!(!put_bits(&mut raw, 0, 1 << 31, 2, 3));
        assert!(!put_bits(&mut raw, 1, 1, 1, 1));
    }
}
