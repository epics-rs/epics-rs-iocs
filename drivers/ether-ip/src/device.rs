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

use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::base::server::device_support::{
    DeviceAlarm, DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf,
};
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::{AlarmSeverity, ProcessContext, Record, ScanType, menu_scan};
use epics_rs::base::types::EpicsValue;

use crate::cip::{self, CipType};
use crate::driver::{self, Plc, TagInfo};

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
    /// Explicit `S <period>`; None means "take the record's SCAN".
    pub period: Option<f64>,
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

/// Parse `@PLC tag[element] flags...`.
///
/// `count` is the number of array elements the record wants (1, or NELM for a
/// waveform); `bits` is >0 for the (multi-)bit records and is then the number
/// of bits (NOBT, or 1 for bi/bo).
pub fn parse_link(text: &str, count: usize, bits: usize) -> Result<Link, String> {
    let text = text.strip_prefix('@').unwrap_or(text);
    let tokens: Vec<&str> = text.split_whitespace().collect();

    let plc_name = (*tokens.first().ok_or("missing PLC in link")?).to_string();
    let tag_text = *tokens.get(1).ok_or("missing tag in link")?;

    let mut special = 0u32;
    let mut period = None;
    let mut bit = 0u32;

    let mut i = 2;
    while i < tokens.len() {
        let t = tokens[i];
        match t {
            "E" => {
                if count != 1 {
                    return Err(format!("array record cannot use the 'E' flag ('{text}')"));
                }
                special |= spco::READ_SINGLE_ELEMENT;
            }
            "S" => {
                let v = tokens.get(i + 1).ok_or("'S' flag needs a scan period")?;
                period = Some(
                    v.parse::<f64>()
                        .map_err(|_| format!("bad scan period '{v}' in link '{text}'"))?,
                );
                special |= spco::SCAN_PERIOD;
                i += 1;
            }
            "B" => {
                let v = tokens.get(i + 1).ok_or("'B' flag needs a bit number")?;
                bit = v
                    .parse::<u32>()
                    .map_err(|_| format!("bad bit number '{v}' in link '{text}'"))?;
                special |= spco::BIT;
                i += 1;
            }
            "FORCE" => special |= spco::FORCE,
            "PLC_ERRORS" => special |= spco::PLC_ERRORS,
            "PLC_TASK_SLOW" => special |= spco::PLC_TASK_SLOW,
            "LIST_ERRORS" => special |= spco::LIST_ERRORS,
            "LIST_TICKS" => special |= spco::LIST_TICKS,
            "LIST_SCAN_TIME" => special |= spco::LIST_SCAN_TIME,
            "LIST_MIN_SCAN_TIME" => special |= spco::LIST_MIN_SCAN_TIME,
            "LIST_MAX_SCAN_TIME" => special |= spco::LIST_MAX_SCAN_TIME,
            "TAG_TRANSFER_TIME" => special |= spco::TAG_TRANSFER_TIME,
            "LIST_TIME" => special |= spco::LIST_TIME,
            other => return Err(format!("invalid flag '{other}' in link '{text}'")),
        }
        i += 1;
    }

    // Split "array_tag[el]" into "array_tag" + el, unless 'E' asked for the
    // element to stay in the tag path (so the PLC does the indexing).
    let single_element = special & spco::READ_SINGLE_ELEMENT != 0;
    let mut string_tag = tag_text.to_string();
    let mut element = 0usize;

    if !single_element && string_tag.ends_with(']') {
        let open = string_tag
            .rfind('[')
            .ok_or_else(|| format!("malformed array tag in '{text}'"))?;
        if open == 0 {
            return Err(format!("malformed array tag in '{text}'"));
        }
        let idx = &string_tag[open + 1..string_tag.len() - 1];
        element = idx
            .parse::<usize>()
            .map_err(|_| format!("malformed array tag in '{text}'"))?;
        special |= spco::INDEX_INCLUDED;
        string_tag.truncate(open);
    }

    if count > 1 && (bits > 0 || special & spco::BIT != 0) {
        return Err(format!("cannot access bits for array records ('{text}')"));
    }

    let mask;
    let last_element;
    if bits > 0 && special & spco::BIT == 0 {
        // A binary record with no explicit bit number: an indexed tag is a
        // BOOL array, which Logix packs into UDINTs, so the bit index folds
        // into a UDINT index plus a mask; an unindexed one is a plain BOOL,
        // masked 255 whatever NOBT is (devEtherIP.c:1086-1100).
        mask = if special & spco::INDEX_INCLUDED != 0 {
            1u32 << (element & 0x1F)
        } else {
            255
        };
        last_element = (element + bits - 1) >> 5;
        element >>= 5;
    } else {
        // No binary record, or an explicit bit number: the element is a real
        // array index and the bit is selected by `B <bit>`.
        last_element = element;
        mask = 1u32 << bit;
    }

    Ok(Link {
        plc_name,
        string_tag,
        element,
        mask,
        special,
        period,
        elements: last_element + count,
    })
}

// ---------------------------------------------------------------------------
// Bit packing (C `get_bits` / `put_bits`)
// ---------------------------------------------------------------------------

/// Gather `bits` consecutive bits out of the BOOL array starting at
/// `element`/`mask`, packing them into an RVAL.
///
/// A multi-bit read tests its first bit as `(value & 0x1) & mask`
/// (devEtherIP.c:209-216), so RVAL bit 0 is 0 unless `mask` covers bit 0.
fn get_bits(raw: &[u8], mut element: usize, mut mask: u32, bits: usize) -> Option<u32> {
    let mut value = cip::get_udint(raw, element)?;
    let mut rval = 0u32;
    let first = if bits == 1 { value } else { value & 0x1 };
    if first & mask != 0 {
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
    /// `rec->udf` as the cycle began: FORCE yields to the PLC while it is set.
    udf: u8,
    link_text: String,
    is_output: bool,
    link: Option<Link>,
    plc: Option<Arc<Plc>>,
    tag: Option<Arc<TagInfo>>,
    io_intr_rx: Option<mpsc::Receiver<()>>,
    alarm: Option<(u16, u16)>,
}

impl EtherIpDevice {
    fn new(link_text: &str, is_output: bool) -> EtherIpDevice {
        EtherIpDevice {
            name: String::new(),
            scan: ScanType::Passive,
            udf: 1,
            link_text: link_text.to_string(),
            is_output,
            link: None,
            plc: None,
            tag: None,
            io_intr_rx: None,
            alarm: None,
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
        let stats = tag.list_stats.lock();
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
        .then(|| strtod_prefix(label))
        .flatten()
        .filter(|p| p.is_finite());
    crate::eip_printf!(
        8,
        "EIP record '{name}' scans at {:.1} secs\n",
        period.unwrap_or(-1.0)
    );
    period
}

/// The number `strtod` takes off the front of `text`, or None when it takes
/// nothing.
fn strtod_prefix(text: &str) -> Option<f64> {
    let t = text.trim_start();
    (1..=t.len())
        .rev()
        .filter(|&n| t.is_char_boundary(n))
        .find_map(|n| t[..n].parse::<f64>().ok())
}

impl DeviceSupport for EtherIpDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn set_record_info(&mut self, name: &str, scan: ScanType) {
        self.name = name.to_string();
        self.scan = scan;
    }

    fn set_process_context(&mut self, ctx: &ProcessContext) {
        self.udf = ctx.udf;
    }

    // Every C init_record/analyze_link failure returns an `S_*` status after
    // an errlogPrintf, never touching pact (`devEtherIP.c:884-1007,1183-1205`)
    // — the record scans on: the `Err` shape, kept by every `?` below.
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        // C's mbbi/mbbo(Direct) `init_record` zero SHFT (devEtherIP.c:1331,
        // 1355,1563,1597): the tag's NOBT bits are the raw value, unshifted.
        if matches!(
            record.record_type(),
            "mbbi" | "mbbo" | "mbbiDirect" | "mbboDirect"
        ) {
            record.put_field("SHFT", EpicsValue::UShort(0))?;
        }
        let (bits, count) = record_shape(record);
        let link = parse_link(&self.link_text, count, bits)
            .map_err(|e| CaError::InvalidValue(format!("devEtherIP: {e}")))?;

        let plc = driver::find_plc(&link.plc_name).ok_or_else(|| {
            CaError::InvalidValue(format!("devEtherIP: unknown PLC '{}'", link.plc_name))
        })?;

        // devEtherIP.c:1021-1035: a non-positive period at each step falls
        // through to the next.
        let positive = |p: &f64| *p > 0.0;
        let period = link
            .period
            .filter(positive)
            .or_else(|| get_period(&self.name, self.scan).filter(positive))
            .or_else(
                || match driver::DEFAULT_RATE_MS.load(std::sync::atomic::Ordering::Relaxed) {
                    0 => None,
                    ms => Some(ms as f64 / 1000.0),
                },
            )
            .unwrap_or_else(|| {
                errlog_vprintf(format_args!(
                    "devEtherIP ({}): cannot decode SCAN field, no scan flag given\n",
                    self.name
                ));
                errlog_vprintf(format_args!(
                    "Device support will use the default of 1 secs, "
                ));
                errlog_vprintf(format_args!("please complete the record configuration\n"));
                1.0
            });

        let tag = plc
            .add_tag(period, &link.string_tag, link.elements)
            .ok_or_else(|| {
                CaError::InvalidValue(format!(
                    "devEtherIP: cannot register tag '{}'",
                    link.string_tag
                ))
            })?;

        // One I/O Intr pulse per new tag value. Depth 1: a pending pulse
        // already says "re-read", and the record reads the current value.
        let (tx, rx) = mpsc::channel(1);
        tag.add_listener(tx);
        self.io_intr_rx = Some(rx);

        self.link = Some(link);
        self.plc = Some(plc);
        self.tag = Some(tag);
        // devEtherIP.c ao/bo/mbbo/mbboDirect_init_record: "return 2; /* don't
        // convert, we have no value, yet */".
        if matches!(record.record_type(), "ao" | "bo" | "mbbo" | "mbboDirect") {
            return Ok(DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched));
        }
        Ok(DeviceInitOutcome::ok())
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

    fn last_alarm(&self) -> Option<DeviceAlarm<'_>> {
        let (stat, sevr) = self.alarm?;
        Some(DeviceAlarm {
            stat,
            sevr,
            msg: "",
        })
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        if self.is_output {
            return self.readback(record);
        }

        // Driver statistics do not touch the tag's value at all.
        if self.link.as_ref().is_some_and(Link::is_statistic) {
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
                    "longin" => cip::get_dint(raw, element)
                        .map(|v| (EpicsValue::Long(v), true))
                        .ok_or(()),
                    "int64in" => cip::get_lint(raw, element)
                        .map(|v| (EpicsValue::Int64(v), true))
                        .ok_or(()),
                    "bi" => get_bits(raw, element, mask, 1)
                        .map(|v| (EpicsValue::Long(v as i32), false))
                        .ok_or(()),
                    "mbbi" | "mbbiDirect" => get_bits(raw, element, mask, nobt(record))
                        .map(|v| (EpicsValue::Long(v as i32), false))
                        .ok_or(()),
                    "stringin" => cip::get_string(raw, element, MAX_STRING_SIZE)
                        .map(|s| (EpicsValue::String(s.into()), true))
                        .ok_or(()),
                    "lsi" => cip::get_string(raw, element, count_field(record, "SIZV"))
                        .map(|s| (EpicsValue::String(s.into()), true))
                        .ok_or(()),
                    "waveform" => read_waveform(raw, element, record).map(|v| (v, true)),
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
                    // never touch udf. longin has no C dset and follows
                    // its LINT twin int64in.
                    let udf = match rtype {
                        "longin" | "int64in" | "waveform" => DeviceUdf::Untouched,
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
        let Some((tag, link)) = self.bound() else {
            self.write_alarm();
            return Ok(());
        };
        let (element, mask) = (link.element, link.mask);
        let rtype = record.record_type();

        let mut data = tag.data.lock();
        let elements = *tag.elements.lock();
        if !data.has_value() || elements <= element {
            drop(data);
            self.write_alarm();
            return Ok(());
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
            "longout" => {
                let val = i32_field(record, "VAL");
                match cip::get_dint(&data.buf, element) {
                    Some(cur) if cur == val => false,
                    Some(_) => cip::put_dint(&mut data.buf, element, val),
                    None => false,
                }
            }
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
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::String(s)) => s.to_string(),
                    _ => String::new(),
                };
                let max = data.buf.len();
                match cip::get_string(&data.buf, element, max) {
                    Some(cur) if cur == val => false,
                    Some(_) => cip::put_string(&mut data.buf, &val),
                    None => false,
                }
            }
            _ => {
                drop(data);
                self.write_alarm();
                return Ok(());
            }
        };

        if changed {
            tag.request_write(&mut data, &self.name);
        }
        drop(data);
        self.no_alarm();
        Ok(())
    }
}

impl EtherIpDevice {
    /// The output half of the driver callback: a new PLC value arrived, so the
    /// record follows it (C `check_ao_callback` & friends). With `FORCE` the
    /// record wins instead -- its value is staged for the next scan.
    fn readback(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let Some((tag, link)) = self.bound() else {
            self.read_alarm();
            return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
        };
        let (element, mask, forced) = (link.element, link.mask, link.forced());
        let rtype = record.record_type();

        let mut data = tag.data.lock();
        let elements = *tag.elements.lock();
        if !data.has_value() || elements <= element {
            drop(data);
            self.read_alarm();
            return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
        }

        let force = forced && self.udf == 0;

        match rtype {
            "ao" => match cip::typecode(&data.buf) {
                Some(CipType::Real) | Some(CipType::Lreal) => {
                    let Some(plc) = cip::get_double(&data.buf, element) else {
                        drop(data);
                        self.read_alarm();
                        return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
                    };
                    let val = f64_field(record, "VAL");
                    if plc != val {
                        if force {
                            cip::put_double(&mut data.buf, element, val);
                            tag.request_write(&mut data, &self.name);
                        } else {
                            record.set_val(EpicsValue::Double(plc))?;
                        }
                    }
                }
                _ => {
                    let Some(plc) = cip::get_dint(&data.buf, element) else {
                        drop(data);
                        self.read_alarm();
                        return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
                    };
                    let rval = i32_field(record, "RVAL");
                    if plc != rval {
                        if force {
                            cip::put_dint(&mut data.buf, element, rval);
                            tag.request_write(&mut data, &self.name);
                        } else if !record.apply_raw_readback(plc) {
                            record.set_val(EpicsValue::Long(plc))?;
                        }
                    }
                }
            },
            "longout" => {
                let Some(plc) = cip::get_dint(&data.buf, element) else {
                    drop(data);
                    self.read_alarm();
                    return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
                };
                let val = i32_field(record, "VAL");
                if plc != val {
                    if force {
                        cip::put_dint(&mut data.buf, element, val);
                        tag.request_write(&mut data, &self.name);
                    } else {
                        record.set_val(EpicsValue::Long(plc))?;
                    }
                }
            }
            "int64out" => {
                let Some(plc) = cip::get_lint(&data.buf, element) else {
                    drop(data);
                    self.read_alarm();
                    return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
                };
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::Int64(v)) => v,
                    _ => 0,
                };
                if plc != val {
                    if force {
                        cip::put_lint(&mut data.buf, element, val);
                        tag.request_write(&mut data, &self.name);
                    } else {
                        record.set_val(EpicsValue::Int64(plc))?;
                    }
                }
            }
            "bo" | "mbbo" | "mbboDirect" => {
                let bits = if rtype == "bo" { 1 } else { nobt(record) };
                let Some(plc) = get_bits(&data.buf, element, mask, bits) else {
                    drop(data);
                    self.read_alarm();
                    return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
                };
                let rval = i32_field(record, "RVAL") as u32;
                if plc != rval {
                    if force {
                        put_bits(&mut data.buf, element, mask, bits, rval);
                        tag.request_write(&mut data, &self.name);
                    } else if !record.apply_raw_readback(plc as i32) {
                        record.set_val(EpicsValue::Long(plc as i32))?;
                    }
                }
            }
            "stringout" | "lso" => {
                let max = data.buf.len();
                let Some(plc) = cip::get_string(&data.buf, element, max) else {
                    drop(data);
                    self.read_alarm();
                    return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
                };
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::String(s)) => s.to_string(),
                    _ => String::new(),
                };
                if plc != val {
                    if force {
                        cip::put_string(&mut data.buf, &val);
                        tag.request_write(&mut data, &self.name);
                    } else {
                        record.set_val(EpicsValue::String(plc.into()))?;
                    }
                }
            }
            _ => {
                drop(data);
                self.read_alarm();
                return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
            }
        }

        drop(data);
        self.no_alarm();
        // C's check_*_callbacks write udf = FALSE only when they copy the
        // PLC value in, but their update condition includes `rec->udf`, so
        // every pass that got this far ends with udf == 0: udf = 1 forces
        // the copy branch (FORCE requires !udf), and the skip/force paths
        // only run with udf already 0. The no-data and failed-get returns
        // above never touch udf in C, hence Untouched there.
        Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
    }
}

/// C `wf_read`. The CIP type picks the array type; the record coerces it to
/// FTVL.
///
/// UPSTREAM FIX: the C reads elements `0 .. NELM`, ignoring the element index
/// the link supplied -- even though `analyze_link` deliberately registers
/// `last_element + count` elements with the driver, which is only meaningful if
/// the read starts at `last_element`. `@PLC arr[5]` on a waveform with NELM=10
/// therefore hands back `arr[0..10]` in the C. We read `arr[5..15]`, the range
/// the registration reserves. Identical behaviour for the (usual) unindexed
/// link, where `element == 0`.
fn read_waveform(raw: &[u8], element: usize, record: &dyn Record) -> Result<EpicsValue, ()> {
    let nelm = count_field(record, "NELM");
    match cip::typecode(raw) {
        Some(CipType::Real) | Some(CipType::Lreal) => {
            let mut v = Vec::with_capacity(nelm);
            for i in 0..nelm {
                v.push(cip::get_double(raw, element + i).ok_or(())?);
            }
            Ok(EpicsValue::DoubleArray(v.into()))
        }
        Some(CipType::Sint) => {
            let mut v = Vec::with_capacity(nelm);
            for i in 0..nelm {
                v.push(cip::get_usint(raw, element + i).ok_or(())?);
            }
            Ok(EpicsValue::CharArray(v.into()))
        }
        Some(_) => {
            let mut v = Vec::with_capacity(nelm);
            for i in 0..nelm {
                v.push(cip::get_dint(raw, element + i).ok_or(())?);
            }
            Ok(EpicsValue::LongArray(v.into()))
        }
        None => Err(()),
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
    match ctx.dtyp {
        DTYP_RESET => Some(Box::new(EtherIpResetDevice::default())),
        DTYP => {
            let (text, is_output) = if !ctx.out.is_empty() {
                (ctx.out, true)
            } else {
                (ctx.inp, false)
            };
            Some(Box::new(EtherIpDevice::new(text, is_output)))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        driver::define_plc("forceudf", "127.0.0.1", 0);
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
            dev.set_process_context(&common.process_context());
            dev.read(&mut record).unwrap();
            assert_eq!(f64_field(&record, "VAL"), want_val, "udf={udf}");
            assert_eq!(tag.data.lock().do_write, want_write, "udf={udf}");
        }
    }

    // -- link parsing --------------------------------------------------------

    #[test]
    fn plain_scalar_link() {
        let l = parse_link("@plc1 REALS", 1, 0).unwrap();
        assert_eq!(l.plc_name, "plc1");
        assert_eq!(l.string_tag, "REALS");
        assert_eq!(l.element, 0);
        assert_eq!(l.elements, 1);
        assert_eq!(l.period, None);
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
    fn scan_and_bit_flags_take_a_value() {
        let l = parse_link("@plc1 Cnt S 0.5 B 3", 1, 0).unwrap();
        assert_eq!(l.period, Some(0.5));
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

    /// devEtherIP.c:1094: 255 for an unindexed binary link, NOBT or not.
    #[test]
    fn unindexed_binary_link_mask() {
        assert_eq!(parse_link("@plc1 aBool", 1, 1).unwrap().mask, 0xFF);
        assert_eq!(parse_link("@plc1 aWord", 1, 4).unwrap().mask, 0xFF);
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
        // Bits 30,31 of word 0 are 1,0; bits 0,1 of word 1 are 1,1. C's
        // first-bit test reads bit 30 as 0, so 4 bits give 0b1100.
        let raw = bits_buf(&[1 << 30, 0b11]);
        assert_eq!(get_bits(&raw, 0, 1 << 30, 4), Some(0b1100));
    }

    /// mbbi NOBT=2 on BOOLs[300] with bits 300 and 301 set: C reads 2, not 3.
    #[test]
    fn multi_bit_read_takes_cs_first_bit_from_bit_zero() {
        let raw = bits_buf(&[0x3000]);
        assert_eq!(get_bits(&raw, 0, 1 << 12, 2), Some(2));
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
        // C reads the first bit back from bit 0 (set here), not bit 4.
        assert_eq!(get_bits(&raw, 0, 1 << 4, 4), Some(0b0100));
        // Neighbouring bits are untouched.
        assert_eq!(cip::get_udint(&raw, 0), Some(0xFFFF_FF5F));
    }

    #[test]
    fn put_bits_across_a_udint_boundary() {
        let mut raw = bits_buf(&[0, 0]);
        assert!(put_bits(&mut raw, 0, 1 << 30, 4, 0b1101));
        assert_eq!(cip::get_udint(&raw, 0), Some(1 << 30));
        assert_eq!(cip::get_udint(&raw, 1), Some(0b11));
        assert_eq!(get_bits(&raw, 0, 1 << 30, 4), Some(0b1100));
    }

    #[test]
    fn put_bits_past_the_end_fails() {
        let mut raw = bits_buf(&[0]);
        assert!(!put_bits(&mut raw, 0, 1 << 31, 2, 3));
        assert!(!put_bits(&mut raw, 1, 1, 1, 1));
    }
}
