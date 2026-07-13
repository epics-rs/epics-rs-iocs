//! Device support for the standard record types (`devOpcua.cpp`).
//!
//! # How a record processes
//!
//! The C runs every OPC UA operation asynchronously: the record's `process`
//! sets `PACT`, asks the client to read or write, and the client later calls
//! back into `dbProcess` with a `ProcessReason` it stashed in the connector,
//! completing the record.
//!
//! **Framework gap.** `epics_rs`'s `DeviceSupport` cannot signal asynchronous
//! completion — `read`/`write` return their result there and then, and
//! `RecordProcessResult::AsyncPending` is reachable from *record* support only.
//! So the record is never left `PACT`.
//!
//! What replaces it: the driver owns each record's update queue and pulses the
//! record's I/O Intr channel whenever an update lands. The pulse is what
//! processes the record ([`DeviceSupport::io_intr_scan_independent`] is `true`,
//! so it fires whatever the record's `SCAN`), and the framework runs that pass
//! as a driver-callback cycle: the *read* stage runs even on an output record,
//! and the device write is suppressed (`processing.rs:1892-1894`, `:2485-2492` —
//! the path asyn's `asyn:READBACK` uses). So the two directions split cleanly:
//!
//! * [`OpcuaDevice::read`] — an update is waiting; deliver it. This is the C's
//!   `pcon->reason != none` branch.
//! * [`OpcuaDevice::write`] — the record processed on its own (a put, a scan, a
//!   `.PROC`); send its value. This is the C's `reason == none` branch.
//!
//! The update the record pops carries the reason it is processing for, which
//! also removes the C's *second* copy of that reason (`RecordConnector::reason`,
//! set around `dbProcess` in `RecordConnector.cpp:56-77`, against the reason
//! already inside the queued update, `Update.h:45`): here the queue is the only
//! source.
//!
//! Deviation, in full: an input record that starts a read of its own (a periodic
//! scan, or a `.PROC`) does not stay `PACT` waiting for the value. It completes
//! at once with the value it already had, and the value that arrives processes it
//! again through the I/O Intr pulse. Every value still reaches the record, in
//! order, exactly once; what differs is that one extra process pass publishes the
//! unchanged value first, and that the record is not `ACTIVE` in between.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::SystemTime;

use async_opcua::types::Variant;
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::base::server::device_support::{DeviceReadOutcome, DeviceSupport};
use epics_rs::base::server::recgbl::alarm_status::{COMM_ALARM, READ_ALARM, WRITE_ALARM};
use epics_rs::base::server::record::{ProcessContext, Record, ScanType};
use epics_rs::base::types::EpicsValue;
use epics_rs::ca::server::ioc_app::DeviceSupportContext;

use crate::link::{Bini, InfoDefaults, RecordKind, parse_link};
use crate::queue::{ConnectionStatus, ProcessReason, Update};
use crate::registry::{Binding, Registry};
use crate::session::{Priority, Request};
use crate::value::{self, EnumChoices};

/// The DTYP every OPC UA record uses (`opcua.dbd`: `device(ai, INST_IO,
/// devAiOpcua, "OPCUA")`).
pub const DTYP: &str = "OPCUA";

/// INVALID — the severity the C device support raises on every failure
/// (`recGblSetSevr(prec, *_ALARM, INVALID_ALARM)`).
const INVALID: u16 = 3;
/// MINOR — an uncertain status code still carries a value
/// (`DataElementOpen62541Leaf.h:733-737`).
const MINOR: u16 = 1;

/// How many state fields an mbbi/mbbo has (`NUMBER_OF_ENUM_CHOICES`,
/// `devOpcua.cpp:703`).
const ENUM_CHOICES: usize = 16;
const ENUM_VALUE_FIELDS: [&str; ENUM_CHOICES] = [
    "ZRVL", "ONVL", "TWVL", "THVL", "FRVL", "FVVL", "SXVL", "SVVL", "EIVL", "NIVL", "TEVL", "ELVL",
    "TVVL", "TTVL", "FTVL", "FFVL",
];
const ENUM_STRING_FIELDS: [&str; ENUM_CHOICES] = [
    "ZRST", "ONST", "TWST", "THST", "FRST", "FVST", "SXST", "SVST", "EIST", "NIST", "TEST", "ELST",
    "TVST", "TTST", "FTST", "FFST",
];
/// What the C parks in an unused state slot: "as invalid as possible"
/// (`devOpcua.cpp:725-727`). It must not be 0 — an incoming 0 would then match
/// the first unused slot instead of the state that really carries it.
const UNUSED_STATE_VALUE: u32 = u32::MAX;

/// How a record's fields map onto the node's value — the C's per-record `dset`
/// (`devOpcua.cpp:1349-1370`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Op {
    /// longin, longout — VAL as an `epicsInt32`.
    Int32,
    /// int64in, int64out — VAL as an `epicsInt64`.
    Int64,
    /// bi, mbbiDirect — RVAL as an `epicsUInt32`; the record converts it.
    UInt32Rval,
    /// bo — RVAL out; RVAL and VAL back.
    Bo,
    /// mbboDirect — RVAL out; RVAL, VAL and B0..B1F back.
    MbboDirect,
    /// mbbi, mbbo — RVAL (or VAL) plus the state table the server defines.
    Enum,
    /// ai, ao — VAL or RVAL, chosen by LINR.
    Analog,
    /// stringin, stringout — VAL as a 40-byte string.
    String,
    /// lsi, lso — VAL as a long string.
    LongString,
    /// waveform, aai, aao — VAL as an array, typed by FTVL.
    Array,
}

impl Op {
    /// `(op, is_output)` for a record type, or `None` if the type has no OPC UA
    /// device support.
    fn of(record_type: &str) -> Option<(Op, bool)> {
        Some(match record_type {
            "longin" => (Op::Int32, false),
            "longout" => (Op::Int32, true),
            "int64in" => (Op::Int64, false),
            "int64out" => (Op::Int64, true),
            "bi" | "mbbiDirect" => (Op::UInt32Rval, false),
            "bo" => (Op::Bo, true),
            "mbboDirect" => (Op::MbboDirect, true),
            "mbbi" => (Op::Enum, false),
            "mbbo" => (Op::Enum, true),
            "ai" => (Op::Analog, false),
            "ao" => (Op::Analog, true),
            "stringin" => (Op::String, false),
            "stringout" => (Op::String, true),
            "lsi" => (Op::LongString, false),
            "lso" => (Op::LongString, true),
            "waveform" | "aai" => (Op::Array, false),
            "aao" => (Op::Array, true),
            _ => return None,
        })
    }
}

/// What an mbbi/mbbo's state table holds — the C's `SDEF` bits
/// (`devOpcua.cpp:705-707`).
///
/// The C reads `prec->sdef`, which the record sets when the database defines a
/// state string, and then adds two bits of its own to remember whether it has
/// checked for state *values* and whether any are defined. `epics_rs`'s mbbi/mbbo
/// keep their `sdef` private, so the same three facts are derived from the state
/// fields at `init` — which is what the C's `ENUM_VALUES_CHECKED` pass does too,
/// one field at a time.
#[derive(Debug, Default, Clone, Copy)]
struct EnumState {
    /// The database defined a state table (C: `prec->sdef != 0`) — a string or a
    /// value. The server's enumeration must not overwrite it.
    defined_by_db: bool,
    /// The database defined state *values* (C: `SDEF & ENUM_VALUES_DEFINED`), so
    /// RVAL — not VAL — is what goes out, and what the state index comes from.
    values_defined: bool,
    /// The server's enumeration has been written into the state table (C:
    /// `SDEF & ENUMS_BY_OPCUA`), so it is applied once, not on every value.
    from_server: bool,
}

pub struct OpcuaDevice {
    registry: Arc<Registry>,
    link_text: String,
    record_name: String,
    op: Op,
    is_output: bool,
    bound: Option<Binding>,
    /// Pulses the record when an update is queued for it.
    notify: Option<mpsc::Receiver<()>>,
    /// Out-of-band PROPERTY posts — the state table the server's enumeration
    /// defines (`db_post_events(prec, &prec->val, DBE_PROPERTY)`,
    /// `devOpcua.cpp:732`).
    property_tx: Option<mpsc::Sender<Vec<(String, EpicsValue)>>>,
    property_rx: Option<mpsc::Receiver<Vec<(String, EpicsValue)>>>,
    info: HashMap<String, String>,
    enums: EnumState,
    /// The record's UDF at the start of this cycle — the ai smoothing needs it
    /// (`devOpcua.cpp:602`) and it is not a field the device can read.
    udf: bool,
    alarm: Option<(u16, u16)>,
    timestamp: Option<SystemTime>,
}

impl OpcuaDevice {
    pub fn new(registry: Arc<Registry>, link_text: String) -> Self {
        Self {
            registry,
            link_text,
            record_name: String::new(),
            op: Op::Int32,
            is_output: false,
            bound: None,
            notify: None,
            property_tx: None,
            property_rx: None,
            info: HashMap::new(),
            enums: EnumState::default(),
            udf: true,
            alarm: None,
            timestamp: None,
        }
    }

    fn bound(&self) -> CaResult<&Binding> {
        self.bound
            .as_ref()
            .ok_or_else(|| CaError::LinkError(format!("{}: link not bound", self.record_name)))
    }

    /// The record wants a value it does not have: ask the session to read the
    /// node (the C's `pcon->requestOpcuaRead()`).
    ///
    /// Framework gap: a record's PRIO lives in `RecordInstance.common`, which
    /// device support cannot reach (`set_record_info` passes the name and the
    /// scan, nothing else), so the C's three priority queues
    /// (`RequestQueueBatcher`, one per `menuPriority`) collapse to one here.
    fn request_read(&mut self) -> CaResult<DeviceReadOutcome> {
        let bound = self.bound()?;
        if bound.leaf.lock().state == ConnectionStatus::Down {
            self.alarm = Some((COMM_ALARM, INVALID));
        } else {
            let handle = bound.item.lock().client_handle;
            bound
                .session
                .request(Priority::Low, Request::Read { handle });
        }
        // Nothing about the record's value changed on this pass, so its
        // conversion must not run again over an unchanged RVAL.
        Ok(DeviceReadOutcome::computed())
    }

    /// The record's value goes to the server (`opcua_write_*`, the
    /// `reason == none || writeRequest` branch).
    fn send(&mut self, record: &mut dyn Record) -> CaResult<()> {
        let bound = self.bound()?;
        let (state, incoming, choices, linked_to_item) = {
            let leaf = bound.leaf.lock();
            (
                leaf.state,
                leaf.incoming.clone(),
                leaf.choices.clone(),
                leaf.link.linked_to_item,
            )
        };
        match state {
            ConnectionStatus::Down => {
                self.alarm = Some((COMM_ALARM, INVALID));
                return Ok(());
            }
            // The initial read is still on its way; the record's value must not
            // overtake it (`devOpcua.cpp:425`).
            ConnectionStatus::InitialRead => return Ok(()),
            _ => {}
        }

        // The node's type is the type of the value it last delivered — the C
        // switches on `typeKindOf(incomingData)` inside every `writeScalar`.
        let incoming = incoming.unwrap_or(Variant::Empty);
        let outgoing = match self.outgoing_value(record, &incoming, choices.as_deref()) {
            Ok(v) => v,
            Err(e) => {
                log::error!("{}: cannot write: {e}", self.record_name);
                self.alarm = Some((WRITE_ALARM, INVALID));
                return Ok(());
            }
        };

        {
            let mut leaf = bound.leaf.lock();
            leaf.outgoing = Some(outgoing);
            leaf.dirty = true;
        }
        // An element of an opcuaItem does not write on its own: the item record
        // decides when the structure goes out (WOC).
        if !linked_to_item && matches!(state, ConnectionStatus::Up | ConnectionStatus::InitialWrite)
        {
            let handle = bound.item.lock().client_handle;
            bound
                .session
                .request(Priority::Low, Request::Write { handle });
        }
        Ok(())
    }

    /// Build the value to send out of the record's fields.
    fn outgoing_value(
        &self,
        record: &mut dyn Record,
        incoming: &Variant,
        choices: Option<&EnumChoices>,
    ) -> Result<Variant, String> {
        let err = |e: value::ConvError| e.to_string();
        match self.op {
            Op::Int32 => value::write_scalar(long(record, "VAL")?, incoming, choices).map_err(err),
            Op::Int64 => value::write_scalar(int64(record, "VAL")?, incoming, choices).map_err(err),
            // bo, mbboDirect and an mbbo whose state values the database defined
            // all send RVAL; an mbbo without state values sends VAL, the state
            // index itself (`devOpcua.cpp:793-799`).
            Op::Bo | Op::MbboDirect | Op::UInt32Rval => {
                value::write_scalar(ulong(record, "RVAL")?, incoming, choices).map_err(err)
            }
            Op::Enum => {
                if self.enums.values_defined {
                    value::write_scalar(ulong(record, "RVAL")?, incoming, choices).map_err(err)
                } else {
                    value::write_scalar(u32::from(enum_index(record, "VAL")?), incoming, choices)
                        .map_err(err)
                }
            }
            // ao sends VAL when it does not convert, RVAL when it does
            // (`devOpcua.cpp:641-648`).
            Op::Analog => {
                if no_conversion(record) {
                    value::write_scalar(double(record, "VAL")?, incoming, choices).map_err(err)
                } else {
                    value::write_scalar(long(record, "RVAL")?, incoming, choices).map_err(err)
                }
            }
            Op::String => {
                value::write_string(&string(record, "VAL")?, incoming, choices).map_err(err)
            }
            Op::LongString => {
                value::write_string(&long_string(record)?, incoming, choices).map_err(err)
            }
            Op::Array => outgoing_array(record, incoming).map_err(err),
        }
    }

    /// One update the record popped.
    fn deliver(&mut self, update: Update, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        self.timestamp = Some(update.timestamp);
        if update.overrides > 0 {
            log::warn!(
                "{}: {} update(s) lost, the record's queue overran",
                self.record_name,
                update.overrides
            );
        }

        match update.reason {
            ProcessReason::ReadFailure => {
                self.alarm = Some((READ_ALARM, INVALID));
                Ok(DeviceReadOutcome::computed())
            }
            ProcessReason::WriteFailure => {
                self.alarm = Some((WRITE_ALARM, INVALID));
                Ok(DeviceReadOutcome::computed())
            }
            ProcessReason::ConnectionLoss => {
                self.alarm = Some((COMM_ALARM, INVALID));
                Ok(DeviceReadOutcome::computed())
            }
            // The write reached the server; nothing about the record changes.
            ProcessReason::WriteComplete => Ok(DeviceReadOutcome::computed()),
            // `bini=write`: the item is asking the record to send its own value.
            ProcessReason::WriteRequest => {
                if self.is_output {
                    self.send(record)?;
                } else {
                    log::warn!(
                        "{}: bini=write has nothing to write on an input record",
                        self.record_name
                    );
                }
                Ok(DeviceReadOutcome::computed())
            }
            // The item record's READ asks for a fresh value.
            //
            // The C treats a `readRequest` on an *output* record as incoming
            // data: `opcua_write_*` falls through to its read branch, which pops
            // an update that a read request never queued (`devOpcua.cpp:891`,
            // `:783`). Here it does what it says — request the read; the value
            // the server returns reaches the record through the normal
            // `readComplete` path.
            ProcessReason::ReadRequest | ProcessReason::None => self.request_read(),
            ProcessReason::IncomingData | ProcessReason::ReadComplete => {
                self.take_value(update, record)
            }
        }
    }

    /// A value from the server lands in the record.
    fn take_value(
        &mut self,
        update: Update,
        record: &mut dyn Record,
    ) -> CaResult<DeviceReadOutcome> {
        // A bad status carries no value (`DataElementOpen62541Leaf.h:730-734`).
        if update.status.is_bad() {
            self.alarm = Some((READ_ALARM, INVALID));
            return Ok(DeviceReadOutcome::computed());
        }
        let Some(data) = update.data else {
            self.alarm = Some((READ_ALARM, INVALID));
            return Ok(DeviceReadOutcome::computed());
        };
        if update.status.is_uncertain() {
            self.alarm = Some((READ_ALARM, MINOR));
        }

        let choices = self.bound()?.leaf.lock().choices.clone();
        match self.store(&data, choices.as_deref(), record) {
            Ok(outcome) => Ok(outcome),
            Err(e) => {
                log::error!("{}: incoming data unusable: {e}", self.record_name);
                self.alarm = Some((READ_ALARM, INVALID));
                Ok(DeviceReadOutcome::computed())
            }
        }
    }

    /// Convert one value into the record's fields.
    fn store(
        &mut self,
        data: &Variant,
        choices: Option<&EnumChoices>,
        record: &mut dyn Record,
    ) -> Result<DeviceReadOutcome, String> {
        let err = |e: value::ConvError| e.to_string();
        match self.op {
            Op::Int32 => {
                let v: i32 = value::read_scalar(data, choices).map_err(err)?;
                put(record, "VAL", EpicsValue::Long(v))?;
                Ok(DeviceReadOutcome::computed())
            }
            Op::Int64 => {
                let v: i64 = value::read_scalar(data, choices).map_err(err)?;
                put(record, "VAL", EpicsValue::Int64(v))?;
                Ok(DeviceReadOutcome::computed())
            }
            // bi's ZSV/OSV and mbbiDirect's bits come out of the record's own
            // RVAL -> VAL conversion, which is what `ok()` asks it to run.
            Op::UInt32Rval => {
                let v: u32 = value::read_scalar(data, choices).map_err(err)?;
                put(record, "RVAL", EpicsValue::ULong(v))?;
                Ok(DeviceReadOutcome::ok())
            }
            // An output record inverts its own forward conversion: MASK, SHFT,
            // the state table and the breakpoint table all live in the record,
            // and `apply_raw_readback` is the framework's entry into them
            // (`record_trait.rs:533-549`; C `processBo`/`processMbbo`).
            Op::Bo | Op::MbboDirect => {
                let v: u32 = value::read_scalar(data, choices).map_err(err)?;
                raw_readback(record, v)?;
                Ok(DeviceReadOutcome::computed())
            }
            Op::Enum => self.store_enum(data, choices, record),
            Op::Analog => self.store_analog(data, choices, record),
            Op::String => {
                let v = value::read_string(data, choices).map_err(err)?;
                put(record, "VAL", EpicsValue::String(truncate(&v, 39).into()))?;
                Ok(DeviceReadOutcome::computed())
            }
            // lsi/lso clamp the string to SIZV themselves and set LEN from it.
            Op::LongString => {
                let v = value::read_string(data, choices).map_err(err)?;
                put(record, "VAL", EpicsValue::CharArray(v.into_bytes()))?;
                Ok(DeviceReadOutcome::computed())
            }
            Op::Array => {
                store_array(data, record)?;
                Ok(DeviceReadOutcome::computed())
            }
        }
    }

    /// mbbi/mbbo (`opcua_read_enum`, and the read branch of `opcua_write_enum`).
    fn store_enum(
        &mut self,
        data: &Variant,
        choices: Option<&EnumChoices>,
        record: &mut dyn Record,
    ) -> Result<DeviceReadOutcome, String> {
        let raw: u32 = value::read_scalar(data, choices).map_err(|e| e.to_string())?;
        self.update_enum_infos(choices, record)?;

        if !self.enums.values_defined {
            // Without state values the raw value *is* the state index: the C
            // returns 2 ("don't convert") and sets both fields
            // (`devOpcua.cpp:766-770`, `:823-828`).
            put(record, "RVAL", EpicsValue::ULong(raw))?;
            let index = u16::try_from(raw).unwrap_or(u16::MAX);
            put(record, "VAL", EpicsValue::Enum(index))?;
            return Ok(DeviceReadOutcome::computed());
        }
        if self.is_output {
            // The record's readback does the mask, the shift and the state
            // lookup, over the MASK [`init_mask`] installed — which is what the
            // C spells out inline for mbbo (`devOpcua.cpp:812-826`), an output
            // record having no reverse conversion of its own.
            //
            // Deviation: the C shifts `prec->rval` down *in place* before the
            // lookup (`devOpcua.cpp:816`), leaving RVAL holding the shifted
            // value until the record's next forward conversion rebuilds it. Here
            // RVAL keeps the raw the server sent, masked
            // (`mbbo.rs:893-903`).
            raw_readback(record, raw)?;
            Ok(DeviceReadOutcome::computed())
        } else {
            // mbbi converts RVAL itself: mask, shift, state lookup
            // (`mbbiRecord.c::convert`).
            put(record, "RVAL", EpicsValue::ULong(raw))?;
            Ok(DeviceReadOutcome::ok())
        }
    }

    /// Write the server's enumeration into the record's state table, once
    /// (`updateEnumInfos`, `devOpcua.cpp:709-744`).
    ///
    /// The C applies it on the *initial read* only, and only when the database
    /// left the state table empty. Which connection state a record is in when it
    /// processes is no longer observable here — the item advances the state when
    /// it queues the update, not when the record pops it — so the equivalent gate
    /// is that the server's enumeration is applied at most once, and only to a
    /// record whose database definition left the table empty.
    ///
    /// The C posts the new table with `db_post_events(..., DBE_PROPERTY)` from
    /// inside `process`. Here the fields are written inline — the record's own
    /// RVAL -> VAL conversion runs on this same pass and needs them — and the
    /// framework does the PROPERTY post, off the channel the device hands it.
    fn update_enum_infos(
        &mut self,
        choices: Option<&EnumChoices>,
        record: &mut dyn Record,
    ) -> Result<(), String> {
        let Some(choices) = choices else {
            return Ok(());
        };
        if self.enums.defined_by_db || self.enums.from_server {
            return Ok(());
        }

        let mut posts: Vec<(String, EpicsValue)> = Vec::new();
        for (i, (value, name)) in choices.iter().take(ENUM_CHOICES).enumerate() {
            posts.push((ENUM_VALUE_FIELDS[i].to_string(), EpicsValue::ULong(*value)));
            posts.push((
                ENUM_STRING_FIELDS[i].to_string(),
                EpicsValue::String(truncate(name, 25).into()),
            ));
        }
        // Clear the slots the server does not use, so no stale string shows and
        // no unused slot can match an incoming value.
        for i in choices.len().min(ENUM_CHOICES)..ENUM_CHOICES {
            posts.push((
                ENUM_VALUE_FIELDS[i].to_string(),
                EpicsValue::ULong(UNUSED_STATE_VALUE),
            ));
            posts.push((
                ENUM_STRING_FIELDS[i].to_string(),
                EpicsValue::String(String::new().into()),
            ));
        }

        for (field, value) in &posts {
            put(record, field, value.clone())?;
            // The record derives "this state table is defined" (`sdef`, which is
            // what makes its RVAL -> VAL conversion a state lookup) from the
            // table's contents, and re-derives it only at init and from
            // `special()` — the hook a database put goes through
            // (`mbbi.rs:828-843`; C `mbbiRecord.c::special` -> `init_common`).
            // A device write is neither, so the re-derive is asked for here.
            record
                .special(field, true)
                .map_err(|e| format!("cannot re-derive the state table: {e}"))?;
        }
        self.enums.from_server = true;
        self.enums.values_defined = true;
        if let Some(tx) = &self.property_tx {
            let _ = tx.try_send(posts);
        }
        Ok(())
    }

    /// ai/ao (`opcua_read_analog`, and the read branch of `opcua_write_analog`).
    fn store_analog(
        &mut self,
        data: &Variant,
        choices: Option<&EnumChoices>,
        record: &mut dyn Record,
    ) -> Result<DeviceReadOutcome, String> {
        let converts = !no_conversion(record);
        let err = |e: value::ConvError| e.to_string();

        if self.is_output {
            // ao inverts its forward conversion inside the record: ROFF,
            // ASLO/AOFF, ESLO/EOFF or the breakpoint table
            // (`devOpcua.cpp:659-680` == `AoRecord::convert_readback`). Without
            // conversion it is the plain ASLO/AOFF scaling
            // (`apply_float64_readback`).
            if converts {
                let raw: i32 = value::read_scalar(data, choices).map_err(err)?;
                raw_readback(record, raw as u32)?;
            } else {
                let v: f64 = value::read_scalar(data, choices).map_err(err)?;
                if !record.apply_float64_readback(v) {
                    return Err("the record has no float64 readback".to_string());
                }
            }
            return Ok(DeviceReadOutcome::computed());
        }

        if converts {
            // RVAL, and the record's LINR conversion turns it into VAL.
            let raw: i32 = value::read_scalar(data, choices).map_err(err)?;
            put(record, "RVAL", EpicsValue::Long(raw))?;
            return Ok(DeviceReadOutcome::ok());
        }

        // ai with LINR = NO CONVERSION: the device applies ASLO/AOFF and the
        // smoothing itself and hands the record a finished VAL
        // (`devOpcua.cpp:595-611`). The record's own no-conversion path would
        // route the value through RVAL, an `epicsInt32`, and lose its fraction.
        let mut v: f64 = value::read_scalar(data, choices).map_err(err)?;
        let aslo = double(record, "ASLO").unwrap_or(0.0);
        let aoff = double(record, "AOFF").unwrap_or(0.0);
        if aslo != 0.0 {
            v *= aslo;
        }
        v += aoff;

        let smoo = double(record, "SMOO").unwrap_or(0.0);
        let old = double(record, "VAL").unwrap_or(f64::NAN);
        let value = if smoo == 0.0 || self.udf || !old.is_finite() {
            v
        } else {
            old * smoo + v * (1.0 - smoo)
        };
        put(record, "VAL", EpicsValue::Double(value))?;
        Ok(DeviceReadOutcome::computed())
    }
}

impl DeviceSupport for OpcuaDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn init(&mut self, record: &mut dyn Record) -> CaResult<()> {
        let record_type = record.record_type();
        let (op, is_output) = Op::of(record_type).ok_or_else(|| {
            CaError::LinkError(format!(
                "{}: record type '{record_type}' has no OPC UA device support",
                self.record_name
            ))
        })?;
        self.op = op;
        self.is_output = is_output;

        let kind = RecordKind {
            is_output,
            is_item_record: false,
        };
        let defaults = InfoDefaults::from_info(&self.info);
        let link = parse_link(&self.link_text, kind, &defaults, &*self.registry)
            .map_err(|e| CaError::LinkError(format!("{}: {e}", self.record_name)))?;

        // `bini=write` is what an output record's own value is for; an input
        // record has nothing to write.
        if link.bini == Bini::Write && !is_output {
            log::warn!(
                "{}: bini=write on the input record type '{record_type}'",
                self.record_name
            );
        }

        let (notify_tx, notify_rx) = mpsc::channel(1);
        let binding = self
            .registry
            .bind(&self.record_name, link, notify_tx)
            .map_err(|e| CaError::LinkError(format!("{}: {e}", self.record_name)))?;
        self.bound = Some(binding);
        self.notify = Some(notify_rx);

        init_mask(record).map_err(|e| CaError::LinkError(format!("{}: {e}", self.record_name)))?;

        if self.op == Op::Enum {
            let (tx, rx) = mpsc::channel(4);
            self.property_tx = Some(tx);
            self.property_rx = Some(rx);
            let values_defined = ENUM_VALUE_FIELDS
                .iter()
                .any(|f| ulong(record, f).is_ok_and(|v| v != 0));
            let strings_defined = ENUM_STRING_FIELDS
                .iter()
                .any(|f| string(record, f).is_ok_and(|s| !s.is_empty()));
            self.enums = EnumState {
                defined_by_db: values_defined || strings_defined,
                values_defined,
                from_server: false,
            };
        }
        Ok(())
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.record_name = name.to_string();
    }

    fn apply_record_info(&mut self, info: &HashMap<String, String>) {
        self.info = info.clone();
    }

    fn set_process_context(&mut self, ctx: &ProcessContext) {
        self.udf = ctx.udf;
    }

    fn io_intr_receiver(&mut self) -> Option<mpsc::Receiver<()>> {
        self.notify.take()
    }

    fn property_post_receiver(&mut self) -> Option<mpsc::Receiver<Vec<(String, EpicsValue)>>> {
        self.property_rx.take()
    }

    /// Every update reaches its record whatever the record's SCAN — the C
    /// processes it from a `callbackRequest` (`RecordConnector.cpp:88-113`),
    /// which does not consult the scan list either.
    fn io_intr_scan_independent(&self) -> bool {
        true
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        self.alarm = None;
        let popped = { self.bound()?.leaf.lock().queue.pop() };
        match popped {
            Some((update, _)) => self.deliver(update, record),
            // An output record reaches the read stage only on an I/O Intr pulse,
            // which is only fired for an update — an empty queue means another
            // pass already took it.
            None if self.is_output => Ok(DeviceReadOutcome::computed()),
            // The C's `reason == none` on an input record: start a read.
            None => self.request_read(),
        }
    }

    fn write(&mut self, record: &mut dyn Record) -> CaResult<()> {
        self.alarm = None;
        self.send(record)
    }

    fn last_alarm(&self) -> Option<(u16, u16)> {
        self.alarm
    }

    fn last_timestamp(&self) -> Option<SystemTime> {
        self.timestamp
    }
}

// ------------------------------------------------------------------ record fields

/// Install the MASK the mbb* conversions run over (`opcua_init_mask_read` and
/// `opcua_init_mask_write`, `devOpcua.cpp:143-157`, which the C wires into the
/// mbbi, mbbo, mbbiDirect and mbboDirect dsets).
///
/// The record's own init derives an unshifted `MASK = (1 << NOBT) - 1`
/// (`mbboRecord.c:136-137`), which is 0 when the database left NOBT out. This
/// widens that to every bit and shifts the window to where SHFT says the node's
/// value sits. The framework runs `init_record` before the device's `init`
/// (`ioc_builder.rs:305-364`) — the same order as the C — so what is installed
/// here is what the record converts with.
fn init_mask(record: &mut dyn Record) -> Result<(), String> {
    if !matches!(
        record.record_type(),
        "mbbi" | "mbbo" | "mbbiDirect" | "mbboDirect"
    ) {
        return Ok(());
    }
    let nobt = match field(record, "NOBT")? {
        EpicsValue::UShort(v) => u32::from(v),
        EpicsValue::Short(v) => v as u32,
        other => return Err(format!("NOBT is not a short ({other:?})")),
    };
    let mask = if nobt == 0 {
        u32::MAX
    } else {
        ulong(record, "MASK")?
    };
    let shft = ulong(record, "SHFT")?;
    let mask = mask.checked_shl(shft).unwrap_or(0);
    put(record, "MASK", EpicsValue::ULong(mask))
}

fn put(record: &mut dyn Record, name: &str, value: EpicsValue) -> Result<(), String> {
    record
        .put_field(name, value)
        .map_err(|e| format!("cannot set {name}: {e}"))
}

/// Hand a raw value to the record's own inverse conversion (MASK, SHFT, the
/// state table, the breakpoint table).
fn raw_readback(record: &mut dyn Record, raw: u32) -> Result<(), String> {
    if record.apply_raw_readback(raw as i32) {
        Ok(())
    } else {
        Err("the record has no raw readback".to_string())
    }
}

fn field(record: &dyn Record, name: &str) -> Result<EpicsValue, String> {
    record
        .get_field(name)
        .ok_or_else(|| format!("the record has no {name} field"))
}

fn long(record: &dyn Record, name: &str) -> Result<i32, String> {
    match field(record, name)? {
        EpicsValue::Long(v) => Ok(v),
        EpicsValue::ULong(v) => Ok(v as i32),
        EpicsValue::Short(v) => Ok(i32::from(v)),
        EpicsValue::Enum(v) => Ok(i32::from(v)),
        other => Err(format!("{name} is not an integer ({other:?})")),
    }
}

fn int64(record: &dyn Record, name: &str) -> Result<i64, String> {
    match field(record, name)? {
        EpicsValue::Int64(v) => Ok(v),
        EpicsValue::Long(v) => Ok(i64::from(v)),
        other => Err(format!("{name} is not a 64-bit integer ({other:?})")),
    }
}

fn ulong(record: &dyn Record, name: &str) -> Result<u32, String> {
    match field(record, name)? {
        EpicsValue::ULong(v) => Ok(v),
        EpicsValue::Long(v) => Ok(v as u32),
        EpicsValue::UShort(v) => Ok(u32::from(v)),
        EpicsValue::Enum(v) => Ok(u32::from(v)),
        other => Err(format!("{name} is not an unsigned integer ({other:?})")),
    }
}

fn short(record: &dyn Record, name: &str) -> Result<i16, String> {
    match field(record, name)? {
        EpicsValue::Short(v) => Ok(v),
        EpicsValue::UShort(v) => Ok(v as i16),
        EpicsValue::Enum(v) => Ok(v as i16),
        other => Err(format!("{name} is not a short ({other:?})")),
    }
}

fn double(record: &dyn Record, name: &str) -> Result<f64, String> {
    match field(record, name)? {
        EpicsValue::Double(v) => Ok(v),
        EpicsValue::Float(v) => Ok(f64::from(v)),
        EpicsValue::Long(v) => Ok(f64::from(v)),
        other => Err(format!("{name} is not a number ({other:?})")),
    }
}

fn enum_index(record: &dyn Record, name: &str) -> Result<u16, String> {
    match field(record, name)? {
        EpicsValue::Enum(v) => Ok(v),
        EpicsValue::EnumWithChoices { index, .. } => Ok(index),
        EpicsValue::Short(v) => Ok(v as u16),
        EpicsValue::Long(v) => Ok(v as u16),
        other => Err(format!("{name} is not a state index ({other:?})")),
    }
}

fn string(record: &dyn Record, name: &str) -> Result<String, String> {
    match field(record, name)? {
        EpicsValue::String(s) => Ok(s.to_string()),
        other => Err(format!("{name} is not a string ({other:?})")),
    }
}

/// lsi/lso hold their value as bytes, LEN long.
fn long_string(record: &dyn Record) -> Result<String, String> {
    match field(record, "VAL")? {
        EpicsValue::CharArray(bytes) | EpicsValue::UCharArray(bytes) => {
            let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
            Ok(String::from_utf8_lossy(&bytes[..end]).into_owned())
        }
        EpicsValue::String(s) => Ok(s.to_string()),
        other => Err(format!("VAL is not a long string ({other:?})")),
    }
}

/// LINR = 0 is `menuConvertNO_CONVERSION`.
fn no_conversion(record: &dyn Record) -> bool {
    short(record, "LINR").unwrap_or(0) == 0
}

fn truncate(s: &str, max: usize) -> String {
    // EPICS field sizes are byte counts; never split a character in half.
    let mut end = s.len().min(max);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    s[..end].to_string()
}

// ------------------------------------------------------------------------ arrays

/// The record's VAL keeps its element type (its FTVL), so an incoming array is
/// converted to exactly that type — the C picks the `readArray<T>` overload from
/// FTVL (`devOpcua.cpp:1083-1134`).
fn store_array(data: &Variant, record: &mut dyn Record) -> Result<(), String> {
    let nelm = usize::try_from(long(record, "NELM").unwrap_or(0)).unwrap_or(0);
    let current = field(record, "VAL")?;
    let err = |e: value::ConvError| e.to_string();

    let value = match current {
        // FTVL = CHAR doubles as a long string (`devOpcua.cpp:1084-1087`), so a
        // String node lands in it as its bytes.
        EpicsValue::CharArray(_) | EpicsValue::UCharArray(_) => {
            let bytes = match data {
                Variant::String(_) | Variant::ByteString(_) | Variant::LocalizedText(_) => {
                    value::read_string(data, None).map_err(err)?.into_bytes()
                }
                _ => value::read_array_u8(data).map_err(err)?,
            };
            EpicsValue::CharArray(clamp(bytes, nelm))
        }
        EpicsValue::ShortArray(_) => {
            EpicsValue::ShortArray(clamp(value::read_array::<i16>(data).map_err(err)?, nelm))
        }
        EpicsValue::UShortArray(_) => {
            EpicsValue::UShortArray(clamp(value::read_array::<u16>(data).map_err(err)?, nelm))
        }
        EpicsValue::LongArray(_) => {
            EpicsValue::LongArray(clamp(value::read_array::<i32>(data).map_err(err)?, nelm))
        }
        EpicsValue::ULongArray(_) => {
            EpicsValue::ULongArray(clamp(value::read_array::<u32>(data).map_err(err)?, nelm))
        }
        EpicsValue::Int64Array(_) => {
            EpicsValue::Int64Array(clamp(value::read_array::<i64>(data).map_err(err)?, nelm))
        }
        EpicsValue::UInt64Array(_) => {
            EpicsValue::UInt64Array(clamp(value::read_array::<u64>(data).map_err(err)?, nelm))
        }
        EpicsValue::FloatArray(_) => {
            EpicsValue::FloatArray(clamp(value::read_array::<f32>(data).map_err(err)?, nelm))
        }
        EpicsValue::DoubleArray(_) => {
            EpicsValue::DoubleArray(clamp(value::read_array::<f64>(data).map_err(err)?, nelm))
        }
        EpicsValue::StringArray(_) => EpicsValue::StringArray(
            clamp(value::read_string_array(data).map_err(err)?, nelm)
                .into_iter()
                .map(|s| truncate(&s, 39).into())
                .collect(),
        ),
        other => return Err(format!("VAL is not an array ({other:?})")),
    };
    put(record, "VAL", value)
}

/// VAL comes back NORD elements long, which is exactly what the C sends
/// (`writeArray(prec->bptr, prec->nord)`).
fn outgoing_array(record: &mut dyn Record, incoming: &Variant) -> value::Result<Variant> {
    let current = record.get_field("VAL").unwrap_or(EpicsValue::Long(0));
    match current {
        EpicsValue::CharArray(v) | EpicsValue::UCharArray(v) => match incoming {
            Variant::String(_) | Variant::ByteString(_) | Variant::LocalizedText(_) => {
                let end = v.iter().position(|b| *b == 0).unwrap_or(v.len());
                value::write_string(&String::from_utf8_lossy(&v[..end]), incoming, None)
            }
            _ => value::write_array_u8(&v, incoming),
        },
        EpicsValue::ShortArray(v) => value::write_array(&v, incoming),
        EpicsValue::UShortArray(v) => value::write_array(&v, incoming),
        EpicsValue::LongArray(v) => value::write_array(&v, incoming),
        EpicsValue::ULongArray(v) => value::write_array(&v, incoming),
        EpicsValue::Int64Array(v) => value::write_array(&v, incoming),
        EpicsValue::UInt64Array(v) => value::write_array(&v, incoming),
        EpicsValue::FloatArray(v) => value::write_array(&v, incoming),
        EpicsValue::DoubleArray(v) => value::write_array(&v, incoming),
        EpicsValue::StringArray(v) => {
            let strings: Vec<String> = v.iter().map(|s| s.to_string()).collect();
            value::write_string_array(&strings, incoming)
        }
        other => Err(value::ConvError::Unsupported {
            from: format!("{other:?}"),
            to: value::type_name(incoming),
        }),
    }
}

fn clamp<T>(mut v: Vec<T>, nelm: usize) -> Vec<T> {
    if nelm > 0 && v.len() > nelm {
        v.truncate(nelm);
    }
    v
}

// -------------------------------------------------------------------- the factory

/// The device-support factory the IOC registers for DTYP `OPCUA`.
pub fn factory(
    registry: Arc<Registry>,
) -> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    move |ctx: &DeviceSupportContext| {
        if ctx.dtyp != DTYP {
            return None;
        }
        let raw = if ctx.inp.is_empty() { ctx.out } else { ctx.inp };
        // The link text is the db field verbatim, so an INST_IO link still
        // carries its leading '@'.
        let link_text = raw.strip_prefix('@').unwrap_or(raw).trim();
        Some(
            Box::new(OpcuaDevice::new(registry.clone(), link_text.to_string()))
                as Box<dyn DeviceSupport>,
        )
    }
}
