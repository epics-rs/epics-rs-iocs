//! The `opcuaItem` record type (`opcuaItemRecord.cpp`, `opcuaItemRecord.dbd`).
//!
//! One record for one OPC UA node, with no value of its own: it exists so that
//! several records can bind to the *elements* of a structured node (their link
//! names this record instead of a session), and so that the read or write of the
//! whole structure can be ordered from the database — READ, WRITE, DEFACTN, WOC.
//!
//! The C splits the type in two, and so does this port: the record type here
//! (the C's `rset`) owns the fields and the `special()` handling, and
//! [`crate::device_support::OpcuaDevice`] with [`Op::Item`] owns the OPC UA
//! action (the C's `dset`, `opcua_action_item`). The two meet where the C's meet
//! — the C's `prec->dpvt`, and here the [`Binding`] the device support hands the
//! record at init.
//!
//! [`Op::Item`]: crate::device_support::Op
//!
//! No `.dbd` file is involved: the framework's database loader takes the field
//! types from [`Record::declared_fields`] and the menu choices from
//! `menu_field_choices`, and serves every field this record does not declare
//! (INP, SCAN, PINI, the alarm fields) from `dbCommon`.

use std::any::Any;

use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::record::{FieldDesc, ProcessOutcome, Record};
use epics_rs::base::types::{DbFieldType, EpicsValue};

use crate::queue::ProcessReason;
use crate::registry::Binding;

/// `menu(menuDefAction)` — what the record does when it processes for no
/// particular reason.
pub const DEF_ACTION: &[&str] = &["read", "write"];
/// `menu(menuBini)` — what happens on the initial read.
pub const BINI: &[&str] = &["read", "ignore", "write"];
/// `menu(menuWoc)` — write on change, or on command.
pub const WOC: &[&str] = &["manual", "immediate"];

/// `menuDefActionWRITE`.
const DEF_ACTION_WRITE: u16 = 1;
/// `menuWocIMMEDIATE`.
const WOC_IMMEDIATE: u16 = 1;

/// The length of a `DBF_STRING` field, minus its terminator.
const MAX_STRING_SIZE: usize = 40;

static FIELDS: &[FieldDesc] = &[
    FieldDesc::new("VAL", DbFieldType::ULong, false),
    FieldDesc::new("SESS", DbFieldType::String, false),
    FieldDesc::new("SUBS", DbFieldType::String, false),
    FieldDesc::new("DEFACTN", DbFieldType::Enum, false),
    FieldDesc::new("BINI", DbFieldType::Enum, false),
    FieldDesc::new("READ", DbFieldType::Char, false),
    FieldDesc::new("WRITE", DbFieldType::Char, false),
    FieldDesc::new("STATCODE", DbFieldType::ULong, false),
    FieldDesc::new("OSTATCODE", DbFieldType::ULong, false),
    FieldDesc::new("STATTEXT", DbFieldType::String, false),
    FieldDesc::new("WOC", DbFieldType::Enum, false),
];

#[derive(Debug, Default)]
pub struct OpcuaItemRecord {
    /// "Dummy Value" — the record has none. A put to it processes the record,
    /// which is one way to order the default action.
    pub val: u32,
    /// The session and subscription the link named, filled in at init.
    pub sess: String,
    pub subs: String,
    pub defactn: u16,
    pub bini: u16,
    /// A put to either orders that action on the next process, which the put
    /// itself triggers (`pp(TRUE)`).
    pub read: u8,
    pub write: u8,
    pub statcode: u32,
    pub ostatcode: u32,
    pub stattext: String,
    pub woc: u16,

    /// The C's `prec->dpvt`: what ties the record to its item.
    binding: Option<Binding>,
    /// The C's `pcon->reason`, as `special()` sets it — the action a put to READ
    /// or WRITE ordered, consumed by the process pass the put triggers.
    pending: Option<ProcessReason>,
    /// The C clears `prec->udf` on every action that did not fail
    /// (`opcuaItemRecord.cpp::readwrite`).
    acted: bool,
}

impl OpcuaItemRecord {
    /// The device support hands the record its item at init — the C's
    /// `prec->dpvt = pvt.release()` (`opcuaItemRecord.cpp:70`).
    pub fn attach(&mut self, binding: Binding) {
        self.binding = Some(binding);
    }

    pub fn binding(&self) -> Option<&Binding> {
        self.binding.as_ref()
    }

    /// The action a put to READ or WRITE ordered, if any. Taking it is what
    /// keeps it to the one process pass the put triggers, exactly as the C's
    /// `process` resets `pcon->reason` to `none` after every pass
    /// (`opcuaItemRecord.cpp:99`).
    pub fn take_pending(&mut self) -> Option<ProcessReason> {
        self.pending.take()
    }

    /// The item's action reported a failure, so UDF stays set.
    pub fn set_acted(&mut self, ok: bool) {
        if ok {
            self.acted = true;
        }
    }

    /// `DEFACTN` — read, or write.
    pub fn default_reason(&self) -> ProcessReason {
        if self.defactn == DEF_ACTION_WRITE {
            ProcessReason::WriteRequest
        } else {
            ProcessReason::ReadRequest
        }
    }
}

impl Record for OpcuaItemRecord {
    fn record_type(&self) -> &'static str {
        "opcuaItem"
    }

    fn declared_fields(&self) -> &'static [FieldDesc] {
        FIELDS
    }

    fn get_field(&self, name: &str) -> Option<EpicsValue> {
        Some(match name {
            "VAL" => EpicsValue::ULong(self.val),
            "SESS" => EpicsValue::String(self.sess.clone().into()),
            "SUBS" => EpicsValue::String(self.subs.clone().into()),
            "DEFACTN" => EpicsValue::Enum(self.defactn),
            "BINI" => EpicsValue::Enum(self.bini),
            "READ" => EpicsValue::Char(self.read),
            "WRITE" => EpicsValue::Char(self.write),
            "STATCODE" => EpicsValue::ULong(self.statcode),
            "OSTATCODE" => EpicsValue::ULong(self.ostatcode),
            "STATTEXT" => EpicsValue::String(self.stattext.clone().into()),
            "WOC" => EpicsValue::Enum(self.woc),
            _ => return None,
        })
    }

    fn put_field(&mut self, name: &str, value: EpicsValue) -> CaResult<()> {
        let mismatch = || CaError::TypeMismatch(name.into());
        match name {
            "VAL" => self.val = ulong(&value).ok_or_else(mismatch)?,
            "SESS" => self.sess = string(&value).ok_or_else(mismatch)?,
            "SUBS" => self.subs = string(&value).ok_or_else(mismatch)?,
            "DEFACTN" => self.defactn = index(&value).ok_or_else(mismatch)?,
            "BINI" => self.bini = index(&value).ok_or_else(mismatch)?,
            "READ" => self.read = byte(&value).ok_or_else(mismatch)?,
            "WRITE" => self.write = byte(&value).ok_or_else(mismatch)?,
            "STATCODE" => self.statcode = ulong(&value).ok_or_else(mismatch)?,
            "OSTATCODE" => self.ostatcode = ulong(&value).ok_or_else(mismatch)?,
            "STATTEXT" => self.stattext = string(&value).ok_or_else(mismatch)?,
            "WOC" => self.woc = index(&value).ok_or_else(mismatch)?,
            _ => return Err(CaError::FieldNotFound(name.into())),
        }
        Ok(())
    }

    fn menu_field_choices(&self, field: &str) -> Option<&'static [&'static str]> {
        match field {
            "DEFACTN" => Some(DEF_ACTION),
            "BINI" => Some(BINI),
            "WOC" => Some(WOC),
            _ => None,
        }
    }

    /// `pp(TRUE)` in `opcuaItemRecord.dbd`.
    fn process_passive_fields(&self) -> &'static [&'static str] {
        &["VAL", "READ", "WRITE"]
    }

    /// `opcuaItemRecord.cpp::special` — the `SPC_MOD` fields.
    ///
    /// READ and WRITE order the action the process pass that follows them
    /// carries out (both are `pp(TRUE)`, so the put itself is what processes the
    /// record). WOC is not: switching it to `immediate` sends whatever its
    /// elements have already written, there and then.
    fn special(&mut self, field: &str, after: bool) -> CaResult<()> {
        if !after {
            return Ok(());
        }
        match field {
            "READ" => self.pending = Some(ProcessReason::ReadRequest),
            "WRITE" => self.pending = Some(ProcessReason::WriteRequest),
            "WOC" if self.woc == WOC_IMMEDIATE => {
                // `Item::requestWriteIfDirty` (`ItemOpen62541.cpp:83-88`): the
                // item asks its record to process for a write, and the write
                // request travels as an update on the record's own queue — the
                // one path any reason to process takes here.
                if let Some(binding) = &self.binding
                    && binding.item.lock().has_dirty_leaf()
                {
                    binding
                        .leaf
                        .lock()
                        .request_processing(ProcessReason::WriteRequest);
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// The C's `monitor()` (`opcuaItemRecord.cpp:138-148`): STATCODE and
    /// STATTEXT are posted when the status code changed, and OSTATCODE — which
    /// is only there to detect that change — is never posted.
    ///
    /// The framework's change-detection loop posts every field that changed over
    /// the pass, so latching OSTATCODE here is what makes the STATCODE/STATTEXT
    /// posts land exactly on the cycles the C posts them, and
    /// [`Record::event_posted_fields`] keeps OSTATCODE itself out of the loop.
    fn process(&mut self) -> CaResult<ProcessOutcome> {
        self.ostatcode = self.statcode;
        Ok(ProcessOutcome::complete())
    }

    fn event_posted_fields(&self) -> &'static [&'static str] {
        &["OSTATCODE"]
    }

    /// The C clears UDF on any action that did not report a failure, and never
    /// checks UDF for an alarm of its own (`opcuaItemRecord.cpp::readwrite`,
    /// against a `process` that calls neither `recGblCheckUdf` nor a
    /// `checkAlarms`).
    fn value_is_undefined(&self) -> bool {
        !self.acted
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn Any> {
        Some(self)
    }
}

fn ulong(value: &EpicsValue) -> Option<u32> {
    match value {
        EpicsValue::ULong(v) => Some(*v),
        EpicsValue::Long(v) => Some(*v as u32),
        EpicsValue::Short(v) => Some(*v as u32),
        EpicsValue::Char(v) => Some(u32::from(*v)),
        EpicsValue::Double(v) => Some(*v as u32),
        _ => None,
    }
}

fn index(value: &EpicsValue) -> Option<u16> {
    match value {
        EpicsValue::Enum(v) => Some(*v),
        EpicsValue::EnumWithChoices { index, .. } => Some(*index),
        EpicsValue::Short(v) => Some(*v as u16),
        EpicsValue::Long(v) => Some(*v as u16),
        _ => None,
    }
}

fn byte(value: &EpicsValue) -> Option<u8> {
    match value {
        EpicsValue::Char(v) => Some(*v),
        EpicsValue::Short(v) => Some(*v as u8),
        EpicsValue::Long(v) => Some(*v as u8),
        EpicsValue::Enum(v) => Some(*v as u8),
        _ => None,
    }
}

fn string(value: &EpicsValue) -> Option<String> {
    match value {
        EpicsValue::String(s) => {
            let mut s = s.to_string();
            s.truncate(MAX_STRING_SIZE);
            Some(s)
        }
        _ => None,
    }
}
