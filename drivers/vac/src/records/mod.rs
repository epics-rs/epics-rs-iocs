//! Custom record types: `vs` (vacuum gauge) and `digitel` (ion pump).
//!
//! Both are registered from the IOC crate with
//! `IocApplication::register_record_type`; no `.dbd` file is involved, because
//! the framework's database loader resolves field types from
//! [`Record::field_list`](epics_rs::base::server::record::Record::field_list)
//! and menu choices from `menu_field_choices`. A field the record does not
//! declare falls through to `dbCommon` — which is how `INP`, `SCAN` and the
//! alarm-severity fields reach the record instance.

pub mod digitel;
pub mod vs;

/// Declare a record's scalar fields once, generating the `FieldDesc` table and
/// the `get_field` / `put_field` bodies from the same list.
///
/// Each entry is `"NAME": Variant = struct_field, read_only`, where `Variant` is
/// the [`EpicsValue`](epics_rs::base::types::EpicsValue) variant that carries the
/// field's `.dbd` type. `read_only` is `true` for `special(SPC_NOMOD)` fields,
/// which the C record refuses to let a client write.
///
/// Fields absent from the list (`INP`, `SCAN`, `HHSV`, ...) are dbCommon fields
/// and are served by the framework.
macro_rules! record_fields {
    (
        $list:ident, $get:ident, $put:ident, $rec:ty;
        $( $name:literal : $variant:ident = $field:ident , $ro:literal );+ $(;)?
    ) => {
        static $list: &[::epics_rs::base::server::record::FieldDesc] = &[
            $(::epics_rs::base::server::record::FieldDesc {
                name: $name,
                dbf_type: $crate::records::dbf_type_of!($variant),
                read_only: $ro,
            }),+
        ];

        fn $get(r: &$rec, name: &str) -> Option<::epics_rs::base::types::EpicsValue> {
            match name {
                $($name => Some(::epics_rs::base::types::EpicsValue::$variant(r.$field)),)+
                _ => None,
            }
        }

        fn $put(
            r: &mut $rec,
            name: &str,
            value: ::epics_rs::base::types::EpicsValue,
        ) -> ::epics_rs::base::error::CaResult<()> {
            match name {
                $($name => match value {
                    ::epics_rs::base::types::EpicsValue::$variant(v) => {
                        r.$field = v;
                        Ok(())
                    }
                    _ => Err(::epics_rs::base::error::CaError::TypeMismatch(name.into())),
                },)+
                _ => Err(::epics_rs::base::error::CaError::FieldNotFound(name.into())),
            }
        }
    };
}

/// The `DBF_*` type each `EpicsValue` variant carries.
macro_rules! dbf_type_of {
    (Short) => {
        ::epics_rs::base::types::DbFieldType::Short
    };
    (UShort) => {
        ::epics_rs::base::types::DbFieldType::UShort
    };
    (Long) => {
        ::epics_rs::base::types::DbFieldType::Long
    };
    (ULong) => {
        ::epics_rs::base::types::DbFieldType::ULong
    };
    (Float) => {
        ::epics_rs::base::types::DbFieldType::Float
    };
    (Double) => {
        ::epics_rs::base::types::DbFieldType::Double
    };
    (Enum) => {
        ::epics_rs::base::types::DbFieldType::Enum
    };
}

pub(crate) use dbf_type_of;
pub(crate) use record_fields;

/// `recGblSetSevr`'s return value, which the C `checkAlarms` routines branch on
/// to decide whether to latch `LALM`.
///
/// `epics_base_rs::server::recgbl::rec_gbl_set_sevr` returns `()`, so the
/// "did this raise the pending severity?" test is applied here — it is exactly
/// the condition the function itself uses (`recGbl.c:258-261`).
pub(crate) fn set_sevr(
    common: &mut epics_rs::base::server::record::CommonFields,
    stat: u16,
    sevr: epics_rs::base::server::record::AlarmSeverity,
) -> bool {
    let raised = (sevr as u16) > (common.nsev as u16);
    epics_rs::base::server::recgbl::rec_gbl_set_sevr(common, stat, sevr);
    raised
}

/// `menu(menuAlarmSevr)` index → [`AlarmSeverity`]. A record's `HHSV`/`LLSV`/
/// `HSV`/`LSV` are dbCommon-typed menu fields, so they arrive as raw indices.
pub(crate) fn severity_of(index: u16) -> epics_rs::base::server::record::AlarmSeverity {
    use epics_rs::base::server::record::AlarmSeverity;
    match index {
        1 => AlarmSeverity::Minor,
        2 => AlarmSeverity::Major,
        3 => AlarmSeverity::Invalid,
        _ => AlarmSeverity::NoAlarm,
    }
}
