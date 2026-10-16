//! Custom record types: `vs` (vacuum gauge) and `digitel` (ion pump).
//!
//! Their field tables, menus and `device()` rows are generated from the C
//! `.dbd` files vendored under `dbd/` ([`dbd_generated`]) and handed to base by
//! [`register_vac_dbd`], C's `dbLoadDatabase` of `vacSupport.dbd`.

pub mod dbd_generated;
pub mod digitel;
mod dsets;
pub mod vs;

/// Hand vac's generated `.dbd` to base: the two record types, their menus,
/// and the `asyn DigitelPump` / `asyn VacSen` `DTYP` choices. Idempotent.
pub fn register_vac_dbd() {
    use dbd_generated::{DBD, RECORD_TYPES, devices};
    epics_rs::base::server::record::register_contributed_dbd(&DBD);
    for &record_type in RECORD_TYPES {
        let rows = devices(record_type).unwrap_or_default();
        epics_rs::base::server::record::register_dbd_devices(record_type, rows);
    }
}

/// Generate a record's scalar `get_field` / `put_field` bodies from one list.
///
/// Each entry is `"NAME": Variant = struct_field`, where `Variant` is the
/// [`EpicsValue`](epics_rs::base::types::EpicsValue) variant that carries the
/// field's `.dbd` type.
macro_rules! record_fields {
    (
        $get:ident, $put:ident, $rec:ty;
        $( $name:literal : $variant:ident = $field:ident );+ $(;)?
    ) => {
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

pub(crate) use record_fields;

/// `recGblSetSevr`'s return value, which the C `checkAlarms` routines branch on
/// to decide whether to latch `LALM`.
pub(crate) fn set_sevr(
    common: &mut epics_rs::base::server::record::CommonFields,
    stat: u16,
    sevr: epics_rs::base::server::record::AlarmSeverity,
) -> bool {
    epics_rs::base::server::recgbl::rec_gbl_set_sevr(common, stat, sevr)
}

/// `menu(menuAlarmSevr)` index → [`AlarmSeverity`](epics_rs::base::server::record::AlarmSeverity). A record's `HHSV`/`LLSV`/
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

#[cfg(test)]
mod tests {
    use epics_rs::base::server::record::Record;

    /// Every field the `.dbd` declares is served by the record, as the type it
    /// declares.
    #[test]
    fn every_declared_field_is_served_as_declared() {
        let records: [Box<dyn Record>; 2] = [
            Box::new(super::digitel::DigitelRecord::default()),
            Box::new(super::vs::VsRecord::default()),
        ];
        for rec in &records {
            for f in rec.declared_fields() {
                let v = rec
                    .get_field(f.name)
                    .unwrap_or_else(|| panic!("{}.{} not served", rec.record_type(), f.name));
                assert_eq!(
                    v.db_field_type(),
                    f.dbf_type,
                    "{}.{}",
                    rec.record_type(),
                    f.name
                );
            }
        }
    }
}
