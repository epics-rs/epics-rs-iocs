//! IOC wiring: record-type factories and the dynamic device-support factory.
//!
//! An IOC crate registers the two record types with
//! `IocApplication::register_record_type` and installs
//! [`device_support_factory`] with
//! `IocApplication::register_dynamic_device_support`. The factory parses the
//! record's `INP` link (both records carry their asyn octet link in `INP`) and
//! hands back the device support matching the record's `DTYP`.

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::base::server::RecordFactory;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::ioc_app::DeviceSupportContext;

use crate::device_support::digitel_pump::{self, DigitelPump};
use crate::device_support::vac_sen::{self, VacSen};
use crate::records::digitel::DigitelRecord;
use crate::records::vs::VsRecord;

/// Path to the bundled database template directory.
pub const VAC_DB_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/db");

/// The `vs` record-type factory, for `IocApplication::register_record_type`.
pub fn vs_record_factory() -> (&'static str, RecordFactory) {
    ("vs", Box::new(|| Box::new(VsRecord::default())))
}

/// The `digitel` record-type factory, for
/// `IocApplication::register_record_type`.
pub fn digitel_record_factory() -> (&'static str, RecordFactory) {
    ("digitel", Box::new(|| Box::new(DigitelRecord::default())))
}

/// Dynamic device-support factory for both `asyn VacSen` and
/// `asyn DigitelPump`. Returns a fresh device support per record, its asyn link
/// parsed from `INP`. A link that fails to parse yields `None`, so the record
/// falls through to the framework's "no device support" handling.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        if ctx.dtyp == vac_sen::DTYP {
            let link = parse_asyn_link(ctx.inp).ok()?;
            Some(Box::new(VacSen::new(link)) as Box<dyn DeviceSupport>)
        } else if ctx.dtyp == digitel_pump::DTYP {
            let link = parse_asyn_link(ctx.inp).ok()?;
            Some(Box::new(DigitelPump::new(link)) as Box<dyn DeviceSupport>)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vs_factory_builds_a_vs_record() {
        let (name, factory) = vs_record_factory();
        assert_eq!(name, "vs");
        assert_eq!(factory().record_type(), "vs");
    }

    #[test]
    fn digitel_factory_builds_a_digitel_record() {
        let (name, factory) = digitel_record_factory();
        assert_eq!(name, "digitel");
        assert_eq!(factory().record_type(), "digitel");
    }

    #[test]
    fn factory_dispatches_vacsen_dtyp() {
        let f = device_support_factory();
        let ctx = DeviceSupportContext {
            dtyp: vac_sen::DTYP,
            inp: "@asyn(TV1 0)5",
            out: "",
        };
        let ds = f(&ctx).expect("VacSen device support");
        assert_eq!(ds.dtyp(), "asyn VacSen");
    }

    #[test]
    fn factory_dispatches_digitel_dtyp() {
        let f = device_support_factory();
        let ctx = DeviceSupportContext {
            dtyp: digitel_pump::DTYP,
            inp: "@asyn(DIGITEL_1 0)0",
            out: "",
        };
        let ds = f(&ctx).expect("DigitelPump device support");
        assert_eq!(ds.dtyp(), "asyn DigitelPump");
    }

    #[test]
    fn factory_rejects_unknown_dtyp() {
        let f = device_support_factory();
        let ctx = DeviceSupportContext {
            dtyp: "asyn Something",
            inp: "@asyn(TV1 0)5",
            out: "",
        };
        assert!(f(&ctx).is_none());
    }

    #[test]
    fn factory_rejects_unparseable_link() {
        let f = device_support_factory();
        let ctx = DeviceSupportContext {
            dtyp: vac_sen::DTYP,
            inp: "not-an-asyn-link",
            out: "",
        };
        assert!(f(&ctx).is_none());
    }
}
