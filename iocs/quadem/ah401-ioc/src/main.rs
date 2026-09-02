//! Elettra/CaenEls AH401B / AH401D picoammeter IOC — CA + PVA dual-protocol.
//!
//! Usage:
//!   cargo run -p ah401-ioc -- iocs/quadem/ah401-ioc/st.cmd

mod ioc_support;

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::error::CaResult;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let mut ioc = AdIoc::new();
    // `transform` is opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC loads
    // use it, as a C IOC links the owning module's .dbd.
    ioc.register_record_type(
        "transform",
        Box::new(|| {
            Box::new(epics_rs::base::server::records::transform::TransformRecord::default())
        }),
    );
    ioc_support::register(&mut ioc);
    ioc.run_from_args_with_pva().await
}
