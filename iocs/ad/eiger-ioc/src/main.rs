//! Dectris Eiger areaDetector IOC binary — CA + PVA dual-protocol.
//!
//! Usage:
//!   cargo run -p eiger-ioc -- iocs/ad/eiger-ioc/st.cmd

mod ioc_support;

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::error::CaResult;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let mut ioc = AdIoc::new();
    // busy is opt-in on epics-rs main (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC loads
    // use it, as a C IOC links the owning module's .dbd.
    ioc.register_record_type(
        "busy",
        Box::new(|| Box::new(epics_rs::busy::BusyRecord::default())),
    );
    ioc_support::register(&mut ioc);
    ioc.run_from_args_with_pva().await
}
