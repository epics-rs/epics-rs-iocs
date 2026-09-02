//! ADSimDetector areaDetector IOC binary — CA + PVA dual-protocol.
//!
//! Mirrors `iocs/simDetectorIOC/iocBoot/iocSimDetector/st_base.cmd` from
//! upstream ADSimDetector.
//!
//! Usage:
//!   cargo run -p ad-simdetector-ioc -- iocs/ad/simdetector-ioc/st.cmd

mod ioc_support;

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::error::CaResult;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let mut ioc = AdIoc::new();
    ioc_support::register(&mut ioc);
    ioc.run_from_args_with_pva().await
}
