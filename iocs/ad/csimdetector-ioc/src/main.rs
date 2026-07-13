//! ADCSimDetector areaDetector IOC binary — CA + PVA dual-protocol.
//!
//! Mirrors `iocs/ADCSimDetectorIOC/iocBoot/iocADCSimDetector/st.cmd` from
//! upstream ADCSimDetector.
//!
//! Usage:
//!   cargo run -p ad-csimdetector-ioc -- iocs/ad/csimdetector-ioc/st.cmd

mod ioc_support;

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::error::CaResult;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let mut ioc = AdIoc::new();
    ioc_support::register(&mut ioc);
    ioc.run_from_args_with_pva().await
}
