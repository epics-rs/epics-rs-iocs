//! D435i RealSense areaDetector IOC binary — CA + PVA dual-protocol.
//!
//! Usage:
//!   cargo run -p d435i-ioc -- iocs/d435i-ioc/st.cmd

mod ioc_support;

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::error::CaResult;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    // The acquisition task reports every librealsense failure through `log`;
    // with no backend installed those calls are no-ops and a pipeline that
    // never starts looks identical to one with nothing in front of it.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();

    let mut ioc = AdIoc::new();
    ioc_support::register(&mut ioc);
    ioc.run_from_args_with_pva().await
}
