//! D435i RealSense areaDetector IOC binary.
//!
//! Usage:
//!   cargo run --bin d435i_ioc --features ioc -- ioc/st.cmd

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::error::CaResult;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let mut ioc = AdIoc::new();
    d435i::ioc_support::register(&mut ioc);
    ioc.run_from_args().await
}
