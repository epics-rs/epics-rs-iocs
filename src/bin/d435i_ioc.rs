//! D435i RealSense areaDetector IOC binary.
//!
//! Usage:
//!   cargo run --bin d435i_ioc --features ioc -- ioc/st.cmd

use ad_plugins_rs::ioc::AdIoc;
use epics_base_rs::error::CaResult;

#[epics_base_rs::epics_main]
async fn main() -> CaResult<()> {
    let mut ioc = AdIoc::new();
    d435i::ioc_support::register(&mut ioc);
    ioc.run_from_args().await
}
