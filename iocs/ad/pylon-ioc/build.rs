//! Sets the IOC's rpath to the pylon libdir ad-pylon found, as C links the
//! IOC with `-Wl,-rpath,$(PYLON_LIB)` (iocs/pylonIOC/pylonApp/src/Makefile:19).
//! Without the SDK ad-pylon exports nothing and the link fails anyway.
fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    if let Ok(dir) = std::env::var("DEP_PYLONSHIM_LIBDIR") {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
