//! Compiles `src/snl/USBCTR_SNL.st` into `$OUT_DIR`, which `snl` includes,
//! and sets this package's rpath to the libuldaq dir uldaq-sys found: a
//! build script's link args reach only its own package's binaries.
fn main() {
    epics_seq_rs::snc::build::compile_dir("src/snl");
    if let Ok(dir) = std::env::var("DEP_ULDAQ_LIBDIR") {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
