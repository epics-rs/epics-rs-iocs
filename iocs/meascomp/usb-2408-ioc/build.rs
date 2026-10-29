//! Sets this package's rpath to the libuldaq dir uldaq-sys found: a build
//! script's link args reach only its own package's binaries.
fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    if let Ok(dir) = std::env::var("DEP_ULDAQ_LIBDIR") {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
