//! Sets the IOC's rpath to the librealsense2 dir d435i found: a build
//! script's link args reach only its own package's binaries.
fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    if let Ok(dir) = std::env::var("DEP_REALSENSE2_LIBDIR") {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
