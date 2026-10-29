//! Hands the librealsense2 libdir to the binaries that link this crate.
//! realsense-sys finds it through pkg-config but declares no `links`, so its
//! dependents get no metadata; this crate probes the same package and exports
//! the dir as DEP_REALSENSE2_LIBDIR. A link arg reaches only this package's
//! own binaries, so every other binary sets its rpath in its own build script.
fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    // realsense-sys emits the link flags.
    let lib = pkg_config::Config::new()
        .cargo_metadata(false)
        .probe("realsense2")
        .expect("pkg-config failed to find realsense2 package");
    // pkg-config leaves the system dirs out, so a path here may be one the
    // loader does not search.
    if !lib.link_paths.is_empty() {
        let dirs = std::env::join_paths(&lib.link_paths).unwrap();
        let dirs = dirs.to_str().unwrap();
        println!("cargo::metadata=libdir={dirs}");
        println!("cargo::rustc-link-arg=-Wl,-rpath,{dirs}");
    }
}
