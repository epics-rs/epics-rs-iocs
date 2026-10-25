fn main() {
    if let Ok(lib) = pkg_config::probe_library("libuldaq") {
        for path in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", path.display());
        }
        // pkg-config leaves the system dirs out, so a path here may be one
        // the loader does not search. A link arg reaches only this package's
        // binaries; every other binary that links libuldaq sets its rpath
        // from DEP_ULDAQ_LIBDIR in its own build script.
        if !lib.link_paths.is_empty() {
            let dirs = std::env::join_paths(&lib.link_paths).unwrap();
            let dirs = dirs.to_str().unwrap();
            println!("cargo:libdir={dirs}");
            println!("cargo:rustc-link-arg=-Wl,-rpath,{dirs}");
        }
    } else {
        // Fallback: assume libuldaq is installed in standard paths
        println!("cargo:rustc-link-lib=uldaq");
    }
}
