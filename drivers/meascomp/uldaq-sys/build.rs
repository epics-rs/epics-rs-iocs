fn main() {
    if let Ok(lib) = pkg_config::probe_library("libuldaq") {
        for path in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", path.display());
        }
    } else {
        // Fallback: assume libuldaq is installed in standard paths
        println!("cargo:rustc-link-lib=uldaq");
    }
}
