//! Compiles the C++ shim and links the pylon SDK found under `PYLON_ROOT`
//! (default `/opt/pylon`).
//!
//! The SDK is a vendor download, not a package, so it is absent on a stock
//! machine. Like `uldaq-sys`, this script degrades instead of failing: with
//! no SDK it emits the link libraries and stops, which leaves `cargo
//! check`/`clippy` working (the Rust side of the shim is `extern "C"`
//! declarations and needs no headers) and defers the failure to the link of
//! something that actually calls them.
fn main() {
    println!("cargo::rerun-if-env-changed=PYLON_ROOT");
    println!("cargo::rerun-if-changed=shim/pylon_shim.cpp");
    println!("cargo::rerun-if-changed=shim/pylon_shim.h");

    let root = std::env::var("PYLON_ROOT").unwrap_or_else(|_| "/opt/pylon".to_string());
    let include = format!("{root}/include");
    let lib = format!("{root}/lib");
    if !std::path::Path::new(&include)
        .join("pylon/PylonIncludes.h")
        .exists()
    {
        println!(
            "cargo::warning=pylon SDK not found under {root} (set PYLON_ROOT); \
             ad-pylon will not link"
        );
        return;
    }

    cc::Build::new()
        .cpp(true)
        .std("c++14")
        .include(&include)
        .include("shim")
        .flag("-Wno-unknown-pragmas")
        .flag("-Wno-unused-parameter")
        .flag("-Wno-deprecated-declarations")
        // Raised by the SDK headers themselves.
        .flag("-Wno-overloaded-virtual")
        .flag("-Wno-deprecated-copy")
        .file("shim/pylon_shim.cpp")
        .compile("pylonshim");

    println!("cargo::rustc-link-search=native={lib}");
    // The SDK names its GenApi libraries after their version; take whichever
    // pair this installation ships.
    let mut genapi = Vec::new();
    for entry in std::fs::read_dir(&lib).expect("pylon lib directory") {
        let name = entry.unwrap().file_name().into_string().unwrap();
        if let Some(stem) = name.strip_prefix("lib").and_then(|n| n.strip_suffix(".so"))
            && (stem.starts_with("GenApi_") || stem.starts_with("GCBase_"))
        {
            genapi.push(stem.to_string());
        }
    }
    for l in ["pylonbase", "pylonutility"]
        .into_iter()
        .map(String::from)
        .chain(genapi)
    {
        println!("cargo::rustc-link-lib=dylib={l}");
    }
    println!("cargo::rustc-link-arg=-Wl,-rpath,{lib}");
    println!("cargo::rustc-link-arg=-Wl,-E");
}
