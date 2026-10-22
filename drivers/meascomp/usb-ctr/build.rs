//! Compiles every `src/snl/*.st` with snc-rs to `$OUT_DIR/<stem>.rs`, which
//! the companion calling `include_program!("<stem>")` includes.
fn main() {
    // compile_dir panics on a missing directory, and src/snl arrives with
    // USBCTR_SNL.st; until then src is watched for it.
    let snl = std::path::Path::new("src/snl");
    if snl.is_dir() {
        epics_snc_rs::build::compile_dir(snl);
    } else {
        println!("cargo::rerun-if-changed=src");
    }
}
