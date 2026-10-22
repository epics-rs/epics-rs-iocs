//! Compiles `src/snl/USBCTR_SNL.st` into `$OUT_DIR`, which `snl` includes.
fn main() {
    epics_snc_rs::build::compile_dir("src/snl");
}
