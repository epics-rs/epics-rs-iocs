//! Compiles `src/snl/*.st` into `$OUT_DIR`, which the `snl` companions include.
fn main() {
    epics_snc_rs::build::compile_dir("src/snl");
}
