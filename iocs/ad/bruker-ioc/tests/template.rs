//! db/BIS.template ships as ADBruker's, its "SRFM" spelling of the one file
//! format included.

#[test]
fn file_format_keeps_the_upstream_srfm_choice() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/db/BIS.template");
    let template = std::fs::read_to_string(path).unwrap();
    // FileFormat and FileFormat_RBV (ADBruker BIS.template:49, :57).
    let srfm = template
        .lines()
        .filter(|l| l.contains(r#"field(ZRST, "SRFM")"#))
        .count();
    assert_eq!(srfm, 2, "{template}");
}
