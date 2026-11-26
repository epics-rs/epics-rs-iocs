//! st-mvp2001.cmd hands the model-1 record the "ENC_CPR,MAX_mA" OUT parm
//! devMVP2001.cc:256-288 reads the ANO current limit from.

use std::path::Path;

use ioc_conventions::script_fields;

#[test]
fn st_mvp2001_cmd_keeps_both_default_parm_values() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let records = script_fields(&dir.join("st-mvp2001.cmd"));
    assert_eq!(records["MVP:m1"].fields["OUT"], "#C0 S0 @512,1000");
}
