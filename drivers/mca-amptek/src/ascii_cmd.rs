//! ASCII configuration command-string helpers, ported from
//! `AsciiCmdUtilities.cpp`, restricted to what `drvAmptek.cpp` reaches
//! (`CreateResTestReadBackCmd`, `GetDP5ScaStr`, `ReplaceCmdText`,
//! `RemoveCmdByDeviceTypeDP5DxK`, `GetCmdData`/`ReplaceCmdDesc`/
//! `AppendCmdDesc`/`GetCmdDesc` are confirmed unreachable from
//! `drvAmptek.cpp`'s call chain and are not ported).
//!
//! # Fixed (not reproduced) upstream defect
//! `AsciiCmdUtilities.h:17`: `#define Whitespace "\t\n\v\f\r\0x20"` --
//! the trailing `\0x20` is parsed as the escape `\0` (NUL, terminates the
//! C string literal right there) followed by the *literal* characters
//! `x`, `2`, `0`, which never become part of the string
//! `find_first_of`/`strlen` see. So the space character (0x20) the
//! adjacent comment explicitly says should be stripped ("Chr$(32)") is
//! silently never in the whitespace set C's `RemWhitespace` uses --
//! internal spaces in a config line (e.g. `"TPEA = 12.5;"`) survive
//! whitespace removal and get sent to the device as part of the ASCII
//! command, which the DP5 firmware's `KEY=value;` grammar does not
//! tolerate. [`remove_whitespace`] here strips the *intended* set (tab,
//! LF, vertical tab, form feed, CR, space), matching the doc comment
//! rather than the broken macro expansion.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use crate::status::DppType;

/// `CAsciiCmdUtilities::RemWhitespace` (`AsciiCmdUtilities.cpp:21-39`),
/// with the space-character omission fixed -- see the module doc.
fn remove_whitespace(line: &str) -> String {
    line.bytes()
        .filter(|&b| !matches!(b, 0x09 | 0x0A | 0x0B | 0x0C | 0x0D | 0x20))
        .map(|b| b as char)
        .collect()
}

/// `CAsciiCmdUtilities::GetDP5CfgStr` (`AsciiCmdUtilities.cpp:42-94`):
/// read `path`, keep only lines inside a (case-sensitive, matched against
/// the *raw*, not-yet-uppercased line) `[DP5 Configuration File]`
/// section, uppercase each kept line, truncate at the first `;`, discard
/// lines whose first character (before whitespace removal) isn't
/// `A`-`Z`, then strip internal whitespace and concatenate.
///
/// # Restructuring vs. C
/// Reads with [`BufRead::lines`] (arbitrary line length, UTF-8) instead
/// of C's fixed 256-byte `fgets` buffer -- immaterial here since every
/// `KEY=value;` entry a real DP5 config file contains is a handful of
/// characters, far under any chunking boundary that buffer size would
/// introduce. Returns `""` (matching C's `NULL`-open behavior) rather
/// than a Result, since [`crate::driver`]'s only use of this is a
/// best-effort optional config-file load mirroring C's own
/// swallow-and-continue-on-open-failure shape.
pub fn get_dp5_cfg_str(path: &Path) -> String {
    let Ok(file) = File::open(path) else {
        return String::new();
    };
    let reader = BufReader::new(file);
    let mut cfg = String::new();
    let mut in_cfg_section = false;

    for line in reader.lines() {
        let Ok(line) = line else { break };
        if line.starts_with('[') {
            in_cfg_section = line.starts_with("[DP5 Configuration File]");
        }
        if !in_cfg_section {
            continue;
        }
        let upper = line.to_ascii_uppercase();
        let Some(semi) = upper.find(';') else {
            continue;
        };
        if semi == 0 {
            continue;
        }
        let truncated = &upper[..=semi];
        if !truncated.as_bytes()[0].is_ascii_uppercase() {
            continue;
        }
        let stripped = remove_whitespace(truncated);
        if stripped.len() > 1 {
            cfg.push_str(&stripped);
        }
    }
    cfg
}

/// `CAsciiCmdUtilities::RemoveCmd` (`AsciiCmdUtilities.cpp:351-367`):
/// delete the first `"CMD=...;"` occurrence (up to and including the
/// next `;` after the match) from `cfg`. No-op if `cmd` isn't exactly 4
/// characters, `cfg` is under 7 characters, `cmd=` isn't found, or no
/// `;` follows the match.
pub fn remove_cmd(cmd: &str, cfg: &str) -> String {
    if cfg.len() < 7 || cmd.len() != 4 {
        return cfg.to_string();
    }
    let needle = format!("{cmd}=");
    let Some(start) = cfg.find(&needle) else {
        return cfg.to_string();
    };
    let Some(semi_rel) = cfg[start..].find(';') else {
        return cfg.to_string();
    };
    let end = start + semi_rel;
    let mut out = String::with_capacity(cfg.len());
    out.push_str(&cfg[..start]);
    out.push_str(&cfg[end + 1..]);
    out
}

/// The device-type inclusion gates shared by
/// [`create_full_readback_cmd`]/[`remove_cmd_by_device_type`]
/// (`AsciiCmdUtilities.cpp:202-245,370-429`).
struct DeviceGates {
    hvse: bool,
    paps: bool,
    tecs: bool,
    volu: bool,
    con1: bool,
    con2: bool,
    inof: bool,
    boot: bool,
    gate: bool,
    papz: bool,
    sctc: bool,
}

impl DeviceGates {
    fn compute(
        pc5_present: bool,
        dpp_type: DppType,
        is_dp5_rev_dx_gains: bool,
        dpp_eco: u8,
    ) -> Self {
        // DP5 Rev Dx K,L needs PAPZ (`AsciiCmdUtilities.cpp:224-232`).
        let mut is_dp5_dx_k = false;
        let mut is_dp5_dx_l = false;
        if dpp_type == DppType::Dp5 && is_dp5_rev_dx_gains {
            if (dpp_eco & 0x0F) == 0x0A {
                is_dp5_dx_k = true;
            }
            if (dpp_eco & 0x0F) == 0x0B {
                is_dp5_dx_l = true;
            }
        }

        DeviceGates {
            hvse: (dpp_type != DppType::Px5 && pc5_present) || dpp_type == DppType::Px5,
            paps: dpp_type != DppType::Dp5G && dpp_type != DppType::Tb5,
            tecs: (dpp_type == DppType::Dp5 && pc5_present)
                || dpp_type == DppType::Px5
                || dpp_type == DppType::Dp5X,
            volu: dpp_type == DppType::Px5,
            con1: dpp_type != DppType::Dp5 && dpp_type != DppType::Dp5X,
            con2: dpp_type != DppType::Dp5 && dpp_type != DppType::Dp5X,
            inof: dpp_type != DppType::Dp5G && dpp_type != DppType::Tb5,
            boot: dpp_type == DppType::Dp5 || dpp_type == DppType::Dp5X,
            gate: dpp_type == DppType::Dp5 || dpp_type == DppType::Dp5X,
            papz: dpp_type == DppType::Px5 || is_dp5_dx_k || is_dp5_dx_l,
            sctc: dpp_type == DppType::Dp5G || dpp_type == DppType::Tb5,
        }
    }
}

/// `CAsciiCmdUtilities::RemoveCmdByDeviceType`
/// (`AsciiCmdUtilities.cpp:370-429`). `isPREL` in C is
/// `DppType == dppMCA8000D`, but this function already returns via
/// [`remove_mca8000d_cmds`] before reaching that line whenever
/// `dpp_type == Mca8000D` -- so on every path that reaches it, `isPREL`
/// is unconditionally `false`, meaning `PREL` is always removed here.
/// Preserved verbatim (not simplified away) since it documents the same
/// shape as the C source for side-by-side review.
pub fn remove_cmd_by_device_type(
    cfg_in: &str,
    pc5_present: bool,
    dpp_type: DppType,
    is_dp5_rev_dx_gains: bool,
    dpp_eco: u8,
) -> String {
    if dpp_type == DppType::Mca8000D {
        return remove_mca8000d_cmds(cfg_in);
    }

    let g = DeviceGates::compute(pc5_present, dpp_type, is_dp5_rev_dx_gains, dpp_eco);
    let is_prel = dpp_type == DppType::Mca8000D; // always false on this path

    let mut cfg = cfg_in.to_string();
    if !g.hvse {
        cfg = remove_cmd("HVSE", &cfg);
    }
    if !g.paps {
        cfg = remove_cmd("PAPS", &cfg);
    }
    if !g.tecs {
        cfg = remove_cmd("TECS", &cfg);
    }
    if !g.volu {
        cfg = remove_cmd("VOLU", &cfg);
    }
    if !g.con1 {
        cfg = remove_cmd("CON1", &cfg);
    }
    if !g.con2 {
        cfg = remove_cmd("CON2", &cfg);
    }
    if !g.inof {
        cfg = remove_cmd("INOF", &cfg);
    }
    if !g.boot {
        cfg = remove_cmd("BOOT", &cfg);
    }
    if !g.gate {
        cfg = remove_cmd("GATE", &cfg);
    }
    if !g.papz {
        cfg = remove_cmd("PAPZ", &cfg);
    }
    if !g.sctc {
        cfg = remove_cmd("SCTC", &cfg);
    }
    if !is_prel {
        cfg = remove_cmd("PREL", &cfg);
    }
    cfg
}

/// `CAsciiCmdUtilities::Remove_MCA8000D_Cmds` (`AsciiCmdUtilities.cpp:482-529`):
/// unconditional fixed removal list for `dppMCA8000D`.
pub fn remove_mca8000d_cmds(cfg_in: &str) -> String {
    const CMDS: &[&str] = &[
        "CLCK", "TPEA", "GAIF", "GAIN", "RESL", "TFLA", "TPFA", "RTDE", "AINP", "INOF", "CUSP",
        "THFA", "DACO", "DACF", "RTDS", "RTDT", "BLRM", "BLRD", "BLRU", "PRET", "HVSE", "TECS",
        "PAPZ", "PAPS", "TPMO", "SCAH", "SCAI", "SCAL", "SCAO", "SCAW", "BOOT", "CON1", "CON2",
        "VOLU",
    ];
    let mut cfg = cfg_in.to_string();
    for cmd in CMDS {
        cfg = remove_cmd(cmd, &cfg);
    }
    cfg
}

/// `CAsciiCmdUtilities::CreateFullReadBackCmdMCA8000D`
/// (`AsciiCmdUtilities.cpp:308-349`). The trailing duplicate `PDMD=?;` is
/// present in the upstream source (line 319 and line 346) and is
/// preserved here verbatim -- a harmless redundant readback query, not a
/// defect worth fixing at source (asking twice costs one extra
/// `KEY=?;` round-trip, nothing more).
pub fn create_full_readback_cmd_mca8000d() -> String {
    concat!(
        "RESC=?;", "PURE=?;", "MCAS=?;", "MCAC=?;", "SOFF=?;", "GAIA=?;", "PDMD=?;", "THSL=?;",
        "TLLD=?;", "GATE=?;", "AUO1=?;", "PRER=?;", "PREL=?;", "PREC=?;", "PRCL=?;", "PRCH=?;",
        "SCOE=?;", "SCOT=?;", "SCOG=?;", "MCSL=?;", "MCSH=?;", "MCST=?;", "AUO2=?;", "GPED=?;",
        "GPIN=?;", "GPME=?;", "GPGA=?;", "GPMC=?;", "MCAE=?;", "PDMD=?;",
    )
    .to_string()
}

/// `CAsciiCmdUtilities::CreateFullReadBackCmd` (`AsciiCmdUtilities.cpp:202-306`).
pub fn create_full_readback_cmd(
    pc5_present: bool,
    dpp_type: DppType,
    is_dp5_rev_dx_gains: bool,
    dpp_eco: u8,
) -> String {
    if dpp_type == DppType::Mca8000D {
        return create_full_readback_cmd_mca8000d();
    }

    let g = DeviceGates::compute(pc5_present, dpp_type, is_dp5_rev_dx_gains, dpp_eco);
    let mut cfg = String::from("RESC=?;CLCK=?;TPEA=?;GAIF=?;GAIN=?;RESL=?;TFLA=?;TPFA=?;PURE=?;");
    if g.sctc {
        cfg += "SCTC=?;";
    }
    cfg += "RTDE=?;MCAS=?;MCAC=?;SOFF=?;AINP=?;";
    if g.inof {
        cfg += "INOF=?;";
    }
    cfg += "GAIA=?;CUSP=?;PDMD=?;THSL=?;TLLD=?;THFA=?;DACO=?;DACF=?;RTDS=?;RTDT=?;BLRM=?;BLRD=?;BLRU=?;";
    if g.gate {
        cfg += "GATE=?;";
    }
    cfg += "AUO1=?;PRET=?;PRER=?;PREC=?;PRCL=?;PRCH=?;";
    if g.hvse {
        cfg += "HVSE=?;";
    }
    if g.tecs {
        cfg += "TECS=?;";
    }
    if g.papz {
        cfg += "PAPZ=?;";
    }
    if g.paps {
        cfg += "PAPS=?;";
    }
    cfg += "SCOE=?;SCOT=?;SCOG=?;MCSL=?;MCSH=?;MCST=?;AUO2=?;TPMO=?;GPED=?;GPIN=?;GPME=?;GPGA=?;GPMC=?;MCAE=?;";
    if g.volu {
        cfg += "VOLU=?;";
    }
    if g.con1 {
        cfg += "CON1=?;";
    }
    if g.con2 {
        cfg += "CON2=?;";
    }
    if g.boot {
        cfg += "BOOT=?;";
    }
    cfg
}

/// `CAsciiCmdUtilities::GetCmdChunk` (`AsciiCmdUtilities.cpp:558-576`):
/// the largest prefix length of `cmd`, up to 512 bytes, that ends
/// exactly at a `;` boundary -- used to split an oversized ASCII
/// config/readback string into two packets that never cut a `KEY=value;`
/// field in half.
pub fn get_cmd_chunk(cmd: &str) -> usize {
    let mut end = 0usize;
    while let Some(rel) = cmd[end..].find(';') {
        let candidate = end + rel + 1;
        if candidate > 512 {
            break;
        }
        end = candidate;
    }
    end
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn remove_whitespace_strips_the_intended_set_including_space() {
        assert_eq!(remove_whitespace("TPEA = 12.5 ;"), "TPEA=12.5;");
        assert_eq!(remove_whitespace("A\tB\nC\x0BD\x0CE\rF G"), "ABCDEFG");
    }

    #[test]
    fn get_dp5_cfg_str_extracts_only_the_dp5_section() {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        writeln!(
            file,
            "[Other Section]\nJUNK=1;\n[DP5 Configuration File]\nTPEA = 12.5 ;\n; a comment line\nGAIN=100;\n[Next Section]\nGARBAGE=1;\n"
        )
        .unwrap();
        let cfg = get_dp5_cfg_str(file.path());
        assert_eq!(cfg, "TPEA=12.5;GAIN=100;");
    }

    #[test]
    fn get_dp5_cfg_str_missing_file_returns_empty() {
        assert_eq!(get_dp5_cfg_str(Path::new("/nonexistent/path/x.cfg")), "");
    }

    #[test]
    fn get_dp5_cfg_str_section_header_match_is_case_sensitive() {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        writeln!(file, "[dp5 configuration file]\nTPEA=12.5;\n").unwrap();
        assert_eq!(get_dp5_cfg_str(file.path()), "");
    }

    #[test]
    fn remove_cmd_deletes_the_key_and_its_value() {
        assert_eq!(
            remove_cmd("TPEA", "CLCK=AUTO;TPEA=12.5;GAIN=100;"),
            "CLCK=AUTO;GAIN=100;"
        );
    }

    #[test]
    fn remove_cmd_no_op_when_not_found_or_malformed() {
        assert_eq!(
            remove_cmd("ZZZZ", "CLCK=AUTO;TPEA=12.5;"),
            "CLCK=AUTO;TPEA=12.5;"
        );
        assert_eq!(remove_cmd("TOOLONG", "CLCK=AUTO;"), "CLCK=AUTO;");
        assert_eq!(remove_cmd("TPEA", "AB"), "AB");
    }

    /// DP5 (no PC5): `isHVSE`/`isTECS`/`isPAPS` false-branch coverage,
    /// `isCON1`/`isCON2`/`isBOOT`/`isGATE`/`isINOF` true (pure DP5).
    #[test]
    fn create_full_readback_cmd_plain_dp5_no_pc5() {
        let cmd = create_full_readback_cmd(false, DppType::Dp5, false, 0);
        assert!(cmd.contains("GATE=?;"));
        assert!(cmd.contains("BOOT=?;"));
        assert!(cmd.contains("INOF=?;"));
        assert!(!cmd.contains("HVSE=?;"));
        assert!(!cmd.contains("TECS=?;"));
        assert!(!cmd.contains("CON1=?;"));
        assert!(!cmd.contains("CON2=?;"));
        assert!(!cmd.contains("VOLU=?;"));
        assert!(!cmd.contains("PAPZ=?;"));
        assert!(cmd.contains("PAPS=?;"));
    }

    /// PX5: `isHVSE`/`isTECS`/`isVOLU`/`isCON1`/`isCON2`/`isPAPZ` all true.
    #[test]
    fn create_full_readback_cmd_px5() {
        let cmd = create_full_readback_cmd(false, DppType::Px5, false, 0);
        assert!(cmd.contains("HVSE=?;"));
        assert!(cmd.contains("TECS=?;"));
        assert!(cmd.contains("VOLU=?;"));
        assert!(cmd.contains("CON1=?;"));
        assert!(cmd.contains("CON2=?;"));
        assert!(cmd.contains("PAPZ=?;"));
        assert!(!cmd.contains("GATE=?;"));
        assert!(!cmd.contains("BOOT=?;"));
    }

    /// DP5 Rev Dx K (`DPP_ECO & 0x0F == 0x0A`) with `is_dp5_rev_dx_gains`
    /// pulls in PAPZ despite not being a PX5.
    #[test]
    fn create_full_readback_cmd_dp5_rev_dx_k_gets_papz() {
        let cmd = create_full_readback_cmd(false, DppType::Dp5, true, 0x0A);
        assert!(cmd.contains("PAPZ=?;"));
        let cmd_no_gains = create_full_readback_cmd(false, DppType::Dp5, false, 0x0A);
        assert!(!cmd_no_gains.contains("PAPZ=?;"));
    }

    /// DP5G/TB5: `isSCTC` true, `isINOF`/`isPAPS` false.
    #[test]
    fn create_full_readback_cmd_dp5g_and_tb5_use_sctc_not_inof() {
        for t in [DppType::Dp5G, DppType::Tb5] {
            let cmd = create_full_readback_cmd(false, t, false, 0);
            assert!(cmd.contains("SCTC=?;"));
            assert!(!cmd.contains("INOF=?;"));
            assert!(!cmd.contains("PAPS=?;"));
        }
    }

    #[test]
    fn create_full_readback_cmd_mca8000d_delegates() {
        assert_eq!(
            create_full_readback_cmd(false, DppType::Mca8000D, false, 0),
            create_full_readback_cmd_mca8000d()
        );
        assert_eq!(
            create_full_readback_cmd_mca8000d()
                .matches("PDMD=?;")
                .count(),
            2
        );
    }

    #[test]
    fn remove_cmd_by_device_type_mca8000d_delegates_to_fixed_list() {
        let cfg = "CLCK=AUTO;THSL=1;TPEA=12.5;";
        assert_eq!(
            remove_cmd_by_device_type(cfg, false, DppType::Mca8000D, false, 0),
            remove_mca8000d_cmds(cfg)
        );
    }

    /// On every non-MCA8000D path `isPREL` is unconditionally false, so
    /// `PREL` is always stripped -- see this function's doc comment.
    #[test]
    fn remove_cmd_by_device_type_always_strips_prel_on_non_mca8000d_paths() {
        let cfg = "CLCK=AUTO;PREL=5;TPEA=12.5;";
        let out = remove_cmd_by_device_type(cfg, true, DppType::Dp5, false, 0);
        assert!(!out.contains("PREL"));
    }

    #[test]
    fn remove_cmd_by_device_type_removes_gates_not_applicable_to_dp5g() {
        let cfg = "GATE=HI;BOOT=1;CLCK=AUTO;";
        let out = remove_cmd_by_device_type(cfg, false, DppType::Dp5G, false, 0);
        assert!(!out.contains("GATE"));
        assert!(!out.contains("BOOT"));
        assert!(out.contains("CLCK=AUTO;"));
    }

    #[test]
    fn get_cmd_chunk_splits_at_the_last_semicolon_at_or_before_512() {
        let field = "TPEA=12.500000;"; // 15 bytes
        let cmd: String = field.repeat(40); // 600 bytes
        let chunk = get_cmd_chunk(&cmd);
        assert!(chunk <= 512);
        assert_eq!(&cmd.as_bytes()[chunk - 1..chunk], b";");
        // the boundary is the last ';' at or before 512 -- one field
        // short of the 512 cut confirms no field is split.
        assert_eq!(chunk % field.len(), 0);
    }

    #[test]
    fn get_cmd_chunk_short_string_returns_full_length_at_last_semicolon() {
        assert_eq!(get_cmd_chunk("CLCK=AUTO;TPEA=12.5;"), 20);
    }

    #[test]
    fn get_cmd_chunk_no_semicolon_returns_zero() {
        assert_eq!(get_cmd_chunk("NOSEMICOLONHERE"), 0);
    }
}
