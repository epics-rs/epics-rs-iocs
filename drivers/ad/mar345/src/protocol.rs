//! mar345dtb ASCII command formatting and reply matching.
//!
//! Every function here is pure so the wire format can be checked against
//! `mar345.cpp` with fixture strings and no hardware.
//!
//! Framing note: `mar345` never appends a terminator itself. The server asyn
//! port is created by `st.cmd` with `asynOctetSetInputEos("marServer",0,"\n")`
//! and `asynOctetSetOutputEos("marServer",0,"\n")`, so the port appends `\n` to
//! every command and splits replies on `\n`. The strings below carry no
//! terminator.

/// C: `"COMMAND CHANGE %d"` — `image_size` is `imageSizes[res][size]`.
pub fn cmd_change(image_size: i32) -> String {
    format!("COMMAND CHANGE {image_size}")
}

/// C: `epicsSnprintf(toServer, ..., "COMMAND SCAN %s", fullFileName)`.
pub fn cmd_scan(full_file_name: &str) -> String {
    format!("COMMAND SCAN {full_file_name}")
}

/// C: `"COMMAND ERASE"`.
pub const CMD_ERASE: &str = "COMMAND ERASE";

/// C: `"COMMAND SHUTTER OPEN"`.
pub const CMD_SHUTTER_OPEN: &str = "COMMAND SHUTTER OPEN";

/// C: `"COMMAND SHUTTER CLOSE"`.
pub const CMD_SHUTTER_CLOSE: &str = "COMMAND SHUTTER CLOSE";

/// C `changeMode` completion string (note the two spaces): the
/// `waitForCompletion("MODE_CHANGE  Ended o.k.", …)` argument.
pub const DONE_MODE_CHANGE: &str = "MODE_CHANGE  Ended o.k.";

/// C `erase` / `acquireFrame` completion string (note the four spaces): the
/// `waitForCompletion("SCAN_DATA    Ended o.k.", …)` argument.
pub const DONE_SCAN_DATA: &str = "SCAN_DATA    Ended o.k.";

/// C `waitForCompletion`: `strstr(response, doneString)` — a substring test.
pub fn response_done(response: &str, done: &str) -> bool {
    response.contains(done)
}

/// C `acquireFrame`: `epicsSnprintf(fullFileName, ..., "%s.mar%d", tempFileName,`
/// `imageSizes[res][size])` — the base name from `createFileName` with the
/// `.mar<pixels>` extension appended.
pub fn full_file_name(base: &str, image_size: i32) -> String {
    format!("{base}.mar{image_size}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn change_command() {
        assert_eq!(cmd_change(3450), "COMMAND CHANGE 3450");
        assert_eq!(cmd_change(1200), "COMMAND CHANGE 1200");
    }

    #[test]
    fn scan_command() {
        assert_eq!(
            cmd_scan("/data/test_001.mar3450"),
            "COMMAND SCAN /data/test_001.mar3450"
        );
    }

    #[test]
    fn literal_commands_match_c() {
        assert_eq!(CMD_ERASE, "COMMAND ERASE");
        assert_eq!(CMD_SHUTTER_OPEN, "COMMAND SHUTTER OPEN");
        assert_eq!(CMD_SHUTTER_CLOSE, "COMMAND SHUTTER CLOSE");
    }

    #[test]
    fn done_strings_preserve_internal_spacing() {
        // Two spaces after MODE_CHANGE, four spaces after SCAN_DATA — copied
        // byte-for-byte from mar345.cpp.
        assert_eq!(DONE_MODE_CHANGE, "MODE_CHANGE  Ended o.k.");
        assert_eq!(DONE_SCAN_DATA, "SCAN_DATA    Ended o.k.");
    }

    #[test]
    fn response_done_is_a_substring_test() {
        // mar345dtb prefixes/suffixes the status line; strstr matches anywhere.
        assert!(response_done(
            "TASK: SCAN_DATA    Ended o.k. (0 errors)",
            DONE_SCAN_DATA
        ));
        assert!(response_done("MODE_CHANGE  Ended o.k.", DONE_MODE_CHANGE));
        assert!(!response_done(
            "SCAN_DATA    Ended with errors",
            DONE_SCAN_DATA
        ));
        // Wrong internal spacing must not match.
        assert!(!response_done("SCAN_DATA Ended o.k.", DONE_SCAN_DATA));
    }

    #[test]
    fn full_file_name_appends_extension() {
        assert_eq!(
            full_file_name("/data/img_007", 3450),
            "/data/img_007.mar3450"
        );
        assert_eq!(
            full_file_name("/data/img_007", 1200),
            "/data/img_007.mar1200"
        );
    }
}
