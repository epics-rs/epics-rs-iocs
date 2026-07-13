//! The p2util command language: line builders and response parsing.
//!
//! Pure functions over strings — no I/O — which is what the unit tests
//! exercise. Ported from `PhotonIIApp/src/PhotonII.cpp`; every command string
//! is the one the C driver puts on the wire, so p2util sees the same
//! vocabulary.

use crate::types::*;

/// Everything that can go wrong reading a p2util line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// The line is not the "wrote N bytes to <file>" message.
    NotFileWritten,
    /// The message is the right kind but carries no quoted path.
    NoQuotedPath,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotFileWritten => write!(f, "not a file-written message"),
            Self::NoQuotedPath => write!(f, "file-written message carries no quoted file name"),
        }
    }
}

impl std::error::Error for ParseError {}

/// `set --runnumber <n>`: the run number p2util stamps into the file names.
pub fn set_run_number(run_number: i32) -> String {
    format!("set --runnumber {run_number}")
}

/// `set --exposure-time <t>`.
pub fn set_exposure_time(seconds: f64) -> String {
    // C used "%f", i.e. six decimals.
    format!("set --exposure-time {seconds:.6}")
}

/// `set --dr-summation <0|1>`.
pub fn set_dr_summation(enable: i32) -> String {
    format!("set --dr-summation {enable}")
}

/// `set --frame-trigger-source <internal|external>`.
pub fn set_trigger_source(source: TriggerSource) -> String {
    format!("set --frame-trigger-source {}", source.as_str())
}

/// `set --frame-trigger-mode <step|continuous>`.
pub fn set_trigger_type(trigger_type: TriggerType) -> String {
    format!("set --frame-trigger-mode {}", trigger_type.as_str())
}

/// `set --frame-trigger-edge <rising|falling>`.
pub fn set_trigger_edge(edge: TriggerEdge) -> String {
    format!("set --frame-trigger-edge {}", edge.as_str())
}

/// `set --subframes-per-frame <n>`.
pub fn set_num_subframes(subframes: i32) -> String {
    format!("set --subframes-per-frame {subframes}")
}

/// `abort`: interrupt whatever task is underway.
pub fn abort() -> String {
    "abort".to_string()
}

/// `grab ...`: acquire `count` frames of the given type into
/// `<dst_dir>/<basename>`.
///
/// C had no `grab` line for `FrameType=ADC0`: its `switch` covered only Normal
/// and Dark, so an ADC0 acquisition re-sent whatever was still in the command
/// buffer (the preceding `set --runnumber`) and then waited for frames that
/// were never taken. p2util documents the flag (`documentation/p2util_help.txt`
/// line 27: `--adc0frame  Collect a adc0 frame`), so the branch is written out
/// here rather than left to fall through.
pub fn grab(frame_type: FrameType, dst_dir: &str, basename: &str, count: i32) -> String {
    let flag = match frame_type {
        FrameType::Normal => "",
        FrameType::Dark => "--darkframe ",
        FrameType::Adc0 => "--adc0frame ",
    };
    format!("grab {flag}--dstdir {dst_dir} --basename {basename} --count {count}")
}

/// Is this the line that announces a written frame?
pub fn is_file_written(line: &str) -> bool {
    line.contains(FILE_WRITTEN_MARKER)
}

/// Pull the file name out of a "... bytes to "<path>"" message.
///
/// C ran `strchr`/`strrchr` for the quotes and used the result without
/// checking: a message with no quote dereferenced NULL, and one with a single
/// quote made `closeQuote - openQuote - 1` wrap to `SIZE_MAX`, which `strncpy`
/// then used as a length. Both are errors here.
pub fn parse_file_written(line: &str) -> Result<&str, ParseError> {
    if !is_file_written(line) {
        return Err(ParseError::NotFileWritten);
    }
    let open = line.find('"').ok_or(ParseError::NoQuotedPath)?;
    let close = line.rfind('"').ok_or(ParseError::NoQuotedPath)?;
    if close <= open + 1 {
        return Err(ParseError::NoQuotedPath);
    }
    Ok(&line[open + 1..close])
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- command builders -------------------------------------------------

    #[test]
    fn set_run_number_matches_c() {
        assert_eq!(set_run_number(17), "set --runnumber 17");
    }

    #[test]
    fn set_exposure_time_uses_six_decimals_like_c_printf() {
        assert_eq!(set_exposure_time(0.5), "set --exposure-time 0.500000");
        assert_eq!(set_exposure_time(1.0), "set --exposure-time 1.000000");
    }

    #[test]
    fn set_dr_summation_matches_c() {
        assert_eq!(set_dr_summation(1), "set --dr-summation 1");
        assert_eq!(set_dr_summation(0), "set --dr-summation 0");
    }

    #[test]
    fn trigger_commands_match_c() {
        assert_eq!(
            set_trigger_source(TriggerSource::Internal),
            "set --frame-trigger-source internal"
        );
        assert_eq!(
            set_trigger_source(TriggerSource::External),
            "set --frame-trigger-source external"
        );
        assert_eq!(
            set_trigger_type(TriggerType::Step),
            "set --frame-trigger-mode step"
        );
        assert_eq!(
            set_trigger_type(TriggerType::Continuous),
            "set --frame-trigger-mode continuous"
        );
        assert_eq!(
            set_trigger_edge(TriggerEdge::Rising),
            "set --frame-trigger-edge rising"
        );
        assert_eq!(
            set_trigger_edge(TriggerEdge::Falling),
            "set --frame-trigger-edge falling"
        );
    }

    #[test]
    fn set_num_subframes_matches_c() {
        assert_eq!(set_num_subframes(10), "set --subframes-per-frame 10");
    }

    #[test]
    fn abort_is_bare() {
        assert_eq!(abort(), "abort");
    }

    #[test]
    fn grab_normal_matches_c() {
        assert_eq!(
            grab(FrameType::Normal, "/data", "test", 5),
            "grab --dstdir /data --basename test --count 5"
        );
    }

    #[test]
    fn grab_dark_matches_c() {
        assert_eq!(
            grab(FrameType::Dark, "/data", "test", 2),
            "grab --darkframe --dstdir /data --basename test --count 2"
        );
    }

    #[test]
    fn grab_adc0_uses_the_documented_flag() {
        assert_eq!(
            grab(FrameType::Adc0, "/data", "test", 2),
            "grab --adc0frame --dstdir /data --basename test --count 2"
        );
    }

    // --- response parsing -------------------------------------------------

    #[test]
    fn parse_file_written_extracts_the_quoted_path() {
        let line = "p2util: wrote 3145728 bytes to \"/data/test_01_0001.raw\"";
        assert_eq!(
            parse_file_written(line),
            Ok("/data/test_01_0001.raw" as &str)
        );
    }

    #[test]
    fn parse_file_written_keeps_spaces_inside_the_quotes() {
        let line = "wrote 10 bytes to \"/data/my dir/f 1.raw\"";
        assert_eq!(parse_file_written(line), Ok("/data/my dir/f 1.raw" as &str));
    }

    #[test]
    fn parse_file_written_rejects_an_unrelated_line() {
        assert_eq!(
            parse_file_written("grab: starting acquisition"),
            Err(ParseError::NotFileWritten)
        );
    }

    #[test]
    fn parse_file_written_rejects_a_message_with_no_quotes() {
        // C: strchr() returned NULL and was dereferenced.
        assert_eq!(
            parse_file_written("wrote 10 bytes to /data/f.raw"),
            Err(ParseError::NoQuotedPath)
        );
    }

    #[test]
    fn parse_file_written_rejects_a_single_quote() {
        // C: closeQuote == openQuote, so numChars underflowed to SIZE_MAX and
        // strncpy copied until it faulted.
        assert_eq!(
            parse_file_written("wrote 10 bytes to \"/data/f.raw"),
            Err(ParseError::NoQuotedPath)
        );
    }

    #[test]
    fn parse_file_written_rejects_an_empty_quoted_name() {
        assert_eq!(
            parse_file_written("wrote 0 bytes to \"\""),
            Err(ParseError::NoQuotedPath)
        );
    }

    #[test]
    fn is_file_written_matches_only_the_marker() {
        assert!(is_file_written("wrote 12 bytes to \"a\""));
        assert!(!is_file_written("bytes written: 12"));
    }
}
