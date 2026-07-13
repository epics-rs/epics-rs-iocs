//! The PSLViewer command language: command builders and reply parsing.
//!
//! Pure functions over strings and byte slices — no I/O — which is what the
//! unit tests exercise. Ported from `pslApp/src/PSL.cpp`.
//!
//! A PSLViewer server that drives several sub-cameras wraps every reply in a
//! Python-style list (`['Software']`, `[(4,4)]`), which C peeled off by
//! skipping a fixed number of leading characters. The same peel lives in
//! [`quoted_field`] and [`numeric_field`], but bounded: a reply shorter than
//! the skip no longer reads past its end.

use std::collections::BTreeSet;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::ndarray::NDDataType;

use crate::types::*;

/// A set of choices offered by the server, ordered as C's `std::set` ordered
/// them — the order *is* the wire protocol, because a choice is selected by
/// its index in this set.
pub type Choices = BTreeSet<String>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolError {
    /// `GetVersion` did not answer with a `PSLViewer-<version>` string.
    BadVersion(String),
    /// The server is older than this driver supports.
    OldVersion(String),
    /// The image header did not start with a mode this driver knows.
    UnknownImageMode(String),
    /// The image header was not `<mode>;<w>;<h>;<len>;`.
    BadImageHeader,
    /// The server announced an image whose byte count does not match the
    /// geometry it announced in the same header.
    ImageSizeMismatch { announced: usize, expected: usize },
    /// A choice index that no choice answers to.
    NoSuchChoice(i32),
}

impl std::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadVersion(s) => write!(f, "unexpected version string '{s}'"),
            Self::OldVersion(s) => {
                write!(f, "server version '{s}' is older than {MIN_SERVER_VERSION}")
            }
            Self::UnknownImageMode(s) => write!(f, "unknown image mode '{s}'"),
            Self::BadImageHeader => write!(f, "malformed image header"),
            Self::ImageSizeMismatch {
                announced,
                expected,
            } => write!(
                f,
                "server announced {announced} image bytes but its geometry needs {expected}"
            ),
            Self::NoSuchChoice(i) => write!(f, "no choice with index {i}"),
        }
    }
}

impl std::error::Error for ProtocolError {}

// ── replies ──────────────────────────────────────────────────────────────

/// Check the `GetVersion` reply and return the server version.
pub fn parse_version(reply: &str) -> Result<f64, ProtocolError> {
    let start = reply
        .find(VERSION_PREFIX)
        .ok_or_else(|| ProtocolError::BadVersion(reply.to_string()))?;
    let tail = &reply[start + VERSION_PREFIX.len()..];
    let end = tail
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(tail.len());
    let version: f64 = tail[..end]
        .parse()
        .map_err(|_| ProtocolError::BadVersion(reply.to_string()))?;
    if version < MIN_SERVER_VERSION {
        return Err(ProtocolError::OldVersion(reply.to_string()));
    }
    Ok(version)
}

/// Split a choices reply into the set the driver indexes into (C
/// `getChoices`).
///
/// The delimiters are C's: `'`, `,`, `[`, `]` and space. On a multi-camera
/// server only the first bracketed group is used, as in C.
///
/// C also ran `while ((pBracket = strchr(++pBracket, '[')) != NULL);` here — a
/// loop with an empty body whose result was never read. It is not reproduced.
pub fn parse_choices(reply: &str, multi_camera: bool) -> Choices {
    let region = if multi_camera {
        // C: strtok(fromServer_, "[]") — the first run of characters that is
        // neither '[' nor ']'.
        reply
            .split(['[', ']'])
            .find(|s| !s.is_empty())
            .unwrap_or("")
    } else {
        reply
    };
    region
        .split(['\'', ',', '[', ']', ' '])
        .filter(|s| !s.is_empty())
        .map(|s| s.to_string())
        .collect()
}

/// The choice at `index`, in set order.
///
/// C walked the `std::set` with a bare loop and dereferenced the iterator even
/// when it had run off the end, so an out-of-range index was undefined
/// behaviour.
pub fn choice_from_index(choices: &Choices, index: i32) -> Result<&str, ProtocolError> {
    if index < 0 {
        return Err(ProtocolError::NoSuchChoice(index));
    }
    choices
        .iter()
        .nth(index as usize)
        .map(|s| s.as_str())
        .ok_or(ProtocolError::NoSuchChoice(index))
}

/// The index of `choice` in set order, or `None` when the server named
/// something that is not in the set.
pub fn index_of_choice(choices: &Choices, choice: &str) -> Option<i32> {
    choices.iter().position(|c| c == choice).map(|i| i as i32)
}

/// Peel the `['...']` wrapper a multi-camera server puts around a string reply
/// (C `strtok(fromServer_ + 2, "'")`).
pub fn quoted_field(reply: &str, multi_camera: bool) -> &str {
    if !multi_camera {
        return reply;
    }
    // C skipped exactly two characters ("['") and took everything up to the
    // next quote; a reply shorter than that walked off the end.
    let tail = reply.get(2..).unwrap_or("");
    match tail.find('\'') {
        Some(end) => &tail[..end],
        None => tail,
    }
}

/// Peel the `[...]` wrapper a multi-camera server puts around a numeric reply
/// (C `pStart = fromServer_ + 1`).
pub fn numeric_field(reply: &str, multi_camera: bool) -> &str {
    if !multi_camera {
        return reply;
    }
    reply.get(1..).unwrap_or("")
}

/// Pull the integers out of a `(a,b)` or `(a,b,c,d)` reply.
pub fn parse_ints(reply: &str) -> Vec<i32> {
    let mut out = Vec::new();
    let mut cur = String::new();
    for c in reply.chars() {
        if c.is_ascii_digit() || (c == '-' && cur.is_empty()) {
            cur.push(c);
        } else if !cur.is_empty() {
            if let Ok(v) = cur.parse() {
                out.push(v);
            }
            cur.clear();
        }
    }
    if let Ok(v) = cur.parse() {
        out.push(v);
    }
    out
}

/// First floating-point number in a reply.
pub fn parse_first_f64(reply: &str) -> Option<f64> {
    let bytes: Vec<char> = reply.chars().collect();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i].is_ascii_digit() || bytes[i] == '-' || bytes[i] == '+' {
            let start = i;
            i += 1;
            while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == '.') {
                i += 1;
            }
            let s: String = bytes[start..i].iter().collect();
            if let Ok(v) = s.parse::<f64>() {
                return Some(v);
            }
        } else {
            i += 1;
        }
    }
    None
}

/// The `GetExposure` reply, in seconds (C scales `Millisec` and `Microsec`).
pub fn parse_exposure(reply: &str) -> Option<f64> {
    let value = parse_first_f64(reply)?;
    if reply.contains("Millisec") {
        Some(value / 1e3)
    } else if reply.contains("Microsec") {
        Some(value / 1e6)
    } else {
        Some(value)
    }
}

/// The `GetMode` reply: how the server lays out a pixel.
pub fn parse_mode(reply: &str) -> Result<(NDDataType, NDColorMode), ProtocolError> {
    match reply.trim() {
        "L" => Ok((NDDataType::UInt8, NDColorMode::Mono)),
        "I;16" => Ok((NDDataType::UInt16, NDColorMode::Mono)),
        "I" => Ok((NDDataType::UInt32, NDColorMode::Mono)),
        "F" => Ok((NDDataType::Float32, NDColorMode::Mono)),
        // C set NDColorModeMono here while `getImage` gives an "RGB;" frame
        // NDColorModeRGB1 and three dimensions, so the ColorMode readback
        // contradicted the arrays the plugins received. The frame decides.
        "RGB" => Ok((NDDataType::UInt8, NDColorMode::RGB1)),
        other => Err(ProtocolError::UnknownImageMode(other.to_string())),
    }
}

// ── image header ─────────────────────────────────────────────────────────

/// Everything the `GetImage` reply's text header says about the frame that
/// follows it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImageHeader {
    pub data_type: NDDataType,
    pub color_mode: NDColorMode,
    /// Frame width, in pixels.
    pub width: usize,
    /// Frame height, in pixels.
    pub height: usize,
    /// Payload bytes the server says follow the header.
    pub data_len: usize,
    /// Bytes of text before the payload.
    pub header_len: usize,
}

impl ImageHeader {
    /// The NDArray dimensions this frame needs, outermost last.
    pub fn dims(&self) -> Vec<usize> {
        if self.color_mode == NDColorMode::RGB1 {
            vec![3, self.width, self.height]
        } else {
            vec![self.width, self.height]
        }
    }

    /// Bytes the announced geometry actually occupies.
    pub fn expected_bytes(&self) -> usize {
        let per_pixel = self.data_type.element_size()
            * if self.color_mode == NDColorMode::RGB1 {
                3
            } else {
                1
            };
        self.width * self.height * per_pixel
    }
}

/// Parse the text header of a `GetImage` reply: `<mode>;<width>;<height>;<bytes>;`.
///
/// C fell through with `prefixLen = 0` and `dataType = NDUInt8` when the mode
/// was not one it knew, then `sscanf`'d the mode string as if it were the
/// geometry and allocated whatever came out. An unknown mode is an error here.
///
/// C also trusted `dataLen` from the server while sizing the NDArray from the
/// geometry in the same header, so a header claiming more bytes than the
/// geometry holds overran the array. The two must agree.
pub fn parse_image_header(reply: &[u8]) -> Result<ImageHeader, ProtocolError> {
    const MODES: [(&[u8], NDDataType, NDColorMode); 5] = [
        (b"I;16;", NDDataType::UInt16, NDColorMode::Mono),
        (b"L;", NDDataType::UInt8, NDColorMode::Mono),
        (b"I;", NDDataType::UInt32, NDColorMode::Mono),
        (b"F;", NDDataType::Float32, NDColorMode::Mono),
        (b"RGB;", NDDataType::UInt8, NDColorMode::RGB1),
    ];

    let (prefix_len, data_type, color_mode) = MODES
        .iter()
        .find(|(tag, _, _)| reply.starts_with(tag))
        .map(|(tag, dt, cm)| (tag.len(), *dt, *cm))
        .ok_or_else(|| {
            let seen = &reply[..reply.len().min(8)];
            ProtocolError::UnknownImageMode(String::from_utf8_lossy(seen).to_string())
        })?;

    // Three ';'-terminated decimal fields follow the mode.
    let mut fields = [0usize; 3];
    let mut pos = prefix_len;
    for field in fields.iter_mut() {
        let start = pos;
        while pos < reply.len() && reply[pos].is_ascii_digit() {
            pos += 1;
        }
        if pos == start || pos >= reply.len() || reply[pos] != b';' {
            return Err(ProtocolError::BadImageHeader);
        }
        *field = std::str::from_utf8(&reply[start..pos])
            .ok()
            .and_then(|s| s.parse().ok())
            .ok_or(ProtocolError::BadImageHeader)?;
        pos += 1; // the ';'
    }

    let header = ImageHeader {
        data_type,
        color_mode,
        width: fields[0],
        height: fields[1],
        data_len: fields[2],
        header_len: pos,
    };
    if header.width == 0 || header.height == 0 {
        return Err(ProtocolError::BadImageHeader);
    }
    let expected = header.expected_bytes();
    if header.data_len != expected {
        return Err(ProtocolError::ImageSizeMismatch {
            announced: header.data_len,
            expected,
        });
    }
    Ok(header)
}

// ── commands ─────────────────────────────────────────────────────────────

pub fn open(camera: &str) -> String {
    format!("Open;{camera}")
}

pub fn select(camera: &str) -> String {
    format!("Select;{camera}")
}

pub fn set_binning(bin_x: i32, bin_y: i32) -> String {
    format!("SetBinning;({bin_x},{bin_y})")
}

pub fn set_sub_area(min_x: i32, min_y: i32, size_x: i32, size_y: i32) -> String {
    format!(
        "SetSubArea;({min_x},{min_y},{},{})",
        min_x + size_x - 1,
        min_y + size_y - 1
    )
}

pub fn set_fliplr(value: i32) -> String {
    format!("SetFliplr;{value}")
}

pub fn set_flipud(value: i32) -> String {
    format!("SetFlipud;{value}")
}

pub fn set_trigger_mode(mode: &str) -> String {
    format!("SetTriggerMode;{mode}")
}

pub fn set_auto_save(value: i32) -> String {
    format!("SetAutoSave;{value}")
}

pub fn set_record_format(format: &str) -> String {
    format!("SetRecordFormat;{format}")
}

pub fn set_record_number(number: i32) -> String {
    format!("SetRecordNumber;{number}")
}

pub fn set_record_path(path: &str) -> String {
    format!("SetRecordPath;{path}")
}

pub fn set_record_name(name: &str) -> String {
    format!("SetRecordName;{name}")
}

pub fn set_record_tag(tag: &str) -> String {
    format!("SetRecordTag;{tag}")
}

/// Exposure time. Under 10 ms the server wants microseconds and a unit; above
/// it, whole milliseconds (C `writeFloat64`).
pub fn set_exposure(seconds: f64) -> String {
    if seconds < 0.01 {
        format!("SetExposure;({},'Microsec')", (seconds * 1e6 + 0.5) as i64)
    } else {
        format!("SetExpoMS;{}", (seconds * 1e3 + 0.5) as i64)
    }
}

pub fn set_frame_time(seconds: f64) -> String {
    format!("SetFrameTime;{:.6}", seconds * 1000.0)
}

pub fn set_chip_gain(gain: f64) -> String {
    format!("SetChipGain;{gain:.6}")
}

pub fn set_frame_number(frames: i32) -> String {
    format!("SetFrameNumber;{frames}")
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- version ----------------------------------------------------------

    #[test]
    fn parse_version_accepts_a_supported_server() {
        assert_eq!(parse_version("PSLViewer-4.3").unwrap(), 4.3);
        assert_eq!(parse_version("PSLViewer-5.10 ready").unwrap(), 5.10);
    }

    #[test]
    fn parse_version_rejects_an_old_server() {
        assert_eq!(
            parse_version("PSLViewer-4.2"),
            Err(ProtocolError::OldVersion("PSLViewer-4.2".into()))
        );
    }

    #[test]
    fn parse_version_rejects_a_foreign_greeting() {
        assert!(matches!(
            parse_version("something else"),
            Err(ProtocolError::BadVersion(_))
        ));
    }

    // --- choices ----------------------------------------------------------

    #[test]
    fn parse_choices_splits_a_python_list() {
        let choices = parse_choices("['Software', 'Hardware', 'FreeRunning']", false);
        assert_eq!(
            choices.iter().map(String::as_str).collect::<Vec<_>>(),
            vec!["FreeRunning", "Hardware", "Software"]
        );
    }

    #[test]
    fn parse_choices_takes_the_first_group_on_a_multi_camera_server() {
        let choices = parse_choices("[['a','b'],['c','d']]", true);
        assert_eq!(
            choices.iter().map(String::as_str).collect::<Vec<_>>(),
            vec!["a", "b"]
        );
    }

    #[test]
    fn parse_choices_indexes_in_set_order() {
        // The index is the position in the sorted, de-duplicated set — that is
        // what SetTriggerMode;<choice> is selected by.
        let choices = parse_choices("['Software','Hardware','Software']", false);
        assert_eq!(choices.len(), 2);
        assert_eq!(choice_from_index(&choices, 0).unwrap(), "Hardware");
        assert_eq!(choice_from_index(&choices, 1).unwrap(), "Software");
        assert_eq!(index_of_choice(&choices, "Software"), Some(1));
        assert_eq!(index_of_choice(&choices, "None"), None);
    }

    #[test]
    fn choice_from_index_rejects_an_out_of_range_index() {
        let choices = parse_choices("['a','b']", false);
        assert_eq!(
            choice_from_index(&choices, 2),
            Err(ProtocolError::NoSuchChoice(2))
        );
        assert_eq!(
            choice_from_index(&choices, -1),
            Err(ProtocolError::NoSuchChoice(-1))
        );
    }

    #[test]
    fn parse_choices_finds_the_option_names_in_a_get_options_dump() {
        let choices = parse_choices("['Exposure', 'TriggerMode', 'Binning', 'SubArea']", false);
        assert!(choices.contains("TriggerMode"));
        assert!(choices.contains("SubArea"));
        assert!(!choices.contains("Fliplr"));
    }

    // --- field peeling ----------------------------------------------------

    #[test]
    fn quoted_field_peels_a_multi_camera_reply() {
        assert_eq!(quoted_field("['Software']", true), "Software");
        assert_eq!(quoted_field("Software", false), "Software");
    }

    #[test]
    fn quoted_field_survives_a_reply_shorter_than_the_wrapper() {
        // C read past the end of the buffer here.
        assert_eq!(quoted_field("[", true), "");
        assert_eq!(quoted_field("", true), "");
    }

    #[test]
    fn numeric_field_peels_one_bracket() {
        assert_eq!(numeric_field("[(4,4)]", true), "(4,4)]");
        assert_eq!(numeric_field("(4,4)", false), "(4,4)");
        assert_eq!(numeric_field("", true), "");
    }

    // --- values -----------------------------------------------------------

    #[test]
    fn parse_ints_reads_pairs_and_quads() {
        assert_eq!(parse_ints("(4,4)"), vec![4, 4]);
        assert_eq!(parse_ints("(0,0,3999,2670)"), vec![0, 0, 3999, 2670]);
        assert_eq!(parse_ints("(-1,2)"), vec![-1, 2]);
    }

    #[test]
    fn parse_exposure_scales_the_unit() {
        assert_eq!(parse_exposure("(5000000, 'Microsec')").unwrap(), 5.0);
        assert_eq!(parse_exposure("(250, 'Millisec')").unwrap(), 0.25);
        assert_eq!(parse_exposure("(2, 'Sec')").unwrap(), 2.0);
    }

    #[test]
    fn parse_mode_maps_every_documented_mode() {
        assert_eq!(
            parse_mode("L").unwrap(),
            (NDDataType::UInt8, NDColorMode::Mono)
        );
        assert_eq!(
            parse_mode("I;16").unwrap(),
            (NDDataType::UInt16, NDColorMode::Mono)
        );
        assert_eq!(
            parse_mode("I").unwrap(),
            (NDDataType::UInt32, NDColorMode::Mono)
        );
        assert_eq!(
            parse_mode("F").unwrap(),
            (NDDataType::Float32, NDColorMode::Mono)
        );
        assert_eq!(
            parse_mode("RGB").unwrap(),
            (NDDataType::UInt8, NDColorMode::RGB1)
        );
        assert!(matches!(
            parse_mode("Q"),
            Err(ProtocolError::UnknownImageMode(_))
        ));
    }

    // --- image header -----------------------------------------------------

    #[test]
    fn parse_image_header_reads_a_16_bit_frame() {
        let h = parse_image_header(b"I;16;4;3;24;\x00\x01").unwrap();
        assert_eq!(h.data_type, NDDataType::UInt16);
        assert_eq!(h.color_mode, NDColorMode::Mono);
        assert_eq!((h.width, h.height), (4, 3));
        assert_eq!(h.data_len, 24);
        assert_eq!(h.header_len, "I;16;4;3;24;".len());
        assert_eq!(h.dims(), vec![4, 3]);
    }

    #[test]
    fn parse_image_header_reads_an_8_bit_frame() {
        let h = parse_image_header(b"L;2;2;4;").unwrap();
        assert_eq!(h.data_type, NDDataType::UInt8);
        assert_eq!(h.data_len, 4);
        assert_eq!(h.header_len, 8);
    }

    #[test]
    fn parse_image_header_reads_a_32_bit_frame() {
        let h = parse_image_header(b"I;2;2;16;").unwrap();
        assert_eq!(h.data_type, NDDataType::UInt32);
        assert_eq!(h.header_len, 9);
    }

    #[test]
    fn parse_image_header_reads_a_float_frame() {
        let h = parse_image_header(b"F;2;2;16;").unwrap();
        assert_eq!(h.data_type, NDDataType::Float32);
    }

    #[test]
    fn parse_image_header_reads_an_rgb_frame_as_three_dimensions() {
        let h = parse_image_header(b"RGB;2;2;12;").unwrap();
        assert_eq!(h.color_mode, NDColorMode::RGB1);
        assert_eq!(h.dims(), vec![3, 2, 2]);
        assert_eq!(h.expected_bytes(), 12);
    }

    #[test]
    fn parse_image_header_rejects_an_unknown_mode() {
        // C carried on with prefixLen=0 and parsed the mode as the geometry.
        assert!(matches!(
            parse_image_header(b"Q;2;2;4;"),
            Err(ProtocolError::UnknownImageMode(_))
        ));
    }

    #[test]
    fn parse_image_header_rejects_a_truncated_header() {
        assert_eq!(
            parse_image_header(b"I;16;4;3;"),
            Err(ProtocolError::BadImageHeader)
        );
        assert_eq!(
            parse_image_header(b"I;16;;3;24;"),
            Err(ProtocolError::BadImageHeader)
        );
    }

    #[test]
    fn parse_image_header_rejects_zero_geometry() {
        assert_eq!(
            parse_image_header(b"I;16;0;3;0;"),
            Err(ProtocolError::BadImageHeader)
        );
    }

    #[test]
    fn parse_image_header_rejects_a_byte_count_the_geometry_cannot_hold() {
        // C allocated 4*3*2 bytes and then copied the 100000 bytes the server
        // announced into it.
        assert_eq!(
            parse_image_header(b"I;16;4;3;100000;"),
            Err(ProtocolError::ImageSizeMismatch {
                announced: 100000,
                expected: 24,
            })
        );
    }

    // --- commands ---------------------------------------------------------

    #[test]
    fn command_builders_match_c() {
        assert_eq!(open("multiconf"), "Open;multiconf");
        assert_eq!(select("cam1"), "Select;cam1");
        assert_eq!(set_binning(2, 4), "SetBinning;(2,4)");
        assert_eq!(set_sub_area(10, 20, 100, 200), "SetSubArea;(10,20,109,219)");
        assert_eq!(set_fliplr(1), "SetFliplr;1");
        assert_eq!(set_flipud(0), "SetFlipud;0");
        assert_eq!(set_trigger_mode("Software"), "SetTriggerMode;Software");
        assert_eq!(set_auto_save(1), "SetAutoSave;1");
        assert_eq!(set_record_format("TIFF"), "SetRecordFormat;TIFF");
        assert_eq!(set_record_number(7), "SetRecordNumber;7");
        assert_eq!(set_record_path("/data"), "SetRecordPath;/data");
        assert_eq!(set_record_name("img"), "SetRecordName;img");
        assert_eq!(set_record_tag("a comment"), "SetRecordTag;a comment");
        assert_eq!(set_frame_number(5), "SetFrameNumber;5");
    }

    #[test]
    fn set_exposure_switches_units_at_ten_milliseconds() {
        // C: < 0.01 s goes as microseconds, otherwise as whole milliseconds,
        // both rounded with +0.5.
        assert_eq!(set_exposure(0.005), "SetExposure;(5000,'Microsec')");
        assert_eq!(set_exposure(0.0000015), "SetExposure;(2,'Microsec')");
        assert_eq!(set_exposure(0.01), "SetExpoMS;10");
        assert_eq!(set_exposure(1.2345), "SetExpoMS;1235");
    }

    #[test]
    fn set_frame_time_is_in_milliseconds() {
        assert_eq!(set_frame_time(0.5), "SetFrameTime;500.000000");
    }

    #[test]
    fn set_chip_gain_matches_c_printf() {
        assert_eq!(set_chip_gain(2.5), "SetChipGain;2.500000");
    }
}
