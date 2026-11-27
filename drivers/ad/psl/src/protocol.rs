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

use std::cell::Cell;
use std::collections::BTreeSet;

use epics_libcom_rs::cfmt::CFloat;
use epics_libcom_rs::runtime::scanf::{ScanTarget, sscanf};
use epics_libcom_rs::runtime::stdlib::strtol;
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
                write!(
                    f,
                    "server version '{s}' is older than {}",
                    CFloat(MIN_SERVER_VERSION).g()
                )
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
    // C's `serverVersion_` is a `float`.
    let version = Cell::new(0f32);
    if sscanf(
        &reply.as_bytes()[start + VERSION_PREFIX.len()..],
        "%f",
        &[&version],
    ) != 1
    {
        return Err(ProtocolError::BadVersion(reply.to_string()));
    }
    let version = f64::from(version.get());
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

/// C `sscanf(reply, format, ...) == N` for a `format` of `N` `%d`s, written
/// `%lld` so that a value C would wrap into its `int` is `None`.
fn scan_ints<const N: usize>(reply: &[u8], format: &str) -> Option<[i32; N]> {
    let wide: [Cell<i64>; N] = std::array::from_fn(|_| Cell::new(0));
    let out: Vec<&dyn ScanTarget> = wide.iter().map(|v| v as &dyn ScanTarget).collect();
    if sscanf(reply, format, &out) != N as i32 {
        return None;
    }
    let mut values = [0; N];
    for (value, wide) in values.iter_mut().zip(&wide) {
        *value = i32::try_from(wide.get()).ok()?;
    }
    Some(values)
}

/// A `(x,y)` reply: `GetMaximumSize`, `GetSize`, `GetBinning`
/// (C `sscanf(pStart, "(%d,%d)", ...)`).
pub fn parse_pair(reply: &str) -> Option<[i32; 2]> {
    scan_ints(reply.as_bytes(), "(%lld,%lld)")
}

/// The `GetSubArea` reply (C `sscanf(pStart, "(%d,%d,%d,%d)", ...)`) as
/// `[min_x, min_y, size_x, size_y]`, a size being C's `right - minX + 1`.
/// A size past `int`, which C's arithmetic overflows, refuses the reply.
pub fn parse_sub_area(reply: &str) -> Option<[i32; 4]> {
    let [min_x, min_y, right, bottom] = scan_ints(reply.as_bytes(), "(%lld,%lld,%lld,%lld)")?;
    let size = |min: i32, edge: i32| i32::try_from(i64::from(edge) - i64::from(min) + 1).ok();
    Some([min_x, min_y, size(min_x, right)?, size(min_y, bottom)?])
}

/// A one-number reply: `GetFliplr`, `GetFlipud`, `GetRecordNumber`
/// (C `sscanf(pStart, "%d", ...)`).
pub fn parse_int(reply: &str) -> Option<i32> {
    scan_ints(reply.as_bytes(), "%lld").map(|[v]| v)
}

/// The `GetCamNum` reply (C `nCameras_ = atoi(fromServer_)`). A count past
/// `int`, which `atoi` wraps, reads as no count, 0, as junk does.
pub fn parse_cam_num(reply: &str) -> i32 {
    i32::try_from(strtol(reply, 10).0).unwrap_or(0)
}

/// C `sscanf(reply, format, &value)` for a `format` with one `%lf`.
fn scan_f64(reply: &str, format: &str) -> Option<f64> {
    let value = Cell::new(0f64);
    (sscanf(reply.as_bytes(), format, &[&value]) == 1).then(|| value.get())
}

/// The `GetChipGain` reply (C `sscanf(pStart, "%lf", &gain)`).
pub fn parse_gain(reply: &str) -> Option<f64> {
    scan_f64(reply, "%lf")
}

/// The `GetExposure` reply, in seconds (C `sscanf(pStart, "(%lf", &exposure)`,
/// then C scales `Millisec` and `Microsec`).
pub fn parse_exposure(reply: &str) -> Option<f64> {
    let value = scan_f64(reply, "(%lf")?;
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

    /// Bytes the announced geometry actually occupies, `None` past `usize`.
    pub fn expected_bytes(&self) -> Option<usize> {
        let per_pixel = self.data_type.element_size()
            * if self.color_mode == NDColorMode::RGB1 {
                3
            } else {
                1
            };
        self.width.checked_mul(self.height)?.checked_mul(per_pixel)
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

    // C `sscanf(fromServer_ + prefixLen, "%d;%d;%d;%n", ...)`, which left
    // the header length unset when the scan stopped short.
    let fields: [Cell<i64>; 3] = std::array::from_fn(|_| Cell::new(0));
    let text_len = Cell::new(-1i32);
    let out: [&dyn ScanTarget; 4] = [&fields[0], &fields[1], &fields[2], &text_len];
    let body = &reply[prefix_len..];
    if sscanf(body, "%lld;%lld;%lld;%n", &out) != 3 {
        return Err(ProtocolError::BadImageHeader);
    }
    let text_len = usize::try_from(text_len.get()).map_err(|_| ProtocolError::BadImageHeader)?;
    let [width, height, data_len] = fields.map(|v| {
        i32::try_from(v.get())
            .ok()
            .and_then(|v| usize::try_from(v).ok())
    });
    let (Some(width), Some(height), Some(data_len)) = (width, height, data_len) else {
        return Err(ProtocolError::BadImageHeader);
    };

    let header = ImageHeader {
        data_type,
        color_mode,
        width,
        height,
        data_len,
        header_len: prefix_len + text_len,
    };
    if header.width == 0 || header.height == 0 {
        return Err(ProtocolError::BadImageHeader);
    }
    let expected = header
        .expected_bytes()
        .ok_or(ProtocolError::BadImageHeader)?;
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

/// `None` where C's `minX+sizeX-1` overflows its `int`.
pub fn set_sub_area(min_x: i32, min_y: i32, size_x: i32, size_y: i32) -> Option<String> {
    let edge = |min: i32, size: i32| i32::try_from(i64::from(min) + i64::from(size) - 1).ok();
    let (right, bottom) = (edge(min_x, size_x)?, edge(min_y, size_y)?);
    Some(format!("SetSubArea;({min_x},{min_y},{right},{bottom})"))
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
    format!("SetFrameTime;{:.6}", CFloat(seconds * 1000.0))
}

pub fn set_chip_gain(gain: f64) -> String {
    format!("SetChipGain;{gain:.6}", gain = CFloat(gain))
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
        assert_eq!(parse_version("PSLViewer-4.3").unwrap(), f64::from(4.3f32));
        assert_eq!(
            parse_version("PSLViewer-5.10 ready").unwrap(),
            f64::from(5.1f32)
        );
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

    /// PSL.cpp scans each reply with its own `sscanf` format, and the
    /// `GetCamNum` reply with `atoi`, so a number ends where C's conversion
    /// stops. A value C wraps into its `int` is refused.
    #[test]
    fn replies_are_scanned_as_c_sscanf() {
        let mut wrong = Vec::new();
        let pairs = [
            ("(4,4)", Some([4, 4])),
            ("( 4, 4)", Some([4, 4])),
            ("(+4,-4)", Some([4, -4])),
            ("(-1,2)", Some([-1, 2])),
            ("(4.5,4)", None),
            ("(1e3,2)", None),
            ("(0x10,2)", None),
            ("4,4", None),
            ("(4,", None),
            ("(3000000000,2)", None),
        ];
        for (reply, want) in pairs {
            if parse_pair(reply) != want {
                wrong.push(format!("pair {reply:?}: {:?}", parse_pair(reply)));
            }
        }
        let areas = [
            ("(0,0,3999,2670)", Some([0, 0, 4000, 2671])),
            ("(0, 0, 9, 9) junk", Some([0, 0, 10, 10])),
            ("(5,0,4,0)", Some([5, 0, 0, 1])),
            ("(0,0,3999)", None),
            // C's `right - minX + 1` overflows `int` on both.
            ("(-2147483648,0,2147483647,0)", None),
            ("(0,0,0,2147483647)", None),
        ];
        for (reply, want) in areas {
            if parse_sub_area(reply) != want {
                wrong.push(format!("area {reply:?}: {:?}", parse_sub_area(reply)));
            }
        }
        let ints = [
            ("1", Some(1)),
            (" 7", Some(7)),
            ("-1x", Some(-1)),
            ("1.9", Some(1)),
            ("x5", None),
            ("True", None),
            ("", None),
        ];
        for (reply, want) in ints {
            if parse_int(reply) != want {
                wrong.push(format!("int {reply:?}: {:?}", parse_int(reply)));
            }
        }
        let cams = [
            ("2", 2),
            (" 2", 2),
            ("2.5", 2),
            ("+3", 3),
            ("[2]", 0),
            ("abc", 0),
            ("4294967298", 0),
        ];
        for (reply, want) in cams {
            if parse_cam_num(reply) != want {
                wrong.push(format!("cams {reply:?}: {}", parse_cam_num(reply)));
            }
        }
        let headers: [(&[u8], _); 8] = [
            (b"I;16; 4;3;24;", Some((4, 3, 24, 13))),
            (b"I;16;+4;3;24;", Some((4, 3, 24, 13))),
            (b"L;2;\t2; 4;", Some((2, 2, 4, 10))),
            (b"I;16;4;3;24;\x01", Some((4, 3, 24, 12))),
            (b"I;16;4;3;24", None),
            (b"I;16;4x;3;24;", None),
            (b"I;16;-4;3;24;", None),
            (b"I;16;4294967300;3;24;", None),
        ];
        for (reply, want) in headers {
            let got = parse_image_header(reply)
                .ok()
                .map(|h| (h.width, h.height, h.data_len, h.header_len));
            if got != want {
                wrong.push(format!(
                    "header {:?}: {got:?}",
                    String::from_utf8_lossy(reply)
                ));
            }
        }
        assert!(wrong.is_empty(), "{wrong:#?}");
    }

    #[test]
    fn parse_exposure_scales_the_unit() {
        assert_eq!(parse_exposure("(5000000, 'Microsec')").unwrap(), 5.0);
        assert_eq!(parse_exposure("(250, 'Millisec')").unwrap(), 0.25);
        assert_eq!(parse_exposure("(2, 'Sec')").unwrap(), 2.0);
    }

    /// glibc's `%f` cannot push back the `x` of `0x`, an exponent marker
    /// with no digit after it, or the `i` of `infi`: the scan fails there.
    #[test]
    fn a_float_cut_short_fails_the_scan() {
        assert_eq!(parse_gain("0x"), None);
        assert_eq!(parse_gain("1.0E"), None);
        assert_eq!(parse_gain("infi"), None);
        assert_eq!(parse_gain("inf"), Some(f64::INFINITY));
        assert_eq!(parse_gain("0x1p3"), Some(8.0));
        assert_eq!(parse_exposure("(0x, 'Sec')"), None);
        assert!(matches!(
            parse_version("PSLViewer-5.0E"),
            Err(ProtocolError::BadVersion(_))
        ));
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
        assert_eq!(h.expected_bytes(), Some(12));
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

    /// 65536 x 65536 16-bit pixels are 2^33 bytes: a size mismatch with any
    /// announced count on a 64-bit host, a bad header where `usize` is 32
    /// bits and the product does not fit.
    #[test]
    fn parse_image_header_handles_a_geometry_past_u32_max() {
        let reply = b"I;16;65536;65536;8;";
        let h = ImageHeader {
            data_type: NDDataType::UInt16,
            color_mode: NDColorMode::Mono,
            width: 65536,
            height: 65536,
            data_len: 8,
            header_len: 19,
        };
        #[cfg(target_pointer_width = "64")]
        {
            assert_eq!(h.expected_bytes(), Some(1 << 33));
            assert_eq!(
                parse_image_header(reply),
                Err(ProtocolError::ImageSizeMismatch {
                    announced: 8,
                    expected: 1 << 33,
                })
            );
        }
        #[cfg(not(target_pointer_width = "64"))]
        {
            assert_eq!(h.expected_bytes(), None);
            assert_eq!(
                parse_image_header(reply),
                Err(ProtocolError::BadImageHeader)
            );
        }
    }

    // --- commands ---------------------------------------------------------

    #[test]
    fn command_builders_match_c() {
        assert_eq!(open("multiconf"), "Open;multiconf");
        assert_eq!(select("cam1"), "Select;cam1");
        assert_eq!(set_binning(2, 4), "SetBinning;(2,4)");
        assert_eq!(
            set_sub_area(10, 20, 100, 200).as_deref(),
            Some("SetSubArea;(10,20,109,219)")
        );
        // C's `minX+sizeX-1` overflows `int` on the first two.
        assert_eq!(set_sub_area(2147483647, 0, 2, 1), None);
        assert_eq!(set_sub_area(0, -2147483648, 1, -1), None);
        assert_eq!(
            set_sub_area(1, 0, 2147483647, 1).as_deref(),
            Some("SetSubArea;(1,0,2147483647,0)")
        );
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
        assert_eq!(set_chip_gain(f64::NAN), "SetChipGain;nan");
        assert_eq!(set_frame_time(-f64::NAN), "SetFrameTime;-nan");
    }
}
