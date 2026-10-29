//! ADTimePix3's trace lines, as `ADTimePixLog.h`'s `ERR`/`WARN`/`LOG` and
//! `serval_http.cpp`'s `logHttpFailure`/`logHttpWarning` print them. The
//! function part is GCC's `__PRETTY_FUNCTION__`, which C builds by default.

use crate::http::HttpError;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::trace::TraceMask;

const LOG_HTTP_FAILURE: &str = "void ADTimePix::logHttpFailure(const char*, const char*, \
                                const std::string&, long int, const std::string&)";
const LOG_HTTP_WARNING: &str = "void ADTimePix::logHttpWarning(const char*, const char*, \
                                const std::string&, long int, const std::string&)";

pub const REFRESH_PIXEL_CONFIG: &str = "asynStatus ADTimePix::refreshPixelConfigFromServal()";
pub const WRITE_BPC_FILE: &str =
    "asynStatus ADTimePix::writeBPCfile(const std::vector<unsigned char>&)";
pub const EXPORT_MASKED_PELS_JSON: &str =
    "void ADTimePix::exportMaskedPelsJsonFromBpcBuffer(const std::vector<unsigned char>&)";
pub const READ_BPC_FILE: &str = "asynStatus ADTimePix::readBPCfile(std::vector<unsigned char>&)";
pub const EXPECTED_BPC_SIZE: &str = "asynStatus ADTimePix::expectedBPCSize(std::size_t&)";
pub const UPDATE_DETECTOR_HEALTH_FROM_JSON: &str =
    "void ADTimePix::updateDetectorHealthFromJson(const json&)";
pub const UPDATE_DETECTOR_FAMILY: &str =
    "void ADTimePix::updateDetectorFamily(int, const std::string&, const std::string&)";
pub const TIME_PIX_CALLBACK: &str = "void ADTimePix::timePixCallback()";
pub const ACQUIRE_START: &str = "asynStatus ADTimePix::acquireStart()";
pub const ROWS_COLS: &str = "asynStatus ADTimePix::rowsCols(int*, int*, int*, int*, int*)";
pub const PEL_INDEX: &str = "int ADTimePix::pelIndex(int, int)";
pub const PARSE_TCP_PATH: &str =
    "bool ADTimePix::parseTcpPath(const std::string&, std::string&, int&)";
pub const ENSURE_PREVIEW_TCP_PORTS_FREE: &str =
    "asynStatus ADTimePix::ensurePreviewTcpPortsFree(bool)";
pub const STOP_ACQUISITION: &str =
    "asynStatus ADTimePix::stopAcquisition(bool, const std::string&)";
pub const PROCESS_PREVIEW_JSONIMAGE_LINE: &str = "bool ADTimePix::processPreviewJsonimageLine(\
    const ADTimePix::PreviewJsonimageStream&, char*, char*, size_t)";
pub const EMIT_PREVIEW_THRESHOLD_DIFF: &str =
    "void ADTimePix::emitPreviewThresholdDiff(int, int, int, int, const char*, int&)";
pub const PROCESS_IMG_DATA_LINE: &str = "bool ADTimePix::processImgDataLine(char*, char*, size_t)";
pub const CONFIGURE_IMG_ROLLING_WINDOW: &str = "bool ADTimePix::configureImgRollingWindow(size_t)";
pub const PROCESS_IMG_FRAME: &str = "void ADTimePix::processImgFrame(const ImageData&)";
pub const CONFIGURE_PRV_HST_ROLLING_WINDOW: &str =
    "bool ADTimePix::configurePrvHstRollingWindow(size_t)";
pub const PROCESS_PRV_HST_FRAME: &str = "void ADTimePix::processPrvHstFrame(const HistogramData&)";
pub const VALIDATE_DESTINATION: &str =
    "asynStatus ADTimePix::validateDestination(const std::string&)";
pub const CONFIGURE_RAW_CHANNEL: &str = "asynStatus ADTimePix::configureRawChannel(int, json&)";
pub const CONFIGURE_HISTOGRAM_CHANNEL: &str =
    "asynStatus ADTimePix::configureHistogramChannel(json&)";
pub const CONFIGURE_IMAGE_CHANNEL: &str =
    "asynStatus ADTimePix::configureImageChannel(const std::string&, json&)";
pub const CONFIGURE_PREVIEW_SETTINGS: &str =
    "asynStatus ADTimePix::configurePreviewSettings(json&)";
pub const FILE_WRITER: &str = "asynStatus ADTimePix::fileWriter()";
pub const GET_SERVER: &str = "asynStatus ADTimePix::getServer()";
pub const INIT_ACQUISITION: &str = "asynStatus ADTimePix::initAcquisition()";
pub const WRITE_DAC: &str =
    "asynStatus ADTimePix::writeDac(int, int, const std::string&, int, int)";
pub const UPLOAD_DACS: &str = "asynStatus ADTimePix::uploadDACS()";
pub const UPLOAD_BPC: &str = "asynStatus ADTimePix::uploadBPC()";
pub const RESOLVE_CALIBRATION_FILE: &str = "asynStatus ADTimePix::resolveCalibrationFile(\
    const std::string&, const std::string&, ADTimePix3Calibration::PathAccess, std::string&)";
pub const CHECK_IMG_PATH: &str = "asynStatus ADTimePix::checkImgPath()";
pub const CHECK_PRV_IMG_PATH: &str = "asynStatus ADTimePix::checkPrvImgPath()";
pub const CHECK_PRV_IMG1_PATH: &str = "asynStatus ADTimePix::checkPrvImg1Path()";
pub const CHECK_BPC_PATH: &str = "asynStatus ADTimePix::checkBPCPath()";
pub const CHECK_DACS_PATH: &str = "asynStatus ADTimePix::checkDACSPath()";
pub const GET_MEASUREMENT_CONFIG: &str = "asynStatus ADTimePix::getMeasurementConfig()";
pub const SEND_MEASUREMENT_CONFIG: &str = "asynStatus ADTimePix::sendMeasurementConfig()";
pub const REFRESH_ON_RECONNECT: &str = "asynStatus ADTimePix::refreshOnReconnect()";
pub const CHECK_CONNECTION: &str = "asynStatus ADTimePix::checkConnection(bool, bool)";
pub const GET_DASHBOARD: &str = "asynStatus ADTimePix::getDashboard()";
pub const GET_DETECTOR: &str = "asynStatus ADTimePix::getDetector(bool)";
pub const INITIAL_SERVER_CHECK_CONNECTION: &str =
    "asynStatus ADTimePix::initialServerCheckConnection()";
pub const CONSTRUCTOR: &str = "ADTimePix::ADTimePix(const char*, const char*, int, size_t, int, \
     int, int, const char*, const char*)";
pub const WRITE_INT32: &str = "virtual asynStatus ADTimePix::writeInt32(asynUser*, epicsInt32)";
pub const BPC_IMAGE_BYTE_OFFSETS: &str =
    "asynStatus ADTimePix::bpcImageByteOffsets(std::vector<long unsigned int>&)";
pub const READ_INT32_ARRAY: &str =
    "virtual asynStatus ADTimePix::readInt32Array(asynUser*, epicsInt32*, size_t, size_t*)";
pub const WRITE_FLOAT64: &str =
    "virtual asynStatus ADTimePix::writeFloat64(asynUser*, epicsFloat64)";
pub const WRITE_OCTET: &str =
    "virtual asynStatus ADTimePix::writeOctet(asynUser*, const char*, size_t, size_t*)";

/// `epicsSnprintf` into a `char[size]`: at most `size - 1` bytes, cut at a
/// character boundary. C cuts mid-character and leaves a broken UTF-8 tail;
/// that is not copied.
pub fn snprintf(size: usize, mut text: String) -> String {
    let mut end = text.len().min(size.saturating_sub(1));
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    text.truncate(end);
    text
}

/// One trace line on the port thread. Every C macro (ADTimePixLog.h:41-71)
/// ends its format in `\n`, so the terminator is added here, never by a site.
pub fn print(base: &PortDriverBase, mask: TraceMask, line: &str) {
    base.trace_print(mask, format_args!("{line}\n"));
}

/// `ERR`/`ERR_ARGS`, at `ASYN_TRACE_ERROR`.
pub fn err(func: &str, msg: &str) -> String {
    format!("ERROR | ADTimePix::{func}: {msg}")
}

/// `WARN`/`WARN_ARGS`, at `ASYN_TRACE_WARNING`.
pub fn warn(func: &str, msg: &str) -> String {
    format!("WARN | ADTimePix::{func}: {msg}")
}

/// `LOG`/`LOG_ARGS`, at `ASYN_TRACEIO_DRIVER`.
pub fn log(func: &str, msg: &str) -> String {
    format!("ADTimePix::{func}: {msg}")
}

/// C `trimHttpBodyForLog`: control characters become spaces, the body is cut
/// at 512 bytes and marked with `...`.
pub fn trim_body(body: &str) -> String {
    const MAX: usize = 512;
    let bytes = body.as_bytes();
    let kept: Vec<u8> = bytes[..bytes.len().min(MAX)]
        .iter()
        .map(|&c| if c < 32 || c == 127 { b' ' } else { c })
        .collect();
    let mut out = String::from_utf8_lossy(&kept).into_owned();
    if bytes.len() > MAX {
        out.push_str("...");
    }
    out
}

/// `"{context}: {method} {url} HTTP {status}: {body}"`; `None` when the
/// request got a 200 and only its body failed to parse, where C prints no
/// HTTP line. A request that never reached Serval is cpr's status 0.
fn http_line(context: &str, method: &str, url: &str, e: &HttpError) -> Option<String> {
    let code = match e {
        HttpError::Status { code, .. } => i32::from(*code),
        HttpError::Transport(_) => 0,
        HttpError::Parse(_) => return None,
    };
    Some(status_line(context, method, url, code, e.body()))
}

fn status_line(context: &str, method: &str, url: &str, code: i32, body: &str) -> String {
    format!("{context}: {method} {url} HTTP {code}: {}", trim_body(body))
}

pub fn http_failure(context: &str, method: &str, url: &str, e: &HttpError) -> Option<String> {
    http_line(context, method, url, e).map(|m| err(LOG_HTTP_FAILURE, &m))
}

/// C `logHttpFailure` for a reply whose status and body the caller holds.
pub fn http_failure_reply(context: &str, method: &str, url: &str, code: i32, body: &str) -> String {
    err(
        LOG_HTTP_FAILURE,
        &status_line(context, method, url, code, body),
    )
}

pub fn http_warning(context: &str, method: &str, url: &str, e: &HttpError) -> Option<String> {
    http_line(context, method, url, e).map(|m| warn(LOG_HTTP_WARNING, &m))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn http_failure_is_log_http_failure_text() {
        let e = HttpError::Status {
            url: String::new(),
            code: 404,
            body: "no\nsuch".into(),
        };
        assert_eq!(
            http_failure("rotateLayout", "GET", "http://s/x", &e).unwrap(),
            "ERROR | ADTimePix::void ADTimePix::logHttpFailure(const char*, const char*, \
             const std::string&, long int, const std::string&): rotateLayout: GET http://s/x \
             HTTP 404: no such"
        );
        assert_eq!(
            http_failure("c", "GET", "u", &HttpError::Transport("x".into())).unwrap(),
            format!("ERROR | ADTimePix::{LOG_HTTP_FAILURE}: c: GET u HTTP 0: ")
        );
        assert!(http_failure("c", "GET", "u", &HttpError::Parse("x".into())).is_none());
    }

    /// `writeBPCfile` takes the BPC bytes since 99d2321 (ADTimePix.h), and
    /// GCC prints `std::uint8_t` as `unsigned char`.
    #[test]
    fn write_bpc_file_lines_carry_the_vector_signature() {
        assert_eq!(
            err(WRITE_BPC_FILE, "WriteBPC: uploadBPC to SERVAL failed"),
            "ERROR | ADTimePix::asynStatus ADTimePix::writeBPCfile(const \
             std::vector<unsigned char>&): WriteBPC: uploadBPC to SERVAL failed"
        );
    }

    #[test]
    fn long_body_is_cut_at_512() {
        let body = "a".repeat(600);
        let t = trim_body(&body);
        assert_eq!(t.len(), 515);
        assert!(t.ends_with("a..."));
    }
}
