//! Blocking wire transport to `driverPort_`, used from a plain OS thread (no
//! active Tokio runtime) spawned by `write_int32`/`write_float64`, and
//! directly at construction time before the port actor exists.
//!
//! A `PortDriver` method runs inside its port actor's own current-thread
//! Tokio runtime and cannot block there (`PortHandle`'s `_blocking` methods
//! panic via `block_in_place` on a current-thread runtime); `write_int32`
//! instead spawns a plain thread with no Tokio runtime for the branches that
//! need a wire round trip, uses [`WireLink`] here, and joins it. The
//! persistent acquisition worker ([`crate::task`]) needs its own async
//! transport instead (it must `.await` on `ArrayPublisher::publish`, so it
//! runs on a dedicated Tokio runtime) and does not use this module; the two
//! duplicate only the thin write/read loop, not any parsing logic (that is
//! all in [`crate::codec`]).

use std::collections::HashMap;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;

use crate::codec::{
    self, BeginOutcome, ErrorInfo, ResponseAssembler, clean_string, next_counter,
    parse_double_field, parse_integer_field, parse_values_list,
};
use crate::types::{MAX_MESSAGE_SIZE, RunMode, SOCKET_TIMEOUT, SpecsValueType, secs};

/// Reasons a wire exchange failed.
#[derive(Debug)]
pub enum WireError {
    /// `SPECSConnected_ == 0` (`SpecsAnalyser::asynWriteRead`,
    /// `specsAnalyser.cpp:2079-2082,2113-2114`) — no I/O was attempted.
    NotConnected,
    /// The low-level asyn round trip itself failed (I/O error/timeout).
    Io(AsynError),
    /// The reply frame was malformed (bad prefix / counter mismatch).
    Frame(codec::FrameError),
    /// The device returned an `ERROR` response.
    Device(ErrorInfo),
}

impl WireError {
    /// `SpecsAnalyser::sendSimpleCommand`'s `ADStatusMessage` on error
    /// (`specsAnalyser.cpp:1329-1331`, `"{code}: {message}"`).
    pub fn status_message(&self) -> String {
        match self {
            WireError::NotConnected => "Not connected to SPECS".to_string(),
            WireError::Io(e) => format!("SPECS communication error: {e}"),
            WireError::Frame(e) => format!("SPECS protocol framing error: {e:?}"),
            WireError::Device(info) => format!("{}: {}", info.code, info.message),
        }
    }
}

/// The server's answer to the `Connect` command
/// (`SpecsAnalyser::asynPortConnect`, `specsAnalyser.cpp:1861-1869`).
#[derive(Debug, Clone, Default)]
pub struct ConnectInfo {
    pub server_name: String,
    pub protocol_version: String,
}

/// One analyser parameter discovered by `setupEPICSParameters`
/// (`specsAnalyser.cpp:1420-1489`) — everything `write_int32`/construction
/// needs to `create_param` it and seed its initial value.
#[derive(Debug, Clone)]
pub struct DiscoveredParam {
    pub epics_name: String,
    pub raw_name: String,
    pub value: DiscoveredValue,
}

/// Decoded initial value (`None` when the value read failed, matching
/// upstream skipping `setIntegerParam`/etc. on that param — the created
/// param keeps its zero-initialised default).
#[derive(Debug, Clone)]
pub enum DiscoveredValue {
    Int(Option<i32>),
    Double(Option<f64>),
    String(Option<String>),
}

/// The decoded `ValidateSpectrum` reply (`SpecsAnalyser::validateSpectrum`,
/// `specsAnalyser.cpp:979-1073`).
#[derive(Debug, Clone)]
pub struct ValidateResult {
    pub start_energy: f64,
    pub end_energy: f64,
    pub step_width: f64,
    pub samples: i32,
    pub dwell_time: f64,
    pub pass_energy: f64,
    /// Index into `lens_modes`, or `-1` if the reported name was not found
    /// (`specsAnalyser.cpp:1032-1039`, an intentional sentinel, not a bug).
    pub lens_mode: i32,
    /// Index into `scan_ranges`, or `-1` if not found.
    pub scan_range: i32,
}

/// `GetSpectrumDataInfo ParameterName:"OrdinateRange"`'s reply
/// (`SpecsAnalyser::readSpectrumDataInfo`, `specsAnalyser.cpp:1764-1819`).
#[derive(Debug, Clone, Default)]
pub struct OrdinateRange {
    pub unit: String,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

/// Borrowed link to `driverPort_` plus the shared session state: the wire
/// message counter (C `SPECSMsgCounter_`) and the connected flag (C
/// `SPECSConnected_`), both shared with [`crate::task`]. `command_response`
/// locks `wire_state` for its own whole request/reply exchange (including any
/// continuation reads) so two contexts (e.g. `write_int32`'s spawned thread
/// and the acquisition worker) can never interleave frames on the shared
/// connection — the one property C's coarser `this->lock()` (held across an
/// entire multi-command sequence) actually depends on here, since the
/// EPICS param store's own consistency is already serialised by the port
/// actor, independent of this lock.
pub struct WireLink<'a> {
    pub driver_port: &'a PortHandle,
    pub addr: i32,
    pub wire_state: &'a Mutex<u32>,
    pub connected: &'a AtomicBool,
}

impl WireLink<'_> {
    fn user(&self) -> AsynUser {
        AsynUser::new(0)
            .with_addr(self.addr)
            .with_timeout(secs(SOCKET_TIMEOUT))
    }

    /// `SpecsAnalyser::commandResponse` + `SpecsAnalyser::asynWriteRead`
    /// (`specsAnalyser.cpp:1897-2118`). Advances and returns the wire
    /// message counter unconditionally before attempting the round trip,
    /// matching upstream incrementing/publishing `SPECSMsgCounter_` before
    /// `pasynOctetSyncIO->writeRead` regardless of its outcome.
    pub fn command_response(
        &mut self,
        command: &str,
    ) -> Result<HashMap<String, String>, WireError> {
        if !self.connected.load(Ordering::SeqCst) {
            return Err(WireError::NotConnected);
        }
        let mut guard = self.wire_state.lock().unwrap();
        let counter = next_counter(*guard);
        *guard = counter;
        let request = codec::format_request(counter, command);

        let result = self
            .driver_port
            .submit_blocking(
                RequestOp::OctetWriteRead {
                    data: request.into_bytes(),
                    buf_size: MAX_MESSAGE_SIZE,
                    flush: true,
                },
                self.user(),
            )
            .map_err(WireError::Io)?;
        let raw = String::from_utf8_lossy(&result.data.unwrap_or_default()).into_owned();
        let payload = codec::strip_response_frame(&raw, counter).map_err(WireError::Frame)?;

        let mut assembler = ResponseAssembler::new();
        match assembler.begin(payload) {
            BeginOutcome::Ok => {}
            BeginOutcome::NeedsMore => loop {
                let more = self
                    .driver_port
                    .submit_blocking(
                        RequestOp::OctetRead {
                            buf_size: MAX_MESSAGE_SIZE,
                        },
                        self.user(),
                    )
                    .map_err(WireError::Io)?;
                let chunk = String::from_utf8_lossy(&more.data.unwrap_or_default()).into_owned();
                if !assembler.continue_with(&chunk) {
                    break;
                }
            },
            BeginOutcome::Error(info) => {
                // `SpecsAnalyser::commandResponse`, `specsAnalyser.cpp:2021-2024`.
                if info.code == "3" {
                    self.connected.store(false, Ordering::SeqCst);
                }
                return Err(WireError::Device(info));
            }
        }
        Ok(assembler.finish())
    }

    /// `SpecsAnalyser::asynPortConnect` (`specsAnalyser.cpp:1831-1871`,
    /// minus the low-level `pasynOctetSyncIO::connect`/`setInputEos`/
    /// `setOutputEos`, which have no Rust equivalent here — the underlying
    /// port and its EOS are configured once via `st.cmd` iocsh commands, not
    /// per `Connect`). `SPECSConnected_`/`SPECSMsgCounter_` are set *before*
    /// issuing the `Connect` command and are not reverted if it fails,
    /// matching upstream exactly.
    pub fn connect(&mut self) -> Result<ConnectInfo, WireError> {
        *self.wire_state.lock().unwrap() = 0;
        self.connected.store(true, Ordering::SeqCst);
        let data = self.command_response("Connect")?;
        Ok(ConnectInfo {
            server_name: clean_string(
                data.get("ServerName").map(String::as_str).unwrap_or(""),
                "\"",
            ),
            protocol_version: clean_string(
                data.get("ProtocolVersion")
                    .map(String::as_str)
                    .unwrap_or(""),
                "\"",
            ),
        })
    }

    /// `SpecsAnalyser::readDeviceVisibleName` (`specsAnalyser.cpp:1345-1361`).
    pub fn read_device_visible_name(&mut self) -> Result<String, WireError> {
        let data = self.command_response("GetAnalyzerVisibleName")?;
        Ok(clean_string(
            data.get("AnalyzerVisibleName")
                .map(String::as_str)
                .unwrap_or(""),
            "\"",
        ))
    }

    /// `SpecsAnalyser::getAnalyserParameterType` (`specsAnalyser.cpp:1495-1516`).
    pub fn get_analyser_parameter_type(
        &mut self,
        name: &str,
    ) -> Result<Option<SpecsValueType>, WireError> {
        let data = self.command_response(&codec::get_info_command(name))?;
        Ok(data
            .get("ValueType")
            .and_then(|s| SpecsValueType::from_wire(s)))
    }

    /// `SpecsAnalyser::getAnalyserParameter(name, int&)`
    /// (`specsAnalyser.cpp:1518-1537`).
    pub fn get_analyser_parameter_int(&mut self, name: &str) -> Result<i32, WireError> {
        let data = self.command_response(&codec::get_value_command(name))?;
        let value = data.get("Value").map(String::as_str).unwrap_or("");
        Ok(match value {
            "\"false\"" => 0,
            "\"true\"" => 1,
            _ => parse_integer_field(value).unwrap_or(0),
        })
    }

    /// `SpecsAnalyser::getAnalyserParameter(name, double&)`
    /// (`specsAnalyser.cpp:1539-1552`).
    pub fn get_analyser_parameter_double(&mut self, name: &str) -> Result<f64, WireError> {
        let data = self.command_response(&codec::get_value_command(name))?;
        Ok(parse_double_field(data.get("Value").map(String::as_str).unwrap_or("")).unwrap_or(0.0))
    }

    /// `SpecsAnalyser::getAnalyserParameter(name, std::string&)`
    /// (`specsAnalyser.cpp:1554-1568`).
    pub fn get_analyser_parameter_string(&mut self, name: &str) -> Result<String, WireError> {
        let data = self.command_response(&codec::get_value_command(name))?;
        Ok(clean_string(
            data.get("Value").map(String::as_str).unwrap_or(""),
            "\"",
        ))
    }

    /// `SpecsAnalyser::getAnalyserParameter(name, bool&)`
    /// (`specsAnalyser.cpp:1570-1590`). Upstream inverts this mapping
    /// relative to its int/string siblings above (`"false"` → `true`,
    /// `"true"` → `false`) — an unambiguous copy-paste-class typo (the
    /// correct mapping is derivable in-file from the int overload three
    /// functions above), fixed here to match: `"false"` → `false`, `"true"`
    /// → `true`.
    pub fn get_analyser_parameter_bool(&mut self, name: &str) -> Result<bool, WireError> {
        let data = self.command_response(&codec::get_value_command(name))?;
        match data.get("Value").map(String::as_str) {
            Some("\"false\"") => Ok(false),
            Some("\"true\"") => Ok(true),
            _ => Err(WireError::Device(ErrorInfo {
                code: String::new(),
                message: format!("invalid value returned for bool parameter {name}"),
            })),
        }
    }

    /// `SpecsAnalyser::setAnalyserParameter(name, int)`
    /// (`specsAnalyser.cpp:1592-1605`).
    pub fn set_analyser_parameter_int(&mut self, name: &str, value: i32) -> Result<(), WireError> {
        self.command_response(&codec::set_value_int_command(name, value))
            .map(|_| ())
    }

    /// `SpecsAnalyser::setAnalyserParameter(name, double)`
    /// (`specsAnalyser.cpp:1607-1620`).
    pub fn set_analyser_parameter_double(
        &mut self,
        name: &str,
        value: f64,
    ) -> Result<(), WireError> {
        self.command_response(&codec::set_value_double_command(name, value))
            .map(|_| ())
    }

    /// `SpecsAnalyser::setupEPICSParameters` (`specsAnalyser.cpp:1367-1493`),
    /// minus the `createParam`/`setIntegerParam`/etc. calls, which need
    /// direct struct access the caller applies after this returns.
    pub fn setup_epics_parameters(&mut self) -> Result<Vec<DiscoveredParam>, WireError> {
        let data = self.command_response("GetAllAnalyzerParameterNames")?;
        let names = clean_string(
            data.get("ParameterNames").map(String::as_str).unwrap_or(""),
            "[]",
        );
        let mut discovered = Vec::new();
        for (epics_name, raw_name) in codec::parse_parameter_names(&names) {
            let Ok(Some(value_type)) = self.get_analyser_parameter_type(&raw_name) else {
                continue;
            };
            let value = match value_type {
                SpecsValueType::Integer => {
                    DiscoveredValue::Int(self.get_analyser_parameter_int(&raw_name).ok())
                }
                SpecsValueType::Double => {
                    DiscoveredValue::Double(self.get_analyser_parameter_double(&raw_name).ok())
                }
                SpecsValueType::String => {
                    DiscoveredValue::String(self.get_analyser_parameter_string(&raw_name).ok())
                }
                SpecsValueType::Bool => DiscoveredValue::Int(
                    self.get_analyser_parameter_bool(&raw_name)
                        .ok()
                        .map(i32::from),
                ),
            };
            discovered.push(DiscoveredParam {
                epics_name,
                raw_name,
                value,
            });
        }
        Ok(discovered)
    }

    /// `SpecsAnalyser::readSpectrumParameter` (`specsAnalyser.cpp:1663-1738`),
    /// minus the `doCallbacksEnum` publish, which the caller does with
    /// direct struct access. Always returns at least one entry — on any
    /// failure that is the `"Not connected"` sentinel upstream pushes to
    /// avoid an EDM/mbbi `SDEF`-field display problem — and reports whether
    /// the read itself succeeded via the second tuple element.
    pub fn read_spectrum_parameter(&mut self, param_name: &str) -> (Vec<String>, bool) {
        let result = self.command_response(&codec::get_spectrum_command(param_name));
        match result {
            Ok(data) => match data.get("Values") {
                Some(values) => (parse_values_list(values), true),
                None => (vec!["Not connected".to_string()], false),
            },
            Err(_) => (vec!["Not connected".to_string()], false),
        }
    }

    /// `SpecsAnalyser::readSpectrumDataInfo(SPECSOrdinateRange)`
    /// (`specsAnalyser.cpp:1764-1819`, the only case the `switch` supports).
    pub fn read_ordinate_range(&mut self) -> Result<OrdinateRange, WireError> {
        let data = self.command_response(&codec::get_data_info_command("OrdinateRange"))?;
        let unit = match data.get("Unit").map(String::as_str) {
            Some("\"\"") | None => String::new(),
            Some(u) => clean_string(u, "\""),
        };
        Ok(OrdinateRange {
            unit,
            min: data.get("Min").and_then(|s| parse_double_field(s)),
            max: data.get("Max").and_then(|s| parse_double_field(s)),
        })
    }

    /// `SpecsAnalyser::validateSpectrum` (`specsAnalyser.cpp:979-1073`),
    /// including the fixed-energy start/end workaround
    /// (`specsAnalyser.cpp:1054-1065`, intentional device-protocol
    /// compensation, not a defect).
    pub fn validate_spectrum(
        &mut self,
        run_mode: Option<RunMode>,
        kinetic_energy: f64,
        lens_modes: &[String],
        scan_ranges: &[String],
    ) -> Result<ValidateResult, WireError> {
        let data = self.command_response("ValidateSpectrum")?;
        let field_f64 = |name: &str| {
            data.get(name)
                .and_then(|s| parse_double_field(s))
                .unwrap_or(0.0)
        };
        let field_i32 = |name: &str| {
            data.get(name)
                .and_then(|s| parse_integer_field(s))
                .unwrap_or(0)
        };

        let mut start_energy = field_f64("StartEnergy");
        let mut end_energy = field_f64("EndEnergy");
        let pass_energy = field_f64("PassEnergy");

        let lookup = |values: &[String], data_key: &str| -> i32 {
            let raw = data.get(data_key).map(String::as_str).unwrap_or("");
            let cleaned = clean_string(raw, "\"");
            values
                .iter()
                .position(|v| *v == cleaned)
                .map(|i| i as i32)
                .unwrap_or(-1)
        };

        if run_mode == Some(RunMode::Fe) {
            // ***** WORKAROUND FOR FIXED ENERGY START AND END *****
            start_energy = kinetic_energy - 0.1 * pass_energy;
            end_energy = kinetic_energy + 0.1 * pass_energy;
        }

        Ok(ValidateResult {
            start_energy,
            end_energy,
            step_width: field_f64("StepWidth"),
            samples: field_i32("Samples"),
            dwell_time: field_f64("DwellTime"),
            pass_energy,
            lens_mode: lookup(lens_modes, "LensMode"),
            scan_range: lookup(scan_ranges, "ScanRange"),
        })
    }

    /// `SpecsAnalyser::defineSpectrumFAT` (`specsAnalyser.cpp:1106-1145`).
    pub fn define_fat(
        &mut self,
        args: codec::DefineFatArgs,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_fat_command(args, lens_mode, scan_range);
        self.command_response(&cmd).map(|_| ())
    }

    /// `SpecsAnalyser::defineSpectrumSFAT` (`specsAnalyser.cpp:1152-1188`).
    pub fn define_sfat(
        &mut self,
        start_energy: f64,
        end_energy: f64,
        samples: i32,
        dwell_time: f64,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_sfat_command(
            start_energy,
            end_energy,
            samples,
            dwell_time,
            lens_mode,
            scan_range,
        );
        self.command_response(&cmd).map(|_| ())
    }

    /// `SpecsAnalyser::defineSpectrumFRR` (`specsAnalyser.cpp:1190-1229`).
    pub fn define_frr(
        &mut self,
        args: codec::DefineFrrArgs,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_frr_command(args, lens_mode, scan_range);
        self.command_response(&cmd).map(|_| ())
    }

    /// `SpecsAnalyser::defineSpectrumFE` (`specsAnalyser.cpp:1231-1267`).
    pub fn define_fe(
        &mut self,
        kinetic_energy: f64,
        samples: i32,
        dwell_time: f64,
        pass_energy: f64,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_fe_command(
            kinetic_energy,
            samples,
            dwell_time,
            pass_energy,
            lens_mode,
            scan_range,
        );
        self.command_response(&cmd).map(|_| ())
    }

    /// `SpecsAnalyser::readAcquisitionData` (`specsAnalyser.cpp:1269-1307`).
    pub fn read_acquisition_data(
        &mut self,
        start_index: i32,
        end_index: i32,
    ) -> Result<Vec<f64>, WireError> {
        let data = self.command_response(&codec::get_data_command(start_index, end_index))?;
        Ok(codec::parse_data_array(
            data.get("Data").map(String::as_str).unwrap_or(""),
        ))
    }
}

/// `SpecsAnalyser::readRunModes` (`specsAnalyser.cpp:1740-1754`) — hardcoded,
/// no wire I/O.
pub fn read_run_modes() -> Vec<String> {
    vec![
        "Fixed Transmission".to_string(),
        "Snapshot".to_string(),
        "Fixed Retarding Ratio".to_string(),
        "Fixed Energy".to_string(),
    ]
}
