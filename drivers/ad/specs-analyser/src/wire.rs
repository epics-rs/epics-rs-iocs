//! The exchange with `driverPort_`: C's `commandResponse`/`asynWriteRead`
//! and the commands built on them. A [`WireLink`] borrows the driver, so every
//! exchange runs on the port actor, under the driver lock, with
//! `SPECSMsgCounter_` and `SPECSConnected_` kept in the parameter library as
//! C keeps them.

use std::collections::HashMap;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::request::OctetReadOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::codec::{
    self, BeginOutcome, ErrorInfo, ResponseAssembler, clean_string, next_counter,
    parse_double_field, parse_integer_field, parse_values_list,
};
use crate::params::SpecsParams;
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
    /// The reply lacked the named field or it did not decode
    /// (`readIntegerData`/`readDoubleData`, `specsAnalyser.cpp:1635-1661`).
    Field(&'static str),
}

/// Every failure is C's `asynError` except the round trip's own status.
impl From<WireError> for AsynError {
    fn from(e: WireError) -> Self {
        match e {
            WireError::Io(e) => e,
            e => AsynError::Status {
                status: AsynStatus::Error,
                message: e.status_message(),
            },
        }
    }
}

impl WireError {
    /// The asynStatus C's exchange returns for it.
    pub fn status(&self) -> AsynStatus {
        match self {
            WireError::Io(e) => e.status(),
            _ => AsynStatus::Error,
        }
    }

    /// The error's text.
    pub fn status_message(&self) -> String {
        match self {
            WireError::NotConnected => "Not connected to SPECS".to_string(),
            WireError::Io(e) => format!("SPECS communication error: {e}"),
            WireError::Frame(e) => format!("SPECS protocol framing error: {e:?}"),
            WireError::Device(info) => info.status_message(),
            WireError::Field(name) => format!("SPECS reply has no valid {name}"),
        }
    }
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

/// `driverPort_` and the driver's parameter library.
pub struct WireLink<'a> {
    /// C `portUser_`, unconnected when st.cmd named a missing port.
    pub driver_port: &'a SyncIOHandle,
    pub addr: i32,
    pub base: &'a mut PortDriverBase,
    pub p: &'a SpecsParams,
    /// `ADStatusMessage`.
    pub status_message: usize,
}

impl WireLink<'_> {
    fn user(&self) -> AsynUser {
        AsynUser::new(0)
            .with_addr(self.addr)
            .with_timeout(secs(SOCKET_TIMEOUT))
    }

    /// `SpecsAnalyser::sendSimpleCommand` (`specsAnalyser.cpp:1313-1339`),
    /// the path of every exchange. `asynWriteRead` advances and posts
    /// `SPECSMsgCounter_` before its connected checks (`:2056-2090`).
    pub fn command_response(
        &mut self,
        command: &str,
    ) -> Result<HashMap<String, String>, WireError> {
        let current = self
            .base
            .get_int32_param(self.p.msg_counter, 0)
            .unwrap_or(0);
        let counter = next_counter(current.max(0) as u32);
        let _ = self
            .base
            .set_int32_param(self.p.msg_counter, 0, counter as i32);
        let _ = self.base.call_param_callbacks(0);
        let port = match self.driver_port.port() {
            Ok(port) => port,
            Err(e) => {
                let _ = self.base.set_int32_param(self.p.connected, 0, 0);
                return Err(WireError::Io(e));
            }
        };
        if self.base.get_int32_param(self.p.connected, 0).unwrap_or(0) != 1 {
            return Err(WireError::NotConnected);
        }
        let request = codec::format_request(counter, command);

        let (reply, status) = port
            .submit_octet_read_blocking(
                OctetReadOp::WriteRead {
                    data: request.into_bytes(),
                    buf_size: MAX_MESSAGE_SIZE,
                },
                self.user(),
            )
            .into_parts();
        status.map_err(WireError::Io)?;
        let raw = String::from_utf8_lossy(&reply).into_owned();
        let payload = codec::strip_response_frame(&raw, counter).map_err(WireError::Frame)?;

        let mut assembler = ResponseAssembler::new();
        match assembler.begin(payload) {
            BeginOutcome::Ok => {}
            BeginOutcome::NeedsMore => loop {
                let (more, status) = port
                    .submit_octet_read_blocking(
                        OctetReadOp::Read {
                            buf_size: MAX_MESSAGE_SIZE,
                        },
                        self.user(),
                    )
                    .into_parts();
                status.map_err(WireError::Io)?;
                let chunk = String::from_utf8_lossy(&more).into_owned();
                if !assembler.continue_with(&chunk) {
                    break;
                }
            },
            BeginOutcome::Error(info) => {
                // `SpecsAnalyser::commandResponse`, `specsAnalyser.cpp:2021-2024`:
                // disconnect() drops a connected link (`:243-258`, `:1882`).
                if info.code == "3" {
                    let _ = self.base.set_int32_param(self.p.connected, 0, 0);
                }
                let _ = self
                    .base
                    .set_string_param(self.status_message, 0, info.status_message());
                return Err(WireError::Device(info));
            }
        }
        Ok(assembler.finish())
    }

    /// `SpecsAnalyser::asynPortConnect` (`specsAnalyser.cpp:1831-1871`). The
    /// port and its EOS come from st.cmd, so `pasynOctetSyncIO->connect`'s
    /// part is only the check that the port exists.
    pub fn connect(&mut self) -> Result<(), WireError> {
        self.driver_port.port().map_err(WireError::Io)?;
        let _ = self.base.set_int32_param(self.p.msg_counter, 0, 0);
        let _ = self.base.set_int32_param(self.p.connected, 0, 1);
        let data = self.command_response("Connect")?;
        let field =
            |name: &str| clean_string(data.get(name).map(String::as_str).unwrap_or(""), "\"");
        let _ = self
            .base
            .set_string_param(self.p.server_name, 0, field("ServerName"));
        let _ = self
            .base
            .set_string_param(self.p.protocol_version, 0, field("ProtocolVersion"));
        Ok(())
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
        match value {
            "\"false\"" => Ok(0),
            "\"true\"" => Ok(1),
            _ => parse_integer_field(value).ok_or(WireError::Field("Value")),
        }
    }

    /// `SpecsAnalyser::getAnalyserParameter(name, double&)`
    /// (`specsAnalyser.cpp:1539-1552`).
    pub fn get_analyser_parameter_double(&mut self, name: &str) -> Result<f64, WireError> {
        let data = self.command_response(&codec::get_value_command(name))?;
        parse_double_field(data.get("Value").map(String::as_str).unwrap_or(""))
            .ok_or(WireError::Field("Value"))
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
    /// minus the `doCallbacksEnum` publish and the `"Not connected"` entry it
    /// substitutes on failure, which the caller applies.
    pub fn read_spectrum_parameter(&mut self, param_name: &str) -> Result<Vec<String>, WireError> {
        let data = self.command_response(&codec::get_spectrum_command(param_name))?;
        data.get("Values")
            .map(|values| parse_values_list(values))
            .ok_or(WireError::Field("Values"))
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
