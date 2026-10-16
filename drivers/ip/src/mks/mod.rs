//! MKS 937 vacuum gauge controller (`ipApp/src/devAiMKS.c`).
//!
//! Read-only port, one asyn address per gauge (address 0 = gauge 1). The C read
//! the units and the gauge types once during `init_record` and used them to set
//! each ai record's EGU / LOPR / HOPR; a port driver cannot write those fields,
//! so the units, the gauge type and its range are published as parameters and
//! the database wires them where it wants them.
//!
//! Two deviations from `devAiMKS.c`, both deliberate:
//!
//! * The C set `pai->val = 0.` before decoding every reply, so a reply it could
//!   not turn into a pressure left the record reading 0 Torr. On `SYNTAX!` /
//!   `NotCMD!` — which it says the 937 emits spuriously — it then suppressed the
//!   alarm as well (`devAiMKS.c:338-347`), publishing that 0 as *good* data.
//!   Suppressing the alarm was intended; publishing a false perfect vacuum was
//!   not. Here a reply that is not a pressure never touches `MKS_PRESSURE`; the
//!   gauge keeps its last good reading and `MKS_STATUS` says why it did not
//!   move.
//! * The C rejected any reply whose length was not exactly 7 bytes. Replies are
//!   classified by content here, so a short or padded reply lands in
//!   [`Reading::Unknown`] instead of being rejected on length.

pub mod protocol;

use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceMask;

use crate::connect::connect_octet;
use crate::runtime::IpPortRuntime;
use crate::worker::{self, DeviceWorker, Transport};
use protocol::{GaugeType, MAX_GAUGES, Reading};

/// Command timeout (`TIMEOUT`, `devAiMKS.c:65`).
pub const MKS_TIMEOUT: Duration = Duration::from_secs(2);

/// The reply length C accepts from every 937 query (`nread != 7`).
const REPLY_LEN: usize = 7;

/// The MKS port is read-only, so the worker takes no commands.
pub enum MksCommand {}

/// Reading status published alongside the pressure. The C encoded these as
/// record alarms from inside device support; the database attaches the
/// severities now.
pub mod status {
    pub const OK: i32 = 0;
    pub const ABOVE_RANGE: i32 = 1;
    pub const BELOW_RANGE: i32 = 2;
    pub const NO_GAUGE: i32 = 3;
    pub const UNKNOWN: i32 = 4;
}

#[derive(Clone, Copy)]
pub struct MksParams {
    pub pressure: usize,
    pub status: usize,
    pub units: usize,
    pub gauge_type: usize,
    pub low_limit: usize,
    pub high_limit: usize,
}

impl MksParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            pressure: base.create_param("MKS_PRESSURE", ParamType::Float64)?,
            status: base.create_param("MKS_STATUS", ParamType::Int32)?,
            units: base.create_param("MKS_UNITS", ParamType::Octet)?,
            gauge_type: base.create_param("MKS_GAUGE_TYPE", ParamType::Octet)?,
            low_limit: base.create_param("MKS_LOW_LIMIT", ParamType::Float64)?,
            high_limit: base.create_param("MKS_HIGH_LIMIT", ParamType::Float64)?,
        })
    }
}

pub struct MksDriver {
    base: PortDriverBase,
    params: MksParams,
    /// Keeps the worker's receiver open for the life of the port; the MKS port
    /// has no writable parameters.
    _commands: std::sync::mpsc::Sender<MksCommand>,
}

impl MksDriver {
    pub fn new(port_name: &str, commands: std::sync::mpsc::Sender<MksCommand>) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            MAX_GAUGES,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = MksParams::create(&mut base)?;
        Ok(Self {
            base,
            params,
            _commands: commands,
        })
    }

    pub fn params(&self) -> MksParams {
        self.params
    }
}

impl PortDriver for MksDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
}

/// What the controller said about a gauge when the port started.
#[derive(Clone, Copy)]
struct GaugeConfig {
    low: f64,
    high: f64,
}

pub struct MksWorker {
    transport: Transport,
    handle: PortHandle,
    params: MksParams,
    gauges: u8,
    /// Filled by the first successful poll (the C did this in `init_record`,
    /// which meant a controller that was offline at iocInit left the record
    /// dead for the life of the IOC).
    config: Option<Vec<Option<GaugeConfig>>>,
}

impl MksWorker {
    fn transact(&self, command: &str) -> Result<String, String> {
        self.transport.write_read(command.as_bytes())
    }

    fn publish(&self, addr: i32, values: Vec<ParamSetValue>) {
        if !values.is_empty() {
            let _ = self.handle.set_params_and_notify_blocking(addr, values);
        }
    }

    /// One of `initAi`'s `writeReadOnce` reads, refused unless it returns the
    /// 7 characters C requires. `who` stands where C prints the record name.
    fn read_config(&self, command: &str, who: &str) -> Result<String, ()> {
        let reply = self.transact(command).unwrap_or_default();
        if reply.len() != REPLY_LEN {
            self.transport.print(
                TraceMask::ERROR,
                format_args!(
                    "devAiMKS ERROR, record={who}, nread={}, response=\n{reply}",
                    reply.len()
                ),
            );
            return Err(());
        }
        Ok(reply)
    }

    /// Read `SU` and `SG` and publish the units, gauge type and range of every
    /// gauge (`devAiMKS.c::initAi`).
    fn configure(&mut self) -> Result<Vec<Option<GaugeConfig>>, ()> {
        let units = self
            .read_config(protocol::READ_UNITS, "MKS_UNITS")?
            .trim()
            .to_string();
        let types = self.read_config(protocol::READ_GAUGE_TYPES, "MKS_GAUGE_TYPE")?;
        let multiplier = protocol::unit_multiplier(&units);
        let p = self.params;

        let mut config = Vec::with_capacity(usize::from(self.gauges));
        for gauge in 1..=self.gauges {
            let addr = i32::from(gauge) - 1;
            let code = protocol::gauge_type_field(&types, gauge).unwrap_or("");
            let gauge_type = GaugeType::from_code(code);
            let mut values = vec![
                ParamSetValue::new(
                    p.units,
                    addr,
                    ParamValue::Octet(units.clone().into_bytes().into()),
                ),
                ParamSetValue::new(
                    p.gauge_type,
                    addr,
                    ParamValue::Octet(code.as_bytes().into()),
                ),
            ];
            match gauge_type {
                Some(gauge_type) => {
                    let (low, high) = gauge_type.limits();
                    let (low, high) = (low * multiplier, high * multiplier);
                    values.push(ParamSetValue::new(
                        p.low_limit,
                        addr,
                        ParamValue::Float64(low),
                    ));
                    values.push(ParamSetValue::new(
                        p.high_limit,
                        addr,
                        ParamValue::Float64(high),
                    ));
                    config.push(Some(GaugeConfig { low, high }));
                }
                None => {
                    self.transport.print(
                        TraceMask::ERROR,
                        format_args!("devAiMKS, unknown gauge type {code}"),
                    );
                    config.push(None);
                }
            }
            self.publish(addr, values);
        }
        Ok(config)
    }
}

impl DeviceWorker for MksWorker {
    type Command = MksCommand;

    fn poll(&mut self) {
        if self.config.is_none() {
            match self.configure() {
                Ok(config) => self.config = Some(config),
                Err(()) => return,
            }
        }
        let config = self.config.clone().unwrap_or_default();
        let p = self.params;

        for gauge in 1..=self.gauges {
            let addr = i32::from(gauge) - 1;
            // C decodes only a 7-character reply (`callbackAi`).
            let raw = match self.transact(&protocol::read_pressure(gauge)) {
                Ok(raw) if raw.len() == REPLY_LEN => raw,
                _ => continue,
            };
            let limits = config.get(usize::from(gauge - 1)).copied().flatten();
            let (value, code) = match protocol::parse_reading(&raw) {
                Reading::Pressure(value) => (Some(value), status::OK),
                Reading::AboveRange => (limits.map(|c| c.high), status::ABOVE_RANGE),
                Reading::BelowRange => (limits.map(|c| c.low), status::BELOW_RANGE),
                Reading::NoGauge => (None, status::NO_GAUGE),
                // The 937 emits SYNTAX! / NotCMD! spuriously; the C neither
                // alarms nor logs, and leaves the record's value alone.
                Reading::Spurious => continue,
                Reading::Unknown => {
                    self.transport.print(
                        TraceMask::ERROR,
                        format_args!(
                            "devAiMKS record=MKS_PRESSURE[{addr}], unknown response len={} response={raw}",
                            raw.len()
                        ),
                    );
                    (None, status::UNKNOWN)
                }
            };

            let mut values = vec![ParamSetValue::new(p.status, addr, ParamValue::Int32(code))];
            if let Some(value) = value {
                values.push(ParamSetValue::new(
                    p.pressure,
                    addr,
                    ParamValue::Float64(value),
                ));
            }
            self.publish(addr, values);
        }
    }

    fn handle(&mut self, _command: Self::Command) {}
}

/// `MKSConfig(port, octetPort, numGauges, [pollPeriod])`.
pub fn create_mks(
    claim: &PortClaim,
    octet_port: &str,
    gauges: u8,
    poll_period: Duration,
) -> AsynResult<IpPortRuntime> {
    let port_name = claim.name();
    if gauges == 0 || usize::from(gauges) > MAX_GAUGES {
        return Err(crate::asyn_error(format!(
            "MKS: numGauges must be 1..{MAX_GAUGES}, got {gauges}"
        )));
    }

    let io = connect_octet(octet_port, MKS_TIMEOUT).map_err(crate::asyn_error)?;
    let (commands, rx) = std::sync::mpsc::channel::<MksCommand>();
    let driver = MksDriver::new(port_name, commands)?;
    let params = driver.params();

    let (runtime_handle, _actor) = create_port_runtime(claim, driver, RuntimeConfig::default())?;
    let worker = MksWorker {
        transport: io,
        handle: runtime_handle.port_handle().clone(),
        params,
        gauges,
        config: None,
    };
    let thread = worker::spawn(port_name, worker, rx, poll_period);
    Ok(IpPortRuntime::new(runtime_handle, thread))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::worker::test_support::{refused_transport, traced};

    /// A dead controller fails initAi's units read with C's error line,
    /// and the poll stops there.
    #[test]
    fn a_failed_units_read_prints_devaimks_error() {
        let (transport, path, runtime) = refused_transport("MKS_TRACE");
        let mut worker = MksWorker {
            transport,
            handle: runtime.port_handle().clone(),
            params: MksParams {
                pressure: 0,
                status: 1,
                units: 2,
                gauge_type: 3,
                low_limit: 4,
                high_limit: 5,
            },
            gauges: 1,
            config: None,
        };
        worker.poll();
        let lines = traced(&path);
        assert_eq!(lines.len(), 2, "{lines:?}");
        assert!(
            lines[0].ends_with("devAiMKS ERROR, record=MKS_UNITS, nread=0, response="),
            "{lines:?}"
        );
        assert!(worker.config.is_none());
    }
}
