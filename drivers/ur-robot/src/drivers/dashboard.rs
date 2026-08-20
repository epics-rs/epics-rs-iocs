//! `URDashboard` — asyn port driver for the dashboard server.
//!
//! Port of `urRobotApp/src/dashboard_driver.cpp`. MAX_ADDR = 1.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::dashboard::DashboardClient;
use crate::drivers::asyn_error;
use crate::drivers::ioc_ready::IocReady;
use crate::registry::{self, DashboardHandle, DashboardState};
use crate::session::DEFAULT_TIMEOUT;

/// asyn parameter indices, one per `createParam` in the C++ constructor.
#[derive(Clone, Copy)]
pub struct DashboardParams {
    pub is_connected: usize,
    pub load_urp: usize,
    pub play: usize,
    pub stop: usize,
    pub pause: usize,
    pub connect: usize,
    pub disconnect: usize,
    pub shutdown: usize,
    pub is_running: usize,
    pub close_popup: usize,
    pub popup: usize,
    pub close_safety_popup: usize,
    pub power_on: usize,
    pub power_off: usize,
    pub brake_release: usize,
    pub unlock_protective_stop: usize,
    pub restart_safety: usize,
    pub polyscope_version: usize,
    pub serial_number: usize,
    pub robot_mode: usize,
    pub program_state: usize,
    pub robot_model: usize,
    pub loaded_program: usize,
    pub safety_status: usize,
    pub is_program_saved: usize,
    pub is_in_remote_control: usize,
}

impl DashboardParams {
    pub(crate) fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            is_connected: base.create_param("IS_CONNECTED", ParamType::Int32)?,
            load_urp: base.create_param("LOAD_URP", ParamType::Octet)?,
            play: base.create_param("PLAY", ParamType::Int32)?,
            stop: base.create_param("STOP", ParamType::Int32)?,
            pause: base.create_param("PAUSE", ParamType::Int32)?,
            connect: base.create_param("CONNECT", ParamType::Int32)?,
            disconnect: base.create_param("DISCONNECT", ParamType::Int32)?,
            shutdown: base.create_param("SHUTDOWN", ParamType::Int32)?,
            is_running: base.create_param("IS_RUNNING", ParamType::Int32)?,
            close_popup: base.create_param("CLOSE_POPUP", ParamType::Int32)?,
            popup: base.create_param("POPUP", ParamType::Octet)?,
            close_safety_popup: base.create_param("CLOSE_SAFETY_POPUP", ParamType::Int32)?,
            power_on: base.create_param("POWER_ON", ParamType::Int32)?,
            power_off: base.create_param("POWER_OFF", ParamType::Int32)?,
            brake_release: base.create_param("BRAKE_RELEASE", ParamType::Int32)?,
            unlock_protective_stop: base
                .create_param("UNLOCK_PROTECTIVE_STOP", ParamType::Int32)?,
            restart_safety: base.create_param("RESTART_SAFETY", ParamType::Int32)?,
            polyscope_version: base.create_param("POLYSCOPE_VERSION", ParamType::Octet)?,
            serial_number: base.create_param("SERIAL_NUMBER", ParamType::Octet)?,
            robot_mode: base.create_param("ROBOT_MODE", ParamType::Octet)?,
            program_state: base.create_param("PROGRAM_STATE", ParamType::Octet)?,
            robot_model: base.create_param("ROBOT_MODEL", ParamType::Octet)?,
            loaded_program: base.create_param("LOADED_PROGRAM", ParamType::Octet)?,
            safety_status: base.create_param("SAFETY_STATUS", ParamType::Octet)?,
            is_program_saved: base.create_param("IS_PROGRAM_SAVED", ParamType::Int32)?,
            is_in_remote_control: base.create_param("IS_IN_REMOTE_CONTROL", ParamType::Int32)?,
        })
    }

    /// The momentary commands of this port — see the
    /// [module docs](crate::drivers#momentary-commands).
    fn is_command(&self, reason: usize) -> bool {
        reason == self.connect
            || reason == self.disconnect
            || reason == self.play
            || reason == self.stop
            || reason == self.pause
            || reason == self.shutdown
            || reason == self.close_popup
            || reason == self.close_safety_popup
            || reason == self.power_on
            || reason == self.power_off
            || reason == self.brake_release
            || reason == self.unlock_protective_stop
            || reason == self.restart_safety
    }

    /// The poll-owned readbacks that carry the COMM alarm while the
    /// dashboard link is down. Excluded beyond the commands: the octet
    /// outputs (`LOAD_URP`/`POPUP` carry requests, not device state),
    /// `IS_CONNECTED` (the health readback must stay valid at 0 — it is
    /// the one PV that says why), and the three connect-time identity
    /// strings (`POLYSCOPE_VERSION`/`SERIAL_NUMBER`/`ROBOT_MODEL`), which
    /// do not go stale with the link. `alarm_set_is_every_poll_readback`
    /// enforces that a new parameter lands in exactly one group.
    fn alarm_targets(&self) -> Vec<(usize, i32)> {
        [
            self.is_running,
            self.program_state,
            self.robot_mode,
            self.loaded_program,
            self.safety_status,
            self.is_program_saved,
            self.is_in_remote_control,
        ]
        .into_iter()
        .map(|r| (r, 0))
        .collect()
    }
}

/// The dashboard driver.
pub struct DashboardDriver {
    base: PortDriverBase,
    params: DashboardParams,
    client: Arc<Mutex<DashboardClient>>,
    shared: DashboardHandle,
}

impl DashboardDriver {
    pub fn new(port_name: &str, robot_ip: &str) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = DashboardParams::create(&mut base)?;

        let client = Arc::new(Mutex::new(DashboardClient::new(robot_ip, DEFAULT_TIMEOUT)));
        let shared = DashboardHandle::new(robot_ip);
        registry::register_dashboard(port_name, shared.clone()).map_err(asyn_error)?;

        // The C++ constructor connects, then reads the three static strings.
        {
            let mut c = client.lock();
            match c.connect() {
                Ok(()) => log::info!("ur-robot: connected to the dashboard server at {robot_ip}"),
                Err(e) => log::error!("ur-robot: dashboard connect failed: {e}"),
            }
            if c.is_connected() {
                let version = c.polyscope_version().unwrap_or_default();
                base.set_string_param(
                    params.polyscope_version,
                    0,
                    format!(
                        "{}.{}.{}.{}",
                        version.major, version.minor, version.patch, version.build
                    ),
                )?;
                if let Ok(serial) = c.serial_number(version) {
                    base.set_string_param(params.serial_number, 0, serial)?;
                }
                if let Ok(model) = c.robot_model() {
                    base.set_string_param(params.robot_model, 0, model)?;
                }
                base.set_int32_param(params.is_connected, 0, 1)?;
                shared.set(DashboardState {
                    connected: true,
                    robot_mode: String::new(),
                    polyscope: version,
                });
            }
        }

        Ok(Self {
            base,
            params,
            client,
            shared,
        })
    }

    pub fn client(&self) -> Arc<Mutex<DashboardClient>> {
        Arc::clone(&self.client)
    }

    pub fn params(&self) -> DashboardParams {
        self.params
    }

    pub fn shared(&self) -> DashboardHandle {
        self.shared.clone()
    }

    /// `try_connect()`.
    fn try_connect(&mut self) -> bool {
        let mut c = self.client.lock();
        if c.is_connected() {
            return true;
        }
        match c.connect() {
            Ok(()) => true,
            Err(e) => {
                log::error!("ur-robot: dashboard connect failed: {e}");
                false
            }
        }
    }
}

impl PortDriver for DashboardDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let result: AsynResult<()> = (|| {
            let reason = user.reason;
            let p = self.params;
            self.base.params.set_int32(reason, user.addr, value)?;

            if p.is_command(reason) && value == 0 {
                return Ok(());
            }

            if reason == p.connect {
                let ok = self.try_connect();
                let connected = i32::from(ok);
                self.base.set_int32_param(p.is_connected, 0, connected)?;
                let mut s = self.shared.get();
                s.connected = ok;
                self.shared.set(s);
                return if ok {
                    Ok(())
                } else {
                    Err(asyn_error("could not connect to the dashboard server"))
                };
            }

            let mut client = self.client.lock();
            if !client.is_connected() {
                log::warn!("ur-robot: the dashboard is disconnected; no action taken");
                return Err(asyn_error("the dashboard is not connected"));
            }

            let result = if reason == p.play {
                client.play()
            } else if reason == p.stop {
                client.stop()
            } else if reason == p.pause {
                client.pause()
            } else if reason == p.disconnect {
                client.disconnect();
                Ok(())
            } else if reason == p.shutdown {
                client.shutdown()
            } else if reason == p.close_popup {
                client.close_popup()
            } else if reason == p.close_safety_popup {
                client.close_safety_popup()
            } else if reason == p.power_on {
                client.power_on()
            } else if reason == p.power_off {
                client.power_off()
            } else if reason == p.brake_release {
                client.brake_release()
            } else if reason == p.unlock_protective_stop {
                client.unlock_protective_stop()
            } else if reason == p.restart_safety {
                client.restart_safety()
            } else {
                Ok(())
            };

            match result {
                Ok(()) => Ok(()),
                Err(e) => {
                    log::error!("ur-robot: dashboard command failed: {e}");
                    Err(asyn_error("dashboard command failed"))
                }
            }
        })();
        crate::drivers::flush_after(&mut self.base, user.addr, result)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let result: AsynResult<usize> = (|| {
            let reason = user.reason;
            let p = self.params;
            let text = String::from_utf8_lossy(data)
                .trim_end_matches('\0')
                .to_string();
            self.base
                .set_string_param(reason, user.addr, text.clone())?;

            let mut client = self.client.lock();
            if !client.is_connected() {
                log::warn!("ur-robot: the dashboard is disconnected; no action taken");
                return Err(asyn_error("the dashboard is not connected"));
            }

            let result = if reason == p.popup {
                client.popup(&text)
            } else if reason == p.load_urp {
                client.load_urp(&text)
            } else {
                Ok(())
            };

            match result {
                Ok(()) => Ok(data.len()),
                Err(e) => {
                    log::error!("ur-robot: dashboard command failed: {e}");
                    Err(asyn_error("dashboard command failed"))
                }
            }
        })();
        crate::drivers::flush_after(&mut self.base, user.addr, result)
    }
}

/// The dashboard poll thread (`URDashboard::poll`).
pub fn start_poller(
    handle: PortHandle,
    params: DashboardParams,
    client: Arc<Mutex<DashboardClient>>,
    shared: DashboardHandle,
    poll_period: Duration,
    ready: Arc<IocReady>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("ur-dashboard-poll".into())
        .spawn(move || {
            ready.wait();
            let alarm_targets = params.alarm_targets();
            // Starts true so an IOC that boots with the dashboard down
            // raises the COMM alarm on its first cycle.
            let mut was_healthy = true;
            loop {
                let mut updates = Vec::new();
                let mut state = shared.get();

                {
                    let mut c = client.lock();
                    if c.is_connected() {
                        match poll_once(&mut c, params, state.polyscope) {
                            Ok((u, mode)) => {
                                updates = u;
                                state.connected = true;
                                state.robot_mode = mode;
                            }
                            Err(e) => {
                                log::error!("ur-robot: dashboard poll failed: {e}");
                                c.disconnect();
                                state.connected = false;
                                updates.push(ParamSetValue::new(
                                    params.is_connected,
                                    0,
                                    ParamValue::Int32(0),
                                ));
                                updates.push(ParamSetValue::new(
                                    params.is_in_remote_control,
                                    0,
                                    ParamValue::Int32(0),
                                ));
                            }
                        }
                    } else {
                        state.connected = false;
                        updates.push(ParamSetValue::new(
                            params.is_connected,
                            0,
                            ParamValue::Int32(0),
                        ));
                    }
                }

                updates.extend(crate::drivers::health_transition(
                    &alarm_targets,
                    state.connected,
                    &mut was_healthy,
                ));
                shared.set(state);
                let _ = handle.set_params_and_notify_blocking(0, updates);
                std::thread::sleep(poll_period);
            }
        })
        .expect("failed to spawn the dashboard poll thread")
}

/// One poll cycle. Returns the parameter updates and the robot mode string.
fn poll_once(
    client: &mut DashboardClient,
    params: DashboardParams,
    polyscope: crate::dashboard::PolyScopeVersion,
) -> crate::UrResult<(Vec<ParamSetValue>, String)> {
    let running = client.running()?;
    let program_state = client.program_state()?;
    let robot_mode = client.robot_mode()?;
    let loaded_program = client.loaded_program()?;
    let safety_status = client.safety_status()?;
    let program_saved = client.is_program_saved()?;
    let remote = client.is_in_remote_control(polyscope)?;

    let updates = vec![
        ParamSetValue::new(params.is_connected, 0, ParamValue::Int32(1)),
        ParamSetValue::new(params.is_running, 0, ParamValue::Int32(i32::from(running))),
        ParamSetValue::new(params.program_state, 0, ParamValue::Octet(program_state)),
        ParamSetValue::new(params.robot_mode, 0, ParamValue::Octet(robot_mode.clone())),
        ParamSetValue::new(params.loaded_program, 0, ParamValue::Octet(loaded_program)),
        ParamSetValue::new(params.safety_status, 0, ParamValue::Octet(safety_status)),
        ParamSetValue::new(
            params.is_program_saved,
            0,
            ParamValue::Int32(i32::from(program_saved)),
        ),
        ParamSetValue::new(
            params.is_in_remote_control,
            0,
            ParamValue::Int32(i32::from(remote)),
        ),
    ];
    Ok((updates, robot_mode))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The octet pair is the interesting negative: `POPUP` and `LOAD_URP`
    /// look like commands but their payload is the request, and they never
    /// reach the int32 gate anyway.
    #[test]
    fn the_thirteen_commands_are_commands_and_the_readbacks_are_not() {
        let mut base = PortDriverBase::new("dash_is_command", 1, PortFlags::default());
        let p = DashboardParams::create(&mut base).expect("params create");

        for reason in [
            p.connect,
            p.disconnect,
            p.play,
            p.stop,
            p.pause,
            p.shutdown,
            p.close_popup,
            p.close_safety_popup,
            p.power_on,
            p.power_off,
            p.brake_release,
            p.unlock_protective_stop,
            p.restart_safety,
        ] {
            assert!(p.is_command(reason));
        }
        for reason in [
            p.is_connected,
            p.is_running,
            p.popup,
            p.load_urp,
            p.robot_mode,
            p.is_in_remote_control,
        ] {
            assert!(!p.is_command(reason));
        }
    }

    /// Completeness of the alarm set: every parameter this driver creates is
    /// a command, an alarm target, or one of the deliberate exclusions — a
    /// parameter added without classification fails here instead of silently
    /// staying NO_ALARM through an outage.
    #[test]
    fn alarm_set_is_every_poll_readback() {
        let mut base = PortDriverBase::new("dash_alarm_set", 1, PortFlags::default());
        let p = DashboardParams::create(&mut base).expect("params create");

        let targets = p.alarm_targets();
        let excluded = [
            p.is_connected,
            p.load_urp,
            p.popup,
            p.polyscope_version,
            p.serial_number,
            p.robot_model,
        ];
        for reason in 0..base.params.len() {
            let targeted = targets.iter().any(|(r, _)| *r == reason);
            let exempt = excluded.contains(&reason) || p.is_command(reason);
            assert!(
                targeted != exempt,
                "param {reason} must be exactly one of: alarm target, command/exclusion"
            );
        }
    }
}
