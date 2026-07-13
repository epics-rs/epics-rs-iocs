//! `RTDEInOut` — asyn port driver for the RTDE input (output-setting) interface.
//!
//! Port of `urRobotApp/src/rtde_io_driver.cpp`. MAX_ADDR = 8 (one address per
//! digital / analog output channel). The C++ driver has no poll thread: every
//! parameter is write-only.

use std::sync::Arc;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::drivers::asyn_error;
use crate::io::IoInterface;

/// Digital outputs 0..7 and analog outputs 0..1 are addressed by asyn address.
pub const NUM_CHANNELS: usize = 8;

#[derive(Clone, Copy)]
pub struct IoParams {
    pub speed_slider: usize,
    pub set_standard_digital_out: usize,
    pub set_config_digital_out: usize,
    pub set_tool_digital_out: usize,
    pub set_voltage_analog_out: usize,
    pub set_current_analog_out: usize,
}

impl IoParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            speed_slider: base.create_param("SPEED_SLIDER", ParamType::Float64)?,
            set_standard_digital_out: base
                .create_param("SET_STANDARD_DIGITAL_OUT", ParamType::Int32)?,
            set_config_digital_out: base
                .create_param("SET_CONFIG_DIGITAL_OUT", ParamType::Int32)?,
            set_tool_digital_out: base.create_param("SET_TOOL_DIGITAL_OUT", ParamType::Int32)?,
            set_voltage_analog_out: base
                .create_param("SET_VOLTAGE_ANALOG_OUT", ParamType::Float64)?,
            set_current_analog_out: base
                .create_param("SET_CURRENT_ANALOG_OUT", ParamType::Float64)?,
        })
    }
}

pub struct IoDriver {
    base: PortDriverBase,
    params: IoParams,
    iface: Arc<Mutex<Option<IoInterface>>>,
    robot_ip: String,
}

impl IoDriver {
    pub fn new(port_name: &str, robot_ip: &str) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            NUM_CHANNELS,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = IoParams::create(&mut base)?;
        let mut me = Self {
            base,
            params,
            iface: Arc::new(Mutex::new(None)),
            robot_ip: robot_ip.to_string(),
        };
        me.try_connect();
        Ok(me)
    }

    pub fn params(&self) -> IoParams {
        self.params
    }

    fn try_connect(&mut self) -> bool {
        let mut slot = self.iface.lock();
        match slot.as_mut() {
            Some(iface) => match iface.reconnect() {
                Ok(()) => {
                    log::info!("ur-robot: reconnected to the RTDE I/O interface");
                    true
                }
                Err(e) => {
                    log::error!("ur-robot: RTDE I/O reconnect failed: {e}");
                    false
                }
            },
            None => match IoInterface::connect(&self.robot_ip, false) {
                Ok(iface) => {
                    log::info!("ur-robot: connected to the RTDE I/O interface");
                    *slot = Some(iface);
                    true
                }
                Err(e) => {
                    log::error!("ur-robot: RTDE I/O connect failed: {e}");
                    false
                }
            },
        }
    }
}

impl PortDriver for IoDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let p = self.params;
        self.base.params.set_int32(reason, addr, value)?;

        let channel = u8::try_from(addr)
            .map_err(|_| asyn_error(format!("I/O channel {addr} is out of range")))?;
        let level = value != 0;

        let mut slot = self.iface.lock();
        let Some(iface) = slot.as_mut() else {
            return Err(asyn_error("the RTDE I/O interface is not initialised"));
        };

        let result = if reason == p.set_standard_digital_out {
            iface.set_standard_digital_out(channel, level)
        } else if reason == p.set_config_digital_out {
            iface.set_configurable_digital_out(channel, level)
        } else if reason == p.set_tool_digital_out {
            iface.set_tool_digital_out(channel, level)
        } else {
            return Ok(());
        };
        result.map_err(|e| asyn_error(format!("RTDE I/O write failed: {e}")))
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let p = self.params;
        self.base.params.set_float64(reason, addr, value)?;

        let channel = u8::try_from(addr)
            .map_err(|_| asyn_error(format!("I/O channel {addr} is out of range")))?;

        let mut slot = self.iface.lock();
        let Some(iface) = slot.as_mut() else {
            return Err(asyn_error("the RTDE I/O interface is not initialised"));
        };

        let result = if reason == p.speed_slider {
            iface.set_speed_slider(value)
        } else if reason == p.set_voltage_analog_out {
            iface.set_analog_output_voltage(channel, value)
        } else if reason == p.set_current_analog_out {
            iface.set_analog_output_current(channel, value)
        } else {
            return Ok(());
        };
        result.map_err(|e| asyn_error(format!("RTDE I/O write failed: {e}")))
    }
}
