//! The parameters Pixirad adds to the areaDetector base set (C `createParam`).

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

use crate::types::*;

#[derive(Clone, Copy)]
pub struct PixiradParams {
    pub system_reset: usize,
    pub system_info: usize,
    pub colors_collected: usize,
    pub udp_buffers_read: usize,
    pub udp_buffers_max: usize,
    pub udp_buffers_free: usize,
    pub udp_speed: usize,
    pub threshold: [usize; 4],
    pub hit_threshold: usize,
    pub threshold_actual: [usize; 4],
    pub hit_threshold_actual: usize,
    pub count_mode: usize,
    pub auto_calibrate: usize,
    pub hv_value: usize,
    pub hv_state: usize,
    pub hv_mode: usize,
    pub hv_actual: usize,
    pub hv_current: usize,
    pub sync_in_polarity: usize,
    pub sync_out_polarity: usize,
    pub sync_out_function: usize,
    pub cooling_state: usize,
    pub hot_temperature: usize,
    pub box_temperature: usize,
    pub box_humidity: usize,
    pub dew_point: usize,
    pub cooling_status: usize,
    pub peltier_power: usize,
    /// No record binds to this one: `pixiradAutoCal` writes its arguments here
    /// so the port actor is the only thing that ever talks to the box.
    pub autocal_conf: usize,
}

impl PixiradParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            system_reset: base.create_param(PIXIRAD_SYSTEM_RESET, ParamType::Int32)?,
            system_info: base.create_param(PIXIRAD_SYSTEM_INFO, ParamType::Octet)?,
            colors_collected: base.create_param(PIXIRAD_COLORS_COLLECTED, ParamType::Int32)?,
            udp_buffers_read: base.create_param(PIXIRAD_UDP_BUFFERS_READ, ParamType::Int32)?,
            udp_buffers_max: base.create_param(PIXIRAD_UDP_BUFFERS_MAX, ParamType::Int32)?,
            udp_buffers_free: base.create_param(PIXIRAD_UDP_BUFFERS_FREE, ParamType::Int32)?,
            udp_speed: base.create_param(PIXIRAD_UDP_SPEED, ParamType::Float64)?,
            threshold: [
                base.create_param(PIXIRAD_THRESHOLD[0], ParamType::Float64)?,
                base.create_param(PIXIRAD_THRESHOLD[1], ParamType::Float64)?,
                base.create_param(PIXIRAD_THRESHOLD[2], ParamType::Float64)?,
                base.create_param(PIXIRAD_THRESHOLD[3], ParamType::Float64)?,
            ],
            hit_threshold: base.create_param(PIXIRAD_HIT_THRESHOLD, ParamType::Float64)?,
            threshold_actual: [
                base.create_param(PIXIRAD_THRESHOLD_ACTUAL[0], ParamType::Float64)?,
                base.create_param(PIXIRAD_THRESHOLD_ACTUAL[1], ParamType::Float64)?,
                base.create_param(PIXIRAD_THRESHOLD_ACTUAL[2], ParamType::Float64)?,
                base.create_param(PIXIRAD_THRESHOLD_ACTUAL[3], ParamType::Float64)?,
            ],
            hit_threshold_actual: base
                .create_param(PIXIRAD_HIT_THRESHOLD_ACTUAL, ParamType::Float64)?,
            count_mode: base.create_param(PIXIRAD_COUNT_MODE, ParamType::Int32)?,
            auto_calibrate: base.create_param(PIXIRAD_AUTO_CALIBRATE, ParamType::Int32)?,
            hv_value: base.create_param(PIXIRAD_HV_VALUE, ParamType::Float64)?,
            hv_state: base.create_param(PIXIRAD_HV_STATE, ParamType::Int32)?,
            hv_mode: base.create_param(PIXIRAD_HV_MODE, ParamType::Int32)?,
            hv_actual: base.create_param(PIXIRAD_HV_ACTUAL, ParamType::Float64)?,
            hv_current: base.create_param(PIXIRAD_HV_CURRENT, ParamType::Float64)?,
            sync_in_polarity: base.create_param(PIXIRAD_SYNC_IN_POLARITY, ParamType::Int32)?,
            sync_out_polarity: base.create_param(PIXIRAD_SYNC_OUT_POLARITY, ParamType::Int32)?,
            sync_out_function: base.create_param(PIXIRAD_SYNC_OUT_FUNCTION, ParamType::Int32)?,
            cooling_state: base.create_param(PIXIRAD_COOLING_STATE, ParamType::Int32)?,
            hot_temperature: base.create_param(PIXIRAD_HOT_TEMPERATURE, ParamType::Float64)?,
            box_temperature: base.create_param(PIXIRAD_BOX_TEMPERATURE, ParamType::Float64)?,
            box_humidity: base.create_param(PIXIRAD_BOX_HUMIDITY, ParamType::Float64)?,
            dew_point: base.create_param(PIXIRAD_DEW_POINT, ParamType::Float64)?,
            cooling_status: base.create_param(PIXIRAD_COOLING_STATUS, ParamType::Int32)?,
            peltier_power: base.create_param(PIXIRAD_PELTIER_POWER, ParamType::Float64)?,
            autocal_conf: base.create_param(PIXIRAD_AUTOCAL_CONF, ParamType::Octet)?,
        })
    }
}
