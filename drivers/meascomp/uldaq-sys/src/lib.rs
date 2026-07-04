//! Raw FFI bindings to libuldaq (Linux only).
//!
//! Manual bindings covering the subset of the uldaq C API used by
//! the measComp USB-CTR08 and USB-2408-2AO drivers.

#![allow(non_camel_case_types, non_upper_case_globals, dead_code)]

use std::os::raw::c_char;

// ---------------------------------------------------------------------------
// Handle and error types
// ---------------------------------------------------------------------------

pub type DaqDeviceHandle = i64;

pub type UlError = i32;

pub const ERR_NO_ERROR: UlError = 0;
pub const ERR_DEV_NOT_FOUND: UlError = 6;
pub const ERR_DEV_NOT_CONNECTED: UlError = 7;
pub const ERR_ALREADY_ACTIVE: UlError = 16;
pub const ERR_TIMEDOUT: UlError = 20;
pub const ERR_TEMP_OUT_OF_RANGE: UlError = 91;

pub const ERR_MSG_LEN: usize = 512;

// ---------------------------------------------------------------------------
// Device interface
// ---------------------------------------------------------------------------

pub const USB_IFC: u32 = 1 << 0;
pub const BLUETOOTH_IFC: u32 = 1 << 1;
pub const ETHERNET_IFC: u32 = 1 << 2;
pub const ANY_IFC: u32 = USB_IFC | BLUETOOTH_IFC | ETHERNET_IFC;

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone)]
pub struct DaqDeviceDescriptor {
    pub product_name: [c_char; 64],
    pub product_id: u32,
    pub dev_interface: u32,
    pub dev_string: [c_char; 64],
    pub unique_id: [c_char; 64],
    pub reserved: [c_char; 512],
}

impl Default for DaqDeviceDescriptor {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct TransferStatus {
    pub current_scan_count: u64,
    pub current_total_count: u64,
    pub current_index: i64,
    pub reserved: [c_char; 64],
}

impl Default for TransferStatus {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct DaqInChanDescriptor {
    pub channel: i32,
    pub chan_type: u32, // DaqInChanType flags
    pub range: i32,     // Range enum
    pub reserved: [c_char; 64],
}

impl Default for DaqInChanDescriptor {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct AiQueueElement {
    pub channel: i32,
    pub input_mode: i32, // AiInputMode
    pub range: i32,      // Range
    pub reserved: [c_char; 64],
}

impl Default for AiQueueElement {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

// ---------------------------------------------------------------------------
// Scan status
// ---------------------------------------------------------------------------

pub const SS_IDLE: i32 = 0;
pub const SS_RUNNING: i32 = 1;

// ---------------------------------------------------------------------------
// AiInputMode
// ---------------------------------------------------------------------------

pub const AI_DIFFERENTIAL: i32 = 1;
pub const AI_SINGLE_ENDED: i32 = 2;
pub const AI_PSEUDO_DIFFERENTIAL: i32 = 3;

// ---------------------------------------------------------------------------
// AiChanType (bitmask)
// ---------------------------------------------------------------------------

pub const AI_VOLTAGE: i64 = 1 << 0;
pub const AI_TC: i64 = 1 << 1;
pub const AI_RTD: i64 = 1 << 2;
pub const AI_THERMISTOR: i64 = 1 << 3;
pub const AI_DISABLED: i64 = 1 << 30;

// ---------------------------------------------------------------------------
// TcType
// ---------------------------------------------------------------------------

pub const TC_J: i32 = 1;
pub const TC_K: i32 = 2;
pub const TC_T: i32 = 3;
pub const TC_E: i32 = 4;
pub const TC_R: i32 = 5;
pub const TC_S: i32 = 6;
pub const TC_B: i32 = 7;
pub const TC_N: i32 = 8;

// ---------------------------------------------------------------------------
// TempScale
// ---------------------------------------------------------------------------

pub const TS_CELSIUS: i32 = 1;
pub const TS_FAHRENHEIT: i32 = 2;
pub const TS_KELVIN: i32 = 3;
pub const TS_VOLTS: i32 = 4;
pub const TS_NOSCALE: i32 = 5;

// ---------------------------------------------------------------------------
// Range
// ---------------------------------------------------------------------------

pub const BIP60VOLTS: i32 = 1;
pub const BIP30VOLTS: i32 = 2;
pub const BIP15VOLTS: i32 = 3;
pub const BIP20VOLTS: i32 = 4;
pub const BIP10VOLTS: i32 = 5;
pub const BIP5VOLTS: i32 = 6;
pub const BIP4VOLTS: i32 = 7;
pub const BIP2PT5VOLTS: i32 = 8;
pub const BIP2VOLTS: i32 = 9;
pub const BIP1PT25VOLTS: i32 = 10;
pub const BIP1VOLTS: i32 = 11;
pub const BIPPT625VOLTS: i32 = 12;
pub const BIPPT5VOLTS: i32 = 13;
pub const BIPPT25VOLTS: i32 = 14;
pub const BIPPT125VOLTS: i32 = 15;
pub const BIPPT2VOLTS: i32 = 16;
pub const BIPPT1VOLTS: i32 = 17;
pub const BIPPT078VOLTS: i32 = 18;
pub const BIPPT05VOLTS: i32 = 19;
pub const BIPPT01VOLTS: i32 = 20;
pub const BIPPT005VOLTS: i32 = 21;
pub const BIP3VOLTS: i32 = 22;
pub const BIPPT312VOLTS: i32 = 23;
pub const BIPPT156VOLTS: i32 = 24;

pub const UNI10VOLTS: i32 = 1005;
pub const UNI5VOLTS: i32 = 1006;

// ---------------------------------------------------------------------------
// DigitalPortType
// ---------------------------------------------------------------------------

pub const AUXPORT: i32 = 1;
pub const AUXPORT0: i32 = 1;
pub const AUXPORT1: i32 = 2;
pub const AUXPORT2: i32 = 3;
pub const FIRSTPORTA: i32 = 10;
pub const FIRSTPORTB: i32 = 11;
pub const FIRSTPORTCL: i32 = 12;
pub const FIRSTPORTCH: i32 = 13;

// ---------------------------------------------------------------------------
// DigitalDirection
// ---------------------------------------------------------------------------

pub const DD_INPUT: i32 = 1;
pub const DD_OUTPUT: i32 = 2;

// ---------------------------------------------------------------------------
// TmrIdleState
// ---------------------------------------------------------------------------

pub const TMRIS_LOW: i32 = 1;
pub const TMRIS_HIGH: i32 = 2;

// ---------------------------------------------------------------------------
// PulseOutOption
// ---------------------------------------------------------------------------

pub const PO_DEFAULT: i32 = 0;
pub const PO_EXTTRIGGER: i32 = 1 << 5;
pub const PO_RETRIGGER: i32 = 1 << 6;

// ---------------------------------------------------------------------------
// ScanOption (bitmask)
// ---------------------------------------------------------------------------

pub const SO_DEFAULTIO: i32 = 0;
pub const SO_SINGLEIO: i32 = 1 << 0;
pub const SO_BLOCKIO: i32 = 1 << 1;
pub const SO_BURSTIO: i32 = 1 << 2;
pub const SO_CONTINUOUS: i32 = 1 << 3;
pub const SO_EXTCLOCK: i32 = 1 << 4;
pub const SO_EXTTRIGGER: i32 = 1 << 5;
pub const SO_RETRIGGER: i32 = 1 << 6;
pub const SO_BURSTMODE: i32 = 1 << 7;
pub const SO_PACEROUT: i32 = 1 << 8;

// ---------------------------------------------------------------------------
// Scan flags
// ---------------------------------------------------------------------------

pub const AINSCAN_FF_DEFAULT: i32 = 0;
pub const AINSCAN_FF_NOSCALEDATA: i32 = 1 << 0;

pub const AIN_FF_DEFAULT: i32 = 0;
pub const AIN_FF_NOSCALEDATA: i32 = 1 << 0;

pub const AOUTSCAN_FF_DEFAULT: i32 = 0;
pub const AOUTSCAN_FF_NOSCALEDATA: i32 = 1 << 0;

pub const AOUT_FF_DEFAULT: i32 = 0;
pub const AOUT_FF_NOSCALEDATA: i32 = 1 << 0;

pub const AOUTARRAY_FF_DEFAULT: i32 = 0;
pub const AOUTARRAY_FF_NOSCALEDATA: i32 = 1 << 0;
pub const AOUTARRAY_FF_SIMULTANEOUS: i32 = 1 << 2;

pub const TIN_FF_DEFAULT: i32 = 0;
pub const TIN_FF_WAIT_FOR_NEW_DATA: i32 = 1;

pub const CINSCAN_FF_DEFAULT: i32 = 0;
pub const CINSCAN_FF_CTR16_BIT: i32 = 1 << 0;
pub const CINSCAN_FF_CTR32_BIT: i32 = 1 << 1;
pub const CINSCAN_FF_CTR64_BIT: i32 = 1 << 2;
pub const CINSCAN_FF_NOCLEAR: i32 = 1 << 3;

pub const DAQINSCAN_FF_DEFAULT: i32 = 0;
pub const DAQINSCAN_FF_NOSCALEDATA: i32 = 1 << 0;
pub const DAQINSCAN_FF_NOCLEAR: i32 = 1 << 3;

// ---------------------------------------------------------------------------
// Counter measurement
// ---------------------------------------------------------------------------

pub const CMT_COUNT: i32 = 1 << 0;

pub const CMM_DEFAULT: i32 = 0;
pub const CMM_CLEAR_ON_READ: i32 = 1 << 0;
pub const CMM_COUNT_DOWN: i32 = 1 << 1;
pub const CMM_OUTPUT_ON: i32 = 1 << 5;
pub const CMM_OUTPUT_INITIAL_STATE_HIGH: i32 = 1 << 6;
pub const CMM_NO_RECYCLE: i32 = 1 << 7;
pub const CMM_RANGE_LIMIT_ON: i32 = 1 << 8;
pub const CMM_GATING_ON: i32 = 1 << 9;
pub const CMM_INVERT_GATE: i32 = 1 << 10;

pub const CED_RISING_EDGE: i32 = 1;
pub const CED_FALLING_EDGE: i32 = 2;

pub const CTS_TICK_20PT83ns: i32 = 1;
pub const CTS_TICK_208PT3ns: i32 = 2;
pub const CTS_TICK_2083PT3ns: i32 = 3;
pub const CTS_TICK_20833PT3ns: i32 = 4;

pub const CDM_NONE: i32 = 0;

pub const CDT_DEBOUNCE_0ns: i32 = 0;

pub const CF_DEFAULT: i32 = 0;

// Counter register types (bitmask)
pub const CRT_COUNT: i32 = 1 << 0;
pub const CRT_LOAD: i32 = 1 << 1;
pub const CRT_MIN_LIMIT: i32 = 1 << 2;
pub const CRT_MAX_LIMIT: i32 = 1 << 3;
pub const CRT_OUTPUT_VAL0: i32 = 1 << 4;
pub const CRT_OUTPUT_VAL1: i32 = 1 << 5;

// ---------------------------------------------------------------------------
// DaqInChanType (bitmask)
// ---------------------------------------------------------------------------

pub const DAQI_ANALOG_DIFF: u32 = 1 << 0;
pub const DAQI_ANALOG_SE: u32 = 1 << 1;
pub const DAQI_DIGITAL: u32 = 1 << 2;
pub const DAQI_CTR16: u32 = 1 << 3;
pub const DAQI_CTR32: u32 = 1 << 4;
pub const DAQI_CTR48: u32 = 1 << 5;

// ---------------------------------------------------------------------------
// TriggerType
// ---------------------------------------------------------------------------

pub const TRIG_NONE: i32 = 0;
pub const TRIG_POS_EDGE: i32 = 1 << 0;
pub const TRIG_NEG_EDGE: i32 = 1 << 1;
pub const TRIG_HIGH: i32 = 1 << 2;
pub const TRIG_LOW: i32 = 1 << 3;
pub const TRIG_RISING: i32 = 1 << 6;
pub const TRIG_FALLING: i32 = 1 << 7;

// ---------------------------------------------------------------------------
// AoConfigItem / AoSyncMode
// ---------------------------------------------------------------------------

pub const AO_CFG_SYNC_MODE: i32 = 1;
pub const AOSM_SLAVE: i64 = 0;
pub const AOSM_MASTER: i64 = 1;

// ---------------------------------------------------------------------------
// AiConfigItem
// ---------------------------------------------------------------------------

pub const AI_CFG_CHAN_TYPE: i32 = 1;
pub const AI_CFG_CHAN_TC_TYPE: i32 = 2;
pub const AI_CFG_CHAN_SENSOR_CONNECTION_TYPE: i32 = 10;
pub const AI_CFG_CHAN_OTD_MODE: i32 = 11;

// AiConfigItemDbl
pub const AI_CFG_CHAN_DATA_RATE: i32 = 1003;

// ---------------------------------------------------------------------------
// DevConfigItemStr
// ---------------------------------------------------------------------------

pub const DEV_CFG_VER_STR: i32 = 2000;
pub const DEV_CFG_IP_ADDR_STR: i32 = 2001;

// DevVersionType (used as index for DEV_CFG_VER_STR)
pub const DEV_VER_FW_MAIN: u32 = 0;

// ---------------------------------------------------------------------------
// UlInfoItemStr
// ---------------------------------------------------------------------------

pub const UL_INFO_VER_STR: i32 = 2000;

// ---------------------------------------------------------------------------
// AiInfoItem
// ---------------------------------------------------------------------------

pub const AI_INFO_RESOLUTION: i32 = 1;
pub const AI_INFO_NUM_CHANS: i32 = 2;
pub const AI_INFO_NUM_CHANS_BY_TYPE: i32 = 4;

// AoInfoItem
pub const AO_INFO_NUM_CHANS: i32 = 2;
pub const AO_INFO_RESOLUTION: i32 = 1;

// DioInfoItem
pub const DIO_INFO_NUM_PORTS: i32 = 1;
pub const DIO_INFO_NUM_BITS: i32 = 4;

// OTD mode
pub const OTD_DISABLED: i64 = 1;
pub const OTD_ENABLED: i64 = 2;

// ---------------------------------------------------------------------------
// extern "C" function declarations
// ---------------------------------------------------------------------------

unsafe extern "C" {
    // -- Device discovery & connection --

    pub fn ulGetDaqDeviceInventory(
        interface_types: u32,
        descriptors: *mut DaqDeviceDescriptor,
        num_descriptors: *mut u32,
    ) -> UlError;

    pub fn ulCreateDaqDevice(descriptor: DaqDeviceDescriptor) -> DaqDeviceHandle;

    pub fn ulConnectDaqDevice(handle: DaqDeviceHandle) -> UlError;

    pub fn ulDisconnectDaqDevice(handle: DaqDeviceHandle) -> UlError;

    pub fn ulReleaseDaqDevice(handle: DaqDeviceHandle) -> UlError;

    pub fn ulGetNetDaqDeviceDescriptor(
        host: *const c_char,
        port: u16,
        ifc_name: *const c_char,
        descriptor: *mut DaqDeviceDescriptor,
        timeout: f64,
    ) -> UlError;

    // -- Device info --

    pub fn ulDevGetConfigStr(
        handle: DaqDeviceHandle,
        config_item: i32,
        index: u32,
        config_str: *mut c_char,
        max_len: *mut u32,
    ) -> UlError;

    pub fn ulGetInfoStr(
        info_item: i32,
        index: u32,
        info_str: *mut c_char,
        max_len: *mut u32,
    ) -> UlError;

    pub fn ulGetErrMsg(err_code: UlError, err_msg: *mut c_char) -> UlError;

    pub fn ulAIGetInfo(
        handle: DaqDeviceHandle,
        info_item: i32,
        index: u32,
        info_value: *mut i64,
    ) -> UlError;

    pub fn ulAOGetInfo(
        handle: DaqDeviceHandle,
        info_item: i32,
        index: u32,
        info_value: *mut i64,
    ) -> UlError;

    pub fn ulDIOGetInfo(
        handle: DaqDeviceHandle,
        info_item: i32,
        index: u32,
        info_value: *mut i64,
    ) -> UlError;

    // -- AI config --

    pub fn ulAISetConfig(
        handle: DaqDeviceHandle,
        config_item: i32,
        index: u32,
        config_value: i64,
    ) -> UlError;

    pub fn ulAISetConfigDbl(
        handle: DaqDeviceHandle,
        config_item: i32,
        index: u32,
        config_value: f64,
    ) -> UlError;

    // -- AO config --

    pub fn ulAOSetConfig(
        handle: DaqDeviceHandle,
        config_item: i32,
        index: u32,
        config_value: i64,
    ) -> UlError;

    // -- Digital I/O --

    pub fn ulDIn(handle: DaqDeviceHandle, port_type: i32, data: *mut u64) -> UlError;

    pub fn ulDOut(handle: DaqDeviceHandle, port_type: i32, data: u64) -> UlError;

    pub fn ulDBitOut(
        handle: DaqDeviceHandle,
        port_type: i32,
        bit_num: i32,
        bit_value: u32,
    ) -> UlError;

    pub fn ulDConfigPort(handle: DaqDeviceHandle, port_type: i32, direction: i32) -> UlError;

    pub fn ulDConfigBit(
        handle: DaqDeviceHandle,
        port_type: i32,
        bit_num: i32,
        direction: i32,
    ) -> UlError;

    // -- Counter --

    pub fn ulCIn(handle: DaqDeviceHandle, counter_num: i32, data: *mut u64) -> UlError;

    pub fn ulCLoad(
        handle: DaqDeviceHandle,
        counter_num: i32,
        register_type: i32,
        load_value: u64,
    ) -> UlError;

    pub fn ulCClear(handle: DaqDeviceHandle, counter_num: i32) -> UlError;

    pub fn ulCConfigScan(
        handle: DaqDeviceHandle,
        counter_num: i32,
        measurement_type: i32,
        measurement_mode: i32,
        edge_detection: i32,
        tick_size: i32,
        debounce_mode: i32,
        debounce_time: i32,
        flags: i32,
    ) -> UlError;

    pub fn ulCInScan(
        handle: DaqDeviceHandle,
        low_counter: i32,
        high_counter: i32,
        samples_per_counter: i32,
        rate: *mut f64,
        options: i32,
        flags: i32,
        data: *mut u64,
    ) -> UlError;

    pub fn ulCInScanStatus(
        handle: DaqDeviceHandle,
        status: *mut i32,
        xfer_status: *mut TransferStatus,
    ) -> UlError;

    pub fn ulCInScanStop(handle: DaqDeviceHandle) -> UlError;

    // -- Timer / Pulse output --

    pub fn ulTmrPulseOutStart(
        handle: DaqDeviceHandle,
        timer_num: i32,
        frequency: *mut f64,
        duty_cycle: *mut f64,
        pulse_count: u64,
        initial_delay: *mut f64,
        idle_state: i32,
        options: i32,
    ) -> UlError;

    pub fn ulTmrPulseOutStop(handle: DaqDeviceHandle, timer_num: i32) -> UlError;

    // -- Analog input --

    pub fn ulAIn(
        handle: DaqDeviceHandle,
        channel: i32,
        input_mode: i32,
        range: i32,
        flags: i32,
        data: *mut f64,
    ) -> UlError;

    pub fn ulTIn(
        handle: DaqDeviceHandle,
        channel: i32,
        scale: i32,
        flags: i32,
        data: *mut f64,
    ) -> UlError;

    pub fn ulAInScan(
        handle: DaqDeviceHandle,
        low_chan: i32,
        high_chan: i32,
        input_mode: i32,
        range: i32,
        samples_per_chan: i32,
        rate: *mut f64,
        options: i32,
        flags: i32,
        data: *mut f64,
    ) -> UlError;

    pub fn ulAInScanStatus(
        handle: DaqDeviceHandle,
        status: *mut i32,
        xfer_status: *mut TransferStatus,
    ) -> UlError;

    pub fn ulAInScanStop(handle: DaqDeviceHandle) -> UlError;

    pub fn ulAInLoadQueue(
        handle: DaqDeviceHandle,
        queue: *const AiQueueElement,
        num_elements: u32,
    ) -> UlError;

    pub fn ulAInSetTrigger(
        handle: DaqDeviceHandle,
        trig_type: i32,
        trig_chan: i32,
        level: f64,
        variance: f64,
        retrigger_sample_count: u32,
    ) -> UlError;

    // -- Analog output --

    pub fn ulAOut(
        handle: DaqDeviceHandle,
        channel: i32,
        range: i32,
        flags: i32,
        data: f64,
    ) -> UlError;

    pub fn ulAOutArray(
        handle: DaqDeviceHandle,
        low_chan: i32,
        high_chan: i32,
        range: *const i32,
        flags: i32,
        data: *mut f64,
    ) -> UlError;

    pub fn ulAOutScan(
        handle: DaqDeviceHandle,
        low_chan: i32,
        high_chan: i32,
        range: i32,
        samples_per_chan: i32,
        rate: *mut f64,
        options: i32,
        flags: i32,
        data: *mut f64,
    ) -> UlError;

    pub fn ulAOutScanStatus(
        handle: DaqDeviceHandle,
        status: *mut i32,
        xfer_status: *mut TransferStatus,
    ) -> UlError;

    pub fn ulAOutScanStop(handle: DaqDeviceHandle) -> UlError;

    // -- DAQ input scan (MCS) --

    pub fn ulDaqInScan(
        handle: DaqDeviceHandle,
        chan_descriptors: *const DaqInChanDescriptor,
        num_chans: i32,
        samples_per_chan: i32,
        rate: *mut f64,
        options: i32,
        flags: i32,
        data: *mut f64,
    ) -> UlError;

    pub fn ulDaqInScanStatus(
        handle: DaqDeviceHandle,
        status: *mut i32,
        xfer_status: *mut TransferStatus,
    ) -> UlError;

    pub fn ulDaqInScanStop(handle: DaqDeviceHandle) -> UlError;

    pub fn ulDaqInSetTrigger(
        handle: DaqDeviceHandle,
        trig_type: i32,
        trig_chan_descriptor: DaqInChanDescriptor,
        level: f64,
        variance: f64,
        retrigger_sample_count: u32,
    ) -> UlError;
}
