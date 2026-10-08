//! Port of ADGenICam `ADGenICam.cpp`: the areaDetector base driver that maps
//! GenICam features onto asyn parameters created on demand from a record's
//! drvInfo string.

use std::sync::Arc;

use asyn_rs::error::{AsynError, AsynResult};
use asyn_rs::param::{EnumEntry, ParamType};
use asyn_rs::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase};
use asyn_rs::trace::TraceMask;
use asyn_rs::user::AsynUser;

use ad_core_rs::driver::{ADDriver, ADDriverBase};

use crate::feature::{FeatureIndex, GCFeatureType, GenICamFeature, GenICamFeatureSet, GenICamNode};

const DRIVER_NAME: &str = "ADGenICam";

/// Parameters ADGenICam adds on top of ADDriver.
#[derive(Clone, Copy, Debug)]
pub struct GCParams {
    pub frame_rate: usize,
    pub frame_rate_enable: usize,
    pub trigger_source: usize,
    pub trigger_overlap: usize,
    pub trigger_software: usize,
    pub exposure_mode: usize,
    pub exposure_auto: usize,
    pub gain_auto: usize,
    pub pixel_format: usize,
}

impl GCParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            frame_rate: base.create_param("GC_FRAMERATE", ParamType::Float64)?,
            frame_rate_enable: base.create_param("GC_FRAMERATE_ENABLE", ParamType::Int32)?,
            trigger_source: base.create_param("GC_TRIGGER_SOURCE", ParamType::Int32)?,
            trigger_overlap: base.create_param("GC_TRIGGER_OVERLAP", ParamType::Int32)?,
            trigger_software: base.create_param("GC_TRIGGER_SOFTWARE", ParamType::Int32)?,
            exposure_mode: base.create_param("GC_EXPOSURE_MODE", ParamType::Int32)?,
            exposure_auto: base.create_param("GC_EXPOSURE_AUTO", ParamType::Int32)?,
            gain_auto: base.create_param("GC_GAIN_AUTO", ParamType::Int32)?,
            pixel_format: base.create_param("GC_PIXEL_FORMAT", ParamType::Int32)?,
        })
    }
}

/// The driver state a backend works on while the port actor holds it.
pub struct DriverCtx<'a> {
    pub ad: &'a mut ADDriverBase,
    pub gc: &'a GCParams,
    pub features: &'a mut GenICamFeatureSet,
}

/// What a camera SDK supplies: C ADGenICam's pure virtuals plus the
/// `asynPortDriver` methods a derived driver overrides.
pub trait GenICamBackend: Send + Sync + 'static {
    fn create_node(
        &mut self,
        feature_name: &str,
        feature_type: GCFeatureType,
    ) -> Box<dyn GenICamNode>;

    fn start_capture(&mut self, ctx: &mut DriverCtx<'_>) -> AsynResult<()>;

    fn stop_capture(&mut self, ctx: &mut DriverCtx<'_>) -> AsynResult<()>;

    /// Called before the port is marked connected. Returns true when the
    /// camera connection was (re)established by this call, so that every
    /// feature is read again.
    fn connect(&mut self, _ctx: &mut DriverCtx<'_>) -> AsynResult<bool> {
        Ok(false)
    }

    /// The port is being disconnected: release the camera. `connect` brings
    /// it back.
    fn disconnect(&mut self, _ctx: &mut DriverCtx<'_>) {}

    /// Called after an int32 parameter was stored, for the backend's own
    /// parameters.
    fn int32_written(&mut self, _ctx: &mut DriverCtx<'_>, _reason: usize, _value: i32) {}

    /// True for an enum feature whose records keep the choices of their
    /// database instead of the camera's.
    fn keep_db_enum(&self, _feature_name: &str) -> bool {
        false
    }

    fn report(&self, _out: &mut dyn std::fmt::Write, _level: i32) {}
}

pub struct ADGenICam<B: GenICamBackend> {
    pub ad: ADDriverBase,
    pub gc: GCParams,
    pub features: GenICamFeatureSet,
    pub backend: B,
    first_drv_user_create: bool,
    was_acquiring: bool,
}

struct StdParam {
    index: usize,
    feature_name: &'static str,
    feature_type: GCFeatureType,
}

impl<B: GenICamBackend> ADGenICam<B> {
    /// `make_backend` runs once the ADDriver and ADGenICam parameters exist,
    /// so the backend can create its own after them. The `FeatureIndex` lets
    /// the backend's own threads find the parameter a feature is bound to.
    pub fn new(
        port_name: &str,
        max_memory: usize,
        make_backend: impl FnOnce(&mut ADDriverBase, FeatureIndex) -> AsynResult<B>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, 0, 0, max_memory)?;
        let gc = GCParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_int32_param(ad.params.base.data_type, 0, 1)?; // NDUInt8
        base.set_int32_param(ad.params.base.color_mode, 0, 0)?; // NDColorModeMono
        base.set_int32_param(ad.params.base.array_size_z, 0, 0)?;
        base.set_int32_param(ad.params.min_x, 0, 0)?;
        base.set_int32_param(ad.params.min_y, 0, 0)?;
        base.set_string_param(ad.params.string_to_server, 0, "<not used by driver>")?;
        base.set_string_param(ad.params.string_from_server, 0, "<not used by driver>")?;

        let features = GenICamFeatureSet::new();
        let backend = make_backend(&mut ad, features.index())?;
        Ok(Self {
            ad,
            gc,
            features,
            backend,
            first_drv_user_create: true,
            was_acquiring: false,
        })
    }

    fn split(&mut self) -> (&mut B, DriverCtx<'_>) {
        (
            &mut self.backend,
            DriverCtx {
                ad: &mut self.ad,
                gc: &self.gc,
                features: &mut self.features,
            },
        )
    }

    fn start_capture(&mut self) -> AsynResult<()> {
        let (backend, mut ctx) = self.split();
        backend.start_capture(&mut ctx)
    }

    fn stop_capture(&mut self) -> AsynResult<()> {
        let (backend, mut ctx) = self.split();
        backend.stop_capture(&mut ctx)
    }

    fn pause_acquisition(&mut self) -> AsynResult<()> {
        let acquiring = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.acquire, 0)?;
        self.was_acquiring = acquiring != 0;
        if self.was_acquiring {
            self.stop_capture()?;
        }
        Ok(())
    }

    fn resume_acquisition(&mut self) -> AsynResult<()> {
        if self.was_acquiring {
            self.start_capture()?;
        }
        Ok(())
    }

    fn set_image_params(&mut self) -> AsynResult<()> {
        let p = self.ad.params;
        let indices = [p.size_x, p.size_y, p.min_x, p.min_y, p.bin_x, p.bin_y];
        self.pause_acquisition()?;
        let modes = self.features.modes;
        for index in indices {
            if let Some(f) = self.features.get_by_index(index) {
                // On some cameras some features will not be writeable
                if f.node().is_writable() {
                    f.write(&mut self.ad.port_base, &modes, None, false);
                }
            }
        }
        for index in indices {
            if let Some(f) = self.features.get_by_index(index) {
                f.read(&mut self.ad.port_base, &modes, true);
            }
        }
        self.resume_acquisition()
    }

    fn read_status(&mut self) -> AsynResult<()> {
        self.features.read_all(&mut self.ad.port_base);
        self.ad.port_base.call_param_callbacks(0)
    }

    /// Write `value` to the feature bound to `reason`, if any, then read every
    /// feature back: a GenICam write can move the value, range or availability
    /// of any other node.
    fn write_feature(&mut self, reason: usize, value: f64) {
        let modes = self.features.modes;
        let Some(f) = self.features.get_by_index(reason) else {
            return;
        };
        f.write(&mut self.ad.port_base, &modes, Some(value), true);
        self.features.read_all(&mut self.ad.port_base);
    }

    fn create_feature(
        &mut self,
        asyn_name: &str,
        asyn_type: ParamType,
        asyn_index: Option<usize>,
        feature_name: &str,
        feature_type: GCFeatureType,
    ) -> AsynResult<GenICamFeature> {
        let asyn_index = match asyn_index {
            Some(i) => i,
            None => self.ad.port_base.create_param(asyn_name, asyn_type)?,
        };
        let node = self.backend.create_node(feature_name, feature_type);
        Ok(GenICamFeature::new(
            asyn_name,
            asyn_type,
            asyn_index,
            feature_name,
            node,
        ))
    }

    /// Insert `feature` and read it once so that EPICS output records
    /// initialize to the camera's value.
    fn adopt(&mut self, mut feature: GenICamFeature) {
        let modes = self.features.modes;
        feature.read(&mut self.ad.port_base, &modes, true);
        self.features.insert(feature);
    }

    /// Bind one asyn parameter to the first implemented feature of `candidates`.
    fn create_multi_feature(
        &mut self,
        asyn_index: usize,
        candidates: &[(&str, GCFeatureType)],
    ) -> AsynResult<()> {
        let asyn_name = self.param_name(asyn_index)?;
        let asyn_type = self.param_type(asyn_index)?;
        for (feature_name, feature_type) in candidates {
            let f = self.create_feature(
                &asyn_name,
                asyn_type,
                Some(asyn_index),
                feature_name,
                *feature_type,
            )?;
            if f.node().is_implemented() {
                self.adopt(f);
                return Ok(());
            }
        }
        Ok(())
    }

    fn param_name(&self, index: usize) -> AsynResult<String> {
        self.ad
            .port_base
            .params
            .param_name(index)
            .map(str::to_string)
            .ok_or_else(|| AsynError::ParamNotFound(index.to_string()))
    }

    fn param_type(&self, index: usize) -> AsynResult<ParamType> {
        self.ad
            .port_base
            .params
            .param_type(index)
            .ok_or_else(|| AsynError::ParamNotFound(index.to_string()))
    }

    /// Map the standard ADDriver parameters onto GenICam features.
    pub fn add_ad_driver_features(&mut self) -> AsynResult<()> {
        use GCFeatureType::*;
        let p = self.ad.params;
        let gc = self.gc;
        let params = [
            StdParam {
                index: p.image_mode,
                feature_name: "AcquisitionMode",
                feature_type: Enum,
            },
            StdParam {
                index: p.base.manufacturer,
                feature_name: "DeviceVendorName",
                feature_type: String,
            },
            StdParam {
                index: p.base.model,
                feature_name: "DeviceModelName",
                feature_type: String,
            },
            StdParam {
                index: p.max_size_x,
                feature_name: "WidthMax",
                feature_type: Integer,
            },
            StdParam {
                index: p.max_size_y,
                feature_name: "HeightMax",
                feature_type: Integer,
            },
            StdParam {
                index: p.size_x,
                feature_name: "Width",
                feature_type: Integer,
            },
            StdParam {
                index: p.size_y,
                feature_name: "Height",
                feature_type: Integer,
            },
            StdParam {
                index: p.min_x,
                feature_name: "OffsetX",
                feature_type: Integer,
            },
            StdParam {
                index: p.min_y,
                feature_name: "OffsetY",
                feature_type: Integer,
            },
            StdParam {
                index: p.bin_x,
                feature_name: "BinningHorizontal",
                feature_type: Integer,
            },
            StdParam {
                index: p.bin_y,
                feature_name: "BinningVertical",
                feature_type: Integer,
            },
            StdParam {
                index: p.num_images,
                feature_name: "AcquisitionFrameCount",
                feature_type: Integer,
            },
            StdParam {
                index: p.gain,
                feature_name: "Gain",
                feature_type: Double,
            },
            StdParam {
                index: p.trigger_mode,
                feature_name: "TriggerMode",
                feature_type: Enum,
            },
            StdParam {
                index: gc.trigger_source,
                feature_name: "TriggerSource",
                feature_type: Enum,
            },
            StdParam {
                index: gc.trigger_overlap,
                feature_name: "TriggerOverlap",
                feature_type: Enum,
            },
            StdParam {
                index: gc.trigger_software,
                feature_name: "TriggerSoftware",
                feature_type: Cmd,
            },
            StdParam {
                index: gc.exposure_mode,
                feature_name: "ExposureMode",
                feature_type: Enum,
            },
            StdParam {
                index: gc.exposure_auto,
                feature_name: "ExposureAuto",
                feature_type: Enum,
            },
            StdParam {
                index: gc.gain_auto,
                feature_name: "GainAuto",
                feature_type: Enum,
            },
            StdParam {
                index: gc.pixel_format,
                feature_name: "PixelFormat",
                feature_type: Enum,
            },
        ];

        for sp in params {
            let asyn_name = self.param_name(sp.index)?;
            let asyn_type = self.param_type(sp.index)?;
            let f = self.create_feature(
                &asyn_name,
                asyn_type,
                Some(sp.index),
                sp.feature_name,
                sp.feature_type,
            )?;
            if !f.node().is_implemented() {
                continue;
            }
            // areaDetector ImageMode maps to GenICam AcquisitionMode, whose
            // enum strings are consistent across cameras but whose values are
            // not.
            if sp.index == p.image_mode {
                for (s, v) in f.node().read_enum_choices().unwrap_or_default() {
                    match s.as_str() {
                        "Continuous" => self.features.modes.continuous = Some(v),
                        "MultiFrame" => self.features.modes.multi_frame = Some(v),
                        "SingleFrame" => self.features.modes.single_frame = Some(v),
                        _ => {}
                    }
                }
            }
            self.adopt(f);
        }

        self.create_multi_feature(
            gc.frame_rate_enable,
            &[
                ("AcquisitionFrameRateEnable", Boolean),
                ("AcquisitionFrameRateEnabled", Boolean),
            ],
        )?;
        self.create_multi_feature(
            p.acquire_time,
            &[("ExposureTime", Double), ("ExposureTimeAbs", Double)],
        )?;
        let frame_rate = [
            ("AcquisitionFrameRate", Double),
            ("AcquisitionFrameRateAbs", Double),
        ];
        self.create_multi_feature(gc.frame_rate, &frame_rate)?;
        self.create_multi_feature(p.acquire_period, &frame_rate)?;
        // DeviceID is used by AVT
        self.create_multi_feature(
            p.base.serial_number,
            &[("DeviceSerialNumber", String), ("DeviceID", String)],
        )?;
        // DeviceVersion is used by Mikrotron
        self.create_multi_feature(
            p.base.firmware_version,
            &[("DeviceFirmwareVersion", String), ("DeviceVersion", String)],
        )?;
        self.create_multi_feature(
            p.gain,
            &[
                ("Gain", Double),
                ("GainRaw", Integer),
                ("GainRawChannelA", Integer),
            ],
        )?;
        self.create_multi_feature(
            p.temperature_actual,
            &[("DeviceTemperature", Double), ("TemperatureAbs", Double)],
        )?;
        Ok(())
    }

    /// Print one feature in full; the body of iocsh `genicamShowFeature`.
    pub fn show_feature(&mut self, feature_name: &str) -> String {
        let mut out = String::new();
        match self.features.get_by_name(feature_name) {
            Some(f) => f.report(&mut out, 2),
            None => self.ad.port_base.trace_print(
                TraceMask::ERROR,
                &format!("{DRIVER_NAME}::showFeature cannot find feature {feature_name}\n"),
            ),
        }
        out
    }
}

/// Parse `YY_X_name` drvInfo: YY is `GC` for camera features, X the feature
/// type letter, name the GenICam feature name.
fn parse_feature_drv_info(drv_info: &str) -> Option<(GCFeatureType, ParamType, &str)> {
    let b = drv_info.as_bytes();
    if b.len() <= 5 || b[2] != b'_' || b[4] != b'_' {
        return None;
    }
    let (feature_type, asyn_type) = match b[3] {
        b'B' => (GCFeatureType::Boolean, ParamType::Int32),
        b'C' => (GCFeatureType::Cmd, ParamType::Int32),
        b'D' => (GCFeatureType::Double, ParamType::Float64),
        b'E' => (GCFeatureType::Enum, ParamType::Int32),
        b'I' => (GCFeatureType::Integer, ParamType::Int64),
        b'S' => (GCFeatureType::String, ParamType::Octet),
        _ => return None,
    };
    Some((feature_type, asyn_type, drv_info.get(5..)?))
}

impl<B: GenICamBackend> PortDriver for ADGenICam<B> {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.ad.params;
        let acquiring = self.ad.port_base.get_int32_param(p.acquire, 0)? != 0;
        let mut status = Ok(());

        // Set the value in the parameter library. This may change later.
        if reason == p.acquire {
            self.ad.set_acquire(value)?;
        } else {
            self.ad.port_base.set_int32_param(reason, 0, value)?;
        }

        if reason < self.gc.frame_rate {
            if reason == p.shutter_control {
                self.ad.set_shutter(value != 0)?;
            } else {
                self.ad.write_int32_pool(reason, value)?;
            }
        }

        if reason == p.acquire {
            if value != 0 && !acquiring {
                status = self.start_capture();
            } else if value == 0 && acquiring {
                status = self.stop_capture();
            }
        } else if [p.size_x, p.size_y, p.min_x, p.min_y, p.bin_x, p.bin_y].contains(&reason) {
            status = self.set_image_params();
        } else if reason == p.read_status {
            status = self.read_status();
        }

        let pauses = reason == self.gc.pixel_format || reason == p.num_images;
        if pauses {
            self.pause_acquisition()?;
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        self.write_feature(reason, value as f64);
        if pauses {
            std::thread::sleep(std::time::Duration::from_millis(100));
            self.resume_acquisition()?;
        }

        let (backend, mut ctx) = self.split();
        backend.int32_written(&mut ctx, reason, value);

        self.ad.port_base.call_param_callbacks(0)?;
        status
    }

    fn write_int64(&mut self, user: &mut AsynUser, value: i64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad.port_base.set_int64_param(reason, 0, value)?;
        let modes = self.features.modes;
        if let Some(f) = self.features.get_by_index(reason) {
            // An f64 cannot carry every i64; the integer goes through the
            // parameter just stored instead.
            f.write(&mut self.ad.port_base, &modes, None, true);
            self.features.read_all(&mut self.ad.port_base);
        }
        self.ad.port_base.call_param_callbacks(0)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad.port_base.set_float64_param(reason, 0, value)?;
        self.write_feature(reason, value);
        self.ad.port_base.call_param_callbacks(0)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        self.ad
            .port_base
            .set_string_param(reason, user.addr, data.to_vec())?;
        // C++ `asynNDArrayDriver::writeOctet`, which ADGenICam inherits.
        self.ad
            .write_octet(reason, &String::from_utf8_lossy(data))?;
        // C ADGenICam has no writeOctet, so a string feature never reaches its
        // camera; here it is written like a feature of any other type.
        let modes = self.features.modes;
        if let Some(f) = self.features.get_by_index(reason) {
            f.write(&mut self.ad.port_base, &modes, None, true);
            self.features.read_all(&mut self.ad.port_base);
        }
        self.ad.port_base.call_param_callbacks(user.addr)?;
        Ok(data.len())
    }

    fn read_enum(&mut self, user: &AsynUser) -> AsynResult<(usize, Arc<[EnumEntry]>)> {
        let reason = user.reason;
        let not_generated = || {
            Err(AsynError::ParamNotFound(format!(
                "no enum table for {reason}"
            )))
        };
        // ImageMode keeps the EPICS choices, not the camera's.
        if reason == self.ad.params.image_mode {
            return not_generated();
        }
        let Some(f) = self.features.get_by_index(reason) else {
            return not_generated();
        };
        if !matches!(
            f.feature_type(),
            GCFeatureType::Enum | GCFeatureType::Unknown
        ) || self.backend.keep_db_enum(f.feature_name())
        {
            return not_generated();
        }
        let table = f.enum_table().map_err(|e| AsynError::Status {
            status: asyn_rs::error::AsynStatus::Error,
            message: e,
        })?;
        let current = self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0);
        let index = table.iter().position(|(_, v)| *v == current).unwrap_or(0);
        let entries: Vec<EnumEntry> = table
            .into_iter()
            .map(|(string, value)| EnumEntry {
                string,
                value,
                severity: 0,
            })
            .collect();
        Ok((index, entries.into()))
    }

    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        // The first time this is called, add the standard ADDriver parameters
        // that map to GenICam features.
        if self.first_drv_user_create {
            self.first_drv_user_create = false;
            self.add_ad_driver_features()?;
        }

        if self.ad.port_base.find_param(&req.drv_info).is_none()
            && let Some((feature_type, asyn_type, feature_name)) =
                parse_feature_drv_info(&req.drv_info)
        {
            let f =
                self.create_feature(&req.drv_info, asyn_type, None, feature_name, feature_type)?;
            self.adopt(f);
        }

        let reason = self
            .ad
            .port_base
            .find_param(&req.drv_info)
            .ok_or_else(|| AsynError::ParamNotFound(req.drv_info.clone()))?;
        Ok(DrvUserInfo::from_reason(reason))
    }

    fn connect(&mut self, user: &AsynUser) -> AsynResult<()> {
        let (backend, mut ctx) = self.split();
        if backend.connect(&mut ctx)? {
            // If this is the first successful connection, the ADDriver
            // features could not be created before now.
            if !self.first_drv_user_create && !self.features.has_index(self.ad.params.base.model) {
                self.add_ad_driver_features()?;
            }
            self.features.read_all(&mut self.ad.port_base);
            self.ad.port_base.call_param_callbacks(0)?;
        }
        self.ad.port_base.set_addr_connected(user.addr, true);
        Ok(())
    }

    fn disconnect(&mut self, user: &AsynUser) -> AsynResult<()> {
        let (backend, mut ctx) = self.split();
        backend.disconnect(&mut ctx);
        self.ad.port_base.call_param_callbacks(0)?;
        self.ad.port_base.set_addr_connected(user.addr, false);
        Ok(())
    }

    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        self.backend.report(out, level);
        if level > 0 {
            self.features.report(out, level);
        }
        self.ad.port_base.report_params(out, level);
    }
}

impl<B: GenICamBackend> ADDriver for ADGenICam<B> {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drv_info_letters_pick_the_asyn_type() {
        let (ft, at, name) = parse_feature_drv_info("GC_I_Width").unwrap();
        assert_eq!(
            (ft, at, name),
            (GCFeatureType::Integer, ParamType::Int64, "Width")
        );
        let (ft, at, _) = parse_feature_drv_info("GC_D_Gain").unwrap();
        assert_eq!((ft, at), (GCFeatureType::Double, ParamType::Float64));
        let (ft, at, _) = parse_feature_drv_info("GC_E_PixelFormat").unwrap();
        assert_eq!((ft, at), (GCFeatureType::Enum, ParamType::Int32));
        let (ft, at, _) = parse_feature_drv_info("GC_S_DeviceUserID").unwrap();
        assert_eq!((ft, at), (GCFeatureType::String, ParamType::Octet));
    }

    #[test]
    fn drv_info_that_is_not_a_feature_is_left_alone() {
        assert!(parse_feature_drv_info("ACQUIRE").is_none());
        assert!(parse_feature_drv_info("GC_FRAMERATE").is_none());
        assert!(parse_feature_drv_info("GC_X_Width").is_none());
        assert!(parse_feature_drv_info("GC_I_").is_none());
    }
}
