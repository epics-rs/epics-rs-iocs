//! The ADTimePix3 asyn port driver (port of the `ADTimePix` class,
//! `ADTimePix.cpp` + `serval_http.cpp` + `acquire.cpp` + `mask_io.cpp`).

use std::sync::Arc;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase};
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::EnumEntry;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use serde_json::json;

use crate::adlog;
use crate::family::DetectorFamily;
use crate::http::{HttpError, ServalHttp, TIMEOUT_CONFIG, TIMEOUT_POLL, decode_base64};
use crate::mask::{self, Geometry, PIXEL_CONFIG_BYTES};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Shared};

/// The line C prints when a Serval request fails: `logHttpFailure` or
/// `logHttpWarning` with its context, method and path, or nothing.
enum HttpPrint<'a> {
    None,
    Failure(&'a str, &'a str, &'a str),
    Warning(&'a str, &'a str, &'a str),
}

/// C's `maxAddr` (ADTimePix.cpp:1069): 0 = PrvImg, 1 = Img, 2 = Img running
/// sum, 3 = Img sum-of-N, 4 = PrvHst sum-of-N, 5 = PrvHst running sum,
/// 6 = PrvHst frame, 7 = PrvHst ToF axis. The chip DACs bind addresses 0-7 and
/// the voltage rails 0-5 on the same port.
pub const MAX_ADDR: usize = 8;

/// ADTIMEPIX_VERSION.REVISION.MODIFICATION (ADTimePix.h:20-22).
const DRIVER_VERSION: &str = "1.6.4";

/// C `calculateImgMemoryUsageMB()` before the first frame (ADTimePix.cpp:1545):
/// the 512x512 UINT16 current frame, the 512x512 default for the two Int64 and
/// one Int32 waveforms, plus the fixed 0.1 MB overhead.
const INITIAL_IMG_MEMORY_MB: f64 =
    (512.0 * 512.0 * (2.0 + 8.0 + 8.0 + 4.0)) / (1024.0 * 1024.0) + 0.1;

/// The per-chip DAC names Serval takes on `/detector/chips/<n>/dacs/`
/// (ADTimePix.cpp:1123-1147). These are on-wire names, not display names.
type DacParam = fn(&TimePixParams) -> usize;

const DAC_NAMES: [(&str, DacParam); 18] = [
    ("Ibias_Preamp_ON", |p| p.preamp_on),
    ("Ibias_Preamp_OFF", |p| p.preamp_of_f),
    ("VPreamp_NCAS", |p| p.v_preamp_nc_as),
    ("Ibias_Ikrum", |p| p.ikrum),
    ("Vfbk", |p| p.vfbk),
    ("Vthreshold_fine", |p| p.vthreshold_fine),
    ("Vthreshold_coarse", |p| p.vthreshold_coarse),
    ("Ibias_DiscS1_ON", |p| p.disc_s1_on),
    ("Ibias_DiscS1_OFF", |p| p.disc_s1_of_f),
    ("Ibias_DiscS2_ON", |p| p.disc_s2_on),
    ("Ibias_DiscS2_OFF", |p| p.disc_s2_of_f),
    ("Ibias_PixelDAC", |p| p.pixel_da_c),
    ("Ibias_TPbufferIn", |p| p.t_pbuffer_in),
    ("Ibias_TPbufferOut", |p| p.t_pbuffer_out),
    ("VTP_coarse", |p| p.vt_pcoarse),
    ("VTP_fine", |p| p.vt_pfine),
    ("Ibias_CP_PLL", |p| p.cp_pl_l),
    ("PLL_Vcntrl", |p| p.pl_l_vcntrl),
];

/// The DAC table as `(Serval name, parameter index)` pairs.
pub fn dac_params(p: &TimePixParams) -> Vec<(&'static str, usize)> {
    DAC_NAMES.iter().map(|(name, f)| (*name, f(p))).collect()
}

/// C `ADTimePix3Action::oneShotDecision` (one_shot_action.h): an action
/// parameter runs on 1 only and always reads back 0.
fn one_shot(value: i32) -> bool {
    value == 1
}

/// C `kMpx3BothCountersTriggerMsg` (ADTimePix.cpp:2129-2130).
const MPX3_TRIGGER_MSG: &str =
    "MPX3: BothCounters requires non-Continuous TriggerMode (use AutoTrgSt_TmrSp=4)";
/// C `kMpx3BothCountersPixelDepthMsg` (ADTimePix.cpp:2132-2133).
const MPX3_PIXEL_DEPTH_MSG: &str =
    "MPX3: PixelDepth 24 incompatible with BothCounters (use 12-bit)";

pub struct TimePix3Driver {
    pub ad: ADDriverBase,
    pub p: TimePixParams,
    pub http: Arc<ServalHttp>,
    pub shared: Arc<Shared>,
    cmd_tx: rt::CommandSender<Command>,
}

impl TimePix3Driver {
    pub fn new(
        port_name: &str,
        http: Arc<ServalHttp>,
        shared: Arc<Shared>,
        max_memory: usize,
        cmd_tx: rt::CommandSender<Command>,
    ) -> AsynResult<Self> {
        // ADDriver(portName, 8, ..., ASYN_MULTIDEVICE | ASYN_CANBLOCK, ...),
        // ADTimePix.cpp:1072-1076.
        let mut ad = ADDriverBase::new(
            port_name,
            MAX_ADDR,
            0,
            0,
            max_memory,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
        )?;
        let p = TimePixParams::create(&mut ad.port_base)?;

        let mut driver = Self {
            ad,
            p,
            http,
            shared,
            cmd_tx,
        };
        driver.init_params()?;
        Ok(driver)
    }

    /// C's constructor seeds (ADTimePix.cpp:1414-1553, ADTimePix3 R1-6-4).
    /// Manufacturer and Model come from the dashboard, and the PrvHst sum
    /// parameters from the first histogram frame or write, as in C.
    fn init_params(&mut self) -> AsynResult<()> {
        let server = self.http.base_url().to_string();
        let p = self.p;
        let ad = &self.ad.params;
        let base = &mut self.ad.port_base;
        base.set_string_param(ad.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_string_param(p.server_name, 0, server)?;
        base.set_int32_param(p.img_accumulation_enable, 0, 1)?;
        base.set_int32_param(p.prv_hst_accumulation_enable, 0, 1)?;
        base.set_int32_param(p.img_frames_to_sum, 0, 10)?;
        base.set_int32_param(p.img_sum_update_interval_frames, 0, 1)?;
        base.set_int64_param(p.img_total_counts, 0, 0)?;
        base.set_float64_param(p.img_processing_time, 0, 0.0)?;
        base.set_float64_param(p.img_memory_usage, 0, INITIAL_IMG_MEMORY_MB)?;
        base.set_int32_param(p.write_processed_img, 0, 0)?;
        base.set_int32_param(p.processed_img_output_type, 0, 0)?;
        base.set_int32_param(p.write_processed_hst, 0, 0)?;
        base.set_int32_param(p.processed_hst_output_type, 0, 0)?;
        base.set_int32_param(ad.num_images, 0, 0)?;
        Ok(())
    }

    fn int(&self, index: usize, addr: i32) -> i32 {
        self.ad.port_base.get_int32_param(index, addr).unwrap_or(0)
    }

    fn dbl(&self, index: usize, addr: i32) -> f64 {
        self.ad
            .port_base
            .get_float64_param(index, addr)
            .unwrap_or(0.0)
    }

    fn text(&self, index: usize, addr: i32) -> String {
        String::from_utf8_lossy(
            self.ad
                .port_base
                .get_string_param(index, addr)
                .unwrap_or_default(),
        )
        .into_owned()
    }

    /// Publish the HTTP status of the last request, as C's `TPX3_HTTP_CODE`
    /// does, print the line C prints for it, and turn the failure into an
    /// asyn error.
    fn http_failed(&mut self, what: &str, e: &HttpError, print: HttpPrint) -> AsynError {
        let _ = self
            .ad
            .port_base
            .set_int32_param(self.p.http_code, 0, e.code());
        let url = |path: &str| format!("{}{path}", self.http.base_url());
        let line = match print {
            HttpPrint::None => None,
            HttpPrint::Failure(context, method, path) => {
                adlog::http_failure(context, method, &url(path), e).map(|l| (TraceMask::ERROR, l))
            }
            HttpPrint::Warning(context, method, path) => {
                adlog::http_warning(context, method, &url(path), e).map(|l| (TraceMask::WARNING, l))
            }
        };
        if let Some((mask, line)) = line {
            self.ad
                .port_base
                .trace_print(mask, format_args!("{}", line));
        }
        AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: {what}: {e}"),
        }
    }

    fn http_ok(&mut self) {
        let _ = self.ad.port_base.set_int32_param(self.p.http_code, 0, 200);
    }

    /// C `mpx3BothCountersTriggerConflict` (ADTimePix.cpp:2135-2143).
    fn mpx3_trigger_conflict(&self, trigger_mode: i32) -> bool {
        self.shared.family() == DetectorFamily::Mpx3
            && self.int(self.p.both_counters, 0) != 0
            && trigger_mode == 5
    }

    /// C `mpx3BothCountersPixelDepthConflict` (ADTimePix.cpp:2145-2153).
    fn mpx3_pixel_depth_conflict(&self, pixel_depth: i32) -> bool {
        self.shared.family() == DetectorFamily::Mpx3
            && self.int(self.p.both_counters, 0) != 0
            && pixel_depth == 24
    }

    /// C `LOG`/`LOG_ARGS` in `writeInt32`.
    fn log_write_int32(&self, msg: &str) {
        self.ad.port_base.trace_print(
            TraceMask::IO_DRIVER,
            format_args!("{}", adlog::log(adlog::WRITE_INT32, msg)),
        );
    }

    fn set_text(&mut self, index: usize, text: &str) -> AsynResult<()> {
        self.ad.port_base.set_string_param(index, 0, text)
    }

    // -- Serval requests -----------------------------------------------------

    /// C `initAcquisition` (serval_http.cpp:2254): read `/detector/config`,
    /// merge the driver's settings into it, PUT it back.
    fn init_acquisition(&mut self) -> AsynResult<()> {
        let current = self
            .http
            .get_json(serval::DETECTOR_CONFIG, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /detector/config", &e, HttpPrint::None))?;

        let cfg = serval::DetectorConfig {
            trigger_mode: self.int(self.ad.params.trigger_mode, 0),
            exposure_time: self.dbl(self.ad.params.acquire_time, 0),
            trigger_period: self.dbl(self.ad.params.acquire_period, 0),
            trigger_delay: self.dbl(self.p.trigger_delay, 0),
            global_timestamp_interval: self.dbl(self.p.global_timestamp_interval, 0),
            n_triggers: self.int(self.ad.params.num_images, 0),
            bias_voltage: self.int(self.p.bias_volt, 0),
            bias_enabled: self.int(self.p.bias_enable, 0) != 0,
            chain_mode: self.int(self.p.chain_mode, 0),
            polarity: self.int(self.p.polarity, 0),
            trigger_in: self.int(self.p.trigger_in, 0),
            trigger_out: self.int(self.p.trigger_out, 0),
            log_level: self.int(self.p.log_level, 0),
            external_reference_clock: self.int(self.p.external_reference_clock, 0) != 0,
            periph_clk80: self.int(self.p.periph_clk80, 0) != 0,
            tdc0: self.int(self.p.tdc0, 0),
            tdc1: self.int(self.p.tdc1, 0),
        };
        let body = serval::detector_config_body(&current, &cfg).map_err(|m| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: /detector/config: {m}"),
        })?;
        self.http
            .put_json(serval::DETECTOR_CONFIG, &body, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("PUT /detector/config", &e, HttpPrint::None))?;
        self.http_ok();
        Ok(())
    }

    /// C `sendMeasurementConfig` (serval_http.cpp:2033).
    fn send_measurement_config(&mut self) -> AsynResult<()> {
        let current = self
            .http
            .get_json(serval::MEASUREMENT_CONFIG, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /measurement/config", &e, HttpPrint::None))?;
        let cfg = serval::MeasurementConfig {
            scan_width: self.int(self.p.stem_scan_width, 0),
            scan_height: self.int(self.p.stem_scan_height, 0),
            dwell_time: self.dbl(self.p.stem_dwell_time, 0),
            radius_outer: self.int(self.p.stem_radius_outer, 0),
            radius_inner: self.int(self.p.stem_radius_inner, 0),
            tdc_reference: self.text(self.p.tof_tdc_reference, 0),
            tof_min: self.dbl(self.p.tof_min, 0),
            tof_max: self.dbl(self.p.tof_max, 0),
        };
        let body = serval::measurement_config_body(&current, &cfg);
        self.http
            .put_json(serval::MEASUREMENT_CONFIG, &body, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    "PUT /measurement/config",
                    &e,
                    HttpPrint::Failure(
                        "sendMeasurementConfig PUT /measurement/config",
                        "PUT",
                        serval::MEASUREMENT_CONFIG,
                    ),
                )
            })?;
        self.http_ok();
        Ok(())
    }

    /// The destination the enabled channels describe (C `fileWriter`,
    /// serval_http.cpp:2095).
    fn destination(&self) -> serval::Destination {
        let mut d = serval::Destination::default();

        for (enable, base, pat, split, queue) in [
            (
                self.p.write_raw,
                self.p.raw_base,
                self.p.raw_file_pat,
                self.p.raw_split_strategy,
                self.p.raw_queue_size,
            ),
            (
                self.p.write_raw1,
                self.p.raw1_base,
                self.p.raw1_file_pat,
                self.p.raw1_split_strategy,
                self.p.raw1_queue_size,
            ),
        ] {
            if self.int(enable, 0) != 0 {
                d.raw.push(serval::RawChannel {
                    base: self.text(base, 0),
                    file_pattern: self.text(pat, 0),
                    split_strategy: self.int(split, 0),
                    queue_size: self.int(queue, 0),
                });
            }
        }

        let image = |this: &Self, base, pat, format, mode, int_size, int_mode, stop, queue| {
            serval::ImageChannel {
                base: this.text(base, 0),
                file_pattern: this.text(pat, 0),
                format: this.int(format, 0),
                mode: this.int(mode, 0),
                integration_size: this.int(int_size, 0),
                integration_mode: this.int(int_mode, 0),
                stop_on_disk_limit: this.int(stop, 0) != 0,
                queue_size: this.int(queue, 0),
            }
        };

        if self.int(self.p.write_img, 0) != 0 {
            d.image.push(image(
                self,
                self.p.img_base,
                self.p.img_file_pat,
                self.p.img_format,
                self.p.img_mode,
                self.p.img_int_size,
                self.p.img_int_mode,
                self.p.img_stp_on_dsk_lim,
                self.p.img_queue_size,
            ));
        }
        if self.int(self.p.write_img1, 0) != 0 {
            d.image.push(image(
                self,
                self.p.img1_base,
                self.p.img1_file_pat,
                self.p.img1_format,
                self.p.img1_mode,
                self.p.img1_int_size,
                self.p.img1_int_mode,
                self.p.img1_stp_on_dsk_lim,
                self.p.img1_queue_size,
            ));
        }
        if self.int(self.p.write_prv_img, 0) != 0 {
            d.preview_image.push(image(
                self,
                self.p.prv_img_base,
                self.p.prv_img_file_pat,
                self.p.prv_img_format,
                self.p.prv_img_mode,
                self.p.prv_img_int_size,
                self.p.prv_img_int_mode,
                self.p.prv_img_stp_on_dsk_lim,
                self.p.prv_img_queue_size,
            ));
        }
        if self.int(self.p.write_prv_img1, 0) != 0 {
            d.preview_image.push(image(
                self,
                self.p.prv_img1_base,
                self.p.prv_img1_file_pat,
                self.p.prv_img1_format,
                self.p.prv_img1_mode,
                self.p.prv_img1_int_size,
                self.p.prv_img1_int_mode,
                self.p.prv_img1_stp_on_dsk_lim,
                self.p.prv_img1_queue_size,
            ));
        }
        if self.int(self.p.write_prv_hst, 0) != 0 {
            d.preview_histogram = Some(serval::HistogramChannel {
                image: image(
                    self,
                    self.p.prv_hst_base,
                    self.p.prv_hst_file_pat,
                    self.p.prv_hst_format,
                    self.p.prv_hst_mode,
                    self.p.prv_hst_int_size,
                    self.p.prv_hst_int_mode,
                    self.p.prv_hst_stp_on_dsk_lim,
                    self.p.prv_hst_queue_size,
                ),
                number_of_bins: self.int(self.p.prv_hst_num_bins, 0),
                bin_width: self.dbl(self.p.prv_hst_bin_width, 0),
                offset: self.dbl(self.p.prv_hst_offset, 0),
            });
        }
        d.preview = Some(serval::PreviewSettings {
            period: self.dbl(self.p.prv_period, 0),
            sampling_mode: self.int(self.p.prv_sampling_mode, 0),
        });
        d
    }

    /// C `fileWriter` (serval_http.cpp:2095).
    fn file_writer(&mut self) -> AsynResult<()> {
        let dest = self.destination();
        let body = serval::destination_body(&dest).map_err(|m| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: /server/destination: {m}"),
        })?;
        self.http
            .put_json(serval::SERVER_DESTINATION, &body, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    "PUT /server/destination",
                    &e,
                    HttpPrint::Failure(
                        "sendConfiguration PUT /server/destination",
                        "PUT",
                        serval::SERVER_DESTINATION,
                    ),
                )
            })?;
        self.http_ok();

        // The stream workers follow the Base PVs.
        self.shared.set_stream_paths(
            dest.preview_image.first().map(|c| c.base.clone()),
            dest.image.first().map(|c| c.base.clone()),
            dest.preview_histogram
                .as_ref()
                .map(|h| h.image.base.clone()),
        );
        Ok(())
    }

    /// C `getServer` (serval_http.cpp:1420): read back the configured channels.
    fn get_server(&mut self) -> AsynResult<()> {
        let dest = self
            .http
            .get_json(serval::SERVER_DESTINATION, TIMEOUT_POLL)
            .map_err(|e| self.http_failed("GET /server/destination", &e, HttpPrint::None))?;
        self.http_ok();

        let raw = dest.get("Raw").and_then(|v| v.as_array());
        for (n, param) in [(0, self.p.write_raw_read), (1, self.p.write_raw1_read)] {
            let on = raw.is_some_and(|a| a.len() > n);
            self.ad.port_base.set_int32_param(param, 0, i32::from(on))?;
        }
        let img = dest.get("Image").and_then(|v| v.as_array());
        for (n, param) in [(0, self.p.write_img_read), (1, self.p.write_img1_read)] {
            let on = img.is_some_and(|a| a.len() > n);
            self.ad.port_base.set_int32_param(param, 0, i32::from(on))?;
        }
        let prv = dest
            .pointer("/Preview/ImageChannels")
            .and_then(|v| v.as_array());
        for (n, param) in [
            (0, self.p.write_prv_img_read),
            (1, self.p.write_prv_img1_read),
        ] {
            let on = prv.is_some_and(|a| a.len() > n);
            self.ad.port_base.set_int32_param(param, 0, i32::from(on))?;
        }
        let hst = dest
            .pointer("/Preview/HistogramChannels")
            .and_then(|v| v.as_array());
        self.ad.port_base.set_int32_param(
            self.p.write_prv_hst_read,
            0,
            i32::from(hst.is_some_and(|a| !a.is_empty())),
        )?;
        Ok(())
    }

    /// C `writeDac` (serval_http.cpp:2199).
    fn write_dac(&mut self, chip: i32, name: &str, value: i32) -> AsynResult<()> {
        if !(0..MAX_ADDR as i32).contains(&chip) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("timepix3: chip {chip} is outside 0..{}", MAX_ADDR - 1),
            });
        }
        let path = serval::chip_dacs(chip);
        let mut dacs = self.http.get_json(&path, TIMEOUT_CONFIG).map_err(|e| {
            let context = format!("writeDac GET chip {chip}");
            self.http_failed(
                &format!("GET {path}"),
                &e,
                HttpPrint::Failure(&context, "GET", &path),
            )
        })?;
        dacs[name] = json!(value);
        self.http
            .put_json(&path, &dacs, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    &format!("PUT {path}"),
                    &e,
                    HttpPrint::Failure("writeDac PUT", "PUT", &path),
                )
            })?;
        self.http_ok();
        Ok(())
    }

    /// C `rotateLayout` (serval_http.cpp:476).
    fn rotate_layout(&mut self, orientation: i32) -> AsynResult<()> {
        let path = serval::layout_rotate(orientation).ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: orientation {orientation} is not one of the eight"),
        })?;
        self.http.get(&path, TIMEOUT_CONFIG).map_err(|e| {
            self.http_failed(
                "GET /detector/layout/rotate",
                &e,
                HttpPrint::Failure("rotateLayout", "GET", &path),
            )
        })?;
        self.http_ok();
        self.shared.set_orientation(orientation);
        Ok(())
    }

    /// C `uploadBPC` / `uploadDACS` (mask_io.cpp:339): Serval loads the file
    /// from its own filesystem, so only the path travels.
    fn config_load(&mut self, format: &str, path: usize, name: usize) -> AsynResult<()> {
        let file = format!("{}{}", self.text(path, 0), self.text(name, 0));
        let url = serval::config_load(format, &file);
        self.http.get(&url, TIMEOUT_CONFIG).map_err(|e| {
            let context = if format == "dacs" {
                "uploadDACS GET /config/load dacs"
            } else {
                "uploadBPC GET /config/load pixelconfig"
            };
            self.http_failed(
                &format!("GET /config/load ({format})"),
                &e,
                HttpPrint::Failure(context, "GET", &url),
            )
        })?;
        self.http_ok();
        Ok(())
    }

    // -- Mask ----------------------------------------------------------------

    /// The layout Serval reported, as the mask code needs it.
    fn geometry(&self) -> AsynResult<Geometry> {
        Geometry::new(
            self.int(self.p.row_len, 0),
            self.int(self.p.number_of_chips, 0),
            self.int(self.p.number_of_rows, 0),
            self.int(self.p.detector_orientation, 0),
        )
        .ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "timepix3: the detector layout is not known yet (no /detector reply)".into(),
        })
    }

    fn bpc_path(&self) -> std::path::PathBuf {
        std::path::PathBuf::from(format!(
            "{}{}",
            self.text(self.p.bp_c_file_path, 0),
            self.text(self.p.bp_c_file_name, 0)
        ))
    }

    /// The one mask drawing entry point (C's `readInt32Array` mask branch,
    /// mask_io.cpp:88-170, which mutates the *record's* buffer in place).
    ///
    /// The driver owns the mask here, so a re-read of the waveform cannot
    /// re-apply an operation and the record's buffer is never the source of
    /// truth.
    fn mask_op(&mut self) -> AsynResult<()> {
        let geom = self.geometry()?;
        let on = self.int(self.p.mask_on_off_pel, 0) != 0;
        let mut mask = self.shared.take_mask(geom.pixel_count());

        if self.int(self.p.mask_reset, 0) == 1 {
            mask::mask_reset(&geom, &mut mask, on);
        } else if self.int(self.p.mask_rectangle, 0) == 1 {
            mask::mask_rectangle(
                &geom,
                &mut mask,
                self.int(self.p.mask_min_x, 0),
                self.int(self.p.mask_size_x, 0),
                self.int(self.p.mask_min_y, 0),
                self.int(self.p.mask_size_y, 0),
                on,
            );
        } else if self.int(self.p.mask_circle, 0) == 1 {
            mask::mask_circle(
                &geom,
                &mut mask,
                self.int(self.p.mask_min_x, 0),
                self.int(self.p.mask_min_y, 0),
                self.int(self.p.mask_radius, 0),
                on,
            );
        } else if self.int(self.p.mask_pel, 0) == 1 {
            let bpc = self.read_bpc()?;
            mask::mask_from_bpc(&geom, &bpc, &mut mask);
        } else if self.int(self.p.mask_write, 0) == 1 {
            let mut bpc = self.read_bpc()?;
            mask::apply_mask_to_bpc(&geom, &mask, &mut bpc);
            self.write_mask_file(&bpc)?;
        }

        let n = mask.iter().filter(|&&v| v & 1 != 0).count();
        self.shared.put_mask(mask);
        self.ad.port_base.set_int32_param(
            self.p.bp_cmasked,
            0,
            i32::try_from(n).unwrap_or(i32::MAX),
        )?;
        Ok(())
    }

    /// C `readBPCfile` (mask_io.cpp:363).
    fn read_bpc(&mut self) -> AsynResult<Vec<u8>> {
        let path = self.bpc_path();
        let bpc = std::fs::read(&path).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: reading {}: {e}", path.display()),
        })?;
        let n = bpc.iter().filter(|&&b| b & 1 != 0).count();
        self.ad
            .port_base
            .set_int32_param(self.p.bp_cn, 0, i32::try_from(n).unwrap_or(i32::MAX))?;
        Ok(bpc)
    }

    /// C `writeBPCfile` (mask_io.cpp:430): write the edited BPC to the mask
    /// file name, then have Serval load *that* file.
    fn write_mask_file(&mut self, bpc: &[u8]) -> AsynResult<()> {
        let dir = self.text(self.p.bp_c_file_path, 0);
        let mut name = self.text(self.p.mask_file_name, 0);
        if name.is_empty() {
            name = "mask.bpc".to_string();
            self.ad
                .port_base
                .set_string_param(self.p.mask_file_name, 0, name.clone())?;
        }
        let path = std::path::PathBuf::from(format!("{dir}{name}"));
        std::fs::write(&path, bpc).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: writing {}: {e}", path.display()),
        })?;

        let url = serval::config_load("pixelconfig", &format!("{dir}{name}"));
        self.http.get(&url, TIMEOUT_CONFIG).map_err(|e| {
            let err = self.http_failed(
                "GET /config/load (pixelconfig)",
                &e,
                HttpPrint::Failure("uploadBPC GET /config/load pixelconfig", "GET", &url),
            );
            self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "{}",
                    adlog::err(
                        adlog::WRITE_BPC_FILE,
                        "WriteBPC: uploadBPC to SERVAL failed",
                    )
                ),
            );
            err
        })?;
        self.http_ok();
        Ok(())
    }

    /// C `refreshPixelConfigFromServal` (serval_http.cpp:1000).
    fn refresh_pixel_config(&mut self) -> AsynResult<()> {
        let geom = self.geometry()?;
        let bpc = self.read_bpc()?;
        let mut serval_linear = vec![0u8; geom.num_chips * PIXEL_CONFIG_BYTES];

        for chip in 0..geom.num_chips {
            let path = serval::chip_pixel_config(i32::try_from(chip).unwrap_or(0));
            let reply = self.http.get(&path, TIMEOUT_POLL).map_err(|e| {
                let err = self.http_failed(&format!("GET {path}"), &e, HttpPrint::None);
                self.ad.port_base.trace_print(
                    TraceMask::ERROR,
                    format_args!(
                        "{}",
                        adlog::err(
                            adlog::REFRESH_PIXEL_CONFIG,
                            &format!("chip {chip} PixelConfig GET failed: HTTP {}", e.code()),
                        )
                    ),
                );
                err
            })?;
            // Serval answers with a JSON string, or with the bare base64.
            let encoded = match serde_json::from_str::<serde_json::Value>(&reply) {
                Ok(v) => serval::json_to_string(&v),
                Err(_) => reply.trim().to_string(),
            };
            let addr = i32::try_from(chip).unwrap_or(0);
            let Some(decoded) = decode_base64(&encoded) else {
                self.ad
                    .port_base
                    .set_int32_param(self.p.pixel_config_match_bp_c, addr, -1)?;
                self.ad.port_base.set_string_param(
                    self.p.pixel_config_status,
                    addr,
                    "base64 decode failed",
                )?;
                continue;
            };

            let offset = chip * PIXEL_CONFIG_BYTES;
            let len = decoded.len();
            self.ad.port_base.set_int32_param(
                self.p.pixel_config_len,
                addr,
                i32::try_from(len).unwrap_or(i32::MAX),
            )?;

            let file_len = bpc.len().saturating_sub(offset).min(PIXEL_CONFIG_BYTES);
            let (match_code, mismatch, status) = if file_len == 0 {
                (
                    3,
                    0i64,
                    format!("BPC too small for chip (need offset {offset})"),
                )
            } else {
                let n = decoded
                    .iter()
                    .zip(&bpc[offset..offset + file_len])
                    .filter(|(a, b)| a != b)
                    .count() as i64;
                if n > 0 {
                    (0, n, format!("Mismatch {n} bytes"))
                } else if len != file_len {
                    (
                        3,
                        0,
                        format!("Length mismatch (decoded {len}, chip file {file_len})"),
                    )
                } else {
                    (1, 0, "OK, matches BPC".to_string())
                }
            };
            for (i, &b) in decoded.iter().take(PIXEL_CONFIG_BYTES).enumerate() {
                if let Some(slot) = serval_linear.get_mut(offset + i) {
                    *slot = b;
                }
            }
            self.ad
                .port_base
                .set_int32_param(self.p.pixel_config_match_bp_c, addr, match_code)?;
            self.ad.port_base.set_int64_param(
                self.p.pixel_config_mismatch_bytes,
                addr,
                mismatch,
            )?;
            self.ad
                .port_base
                .set_string_param(self.p.pixel_config_status, addr, status)?;
            self.ad.port_base.call_param_callbacks(addr)?;
        }

        let mut diff = vec![0i32; geom.pixel_count()];
        mask::pixel_config_diff(&geom, &serval_linear, &bpc, &mut diff);
        self.ad
            .port_base
            .params
            .set_int32_array(self.p.pixel_config_diff, 0, diff)?;

        let masked = mask::masked_pixels(&geom, &bpc);
        self.export_masked_pixels(&masked)?;
        Ok(())
    }

    /// C `exportMaskedPelsJsonFromBpcBuffer` (serval_http.cpp:900).
    fn export_masked_pixels(&mut self, masked: &[(usize, usize)]) -> AsynResult<()> {
        let path = self.text(self.p.masked_pels_json_path, 0);
        self.ad.port_base.set_int32_param(
            self.p.masked_pels_count,
            0,
            i32::try_from(masked.len()).unwrap_or(i32::MAX),
        )?;
        if path.is_empty() {
            self.ad
                .port_base
                .set_string_param(self.p.masked_pels_export_status, 0, "no path")?;
            return Ok(());
        }
        let body = json!({
            "MaskedPixels": masked
                .iter()
                .map(|&(x, y)| json!({ "x": x, "y": y }))
                .collect::<Vec<_>>(),
        });
        let status =
            match std::fs::write(&path, serde_json::to_vec_pretty(&body).unwrap_or_default()) {
                Ok(()) => format!("wrote {} pixels", masked.len()),
                Err(e) => format!("write failed: {e}"),
            };
        self.ad
            .port_base
            .set_string_param(self.p.masked_pels_export_status, 0, status)?;
        Ok(())
    }

    // -- Acquisition ---------------------------------------------------------

    /// C `acquireStart` (acquire.cpp:62).
    fn acquire_start(&mut self) -> AsynResult<()> {
        let measurement = self
            .http
            .get_json(serval::MEASUREMENT, TIMEOUT_POLL)
            .map_err(|e| self.http_failed("GET /measurement", &e, HttpPrint::None))?;
        if serval::measurement_is_running(&measurement) {
            self.http
                .get(serval::MEASUREMENT_STOP, TIMEOUT_POLL)
                .map_err(|e| {
                    self.http_failed(
                        "GET /measurement/stop",
                        &e,
                        HttpPrint::Warning(
                            "acquireStart stop prior measurement",
                            "GET",
                            serval::MEASUREMENT_STOP,
                        ),
                    )
                })?;
        }

        self.init_acquisition()?;
        self.file_writer()?;
        self.get_server()?;

        self.http
            .get(serval::MEASUREMENT_START, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    "GET /measurement/start",
                    &e,
                    HttpPrint::Failure(
                        "acquireStart GET /measurement/start",
                        "GET",
                        serval::MEASUREMENT_START,
                    ),
                )
            })?;
        self.http_ok();

        self.cmd_tx
            .try_send(Command::AcquisitionStarted)
            .map_err(|_| AsynError::Status {
                status: AsynStatus::Error,
                message: "timepix3: the acquisition task is not running".into(),
            })
    }

    /// C `acquireStop` (acquire.cpp:619).
    fn acquire_stop(&mut self) -> AsynResult<()> {
        self.http
            .get(serval::MEASUREMENT_STOP, TIMEOUT_POLL)
            .map_err(|e| {
                self.http_failed(
                    "GET /measurement/stop",
                    &e,
                    HttpPrint::Failure(
                        "acquireStop GET /measurement/stop",
                        "GET",
                        serval::MEASUREMENT_STOP,
                    ),
                )
            })?;
        self.http_ok();
        let _ = self.cmd_tx.try_send(Command::AcquisitionStopped);
        Ok(())
    }
}

impl ADDriver for TimePix3Driver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

impl PortDriver for TimePix3Driver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let acquiring = self.int(self.ad.params.acquire, 0) != 0;

        self.ad.port_base.set_int32_param(reason, addr, value)?;

        let p = self.p;
        let result = if reason == self.ad.params.acquire {
            if value != 0 && !acquiring {
                self.acquire_start()
            } else if value == 0 && acquiring {
                self.acquire_stop()
            } else {
                Ok(())
            }
        } else if let Some((name, _)) = DAC_NAMES.iter().find(|(_, f)| f(&p) == reason) {
            // Every per-chip DAC is written to the chip the asyn address names.
            self.write_dac(addr, name, value)
        } else if reason == p.write_raw
            || reason == p.write_raw1
            || reason == p.write_img
            || reason == p.write_img1
            || reason == p.write_prv_img
            || reason == p.write_prv_img1
            || reason == p.write_prv_hst
        {
            self.get_server()
        } else if reason == p.refresh_connection {
            self.ad
                .port_base
                .set_int32_param(p.refresh_connection, 0, 0)?;
            if one_shot(value) {
                self.cmd_tx
                    .try_send(Command::RefreshConnection)
                    .map_err(|_| AsynError::Status {
                        status: AsynStatus::Error,
                        message: "timepix3: the connection task is not running".into(),
                    })
            } else {
                Ok(())
            }
        } else if reason == p.refresh_pixel_config {
            self.ad
                .port_base
                .set_int32_param(p.refresh_pixel_config, 0, 0)?;
            if addr == 0 && one_shot(value) {
                self.refresh_pixel_config()
            } else {
                Ok(())
            }
        } else if reason == p.apply_config {
            self.ad.port_base.set_int32_param(p.apply_config, 0, 0)?;
            if one_shot(value) {
                self.file_writer().and_then(|()| self.get_server())
            } else {
                Ok(())
            }
        } else if reason == p.write_bp_c_file || reason == p.write_da_cs_file {
            self.ad.port_base.set_int32_param(reason, addr, 0)?;
            if !one_shot(value) {
                Ok(())
            } else if reason == p.write_bp_c_file {
                self.config_load("pixelconfig", p.bp_c_file_path, p.bp_c_file_name)
            } else {
                self.config_load("dacs", p.da_cs_file_path, p.da_cs_file_name)
            }
        } else if reason == p.write_data {
            self.file_writer().and_then(|()| self.get_server())
        } else if reason == p.detector_orientation {
            let value = if (0..8).contains(&value) { value } else { 0 };
            self.ad
                .port_base
                .set_int32_param(p.detector_orientation, 0, value)?;
            self.rotate_layout(value)
        } else if reason == p.bias_volt
            || reason == p.bias_enable
            || reason == p.trigger_in
            || reason == p.trigger_out
            || reason == p.log_level
            || reason == p.external_reference_clock
            || reason == p.chain_mode
            || reason == p.polarity
            || reason == p.periph_clk80
            || reason == p.tdc0
            || reason == p.tdc1
            || reason == p.gain_mode
            || reason == p.charge_summing
            || reason == p.colour
            || reason == p.pixel_depth
            || reason == p.counter_select_in
            || reason == p.counter_select_out
            || reason == p.i_delay0
            || reason == p.i_delay1
            || reason == p.i_delay2
            || reason == p.i_delay3
        {
            if reason == p.gain_mode && !(0..=3).contains(&value) {
                self.ad.port_base.set_int32_param(p.gain_mode, 0, 0)?;
            }
            let mut value = value;
            if reason == p.pixel_depth && ![1, 6, 12, 24].contains(&value) {
                value = 12;
                self.ad.port_base.set_int32_param(p.pixel_depth, 0, value)?;
            }
            if reason == p.pixel_depth && self.mpx3_pixel_depth_conflict(value) {
                // C returns before its closing callbacks and line; its
                // callParamCallbacks(ADTimePixPixelDepth) calls pass a
                // parameter index as the address and post nothing (UPSTREAM
                // DEFECT, ADTimePix.cpp:664-666), so the readbacks are posted
                // here as on every other path.
                self.log_write_int32(MPX3_PIXEL_DEPTH_MSG);
                self.ad.port_base.set_int32_param(p.pixel_depth, 0, 12)?;
                self.set_text(self.ad.params.status_message, MPX3_PIXEL_DEPTH_MSG)?;
                self.set_text(p.write_msg, MPX3_PIXEL_DEPTH_MSG)?;
                self.ad.port_base.set_int32_param(p.http_code, 0, 400)?;
                let _ = self.ad.port_base.call_param_callbacks(addr);
                self.ad.port_base.trace_print(
                    TraceMask::ERROR,
                    format_args!("{}", adlog::err(adlog::WRITE_INT32, MPX3_PIXEL_DEPTH_MSG)),
                );
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: MPX3_PIXEL_DEPTH_MSG.into(),
                });
            }
            self.init_acquisition()
        } else if reason == p.both_counters {
            if value != 0 {
                if self.mpx3_trigger_conflict(self.int(self.ad.params.trigger_mode, 0)) {
                    self.log_write_int32(
                        "MPX3 BothCounters: auto-switching TriggerMode 5 (CONTINUOUS) -> 4 (AUTOTRIGSTART_TIMERSTOP)",
                    );
                    self.ad
                        .port_base
                        .set_int32_param(self.ad.params.trigger_mode, 0, 4)?;
                }
                for ths in [p.prv_img_ths, p.prv_img1_ths, p.img_ths, p.img1_ths] {
                    self.set_text(ths, "0,1")?;
                }
                if self.mpx3_pixel_depth_conflict(self.int(p.pixel_depth, 0)) {
                    self.log_write_int32(MPX3_PIXEL_DEPTH_MSG);
                    self.ad.port_base.set_int32_param(p.pixel_depth, 0, 12)?;
                    self.set_text(self.ad.params.status_message, MPX3_PIXEL_DEPTH_MSG)?;
                }
            }
            self.init_acquisition()
        } else if reason == self.ad.params.num_images || reason == self.ad.params.trigger_mode {
            if reason == self.ad.params.trigger_mode
                && !(0..serval::TRIGGER_MODES.len() as i32).contains(&value)
            {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.trigger_mode, 0, 0)?;
            }
            let trigger_mode = self.int(self.ad.params.trigger_mode, 0);
            if reason == self.ad.params.trigger_mode && self.mpx3_trigger_conflict(trigger_mode) {
                self.log_write_int32(MPX3_TRIGGER_MSG);
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.trigger_mode, 0, 4)?;
                self.set_text(self.ad.params.status_message, MPX3_TRIGGER_MSG)?;
            }
            // Single image mode carries exactly one image (C ADTimePix.cpp:519).
            if reason == self.ad.params.num_images
                && self.int(self.ad.params.image_mode, 0) == 0
                && value != 1
            {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.num_images, 0, 1)?;
            }
            self.init_acquisition()
        } else if reason == self.ad.params.image_mode {
            if acquiring {
                self.acquire_stop()?;
            }
            if value == 0 {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.num_images, 0, 1)?;
                self.init_acquisition()
            } else {
                Ok(())
            }
        } else if reason == p.stem_scan_width
            || reason == p.stem_scan_height
            || reason == p.stem_radius_outer
            || reason == p.stem_radius_inner
        {
            self.send_measurement_config()
        } else if reason == p.img_frames_to_sum {
            self.shared.img.lock().set_frames_to_sum(value);
            Ok(())
        } else if reason == p.img_sum_update_interval_frames {
            self.shared.img.lock().set_update_interval(value);
            Ok(())
        } else if reason == p.prv_hst_frames_to_sum {
            self.shared.hst.lock().set_frames_to_sum(value);
            Ok(())
        } else if reason == p.prv_hst_sum_update_interval {
            self.shared.hst.lock().set_update_interval(value);
            Ok(())
        } else if reason == p.img_image_data_reset {
            if value == 1 {
                self.shared.img.lock().reset();
                self.ad
                    .port_base
                    .set_int32_param(p.img_image_data_reset, 0, 0)?;
            }
            Ok(())
        } else if reason == p.prv_hst_data_reset {
            if value == 1 {
                self.shared.hst.lock().reset();
                self.ad
                    .port_base
                    .set_int32_param(p.prv_hst_data_reset, 0, 0)?;
            }
            Ok(())
        } else if reason == p.mask_reset
            || reason == p.mask_rectangle
            || reason == p.mask_circle
            || reason == p.mask_pel
            || reason == p.mask_write
        {
            let r = self.mask_op();
            // These are one-shot triggers, exactly as the db's autosave-free
            // bo records expect.
            self.ad.port_base.set_int32_param(reason, 0, 0)?;
            r
        } else if reason == p.health {
            self.cmd_tx
                .try_send(Command::RefreshStatus)
                .map_err(|_| AsynError::Status {
                    status: AsynStatus::Error,
                    message: "timepix3: the connection task is not running".into(),
                })
        } else if reason < p.http_code {
            // ADTIMEPIX_FIRST_PARAM is ADTimePixHttpCode (ADTimePix.h:467,
            // ADTimePix.cpp:860-863).
            ADDriver::ad_write_int32(self, user, value)
        } else {
            Ok(())
        };

        // The write's outcome is the record's, and it must survive the
        // callbacks.
        //
        // UPSTREAM DEFECT (ADTimePix.cpp:409-411, 448-450, 505-506): C returns
        // `asynError` from several branches *before* `callParamCallbacks`, so
        // the readbacks (and, for ADAcquire, the latched Acquire=1) are never
        // refreshed after a failed write and the operator sees the detector as
        // acquiring when it is not.
        let _ = self.ad.port_base.call_param_callbacks(addr);
        match &result {
            Err(e) => self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "{}",
                    adlog::err(
                        adlog::WRITE_INT32,
                        &format!(
                            "ERROR status={} function={reason}, value={value}",
                            e.status() as i32
                        ),
                    )
                ),
            ),
            Ok(()) => self.ad.port_base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "{}",
                    adlog::log(
                        adlog::WRITE_INT32,
                        &format!("function={reason} value={value}"),
                    )
                ),
            ),
        }
        result
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        // UPSTREAM DEFECT (ADTimePix.cpp:791): C's writeFloat64 calls
        // `setDoubleParam(function, value)` — the two-argument form, which
        // always writes address 0 — on a maxAddr=8 multi-device port, so every
        // per-chip float write lands on chip 0. The address is honoured here.
        let addr = user.addr;
        let acquiring = self.int(self.ad.params.acquire, 0) != 0;

        self.ad.port_base.set_float64_param(reason, addr, value)?;

        let p = self.p;
        let result = if reason == self.ad.params.acquire_time
            || reason == self.ad.params.acquire_period
            || reason == p.trigger_delay
            || reason == p.global_timestamp_interval
        {
            if acquiring {
                self.acquire_stop()?;
            }
            self.init_acquisition()
        } else if reason == p.stem_dwell_time || reason == p.tof_min || reason == p.tof_max {
            self.send_measurement_config()
        } else {
            Ok(())
        };

        let _ = self.ad.port_base.call_param_callbacks(addr);
        match &result {
            Err(e) => self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "{}",
                    adlog::err(
                        adlog::WRITE_FLOAT64,
                        &format!(
                            "ERROR status = {}, function ={reason}, value = {value:.6}",
                            e.status() as i32
                        ),
                    )
                ),
            ),
            Ok(()) => self.ad.port_base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "{}",
                    adlog::log(
                        adlog::WRITE_FLOAT64,
                        &format!("function={reason} value={value:.6}"),
                    )
                ),
            ),
        }
        result
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let addr = user.addr;
        let value = String::from_utf8_lossy(data).into_owned();
        self.ad.port_base.set_string_param(reason, addr, value)?;

        let p = self.p;
        // A path PV's "does it exist" readback (C `checkBPCPath`,
        // mask_io.cpp:320).
        for (path, exists) in [
            (p.bp_c_file_path, p.bp_c_file_path_exists),
            (p.da_cs_file_path, p.da_cs_file_path_exists),
        ] {
            if reason == path {
                let dir = self.text(path, 0);
                let ok = !dir.is_empty() && std::path::Path::new(&dir).is_dir();
                self.ad
                    .port_base
                    .set_int32_param(exists, 0, i32::from(ok))?;
            }
        }
        if reason == p.tof_tdc_reference {
            self.send_measurement_config()?;
        }

        // UPSTREAM DEFECT (ADTimePix.cpp:371): C's writeOctet sets the
        // parameter's status to reflect the validation it just did and then
        // calls `callParamCallbacks`, which re-sets the status from the
        // parameter library and discards it — the record never shows the
        // failure. The status is left to the return value here.
        self.ad.port_base.call_param_callbacks(addr)?;
        let name = self.ad.port_base.params.param_name(reason).unwrap_or("");
        // C prints the value as the `char *` it was handed: up to the NUL.
        let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
        let value = String::from_utf8_lossy(&data[..end]);
        self.ad.port_base.trace_print(
            TraceMask::IO_DRIVER,
            format_args!(
                "{}",
                adlog::log(
                    adlog::WRITE_OCTET,
                    &format!("function={reason}, paramName={name}, value={value}"),
                )
            ),
        );
        Ok(data.len())
    }

    /// C `readEnum` (ADTimePix.cpp:979-1023): the orientation, trigger-mode
    /// and pipeline-state menus come from the driver, not the template.
    fn read_enum(&mut self, user: &AsynUser) -> AsynResult<(usize, Arc<[EnumEntry]>)> {
        let entry = |(value, s): (i32, &str)| EnumEntry {
            string: s.to_string(),
            value,
            severity: 0,
        };
        let entries: Arc<[EnumEntry]> = if user.reason == self.p.detector_orientation {
            (0..).zip(serval::ORIENTATIONS).map(entry).collect()
        } else if user.reason == self.ad.params.trigger_mode {
            (0..).zip(serval::TRIGGER_MODE_LABELS).map(entry).collect()
        } else if user.reason == self.p.pipeline_state {
            serval::PIPELINE_STATES.into_iter().map(entry).collect()
        } else {
            return self.ad.port_base.params.get_enum(user.reason, user.addr);
        };
        let index = usize::try_from(self.int(user.reason, user.addr)).unwrap_or(0);
        Ok((index, entries))
    }

    /// The mask waveform reads back what the driver drew (C `readInt32Array`,
    /// mask_io.cpp:18).
    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let reason = user.reason;
        if reason == self.p.mask_bp_c {
            let mask = self.shared.mask.lock();
            let n = buf.len().min(mask.len());
            buf[..n].copy_from_slice(&mask[..n]);
            buf[n..].fill(0);
            return Ok(buf.len());
        }
        if reason == self.p.bp_c {
            let bpc = self.read_bpc()?;
            for (slot, &b) in buf.iter_mut().zip(bpc.iter()) {
                // Bit 8 marks a masked pixel for the calibration view
                // (mask_io.cpp:186).
                *slot = i32::from(b) | if b & 1 != 0 { 1 << 8 } else { 0 };
            }
            if bpc.len() < buf.len() {
                buf[bpc.len()..].fill(0);
            }
            return Ok(buf.len());
        }
        // C answers every other reason with nElements, zero-filled past the
        // data, including before the first value (mask_io.cpp:23-74, :205).
        let data = self
            .ad
            .port_base
            .params
            .get_int32_array(reason, user.addr)
            .unwrap_or_default();
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        buf[n..].fill(0);
        Ok(buf.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;

    /// An image frame read before the first frame, and a pixel-config diff
    /// shorter than the waveform, both come back as nElements zero-filled
    /// (mask_io.cpp:23-74).
    #[test]
    fn an_int32_array_read_fills_the_waveform() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_READ_ARRAY",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let mut buf = [7; 4];
        let n = det
            .read_int32_array(&AsynUser::new(p.img_image_frame), &mut buf)
            .unwrap();
        assert_eq!((n, buf), (4, [0; 4]));
        det.ad
            .port_base
            .params
            .set_int32_array(p.pixel_config_diff, 0, vec![5, 6])
            .unwrap();
        let mut buf = [7; 4];
        let n = det
            .read_int32_array(&AsynUser::new(p.pixel_config_diff), &mut buf)
            .unwrap();
        assert_eq!((n, buf), (4, [5, 6, 0, 0]));
    }

    /// A Medipix3 counting both counters refuses 24-bit depth and CONTINUOUS
    /// triggering (ADTimePix.cpp:658-671, :675-702, :709-714).
    #[test]
    fn medipix3_both_counters_refuses_24_bit_and_continuous() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        shared.note_family(DetectorFamily::Mpx3);
        let mut det = TimePix3Driver::new(
            "TPX_MPX3_BOTH",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared,
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let (trigger_mode, status_message) =
            (det.ad.params.trigger_mode, det.ad.params.status_message);
        det.ad
            .port_base
            .set_int32_param(trigger_mode, 0, 5)
            .unwrap();
        det.ad
            .port_base
            .set_int32_param(p.pixel_depth, 0, 24)
            .unwrap();
        let _ = det.write_int32(&mut AsynUser::new(p.both_counters), 1);
        assert_eq!(det.int(trigger_mode, 0), 4);
        assert_eq!(det.int(p.pixel_depth, 0), 12);
        assert_eq!(det.text(p.img1_ths, 0), "0,1");
        assert_eq!(det.text(status_message, 0), MPX3_PIXEL_DEPTH_MSG);

        let e = det
            .write_int32(&mut AsynUser::new(p.pixel_depth), 24)
            .unwrap_err();
        assert!(e.to_string().contains(MPX3_PIXEL_DEPTH_MSG));
        assert_eq!(det.int(p.pixel_depth, 0), 12);
        assert_eq!(det.int(p.http_code, 0), 400);
        assert_eq!(det.text(p.write_msg, 0), MPX3_PIXEL_DEPTH_MSG);

        let _ = det.write_int32(&mut AsynUser::new(trigger_mode), 5);
        assert_eq!(det.int(trigger_mode, 0), 4);
        assert_eq!(det.text(status_message, 0), MPX3_TRIGGER_MSG);
    }

    /// The trigger-mode menu is C's nine labels, FOLLOWING included, and the
    /// pipeline state carries -1 for Unknown (ADTimePix.cpp:900-926).
    #[test]
    fn the_driver_supplies_the_trigger_mode_and_pipeline_menus() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_READ_ENUM",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let (_, menu) = det
            .read_enum(&AsynUser::new(det.ad.params.trigger_mode))
            .unwrap();
        assert_eq!(menu.len(), 9);
        assert_eq!(menu[7].string, "SwReSt_SwReSp");
        assert_eq!((menu[8].string.as_str(), menu[8].value), ("FOLLOWING", 8));
        let (_, menu) = det.read_enum(&AsynUser::new(det.p.pipeline_state)).unwrap();
        assert_eq!((menu[4].string.as_str(), menu[4].value), ("Unknown", -1));
        let (_, menu) = det
            .read_enum(&AsynUser::new(det.p.detector_orientation))
            .unwrap();
        assert_eq!(menu[7].string, "LEFT_MIRRORED");
        assert!(det.read_enum(&AsynUser::new(det.p.gain_mode)).is_err());
    }

    /// An out-of-range GainMode, PixelDepth, TriggerMode or orientation is
    /// stored as C's default (ADTimePix.cpp:631-634, :649-657, :705-708).
    #[test]
    fn an_out_of_range_setting_reads_back_the_default() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_CLAMP",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let trigger_mode = det.ad.params.trigger_mode;
        for (reason, written, stored) in [
            (p.gain_mode, 4, 0),
            (p.pixel_depth, 8, 12),
            (trigger_mode, 9, 0),
            (p.detector_orientation, 8, 0),
        ] {
            let _ = det.write_int32(&mut AsynUser::new(reason), written);
            assert_eq!(det.int(reason, 0), stored, "reason {reason}");
        }
    }

    /// The Medipix3 detector settings go to Serval like the Timepix3 ones
    /// (ADTimePix.cpp:643-673); with no Serval behind the driver the write
    /// fails.
    #[test]
    fn a_medipix3_setting_reaches_init_acquisition() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MPX3_SETTING",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for reason in [p.gain_mode, p.pixel_depth, p.colour, p.i_delay3] {
            assert!(det.write_int32(&mut AsynUser::new(reason), 1).is_err());
        }
    }

    /// The one-shot actions run on 1 only and read back 0 whatever was
    /// written (one_shot_action.h, ADTimePix.cpp:563-584, :614-621); with no
    /// Serval behind the driver, running one fails the write.
    #[test]
    fn a_one_shot_action_runs_on_one_only() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_ONE_SHOT",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for reason in [
            p.apply_config,
            p.write_bp_c_file,
            p.write_da_cs_file,
            p.refresh_pixel_config,
        ] {
            det.write_int32(&mut AsynUser::new(reason), 2).unwrap();
            assert_eq!(det.int(reason, 0), 0);
        }
        assert!(
            det.write_int32(&mut AsynUser::new(p.apply_config), 1)
                .is_err()
        );
        assert_eq!(det.int(p.apply_config, 0), 0);
        assert!(
            det.write_int32(&mut AsynUser::new(p.write_bp_c_file), 1)
                .is_err()
        );
        assert_eq!(det.int(p.write_bp_c_file, 0), 0);
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_BASE_WRITE",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(p.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        det.write_int32(&mut AsynUser::new(p.shutter_control), 1)
            .unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }
}
