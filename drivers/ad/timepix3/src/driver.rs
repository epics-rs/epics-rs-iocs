//! The ADTimePix3 asyn port driver (port of the `ADTimePix` class,
//! `ADTimePix.cpp` + `serval_http.cpp` + `acquire.cpp` + `mask_io.cpp`).

use std::sync::Arc;

use epics_rs::ad_core::driver::ADDriverBase;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{NDArrayOutput, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::user::AsynUser;
use serde_json::json;

use crate::http::{HttpError, ServalHttp, TIMEOUT_CONFIG, TIMEOUT_POLL, decode_base64};
use crate::mask::{self, Geometry, PIXEL_CONFIG_BYTES};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Shared};

/// C's `maxAddr` (ADTimePix.cpp:1069): 0 = PrvImg, 1 = Img, 2 = Img running
/// sum, 3 = Img sum-of-N, 4 = PrvHst sum-of-N, 5 = PrvHst running sum,
/// 6 = PrvHst frame, 7 = PrvHst ToF axis. The chip DACs bind addresses 0-7 and
/// the voltage rails 0-5 on the same port.
pub const MAX_ADDR: usize = 8;

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
        let mut ad = new_multi_addr_base(port_name, max_memory)?;
        let p = TimePixParams::create(&mut ad.port_base)?;

        let mut driver = Self {
            ad,
            p,
            http,
            shared,
            cmd_tx,
        };
        driver.init_params(port_name)?;
        Ok(driver)
    }

    /// C's constructor tail (ADTimePix.cpp:1440-1520).
    fn init_params(&mut self, port_name: &str) -> AsynResult<()> {
        let server = self.http.base_url().to_string();
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.base.manufacturer, 0, "ASI".into())?;
        base.set_string_param(self.ad.params.base.model, 0, "TimePix3".into())?;
        base.set_string_param(self.p.server_name, 0, server)?;
        base.set_string_param(self.ad.params.base.port_name_self, 0, port_name.into())?;
        base.set_int32_param(self.p.img_frames_to_sum, 0, 10)?;
        base.set_int32_param(self.p.img_sum_update_interval_frames, 0, 1)?;
        base.set_int32_param(self.p.prv_hst_frames_to_sum, 0, 10)?;
        base.set_int32_param(self.p.prv_hst_sum_update_interval, 0, 1)?;
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
        self.ad
            .port_base
            .get_string_param(index, addr)
            .unwrap_or_default()
            .to_string()
    }

    /// Publish the HTTP status of the last request, as C's `TPX3_HTTP_CODE`
    /// does, and turn the failure into an asyn error.
    fn http_failed(&mut self, what: &str, e: &HttpError) -> AsynError {
        let _ = self
            .ad
            .port_base
            .set_int32_param(self.p.http_code, 0, e.code());
        let message = format!("timepix3: {what}: {e}");
        log::error!("{message}");
        AsynError::Status {
            status: AsynStatus::Error,
            message,
        }
    }

    fn http_ok(&mut self) {
        let _ = self.ad.port_base.set_int32_param(self.p.http_code, 0, 200);
    }

    // -- Serval requests -----------------------------------------------------

    /// C `initAcquisition` (serval_http.cpp:2254): read `/detector/config`,
    /// merge the driver's settings into it, PUT it back.
    fn init_acquisition(&mut self) -> AsynResult<()> {
        let current = self
            .http
            .get_json(serval::DETECTOR_CONFIG, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /detector/config", &e))?;

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
            .map_err(|e| self.http_failed("PUT /detector/config", &e))?;
        self.http_ok();
        Ok(())
    }

    /// C `sendMeasurementConfig` (serval_http.cpp:2033).
    fn send_measurement_config(&mut self) -> AsynResult<()> {
        let current = self
            .http
            .get_json(serval::MEASUREMENT_CONFIG, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /measurement/config", &e))?;
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
            .map_err(|e| self.http_failed("PUT /measurement/config", &e))?;
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
            .map_err(|e| self.http_failed("PUT /server/destination", &e))?;
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
            .map_err(|e| self.http_failed("GET /server/destination", &e))?;
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
        let mut dacs = self
            .http
            .get_json(&path, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed(&format!("GET {path}"), &e))?;
        dacs[name] = json!(value);
        self.http
            .put_json(&path, &dacs, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed(&format!("PUT {path}"), &e))?;
        self.http_ok();
        Ok(())
    }

    /// C `rotateLayout` (serval_http.cpp:476).
    fn rotate_layout(&mut self, orientation: i32) -> AsynResult<()> {
        let path = serval::layout_rotate(orientation).ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: orientation {orientation} is not one of the eight"),
        })?;
        self.http
            .get(&path, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /detector/layout/rotate", &e))?;
        self.http_ok();
        self.shared.set_orientation(orientation);
        Ok(())
    }

    /// C `uploadBPC` / `uploadDACS` (mask_io.cpp:339): Serval loads the file
    /// from its own filesystem, so only the path travels.
    fn config_load(&mut self, format: &str, path: usize, name: usize) -> AsynResult<()> {
        let file = format!("{}{}", self.text(path, 0), self.text(name, 0));
        let url = serval::config_load(format, &file);
        self.http
            .get(&url, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed(&format!("GET /config/load ({format})"), &e))?;
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
            let dropped = mask::apply_mask_to_bpc(&geom, &mask, &mut bpc);
            if dropped > 0 {
                log::warn!(
                    "timepix3: {dropped} masked pixels fall outside the BPC file ({} bytes)",
                    bpc.len()
                );
            }
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
        self.http
            .get(&url, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /config/load (pixelconfig)", &e))?;
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
            let reply = self
                .http
                .get(&path, TIMEOUT_POLL)
                .map_err(|e| self.http_failed(&format!("GET {path}"), &e))?;
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
                    "base64 decode failed".into(),
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
            self.ad.port_base.params.set_int64(
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
            self.ad.port_base.set_string_param(
                self.p.masked_pels_export_status,
                0,
                "no path".into(),
            )?;
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
            .map_err(|e| self.http_failed("GET /measurement", &e))?;
        if serval::measurement_is_running(&measurement) {
            self.http
                .get(serval::MEASUREMENT_STOP, TIMEOUT_POLL)
                .map_err(|e| self.http_failed("GET /measurement/stop", &e))?;
        }

        self.init_acquisition()?;
        self.file_writer()?;
        self.get_server()?;

        self.http
            .get(serval::MEASUREMENT_START, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /measurement/start", &e))?;
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
            .map_err(|e| self.http_failed("GET /measurement/stop", &e))?;
        self.http_ok();
        let _ = self.cmd_tx.try_send(Command::AcquisitionStopped);
        Ok(())
    }
}

/// `ADDriverBase::new` builds its port with `max_addr = 1` and
/// `multi_device = false`; ADTimePix3 is an `ASYN_MULTIDEVICE` port with
/// `maxAddr = 8` (ADTimePix.cpp:1069), so the base is assembled here from the
/// same public pieces.
fn new_multi_addr_base(port_name: &str, max_memory: usize) -> AsynResult<ADDriverBase> {
    let mut port_base = PortDriverBase::new(
        port_name,
        MAX_ADDR,
        PortFlags {
            can_block: true,
            multi_device: true,
            ..Default::default()
        },
    );
    let params = ADDriverParams::create(&mut port_base)?;
    port_base.set_string_param(params.base.port_name_self, 0, port_name.into())?;
    port_base.set_int32_param(params.base.array_callbacks, 0, 1)?;
    port_base.set_float64_param(
        params.base.pool_max_memory,
        0,
        max_memory as f64 / 1_048_576.0,
    )?;
    port_base.set_int32_param(params.image_mode, 0, 0)?;
    port_base.set_int32_param(params.num_images, 0, 1)?;
    port_base.set_float64_param(params.acquire_time, 0, 1.0)?;
    port_base.set_float64_param(params.acquire_period, 0, 1.0)?;

    Ok(ADDriverBase {
        port_base,
        params,
        pool: Arc::new(NDArrayPool::new(max_memory)),
        array_output: NDArrayOutput::new(),
        queued_counter: Arc::new(QueuedArrayCounter::new()),
        last_array: None,
    })
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

        self.ad.port_base.params.set_int32(reason, addr, value)?;

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
        } else if reason == p.apply_config || reason == p.write_data {
            let r = self.file_writer().and_then(|()| self.get_server());
            self.ad.port_base.set_int32_param(p.apply_config, 0, 0)?;
            r
        } else if reason == p.refresh_connection {
            self.ad
                .port_base
                .set_int32_param(p.refresh_connection, 0, 0)?;
            self.cmd_tx
                .try_send(Command::RefreshConnection)
                .map_err(|_| AsynError::Status {
                    status: AsynStatus::Error,
                    message: "timepix3: the connection task is not running".into(),
                })
        } else if reason == p.refresh_pixel_config {
            if addr == 0 && value == 1 {
                self.ad
                    .port_base
                    .set_int32_param(p.refresh_pixel_config, 0, 0)?;
                self.refresh_pixel_config()
            } else {
                Ok(())
            }
        } else if reason == p.write_bp_c_file {
            self.config_load("pixelconfig", p.bp_c_file_path, p.bp_c_file_name)
        } else if reason == p.write_da_cs_file {
            self.config_load("dacs", p.da_cs_file_path, p.da_cs_file_name)
        } else if reason == p.detector_orientation {
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
            || reason == self.ad.params.trigger_mode
        {
            self.init_acquisition()
        } else if reason == self.ad.params.num_images {
            // Single image mode carries exactly one image (C ADTimePix.cpp:519).
            if self.int(self.ad.params.image_mode, 0) == 0 && value != 1 {
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
        self.ad.port_base.call_param_callbacks(addr)?;
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

        self.ad.port_base.params.set_float64(reason, addr, value)?;

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

        self.ad.port_base.call_param_callbacks(addr)?;
        result
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let addr = user.addr;
        let value = String::from_utf8_lossy(data).into_owned();
        self.ad.port_base.params.set_string(reason, addr, value)?;

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
        Ok(data.len())
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
        let data = self
            .ad
            .port_base
            .params
            .get_int32_array(reason, user.addr)?;
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }
}
