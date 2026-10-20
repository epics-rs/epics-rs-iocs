//! The acquisition task (port of `mythen::acquisitionTask` and
//! `mythen::dataCallback`, mythen.cpp:838 and :950).

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use crate::detector::Detector;
use crate::driver::MythenParams;
use crate::protocol::{CHANNELS_PER_MODULE, READ_MODE_RAW, decode_raw_readout, words_from_bytes};
use crate::transport::M1K_TIMEOUT;

/// Corrected readout is always 24-bit (C `dataCallback`, mythen.cpp:976).
const CORRECTED_NBITS: i32 = 24;

/// Everything the acquisition task needs.
pub struct Shared {
    pub handle: PortHandle,
    pub p: MythenParams,
    pub ad: ADDriverParams,
    pub det: Arc<Detector>,
    pub output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub pool: DriverPool,
}

impl Shared {
    async fn set_int(&self, reason: usize, value: i32) {
        let _ = self
            .handle
            .set_params_and_notify(
                0,
                vec![ParamSetValue::new(reason, 0, ParamValue::Int32(value))],
            )
            .await;
    }

    async fn get_int(&self, reason: usize) -> i32 {
        self.handle.read_int32(reason, 0).await.unwrap_or(0)
    }

    async fn get_f64(&self, reason: usize) -> f64 {
        self.handle.read_float64(reason, 0).await.unwrap_or(0.0)
    }
}

/// Start the acquisition task; it runs until the process exits.
pub fn start(
    shared: Arc<Shared>,
    mut start_rx: rt::CommandReceiver<()>,
) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("mythenAcquisitionTask", move || async move {
        while start_rx.recv().await.is_some() {
            acquire(&shared).await;
        }
    })
}

/// One acquisition, from the start signal to the last frame.
async fn acquire(s: &Arc<Shared>) {
    let det = &s.det;
    let read_mode = det.state.read_mode.load(Ordering::Acquire);
    let nbits = det.state.nbits.load(Ordering::Acquire);
    // Without a module count there is no readout length, and a readout with no
    // length would send the command and read none of the reply — the frame
    // would sit in the socket and be picked up as the answer to the next
    // command, for the life of the process. Refuse instead.
    let (Some(nmodules), Some(expect)) = (det.nmodules(), det.readout_len()) else {
        det.state.acquiring.store(false, Ordering::Release);
        s.set_int(s.ad.status, ADStatus::Disconnected as i32).await;
        s.set_int(s.ad.acquire, 0).await;
        return;
    };
    let acquire_time = s.get_f64(s.ad.acquire_time).await;
    let timeout = M1K_TIMEOUT + Duration::from_secs_f64(acquire_time.max(0.0));
    // Raw readout packs several channels into one 32-bit word; a corrected
    // readout is one word per channel.
    let decode_nbits = if read_mode == READ_MODE_RAW {
        nbits
    } else {
        CORRECTED_NBITS
    };

    let mut status = read_status(s).await;

    // Every readout that arrives whole is published, and the status is only
    // re-read when one does not: a running detector answers each `-readout`
    // with the next frame, and the frame after the last one is what makes the
    // status go back to Idle and ends the loop (C, mythen.cpp:883-921).
    while status != ADStatus::Error {
        match det.readout(expect, timeout) {
            Ok(reply) if reply.len() == expect.get() => {
                if !publish(s, &reply, nmodules, decode_nbits).await {
                    status = read_status(s).await;
                }
            }
            Ok(_) => {
                status = read_status(s).await;
            }
            Err(e) => {
                // C re-reads it for a failed readout too, before the report
                // (mythen.cpp:908-917).
                status = read_status(s).await;
                det.trace_error(&format!(
                    "mythen:acquisitionTask: error using readout command status={}, nRead=0, \
                     eomReason=0\n",
                    e.status() as i32
                ));
                break;
            }
        }
        if !det.state.acquiring.load(Ordering::Acquire) {
            break;
        }
        if !matches!(status, ADStatus::Acquire | ADStatus::Readout) {
            break;
        }
    }

    det.state.acquiring.store(false, Ordering::Release);
    let image_mode = s.get_int(s.ad.image_mode).await;
    if status == ADStatus::Error {
        // C aborts the detector on the way out (mythen.cpp:939-946).
        det.trace_error("mythen:acquisitionTask: error timed out waiting for data\n");
        let _ = det.stop();
    } else {
        println!("Acquisition finish");
        if image_mode == ImageMode::Single as i32 || image_mode == ImageMode::Multiple as i32 {
            println!("ADAcquire Callback");
        }
    }
    s.set_int(s.ad.acquire, 0).await;
    // C leaves ADAcquire set when the image mode is Continuous, and then spins
    // in `while (1)` with the driver lock held because neither `acquire` nor
    // `acquiring_` ever clears (mythen.cpp:928-936). The image mode the driver
    // offers is Single or Multiple only (mythen.template re-opens the mbbo
    // without "Continuous"), so the acquisition ends here in every mode the
    // detector can be put into.
    s.set_int(s.ad.status, ADStatus::Idle as i32).await;
}

async fn read_status(s: &Arc<Shared>) -> ADStatus {
    let status = match s.det.get_status() {
        Ok(status) => status,
        // writeReadMeter has printed C's line.
        Err(_) => ADStatus::Error,
    };
    s.set_int(s.ad.status, status as i32).await;
    status
}

/// One readout on its way out (C `dataCallback`, mythen.cpp:950).
///
/// Returns false when the detector answered with an error word, which is what
/// makes the caller re-read the status.
async fn publish(s: &Arc<Shared>, reply: &[u8], nmodules: usize, nbits: i32) -> bool {
    let words = words_from_bytes(reply);
    match words.first() {
        // C `pData[0] < 0` — the detector reports a failed readout in the
        // first word.
        Some(&first) if (first as i32) < 0 => return false,
        None => return false,
        Some(_) => {}
    }

    let channels = decode_raw_readout(nmodules, nbits, &words);
    // C declares the NDArray one module wide (`dims[0] = MAX_DIMS`) while it
    // fills 1280 * nmodules elements, so a second module is invisible to every
    // downstream plugin (mythen.cpp:965, :1044). The array is as wide as the
    // data it carries.
    let width = nmodules * CHANNELS_PER_MODULE;
    debug_assert_eq!(channels.len(), width);

    // C allocates before it counts (mythen.cpp:973, :993) and dereferences a
    // refused alloc; the frame is dropped after the pool's reached-limit line.
    let Ok(mut array) = s.pool.alloc_with(
        vec![NDDimension::new(width), NDDimension::new(1)],
        NDDataBuffer::U32(channels),
    ) else {
        return true;
    };

    let mut attributes = NDAttributeList::new();
    attributes.add(NDAttribute::new_static(
        "ColorMode",
        "Color Mode",
        NDAttrSource::Driver,
        NDAttrValue::Int32(NDColorMode::Mono as i32),
    ));

    let counter = s.get_int(s.ad.base.array_counter).await + 1;
    s.set_int(s.ad.base.array_counter, counter).await;

    array.unique_id = counter;
    array.timestamp = EpicsTimestamp::now();
    array.time_stamp = array.timestamp.as_f64();
    array.attributes = attributes;

    let array = array.share();
    if s.get_int(s.ad.base.array_callbacks).await != 0 {
        ArrayPublisher::new(s.output.clone())
            .publish(array.clone())
            .await;
    }
    // C releases the previous pArrays[0] after the callbacks (mythen.cpp:1010-1011).
    s.pool.owner().store_array(0, array.array().clone());
    true
}
