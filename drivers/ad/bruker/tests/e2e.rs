//! Two acquisitions, end to end, against a fake BIS: the two TCP sockets, the
//! port actor, the acquisition task, the status task and the SFRM reader, wired
//! together as the IOC wires them.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc;
use std::time::Duration;

use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interpose::eos::EosInterpose;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::sync_io::SyncIOHandle;

use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDataType};
use epics_rs::ad_core::plugin::channel::ndarray_channel;

use bruker::{BrukerRuntime, create_bruker_detector};

/// A read the test expects to succeed, as its bytes.
trait BytesOk {
    fn bytes_ok(self) -> Vec<u8>;
}

impl BytesOk for epics_rs::asyn::request::OctetRead {
    fn bytes_ok(self) -> Vec<u8> {
        let (bytes, status) = self.into_parts();
        status.unwrap();
        bytes
    }
}

/// Privileged port 1: no test can bind it, so a connect is always refused.
/// A freed ephemeral port can be taken by a concurrent test's listener.
const DEAD_ADDR: &str = "127.0.0.1:1";

/// A 4 x 2 frame of 16-bit pixels, 1..=8.
fn sfrm_file() -> Vec<u8> {
    const LINE: usize = 80;
    let lines: [(usize, &str); 9] = [
        (0, "FORMAT :100"),
        (1, "VERSION:11"),
        (2, "HDRBLKS:13"),
        (20, "NOVERFL:0 0 0"),
        (39, "NPIXELB:2 2"),
        (40, "NROWS  :2"),
        (41, "NCOLS  :4"),
        (42, "WORDORD:0"),
        (79, "LINEAR :1 0 0 0 0"),
    ];
    let mut file = vec![b' '; 13 * 512];
    for (line, text) in lines {
        let at = line * LINE;
        file[at..at + text.len()].copy_from_slice(text.as_bytes());
    }
    for pixel in 1u16..=8 {
        file.extend_from_slice(&pixel.to_le_bytes());
    }
    file
}

/// The BIS server: it takes a scan command, writes the frame file, and says on
/// the status socket that it has processed it.
fn fake_bis(command: TcpListener, status: TcpListener) {
    let (processed_tx, processed_rx) = mpsc::channel::<()>();

    std::thread::spawn(move || {
        let (mut stream, _) = status.accept().expect("status connection");
        stream
            .write_all(b"[DETECTORSTATUS /FRAMESIZE=1024 /CCDTEMP=-40.5]\n")
            .expect("status message");
        while processed_rx.recv().is_ok() {
            stream
                .write_all(b"[INSTRUMENTQUEUE /PROCESSING=0]\n")
                .expect("processing message");
        }
    });

    std::thread::spawn(move || {
        let (stream, _) = command.accept().expect("command connection");
        let mut writer: TcpStream = stream.try_clone().expect("clone");
        let mut reader = BufReader::new(stream);
        let mut line = String::new();
        while reader.read_line(&mut line).unwrap_or(0) > 0 {
            if let Some(rest) = line.strip_prefix("[Scan /Filename=") {
                let file = rest.split(' ').next().expect("file name");
                std::fs::write(file, sfrm_file()).expect("frame file");
                writer.write_all(b"[Ok]").expect("reply");
                processed_tx.send(()).expect("processed");
            } else {
                writer.write_all(b"[Ok]").expect("reply");
            }
            line.clear();
        }
    });
}

/// The asynUser C's `pasynOctetSyncIO->connect` hands the driver.
fn user(handle: PortHandle) -> SyncIOHandle {
    SyncIOHandle::from_handle(handle, 0, Duration::ZERO)
}

fn ip_port(name: &str, addr: &str, input_eos: &[u8], output_eos: &[u8]) -> PortHandle {
    let mut driver = DrvAsynIPPort::new(name, addr).expect("ip port");
    // What `drvAsynIPPortConfigure` does before it starts the port: the EOS
    // bytes are only cached, never applied, unless this layer is there.
    driver.install_interpose(Box::new(EosInterpose::default()));
    let (runtime, _) =
        create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
            .expect("ip port runtime");
    let handle = runtime.port_handle().clone();
    handle
        .set_input_eos_blocking(epics_rs::asyn::user::AsynUser::default(), input_eos)
        .expect("input eos");
    if !output_eos.is_empty() {
        handle
            .set_output_eos_blocking(epics_rs::asyn::user::AsynUser::default(), output_eos)
            .expect("output eos");
    }
    // The runtime must outlive the test.
    std::mem::forget(runtime);
    handle
}

/// A detector wired to a fake BIS, a plugin hanging off it, and a directory for
/// the frames.
struct Fixture {
    detector: BrukerRuntime,
    arrays: mpsc::Receiver<Arc<NDArray>>,
    sync: SyncIOHandle,
    dir: PathBuf,
}

impl Fixture {
    fn new(name: &str) -> Self {
        let command = TcpListener::bind("127.0.0.1:0").expect("command listener");
        let status = TcpListener::bind("127.0.0.1:0").expect("status listener");
        let command_addr = format!("127.0.0.1:{}", command.local_addr().unwrap().port());
        let status_addr = format!("127.0.0.1:{}", status.local_addr().unwrap().port());
        fake_bis(command, status);

        let command_port = ip_port(&format!("{name}_COMMAND"), &command_addr, b"]", b"\n");
        let status_port = ip_port(&format!("{name}_STATUS"), &status_addr, b"\n", b"");

        let detector = create_bruker_detector(
            &claim_port(name).unwrap(),
            user(command_port),
            user(status_port),
            0,
        )
        .expect("the detector port");

        // The plugin: it takes every array the driver publishes and parks it
        // where the test thread can pick it up.
        let (sender, mut receiver) = ndarray_channel(&format!("{name}_PLUGIN"), 4);
        detector.connect_downstream(sender);
        let (published, arrays) = mpsc::channel();
        std::thread::spawn(move || {
            while let Some(array) = receiver.blocking_recv() {
                if published.send(array).is_err() {
                    break;
                }
            }
        });

        let dir = std::env::temp_dir().join(format!("bruker-e2e-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("frame directory");

        // The test thread is a plain thread, so the blocking parameter API is
        // the right one here: it stands in for the IOC's record layer.
        let sync =
            SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));

        let fixture = Fixture {
            detector,
            arrays,
            sync,
            dir,
        };
        fixture.set_up_files();
        fixture
    }

    fn set_up_files(&self) {
        let ad = self.detector.ad_params;
        let path = format!("{}/", self.dir.display());
        self.sync
            .write_octet(ad.base.file_path, path.as_bytes())
            .expect("file path");
        self.sync
            .write_octet(ad.base.file_name, b"frame")
            .expect("file name");
        self.sync
            .write_octet(ad.base.file_template, b"%s%s_%3.3d.sfrm")
            .expect("file template");
        self.sync
            .write_int32(ad.base.file_number, 7)
            .expect("file number");
        self.sync
            .write_int32(ad.base.auto_increment, 1)
            .expect("auto increment");
    }

    fn string(&self, reason: usize) -> String {
        let bytes = self
            .detector
            .port_handle()
            .get_string_param_blocking(reason, 0, 256)
            .expect("string parameter");
        String::from_utf8_lossy(&bytes).into_owned()
    }

    /// The next array the plugin is handed, or nothing within `timeout`.
    fn next_array(&self, timeout: Duration) -> Option<Arc<NDArray>> {
        self.arrays.recv_timeout(timeout).ok()
    }

    /// Poll an integer parameter until it equals `want`, returning the last
    /// value read once it matches or `timeout` elapses. The acquisition task
    /// flips `acquire`/`status` *after* it publishes the array, so a bare read
    /// of them the instant the array arrives races that task under load.
    fn wait_int32(&self, reason: usize, want: i32, timeout: Duration) -> i32 {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let got = self.sync.read_int32(reason).expect("int32 parameter");
            if got == want || std::time::Instant::now() >= deadline {
                return got;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.dir).ok();
    }
}

/// BISDetector.cpp:765-766 sets ADMaxSizeX/Y to 4096 over ADDriver's 1.
#[test]
fn max_size_is_bis_4096() {
    let fixture = Fixture::new("APX_MAXSIZE");
    let ad = fixture.detector.ad_params;
    assert_eq!(fixture.sync.read_int32(ad.max_size_x).unwrap(), 4096);
    assert_eq!(fixture.sync.read_int32(ad.max_size_y).unwrap(), 4096);
}

#[test]
fn one_scan_reaches_the_plugins() {
    let fixture = Fixture::new("APX_SCAN");
    let ad = fixture.detector.ad_params;

    fixture
        .sync
        .write_int32(ad.image_mode, ImageMode::Single as i32)
        .expect("image mode");
    fixture
        .sync
        .write_float64(ad.acquire_time, 0.05)
        .expect("acquire time");
    assert_eq!(
        fixture
            .sync
            .read_int32(ad.base.file_path_exists)
            .expect("path exists"),
        1,
        "the driver checks the path on every FilePath write"
    );

    fixture.sync.write_int32(ad.acquire, 1).expect("acquire");

    let array = fixture
        .next_array(Duration::from_secs(20))
        .unwrap_or_else(|| {
            panic!(
                "no array: status={:?} message={:?} to={:?} from={:?}",
                fixture.sync.read_int32(ad.status),
                fixture.string(ad.status_message),
                fixture.string(ad.string_to_server),
                fixture.string(ad.string_from_server),
            )
        });

    assert_eq!(array.dims.len(), 2);
    assert_eq!(array.dims[0].size, 4, "columns are the fast axis");
    assert_eq!(array.dims[1].size, 2);
    // C allocates NDInt32 (BISDetector.cpp:588) while DataType says UInt32.
    match &array.data {
        NDDataBuffer::I32(data) => assert_eq!(data, &vec![1, 2, 3, 4, 5, 6, 7, 8]),
        other => panic!("the frame is NDInt32 as C allocates it, not {other:?}"),
    }
    assert_eq!(
        fixture
            .sync
            .read_int32(ad.base.data_type)
            .expect("data type"),
        NDDataType::UInt32 as u8 as i32
    );
    assert_eq!(array.unique_id, 1);

    // The file name came from FileTemplate/FileNumber, and the number was
    // bumped for the next frame.
    let expected = fixture.dir.join("frame_007.sfrm");
    assert_eq!(
        PathBuf::from(fixture.string(ad.base.full_file_name)),
        expected
    );
    assert!(expected.exists(), "BIS wrote the frame this driver read");
    assert_eq!(
        fixture
            .sync
            .read_int32(ad.base.file_number)
            .expect("file number"),
        8
    );

    // The acquisition ended by itself: ImageMode was Single. The acquisition
    // task flips these after publishing the array, so wait for the transition
    // rather than racing it.
    assert_eq!(fixture.wait_int32(ad.acquire, 0, Duration::from_secs(5)), 0);
    assert_eq!(
        fixture.wait_int32(ad.status, ADStatus::Idle as i32, Duration::from_secs(5)),
        ADStatus::Idle as i32
    );

    // BISTask never sets the array geometry params: they keep the 0 the
    // constructors set (BISDetector.cpp:769-771, asynNDArrayDriver.cpp:967-971).
    for reason in [
        ad.base.array_size_x,
        ad.base.array_size_y,
        ad.base.array_size,
        ad.base.n_dimensions,
    ] {
        assert_eq!(
            fixture.sync.read_int32(reason).unwrap(),
            0,
            "reason {reason}"
        );
    }

    // The status socket was read: BIS said how big a frame is and how cold it
    // is.
    assert_eq!(fixture.sync.read_int32(ad.size_x).expect("size x"), 1024);
    assert_eq!(fixture.sync.read_int32(ad.size_y).expect("size y"), 1024);
    assert!(
        (fixture
            .sync
            .read_float64(ad.temperature)
            .expect("temperature")
            + 40.5)
            .abs()
            < 1e-9,
        "the temperature BIS broadcast"
    );
}

#[test]
fn a_stop_during_the_exposure_publishes_no_frame() {
    // C signalled one event both for the exposure timer and for a user Stop, so
    // a Stop mid-exposure went on to read the frame file and publish it. Here
    // the frame file exists (BIS wrote it as soon as it took the scan) and BIS
    // has already said it has processed it, and still nothing is published.
    let fixture = Fixture::new("APX_STOP");
    let ad = fixture.detector.ad_params;

    fixture
        .sync
        .write_int32(ad.image_mode, ImageMode::Continuous as i32)
        .expect("image mode");
    fixture
        .sync
        .write_float64(ad.acquire_time, 30.0)
        .expect("acquire time");
    fixture.sync.write_int32(ad.acquire, 1).expect("acquire");

    // Let the scan go out and the countdown start. The actor enters Acquire
    // when it processes the acquire write and BIS writes the frame when the
    // acquisition task sends the scan; both lag the write under load and on
    // independent timelines, so wait for each rather than assuming a fixed
    // delay. The 30 s exposure keeps us mid-scan throughout.
    assert_eq!(
        fixture.wait_int32(ad.status, ADStatus::Acquire as i32, Duration::from_secs(5)),
        ADStatus::Acquire as i32
    );
    let frame = fixture.dir.join("frame_007.sfrm");
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while !frame.exists() && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(frame.exists(), "BIS has written the frame");

    fixture.sync.write_int32(ad.acquire, 0).expect("stop");

    let array = fixture.next_array(Duration::from_secs(2));
    assert!(array.is_none(), "a stopped exposure is not a frame");
    // The stop is handled asynchronously; wait for the transition it drives
    // rather than racing the task that applies it.
    assert_eq!(fixture.wait_int32(ad.acquire, 0, Duration::from_secs(5)), 0);
    assert_eq!(
        fixture.wait_int32(ad.status, ADStatus::Idle as i32, Duration::from_secs(5)),
        ADStatus::Idle as i32
    );
    // Nothing was published, so the array counter still holds the 0 the
    // constructor seeds (asynNDArrayDriver parity since ad-core-rs 0.27.0).
    assert_eq!(
        fixture
            .sync
            .read_int32(ad.base.array_counter)
            .expect("seeded at boot"),
        0
    );
}

/// No caller of C's `writeBIS` looks at its status (BISDetector.cpp:349, :362,
/// :531, :659): with BIS gone a binning write still succeeds, and only a
/// binning C refuses is an error.
#[test]
fn a_binning_write_does_not_depend_on_bis() {
    let closed = |name: &str| {
        // The EOS setters need a connection, which a dead BIS never gives.
        let driver = DrvAsynIPPort::new(name, DEAD_ADDR).expect("ip port");
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
                .expect("ip port runtime");
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        handle
    };
    let detector = create_bruker_detector(
        &claim_port("BISDEAD").unwrap(),
        user(closed("BISDEAD_COMMAND")),
        user(closed("BISDEAD_STATUS")),
        0,
    )
    .expect("the detector port");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    let ad = detector.ad_params;

    sync.write_int32(ad.bin_x, 2).expect("binning 2");
    assert_eq!(sync.read_int32(ad.bin_y).unwrap(), 2);
    let e = sync.write_int32(ad.bin_x, 3).unwrap_err();
    assert_eq!(
        e.message(),
        "",
        "C sets no errorMessage for a refused binning"
    );
}

/// BISDetector.cpp:668 hands base-class writes to ADDriver::writeInt32,
/// which with asynNDArrayDriver::writeInt32 closes with an
/// ASYN_TRACEIO_DRIVER line (ADDriver.cpp:111-113, asynNDArrayDriver.cpp:703-705).
#[test]
fn a_base_class_write_runs_both_base_class_write_int32s() {
    use epics_rs::asyn::trace::{TraceFile, TraceMask};
    let closed = |name: &str| {
        let driver = DrvAsynIPPort::new(name, DEAD_ADDR).expect("ip port");
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
                .expect("ip port runtime");
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        handle
    };
    let name = "BISTRACE";
    let detector = create_bruker_detector(
        &claim_port(name).unwrap(),
        user(closed("BISTRACE_COMMAND")),
        user(closed("BISTRACE_STATUS")),
        0,
    )
    .expect("the detector port");
    let h = detector.port_handle().clone();
    let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    h.trace().set_trace_file(
        Some(name),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    h.trace()
        .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
    let reason = detector.ad_params.base.pool_poll_stats;
    h.write_int32_blocking(reason, 0, 1).unwrap();
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    for driver in ["asynNDArrayDriver", "ADDriver"] {
        assert!(
            text.contains(&format!("{driver}:writeInt32: function={reason}")),
            "{driver}: {text:?}"
        );
    }
}

/// A detector whose BIS sockets refuse every connection.
/// An IP port whose peer is gone.
fn closed_port(name: &str) -> PortHandle {
    let driver = DrvAsynIPPort::new(name, DEAD_ADDR).expect("ip port");
    let (runtime, _) =
        create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
            .expect("ip port runtime");
    let handle = runtime.port_handle().clone();
    std::mem::forget(runtime);
    handle
}

fn dead_detector(name: &str) -> BrukerRuntime {
    create_bruker_detector(
        &claim_port(name).unwrap(),
        user(closed_port(&format!("{name}_COMMAND"))),
        user(closed_port(&format!("{name}_STATUS"))),
        0,
    )
    .expect("the detector port")
}

/// BISDetector.cpp:639: setIntegerParam(function, value) is list 0 whatever
/// the address.
#[test]
fn a_write_at_another_address_sets_list_zero() {
    let detector = dead_detector("BISLIST0");
    let h = detector.port_handle().clone();
    let reason = detector.params.num_darks;
    h.write_int32_blocking(reason, 1, 3).unwrap();
    assert_eq!(h.read_int32_blocking(reason, 0).unwrap(), 3);
}

/// BISDetector.cpp:675-678 prints setIntegerParam's asynParamWrongType for
/// an int written to BISSFRMTimeout.
#[test]
fn a_refused_write_prints_the_param_librarys_status() {
    use epics_rs::asyn::trace::TraceFile;
    let name = "BISSTATUS";
    let detector = dead_detector(name);
    let h = detector.port_handle().clone();
    let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    h.trace().set_trace_file(
        Some(name),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    let reason = detector.params.sfrm_timeout;
    assert!(h.write_int32_blocking(reason, 0, 1).is_err());
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    assert!(
        text.contains(&format!(
            "BISDetector:writeInt32: error, status=8 function={reason}, value=1"
        )),
        "{text}"
    );
}

/// BISTask ignores its getDoubleParam status (BISDetector.cpp:498), so a scan
/// with no ADAcquireTime ever written still goes out to BIS, as 0 s.
#[test]
fn a_scan_without_an_acquire_time_still_reaches_bis() {
    let fixture = Fixture::new("APX_NOTIME");
    let ad = fixture.detector.ad_params;

    fixture.sync.write_int32(ad.acquire, 1).expect("acquire");

    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    let mut sent = fixture.string(ad.string_to_server);
    while !sent.starts_with("[Scan ") && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
        sent = fixture.string(ad.string_to_server);
    }
    assert!(sent.contains("/scantime=0.000000"), "last command {sent:?}");
}

/// C ignores both `pasynOctetSyncIO->connect` statuses (BISDetector.cpp:759-760),
/// so a startup script naming socket ports that do not exist still boots the
/// detector; only its BIS traffic fails.
#[test]
fn a_detector_whose_socket_ports_are_missing_still_boots() {
    let (command, status) =
        SyncIOHandle::connect_port("BISNONE_COMMAND", 0, Duration::ZERO, InterfaceType::Octet);
    assert!(status.is_err());
    let (status_user, status) =
        SyncIOHandle::connect_port("BISNONE_STATUS", 0, Duration::ZERO, InterfaceType::Octet);
    assert!(status.is_err());
    let detector = create_bruker_detector(&claim_port("BISNONE").unwrap(), command, status_user, 0)
        .expect("the detector port");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    let ad = detector.ad_params;

    sync.write_int32(ad.bin_x, 2).expect("binning 2");
    assert_eq!(sync.read_int32(ad.bin_y).unwrap(), 2);
}

/// BISDetector.cpp:316-327: writeBIS sets StringFromServer from `fromBIS`,
/// which a writeRead that fails before its read leaves holding the last
/// reply.
#[test]
fn an_exchange_that_never_reads_keeps_the_last_reply() {
    let detector = dead_detector("BISSTALE");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    let ad = detector.ad_params;
    sync.write_octet(ad.string_from_server, b"old").unwrap();
    sync.write_int32(ad.bin_x, 2).unwrap();
    assert_eq!(
        sync.read_octet(ad.string_from_server, 64).bytes_ok(),
        b"old\0"
    );
    assert!(sync.read_octet(ad.string_to_server, 64).bytes_ok().len() > 1);
}

/// A read that times out leaves in `fromBIS` what it received: nothing.
#[test]
fn an_exchange_whose_read_times_out_publishes_what_it_read() {
    let command = TcpListener::bind("127.0.0.1:0").expect("command listener");
    let command_addr = format!("127.0.0.1:{}", command.local_addr().unwrap().port());
    std::thread::spawn(move || {
        let held: Vec<TcpStream> = command.incoming().filter_map(Result::ok).collect();
        drop(held);
    });
    let detector = create_bruker_detector(
        &claim_port("BISSILENT").unwrap(),
        user(ip_port("BISSILENT_COMMAND", &command_addr, b"]", b"")),
        user(closed_port("BISSILENT_STATUS")),
        0,
    )
    .expect("the detector port");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    let ad = detector.ad_params;
    sync.write_octet(ad.string_from_server, b"old").unwrap();
    sync.write_int32(ad.bin_x, 2).unwrap();
    assert_eq!(sync.read_octet(ad.string_from_server, 64).bytes_ok(), b"\0");
}

/// BISDetector.cpp:404-434: statusTask posts every successful read as read,
/// whitespace and empty reads included.
#[test]
fn every_status_read_is_posted_as_read() {
    let status = TcpListener::bind("127.0.0.1:0").expect("status listener");
    let status_addr = format!("127.0.0.1:{}", status.local_addr().unwrap().port());
    std::thread::spawn(move || {
        let (mut stream, _) = status.accept().expect("status connection");
        stream
            .write_all(b"[DETECTORSTATUS /FRAMESIZE=1024 /CCDTEMP=-40.5] \r\n\n")
            .expect("status messages");
        std::thread::sleep(Duration::from_secs(30));
    });
    let detector = create_bruker_detector(
        &claim_port("BISPOSTED").unwrap(),
        user(closed_port("BISPOSTED_COMMAND")),
        user(ip_port("BISPOSTED_STATUS", &status_addr, b"\n", b"")),
        0,
    )
    .expect("the detector port");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    let (reason, size_x) = (detector.params.status, detector.ad_params.size_x);
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    // BIS_STATUS is undefined until statusTask's first post, as in C.
    let read = || {
        (sync.read_int32(size_x).unwrap(), {
            let (bytes, status) = sync.read_octet(reason, 128).into_parts();
            status.ok().map(|()| bytes)
        })
    };
    let mut got = read();
    while got != (1024, Some(b"\0".to_vec())) && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
        got = read();
    }
    assert_eq!(got.0, 1024);
    assert_eq!(got.1.as_deref(), Some(&b"\0"[..]));
}

/// BISDetector.cpp never sets NDDriverVersion, so it keeps the ADCore version
/// asynNDArrayDriver gave it (asynNDArrayDriver.cpp:966).
#[test]
fn the_driver_version_is_left_to_adcore() {
    let missing =
        |name| SyncIOHandle::connect_port(name, 0, Duration::ZERO, InterfaceType::Octet).0;
    let detector = create_bruker_detector(
        &claim_port("BISVERSION").unwrap(),
        missing("BISVERSION_COMMAND"),
        missing("BISVERSION_STATUS"),
        0,
    )
    .expect("the detector port");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    let ad = detector.ad_params;
    assert_eq!(
        sync.read_octet(ad.base.driver_version, 64).bytes_ok(),
        sync.read_octet(ad.base.ad_core_version, 64).bytes_ok()
    );
}

/// `writeBIS` and `statusTask` talk to BIS through `pasynOctetSyncIO`
/// (BISDetector.cpp:316, :404), whose writeRead and read print their
/// TRACEIO_DEVICE lines (asynOctetSyncIO.c:194-200, :219-223, :259-266).
#[test]
fn the_bis_sockets_print_their_sync_io_lines() {
    use epics_rs::asyn::trace::{TraceFile, TraceInfoMask, TraceIoMask, TraceMask};
    let command = TcpListener::bind("127.0.0.1:0").expect("command listener");
    let status = TcpListener::bind("127.0.0.1:0").expect("status listener");
    let command_addr = format!("127.0.0.1:{}", command.local_addr().unwrap().port());
    let status_addr = format!("127.0.0.1:{}", status.local_addr().unwrap().port());
    fake_bis(command, status);

    let name = "BISSYNCIO";
    let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
    let file = Arc::new(std::sync::Mutex::new(std::fs::File::create(&path).unwrap()));
    let traced = |port: &str, addr: &str, input_eos: &[u8], output_eos: &[u8]| {
        let handle = ip_port(port, addr, input_eos, output_eos);
        handle
            .trace()
            .set_trace_file(Some(port), TraceFile::File(file.clone()));
        handle
            .trace()
            .set_trace_mask(Some(port), TraceMask::IO_DEVICE);
        handle
            .trace()
            .set_trace_info_mask(Some(port), TraceInfoMask::PORT);
        handle
            .trace()
            .set_trace_io_mask(Some(port), TraceIoMask::ASCII);
        handle
    };
    let detector = create_bruker_detector(
        &claim_port(name).unwrap(),
        user(traced("BISSYNCIO_COMMAND", &command_addr, b"]", b"\n")),
        user(traced("BISSYNCIO_STATUS", &status_addr, b"\n", b"")),
        0,
    )
    .expect("the detector port");
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    sync.write_int32(detector.ad_params.bin_x, 2)
        .expect("binning 2");

    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    let text = loop {
        let text = std::fs::read_to_string(&path).unwrap();
        if (text.contains("asynOctetSyncIO read:\n[DETECTORSTATUS")
            && text.contains("asynOctetSyncIO wrote:\n[ChangeFrameSize"))
            || std::time::Instant::now() >= deadline
        {
            break text;
        }
        std::thread::sleep(Duration::from_millis(20));
    };
    let _ = std::fs::remove_file(&path);
    assert!(text.contains("BISSYNCIO_COMMAND"), "{text}");
    assert!(
        text.contains("asynOctetSyncIO wrote:\n[ChangeFrameSize"),
        "{text}"
    );
    assert!(
        text.contains("asynOctetSyncIO read:\n[DETECTORSTATUS"),
        "{text}"
    );
}
