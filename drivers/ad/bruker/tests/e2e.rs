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
use epics_rs::asyn::interpose::eos::EosInterpose;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::sync_io::SyncIOHandle;

use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer};
use epics_rs::ad_core::plugin::channel::ndarray_channel;

use bruker::{BrukerRuntime, create_bruker_detector};

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

fn ip_port(name: &str, addr: &str, input_eos: &[u8], output_eos: &[u8]) -> PortHandle {
    let mut driver = DrvAsynIPPort::new(name, addr).expect("ip port");
    // What `drvAsynIPPortConfigure` does before it starts the port: the EOS
    // bytes are only cached, never applied, unless this layer is there.
    driver.install_interpose(Box::new(EosInterpose::default()));
    let (runtime, _) = create_port_runtime(driver, RuntimeConfig::default());
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

        let detector =
            create_bruker_detector(name, command_port, status_port, 0).expect("the detector port");

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
        let bytes = self.sync.read_octet(reason, 256).expect("string parameter");
        String::from_utf8_lossy(&bytes)
            .trim_end_matches('\0')
            .to_string()
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
    match &array.data {
        NDDataBuffer::U32(data) => assert_eq!(data, &vec![1, 2, 3, 4, 5, 6, 7, 8]),
        other => panic!("the frame is 32-bit unsigned, not {other:?}"),
    }
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
    // Nothing was published, so the array counter was never touched: it is
    // still undefined, as it is at boot.
    assert!(fixture.sync.read_int32(ad.base.array_counter).is_err());
}
