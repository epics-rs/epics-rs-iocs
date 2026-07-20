# epics-rs-iocs

[![Rust](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/rust.yml/badge.svg)](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/rust.yml)
[![Cross-platform build](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/cross-platform.yml/badge.svg)](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/cross-platform.yml)

Cargo workspace containing epics-rs based IOC applications.
Each device driver is an independent library crate under `drivers/`, and
each IOC binary lives under `iocs/`.

> **Platform**: Linux is the primary, fully-supported target — every
> crate builds and is tested there. CI additionally builds and tests the
> workspace on macOS and Windows (arm64 + x86_64), **excluding** the
> vendor-SDK crates whose C libraries are Linux-oriented: the measComp
> family (`uldaq-sys`, `meascomp`, `usb-2408`, `usb-ctr`, and their IOCs)
> and the RealSense `d435i` driver/IOC. POSIX-only behaviour degrades
> gracefully off Linux — e.g. the Eiger file-writer's file-permission
> bits are a no-op on Windows.

## Workspace Structure

```
epics-rs-iocs/
├── Cargo.toml                        # Workspace root
├── drivers/
│   ├── meascomp/                     # Measurement Computing (MCC) driver family
│   │   ├── uldaq-sys/                # Raw FFI bindings to libuldaq
│   │   ├── meascomp/                 # Safe wrapper (DaqDevice, DIO, counter, timer, AI/AO)
│   │   ├── usb-ctr/                  # USB-CTR08 PortDriver (counters, pulse gen, DIO)
│   │   └── usb-2408/                 # USB-2408-2AO PortDriver (AI, AO, temp, DIO)
│   └── d435i/                        # RealSense D435i areaDetector driver
├── iocs/
│   ├── usb-ctr-ioc/                  # USB-CTR08 IOC binary + st.cmd
│   ├── usb-2408-ioc/                 # USB-2408-2AO IOC binary + st.cmd
│   └── d435i-ioc/                    # D435i IOC binary + st.cmd + plugin cmds
├── db/                               # Shared EPICS templates
│   ├── meascomp_device.template      # Board info (model, firmware, UL version)
│   ├── meascomp_counter.template     # Counter counts + reset
│   ├── meascomp_pulse_gen.template   # Pulse generator control
│   ├── meascomp_binary_in.template   # Digital input (per-bit)
│   ├── meascomp_binary_out.template  # Digital output (per-bit)
│   ├── meascomp_analog_in.template   # Voltage input + range selector
│   ├── meascomp_temperature.template # Thermocouple input
│   ├── meascomp_analog_out.template  # DAC output
│   ├── d435i_color.template
│   └── d435i_depth.template
└── display/                          # Shared PyDM displays
```

### Adding a New Device

1. Create `drivers/<device>/` with a library crate (driver logic only, no IOC deps)
2. Create `iocs/<device>-ioc/` with a binary crate that depends on the driver
3. Add both to the workspace `members` in the root `Cargo.toml`

### Vendor SDKs and workspace-wide checks

Some driver crates link vendor SDKs, so `cargo clippy --workspace` /
`cargo nextest run --workspace` only pass on a machine with those SDKs
installed:

- **d435i / d435i-ioc** need [librealsense2](https://github.com/IntelRealSense/librealsense)
  (`realsense-sys`'s build script fails without `realsense2.pc` on
  `PKG_CONFIG_PATH`, which fails even `cargo clippy --workspace`).
  Ubuntu: `sudo apt install librealsense2-dev`.
- **meascomp / usb-ctr / usb-2408** (and their IOCs) need
  [libuldaq](https://github.com/mccdaq/uldaq). `clippy`/`check` pass
  without it, but building test or IOC binaries fails at link
  (`-luldaq`).

On a machine without the SDKs, scope checks to the crates you touched,
e.g. `cargo clippy -p motor-newport -p xps-ioc --all-targets -- -D warnings`.

---

# USB-CTR08 Counter/Timer IOC

8-channel 32-bit counter/timer with 4 pulse generators and 8-bit digital I/O.
Ported from [measComp](https://github.com/epics-modules/measComp) `drvUSBCTR`.

## Build

```bash
cargo build -p usb-ctr-ioc --release
```

## Run

Connect a USB-CTR08 to a USB port, then:

```bash
cargo run -p usb-ctr-ioc --release -- iocs/usb-ctr-ioc/st.cmd
```

Edit `iocs/usb-ctr-ioc/st.cmd` to set your device serial number:

```
epicsEnvSet("UNIQUE_ID", "0214D582")
```

An empty `UNIQUE_ID` connects to the first available device, which is
**not safe when multiple MCC devices are plugged in** — both this IOC
and `usb-2408-ioc` would grab the same `descriptors[0]`. Always set
`UNIQUE_ID` explicitly in multi-device setups.

To list all connected MCC devices and their UNIQUE_IDs:

```bash
cargo run -p meascomp --bin list-devices --release
```

Example output:

```
Found 2 device(s):
#    product_name                   unique_id            product_id
0    USB-2408-2AO                   01DA523D             0x00fe
1    USB-CTR08                      01DAB0FB             0x0127
```

Copy the relevant `unique_id` into the matching `st.cmd`.

## Quick Test

```bash
# Start a 1kHz pulse on timer 0
caput USBCTR:PulseGen1Period 0.001
caput USBCTR:PulseGen1Run 1

# Read counter 1
caget USBCTR:Counter1Counts

# Reset counter 1
caput USBCTR:Counter1Reset 1

# Read digital input bit 1
caget USBCTR:Bi1
```

---

# USB-2408-2AO Analog I/O IOC

8-channel 24-bit analog input (voltage + thermocouple), 2-channel 16-bit
analog output, 8-bit digital I/O, and 2 counters.
Ported from [measComp](https://github.com/epics-modules/measComp) `drvMultiFunction`.

## Build

```bash
cargo build -p usb-2408-ioc --release
```

## Run

```bash
cargo run -p usb-2408-ioc --release -- iocs/usb-2408-ioc/st.cmd
```

Edit `iocs/usb-2408-ioc/st.cmd` to set your device serial number:

```
epicsEnvSet("UNIQUE_ID", "01AAA83E")
```

Use the device discovery tool to find the real UNIQUE_ID (see the
USB-CTR08 section above for details):

```bash
cargo run -p meascomp --bin list-devices --release
```

## Quick Test

```bash
# Read voltage on channel 1 (±10V range)
caget USB2408:Ai1

# Read thermocouple temperature on channel 1
caput USB2408:Ai1Type 1          # Switch to TC mode
caput USB2408:Ti1TCType 1        # J-type thermocouple
caget USB2408:Ti1

# Set analog output 1 to mid-scale
caput USB2408:Ao1 32768

# Start waveform digitizer (8 channels, 1000 points)
caput USB2408:WaveDigNumPoints 1000
caput USB2408:WaveDigDwell 0.001
caput USB2408:WaveDigRun 1
```

---

# D435i RealSense areaDetector IOC

An epics-rs (v0.9) based areaDetector IOC for the Intel RealSense D435i
camera. A single pipeline produces three NDArray outputs simultaneously
(Color RGB8, Depth Z16, optional XYZ Pointcloud) and publishes IMU data
as PVs.

The IOC serves both **Channel Access (CA)** and **pvAccess (PVA)** — every
record is reachable via `caget`/`camonitor` *and* `pvget`/`pvmonitor`
simultaneously. PVA is wired through `epics-bridge-rs` (QSRV-equivalent)
via `AdIoc::run_from_args_with_pva`.

## Architecture

```
RealSense Pipeline
    |
    +- ColorFrame (RGB8)   --> RS1       (Color ADDriver port, full plugin chain)
    +- DepthFrame (Z16)    --> RS1_DEPTH (Depth ADDriver port, lean plugin chain)
    +- PointCloud (Float32)--> RS1_PC    (NDArray source only, minimal chain)
    +- AccelFrame          --> RS1:cam1:RSAccelX/Y/Z_RBV
    +- GyroFrame           --> RS1:cam1:RSGyroX/Y/Z_RBV
```

`RS1_PC` is not a full ADDriver port — it is a secondary `NDArrayOutput`
on the color driver, registered in the plugin wiring registry so plugins
can attach via `NDARRAY_PORT=RS1_PC`. It does not carry control records.

## Prerequisites

- Rust toolchain (stable)
- [librealsense2](https://github.com/IntelRealSense/librealsense) (for d435i driver)
  - Ubuntu: `sudo apt install librealsense2-dev`

## Build

```bash
# Debug build
cargo build -p d435i-ioc

# Release build (recommended)
cargo build -p d435i-ioc --release
```

## Run

Connect a D435i camera to a USB 3.0 port, then from the workspace root:

```bash
# Run with debug build
cargo run -p d435i-ioc -- iocs/d435i-ioc/st.cmd

# Run with release build (recommended)
cargo run -p d435i-ioc --release -- iocs/d435i-ioc/st.cmd
```

Or run the compiled binary directly:

```bash
./target/release/d435i-ioc iocs/d435i-ioc/st.cmd
```

> The bin target is `d435i-ioc` (hyphen, not underscore), and the startup
> script path is `iocs/d435i-ioc/st.cmd` relative to the workspace root.

## Startup Script (st.cmd)

Camera settings can be configured in `iocs/d435i-ioc/st.cmd`.

```bash
# d435iConfig(portName, serial, maxSizeX, maxSizeY, maxMemory)
# An empty serial string uses the first available D435i.
d435iConfig("RS1", "", 1920, 1080, 100000000)
```

To target a specific camera, provide its serial number:

```bash
d435iConfig("RS1", "012345678901", 1920, 1080, 100000000)
```

## PV Reference

### Color Port (`RS1:cam1:`)

| PV | Type | Description |
|----|------|-------------|
| `Acquire` | bo | Start/stop acquisition |
| `ImageMode` | mbbo | Single / Multiple / Continuous |
| `ArrayCounter_RBV` | longin | Frame counter |
| `ArraySizeX_RBV` | longin | Image width |
| `ArraySizeY_RBV` | longin | Image height |

### D435i Stream Configuration (`RS1:cam1:`)

| PV | Type | Description |
|----|------|-------------|
| `RSStreamMode` | mbbo | Stream mode selector (see table below) |
| `RSResX_RBV` | longin | Active resolution width (read-only) |
| `RSResY_RBV` | longin | Active resolution height (read-only) |
| `RSFrameRate_RBV` | longin | Active frame rate (read-only) |

Available stream modes (valid for both Color RGB8 and Depth Z16):

| Index | Mode |
|-------|------|
| 0 | 424x240 @ 15fps |
| 1 | 424x240 @ 30fps |
| 2 | 424x240 @ 60fps |
| 3 | 640x360 @ 15fps |
| 4 | 640x360 @ 30fps |
| 5 | 640x360 @ 60fps |
| 6 | 640x480 @ 15fps |
| 7 | 640x480 @ 30fps (default) |
| 8 | 640x480 @ 60fps |
| 9 | 848x480 @ 15fps |
| 10 | 848x480 @ 30fps |
| 11 | 848x480 @ 60fps |
| 12 | 1280x720 @ 6fps |
| 13 | 1280x720 @ 15fps |
| 14 | 1280x720 @ 30fps |

### Sensor Options (`RS1:cam1:`)

| PV | Type | Unit | Description |
|----|------|------|-------------|
| `RSExposure` | ao | us | Exposure time |
| `RSGain` | ao | | Sensor gain |
| `RSAutoExposure` | bo | | Auto-exposure On/Off |
| `RSLaserPower` | ao | mW | IR laser power |
| `RSEmitterEnabled` | bo | | IR emitter On/Off |
| `RSDepthUnits_RBV` | ai | m/unit | Depth scale (read-only) |

### IMU (`RS1:cam1:`)

| PV | Type | Unit | Description |
|----|------|------|-------------|
| `RSAccelX/Y/Z_RBV` | ai | m/s^2 | Accelerometer |
| `RSGyroX/Y/Z_RBV` | ai | rad/s | Gyroscope |

### Device Info (`RS1:cam1:`)

| PV | Type | Description |
|----|------|-------------|
| `Manufacturer_RBV` | stringin | Manufacturer (Intel) |
| `Model_RBV` | stringin | Model name |
| `RSSerial_RBV` | stringin | Serial number |
| `FirmwareVersion_RBV` | stringin | Firmware version |
| `RSConnected_RBV` | bi | Connection status |

### Depth Port (`RS1:depth1:`)

| PV | Type | Description |
|----|------|-------------|
| `ArrayCounter_RBV` | longin | Frame counter |
| `ArraySizeX/Y_RBV` | longin | Image size |
| `UniqueId_RBV` | longin | Unique ID |
| `Manufacturer_RBV` | stringin | Manufacturer |
| `Model_RBV` | stringin | Model name |

### Image Arrays (NDStdArrays Plugin)

| PV | Format | Source port | Description |
|----|--------|-------------|-------------|
| `RS1:image1:ArrayData` | UInt8 (RGB)   | `RS1`       | Color image data |
| `RS1:image2:ArrayData` | Int16 (Mono)  | `RS1_DEPTH` | Depth image data |
| `RS1:image3:ArrayData` | Float32 (XYZ) | `RS1_PC`    | Pointcloud vertices |

## PyDM Displays

GUI displays built with [PyDM](https://slaclab.github.io/pydm/) are provided for detector control and image viewing.

### Requirements

```bash
pip install pydm
```

### Main Control Display (`display/d435i_main.py`)

Full control panel for the D435i camera.

- **Device Info**: Model, Serial, Firmware, Connection status
- **Acquire**: Start/stop acquisition, ImageMode, DetectorState, ArrayCounter
- **Stream Config**: Resolution/frame-rate mode selector
- **Sensor Controls**: Exposure, Gain, AutoExposure, LaserPower, Emitter
- **Depth Info**: Depth units (m/unit)
- **IMU Readback**: Accelerometer/Gyroscope X/Y/Z
- **Image Viewers**: Button to launch dual viewer
- **Array Info**: Color/Depth image dimensions and callback settings

```bash
pydm display/d435i_main.py -m '{"P":"RS1:"}'
```

### Dual Image Viewer (`display/d435i_dual_view.py`)

Side-by-side live display of Color (RGB) and Depth (Z16) images.

- Left panel: Color image (`RS1:image1:ArrayData`)
- Right panel: Depth image (`RS1:image2:ArrayData`)
- Bottom toolbar: Depth colormap selector (inferno, viridis, plasma, etc.), Acquire control

```bash
pydm display/d435i_dual_view.py -m '{"P":"RS1:"}'
```

### Using a Custom Prefix

When running multiple cameras, change the `P` macro:

```bash
pydm display/d435i_main.py -m '{"P":"RS2:"}'
```

## Quick Test

```bash
# Start acquisition
caput RS1:cam1:Acquire 1

# Check color frame count
caget RS1:cam1:ArrayCounter_RBV

# Check depth frame count
caget RS1:depth1:ArrayCounter_RBV

# Read IMU (expect ~9.8 m/s^2 on Y-axis when level)
caget RS1:cam1:RSAccelY_RBV

# Change stream mode to 1280x720 @ 30fps (pipeline restarts automatically)
caput RS1:cam1:RSStreamMode 14

# Stop acquisition
caput RS1:cam1:Acquire 0
```

The same records are available over pvAccess — substitute `pvget`/`pvput`
for `caget`/`caput`:

```bash
pvget  RS1:cam1:ArrayCounter_RBV
pvput  RS1:cam1:Acquire 1
pvmonitor RS1:cam1:RSAccelY_RBV
```

## Plugin Configuration (D435i)

Each NDArray output port has its own plugin script, loaded from
`iocs/d435i-ioc/st.cmd`:

| Script | Applied to | Plugins |
|--------|------------|---------|
| `d435iColorPlugins.cmd` | `RS1` (RGB8)     | Full `commonPlugins.cmd` chain |
| `d435iDepthPlugins.cmd` | `RS1_DEPTH` (Z16) | StdArrays, ROI, ROIStat, Stats, TIFF, HDF5 |
| `d435iPCPlugins.cmd`    | `RS1_PC` (Float32 XYZ) | StdArrays + HDF5 only |
