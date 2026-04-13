# D435i RealSense areaDetector IOC

An epics-rs (v0.9) based areaDetector IOC for the Intel RealSense D435i
camera. A single pipeline produces three NDArray outputs simultaneously
(Color RGB8, Depth Z16, optional XYZ Pointcloud) and publishes IMU data
as PVs.

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
- [librealsense2](https://github.com/IntelRealSense/librealsense) installed on the system
  - macOS: `brew install librealsense`
  - Ubuntu: `sudo apt install librealsense2-dev`

## Build

```bash
# Debug build
cargo build --features ioc

# Release build (recommended)
cargo build --release --features ioc
```

## Run

Connect a D435i camera to a USB 3.0 port, then:

```bash
# Run with debug build
cargo run --features ioc --bin d435i_ioc -- ioc/st.cmd

# Run with release build (recommended)
cargo run --release --features ioc --bin d435i_ioc -- ioc/st.cmd
```

Or run the compiled binary directly:

```bash
./target/release/d435i_ioc ioc/st.cmd
```

## Startup Script (st.cmd)

Camera settings can be configured in `ioc/st.cmd`.

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

## Plugin Configuration

Each NDArray output port has its own plugin script, loaded from
`ioc/st.cmd`:

| Script | Applied to | Plugins |
|--------|------------|---------|
| `d435iColorPlugins.cmd` | `RS1` (RGB8)     | Full `commonPlugins.cmd` (StdArrays image1, ROI, Stats, Over, Trans, Process, CB, Attr, FFT, Codec, TIFF, JPEG, HDF5, Nexus, NetCDF, ColorConvert, PVA, ...) |
| `d435iDepthPlugins.cmd` | `RS1_DEPTH` (Z16) | StdArrays image2, ROI, ROIStat, Stats, TIFF, HDF5 — JPEG/ColorConvert skipped (not meaningful for mono Z16) |
| `d435iPCPlugins.cmd`    | `RS1_PC` (Float32 XYZ) | StdArrays image3 + HDF5 only — most AD plugins cannot process a (3, W, H) vertex array |

To add or remove plugins for a given port, edit the corresponding
`.cmd` file rather than the shared `commonPlugins.cmd`.

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

## Project Structure

```
epics-rs-iocs/
├── Cargo.toml
├── src/
│   ├── lib.rs              # Module declarations
│   ├── types.rs            # AcqCommand, DirtyFlags
│   ├── params.rs           # D435iParams, D435iConfigSnapshot
│   ├── driver.rs           # D435iColorDriver, D435iDepthDriver, runtimes
│   ├── task.rs             # Acquisition loop (pipeline, filters, publish)
│   ├── ioc_support.rs      # d435iConfig command + wiring registration
│   └── bin/
│       └── d435i_ioc.rs    # IOC binary entry point
├── db/
│   ├── d435i_color.template  # Color port EPICS records (standard asyn DTYPs)
│   └── d435i_depth.template  # Depth port EPICS records (NDArrayBase include)
├── display/
│   ├── d435i_main.py         # Main detector control display (PyDM)
│   └── d435i_dual_view.py    # Dual color+depth image viewer (PyDM)
└── ioc/
    ├── st.cmd                   # IOC startup script
    ├── d435iColorPlugins.cmd    # Full plugin chain for RS1 (RGB8)
    ├── d435iDepthPlugins.cmd    # Lean chain for RS1_DEPTH (Z16)
    └── d435iPCPlugins.cmd       # Minimal chain for RS1_PC (XYZ Float32)
```
