# D435i RealSense areaDetector IOC

An epics-rs based areaDetector IOC for the Intel RealSense D435i camera.
A single pipeline outputs Color (RGB8) and Depth (Z16) simultaneously on two separate ports, with IMU data published as PVs.

## Architecture

```
RealSense Pipeline
    |
    +- ColorFrame (RGB8) --> RS1       (Color ADDriver Port)
    +- DepthFrame (Z16)  --> RS1_DEPTH (Depth ADDriver Port)
    +- AccelFrame        --> RS1:cam1:RSAccelX/Y/Z_RBV
    +- GyroFrame         --> RS1:cam1:RSGyroX/Y/Z_RBV
```

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
| `RSResX` / `RSResY` | longout | Requested resolution (default 640x480) |
| `RSFrameRate` | longout | Requested FPS (default 30) |

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

| PV | Format | Description |
|----|--------|-------------|
| `RS1:image1:ArrayData` | UInt8 (RGB) | Color image data |
| `RS1:image2:ArrayData` | Int16 (Mono) | Depth image data |

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

# Change resolution (pipeline restarts automatically during acquisition)
caput RS1:cam1:RSResX 1280
caput RS1:cam1:RSResY 720

# Stop acquisition
caput RS1:cam1:Acquire 0
```

## Project Structure

```
iocs/d435i/
├── Cargo.toml
├── src/
│   ├── lib.rs              # Module declarations
│   ├── types.rs            # AcqCommand, DirtyFlags
│   ├── params.rs           # D435iParams, D435iConfigSnapshot
│   ├── driver.rs           # D435iColorDriver, D435iDepthDriver, runtimes
│   ├── task.rs             # Acquisition loop (pipeline management, frame processing)
│   ├── ioc_support.rs      # IOC registration, device support
│   └── bin/
│       └── d435i_ioc.rs    # IOC binary entry point
├── db/
│   ├── d435i_color.template  # Color port EPICS records
│   └── d435i_depth.template  # Depth port EPICS records
└── ioc/
    └── st.cmd              # IOC startup script
```
