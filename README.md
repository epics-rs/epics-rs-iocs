# epics-rs-iocs

[![Rust](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/rust.yml/badge.svg)](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/rust.yml)
[![Cross-platform build](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/cross-platform.yml/badge.svg)](https://github.com/epics-rs/epics-rs-iocs/actions/workflows/cross-platform.yml)

Cargo workspace of [epics-rs](https://github.com/epics-rs/epics-rs) based
IOC applications — Rust ports of the EPICS device-driver modules. Each
device driver is an independent library crate under `drivers/`, and each
IOC binary lives under `iocs/`. The workspace currently holds **67 driver
crates** and **82 IOC crates**, all consuming a single pinned epics-rs
version (**0.24.3**) declared once in the root `Cargo.toml`.

> **Platform**: Linux is the primary, fully-supported target — every
> crate builds and is tested there. CI additionally builds and tests the
> workspace on macOS and Windows (arm64 + x86_64), **excluding** the
> vendor-SDK crates whose C libraries are Linux-oriented: the measComp
> family (`uldaq-sys`, `meascomp`, `usb-2408`, `usb-ctr`, and their IOCs)
> and the RealSense `d435i` driver/IOC. POSIX-only behaviour degrades
> gracefully off Linux — e.g. the Eiger file-writer's file-permission
> bits are a no-op on Windows.

## Contents

- [Workspace Structure](#workspace-structure)
- [Adding a New Device](#adding-a-new-device)
- [Vendor SDKs and workspace-wide checks](#vendor-sdks-and-workspace-wide-checks)
- **Driver & IOC catalog**
  - [Motor drivers](#motor-drivers)
  - [AreaDetector drivers (part 1)](#areadetector-drivers-part-1)
  - [AreaDetector drivers (part 2)](#areadetector-drivers-part-2)
  - [quadEM, MCA & Scaler](#quadem-mca--scaler)
  - [Vacuum, fieldbus & miscellaneous drivers](#vacuum-fieldbus--miscellaneous-drivers)
- **Worked examples (deep dives)**
  - [USB-CTR08 Counter/Timer IOC](#usb-ctr08-countertimer-ioc)
  - [USB-2408-2AO Analog I/O IOC](#usb-2408-2ao-analog-io-ioc)
  - [D435i RealSense areaDetector IOC](#d435i-realsense-areadetector-ioc)

## Workspace Structure

Drivers are grouped into families (`drivers/<family>/<device>`) where a
family shares a record type, template set, or vendor SDK; standalone
device drivers live directly under `drivers/`. IOC binaries mirror the
same layout under `iocs/`.

```
epics-rs-iocs/
├── Cargo.toml                  # Workspace root — pins epics-rs 0.24.3 for all crates
├── drivers/
│   ├── motor/                  # 27 motor-controller drivers + shared `common` crate
│   │                           #   acs, acsmotion, acstech80, aerotech, amci, attocube,
│   │                           #   faulhaber, ims, kohzu, mclennan, micos, micromo,
│   │                           #   micronix, motorsim, newport, npoint, oms-asyn, oriel,
│   │                           #   parker, phytron, pi, pi-gcs2, pijena, pmac, smaract,
│   │                           #   smartmotor, thorlabs   (all bind the standard motor record)
│   ├── ad/                     # 17 areaDetector drivers
│   │                           #   bruker, csimdetector, eiger, mar345, marccd, merlin,
│   │                           #   mythen, photonii, pilatus, pixirad, psl, pva-driver,
│   │                           #   simdetector, specs-analyser, std-arrays-driver,
│   │                           #   timepix3, url
│   ├── meascomp/               # Measurement Computing (MCC) family
│   │   ├── uldaq-sys/          #   Raw FFI bindings to libuldaq
│   │   ├── meascomp/           #   Safe wrapper (DaqDevice, DIO, counter, timer, AI/AO)
│   │   ├── usb-ctr/            #   USB-CTR08 PortDriver (counters, pulse gen, DIO)
│   │   └── usb-2408/           #   USB-2408-2AO PortDriver (AI, AO, temp, DIO)
│   ├── quadem/                 # quadEM 4-channel electrometers
│   ├── scaler974/              # SIS3820 / Joerger scaler
│   ├── mca/                    # MCA foundation (mcaRecord device support)
│   ├── mca-amptek/             # Amptek DP5 (UDP/NetFinder)
│   ├── mca-rontec/             # Rontec MCA
│   ├── d435i/                  # Intel RealSense D435i areaDetector driver
│   ├── vac/                    # Vacuum gauges + ion pumps (custom vs/digitel records)
│   ├── ip/                     # epics-modules `ip` serial instruments (crate `ip-devices`)
│   ├── ether-ip/               # Allen-Bradley EtherNet/IP + CIP
│   ├── twincat-ads/            # Beckhoff TwinCAT ADS/AMS
│   ├── opcua/                  # OPC-UA client device support
│   ├── ur-robot/               # Universal Robots arm (RTDE/script/dashboard/gripper)
│   ├── love/                   # Love Controls PID controller (RS-485)
│   ├── delaygen/               # SRS DG645 / Colby PDL-100A / Coherent SDG
│   ├── syringepump/            # Teledyne ISCO / ISCO Modbus / Vindum pumps
│   ├── microepsilon/           # capaNCDT6200 displacement sensor
│   ├── yokogawa-gm10/          # Yokogawa GM10 data acquisition
│   └── yokogawa-mw100/         # Yokogawa MW100 data acquisition
├── iocs/                       # One IOC binary crate per device (+ st.cmd, db/, display/)
├── db/                         # Shared EPICS templates (measComp, d435i)
└── display/                    # Shared PyDM displays
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

# Driver & IOC catalog

Every driver is a Rust port of an upstream EPICS module; each section
below links its upstream source and lists the IOC binary, its build/run
command, and the record/PV surface it exposes. The three measComp/RealSense
devices have fuller worked-example write-ups in the
[deep-dive sections](#usb-ctr08-countertimer-ioc) at the end.

## Motor drivers

The MOTOR family ports [epics-modules/motor](https://github.com/epics-modules/motor) (plus the
Delta Tau `tpmac` module for the PMAC/Geobrick) to `epics-rs`: 37 IOC crates under `iocs/motor/`,
each pairing one `db/*.template` with one vendor driver crate under `drivers/motor/` (27 vendor
crates, plus the shared `motor-common` plumbing crate — 28 crate directories in total).

### Motor record (shared)

Every motor IOC in this workspace loads records of the single standard EPICS `motor` record type
at `$(P)$(M)` (or an axis-letter macro variant such as `$(P)$(MX)`, `$(P)$(MU)`, `$(P)$(M0)` for
multi-axis controllers) — confirmed by grepping `record(` across all 54 `db/*.template` files in
`iocs/motor/*/db/`: every single one is `record(motor, ...)`, with no other record type anywhere
in the MOTOR family. Field definitions below are taken directly from the vendored
`motorRecord.dbd` (`~/.cargo/registry/.../motor-rs-0.24.0/dbd/motorRecord.dbd`, mirrored at
`crates/motor-rs/dbd/motorRecord.dbd` in `epics-rs`):

| Field | Type | Purpose |
|---|---|---|
| `.VAL` | `DBF_DOUBLE` | User Desired Value (EGU) |
| `.RBV` | `DBF_DOUBLE` | User Readback Value |
| `.MOVN` | `DBF_SHORT` | Motor is moving |
| `.DMOV` | `DBF_SHORT` | Done moving to value |
| `.HLM` | `DBF_DOUBLE` | User High Limit |
| `.LLM` | `DBF_DOUBLE` | User Low Limit |
| `.VELO` | `DBF_DOUBLE` | Velocity (EGU/s) |
| `.ACCL` | `DBF_DOUBLE` | Seconds to Velocity |
| `.HOMF` | `DBF_SHORT` | Home Forward |
| `.HOMR` | `DBF_SHORT` | Home Reverse |
| `.STOP` | `DBF_SHORT` | Stop |

Individual `db/*.template` files typically set only a handful of these per axis at load time
(`DTYP`, `SCAN`, `VELO`, `ACCL`, `DHLM`/`DLLM`, `MRES`, `EGU`, `PREC` — see e.g.
`iocs/motor/esp300-ioc/db/esp300.template`); the rest are standard motor-record fields provided
by the record type itself regardless of template overrides.

### Motor IOCs

Every row's "extra records" is **"motor record only"** — confirmed by counting `record(...)` vs
`record(motor, ...)` lines in every template in the family; there was no mismatch anywhere.

| IOC crate | driver crate | controller / protocol | st.cmd path | extra records |
|---|---|---|---|---|
| `acsmotion-ioc` | `motor-acsmotion` | ACS SPiiPlus motion controller (serial/TCP ASCII, ACSPL+) | `iocs/motor/acsmotion-ioc/st.cmd` | motor record only |
| `acstech80-ioc` | `motor-acstech80` | ACS Tech80 SPiiPlus (serial/TCP ASCII, ACSPL+) | `iocs/motor/acstech80-ioc/st.cmd` | motor record only |
| `aerotech-ioc` | `motor-aerotech` | Aerotech Ensemble (ASCII over asyn octet); A3200 variant via separate startup | `iocs/motor/aerotech-ioc/st.cmd` (+ `st.a3200.cmd` for A3200) | motor record only |
| `agap-conex-ioc` | `motor-newport` (`agap.rs`, [ports from](https://github.com/epics-modules/motor) `motorNewport/newportApp/src/AGAP_CONEX.cpp`) | Newport CONEX-AGAP two-axis piezo gonio (serial ASCII) | `iocs/motor/agap-conex-ioc/st.cmd` | motor record only |
| `ag-uc-ioc` | `motor-newport` (`agilis.rs`, ports from `AG_UC.cpp`) | Newport Agilis AG-UC2/AG-UC8/AG-UC8PC piezo stepper (serial ASCII) | `iocs/motor/ag-uc-ioc/st.cmd` | motor record only |
| `amci-ioc` | `motor-amci` | AMCI ANF2 / ANG1 stepper controllers (Modbus/TCP registers, not serial ASCII) | `iocs/motor/amci-ioc/st.cmd` | motor record only |
| `attocube-ioc` | `motor-attocube` | Attocube ANC150 (serial ASCII) | `iocs/motor/attocube-ioc/st.cmd` | motor record only |
| `conex-ioc` | `motor-newport` (`conex.rs`, ports from `AG_CONEX.cpp`) | Newport CONEX-AGP/CC/PP/DL/FCL200 (serial ASCII) | `iocs/motor/conex-ioc/st.cmd` | motor record only |
| `esp300-ioc` | `motor-newport` (`esp300.rs`, ports from `drvESP300.cc`+`devESP300.cc`) | Newport ESP100/ESP300/ESP301 (serial/GPIB ASCII) | `iocs/motor/esp300-ioc/st.cmd` | motor record only |
| `faulhaber-ioc` | `motor-faulhaber` | Faulhaber MCDC2805 (serial ASCII) | `iocs/motor/faulhaber-ioc/st.cmd` | motor record only |
| `hxp-ioc` | `motor-newport` (`hxp/`, ports from `HXPDriver.cpp`) | Newport HXP hexapod (TCP RPC socket, 6-axis X/Y/Z/U/V/W) | `iocs/motor/hxp-ioc/st.cmd` | motor record only |
| `ims-ioc` | `motor-ims` | IMS (Intelligent Motion Systems) MDrivePlus/MForce/Lexium (ASCII MCode over asyn octet) | `iocs/motor/ims-ioc/st.cmd` | motor record only |
| `kohzu-ioc` | `motor-kohzu` | Kohzu SC800 (serial ASCII) | `iocs/motor/kohzu-ioc/st.cmd` | motor record only |
| `mcb4b-ioc` | `motor-acs` | ACS MCB-4B stepper motor controller | `iocs/motor/mcb4b-ioc/st.cmd` | motor record only |
| `mclennan-ioc` | `motor-mclennan` | McLennan PM304 (serial ASCII) | `iocs/motor/mclennan-ioc/st.cmd` | motor record only |
| `micos-ioc` | `motor-micos` | Micos SMC corvus / hydra / taurus (serial ASCII, shared-controller read-modify-write for corvus) | `iocs/motor/micos-ioc/st.cmd` (+ `st.hydra.cmd`, `st.taurus.cmd`) | motor record only |
| `micromo-ioc` | `motor-micromo` | MicroMo MVP 2001 motion controller | `iocs/motor/micromo-ioc/st.cmd` | motor record only |
| `micronix-ioc` | `motor-micronix` | Micronix MMC200 (model-3 asyn driver) | `iocs/motor/micronix-ioc/st.cmd` | motor record only |
| `mm3000-ioc` | `motor-newport` (`mm3000.rs`, ports from `drvMM3000.cc`+`devMM3000.cc`) | Newport MM3000 (serial/GPIB ASCII) | `iocs/motor/mm3000-ioc/st.cmd` | motor record only |
| `mm4000-ioc` | `motor-newport` (`mm4000.rs`, ports from `drvMM4000Asyn.c`) | Newport MM4000/MM4005/MM4006 (serial/GPIB ASCII) | `iocs/motor/mm4000-ioc/st.cmd` | motor record only |
| `motorsim-ioc` | `motor-motorsim` | Simulated motor controller (no hardware) | `iocs/motor/motorsim-ioc/st.cmd` | motor record only |
| `newfocus-ioc` | `motor-newport` (`pmnc.rs`, ports from `motorNewFocus/newFocusApp/src/drvPMNC87xx.cc`+`devPMNC87xx.cc`) | Newport/New Focus PMNC 8750/8752 picomotor network controller (ASCII, prompt-framed) | `iocs/motor/newfocus-ioc/st.cmd` | motor record only |
| `npoint-ioc` | `motor-npoint` | nPoint C300 (ASCII SCPI-style) | `iocs/motor/npoint-ioc/st.cmd` | motor record only |
| `oms-asyn-ioc` | `motor-oms-asyn` | Pro-Dex OMS MAXnet / MXA (ASCII over asyn octet; VME OMS MAXv out of scope) | `iocs/motor/oms-asyn-ioc/st.cmd` (+ `st.mxa.cmd` for MXA) | motor record only |
| `oriel-ioc` | `motor-oriel` | Oriel EMC18011 (serial ASCII) | `iocs/motor/oriel-ioc/st.cmd` | motor record only |
| `parker-ioc` | `motor-parker` | Parker ACR-series / OEM-series controllers | `iocs/motor/parker-ioc/st.cmd` (+ `st.acr.cmd` for ACR) | motor record only |
| `phytron-ioc` | `motor-phytron` | Phytron phyMOTION (MCM) / MCC-1 / MCC-2 (ASCII, STX/ETX framing, optional XOR checksum) | `iocs/motor/phytron-ioc/st.cmd` (+ `st.mcc.cmd` for MCC) | motor record only |
| `pigcs2-ioc` | `motor-pi-gcs2` | PI (Physik Instrumente) GCS2 controllers (`PIGCSController`/`PIGCSMotorController`) | `iocs/motor/pigcs2-ioc/st.cmd` | motor record only |
| `pi-ioc` | `motor-pi` | PI legacy model-1 controllers: C-630, C-662, C-663, C-844, C-848, C-862, E-516, E-517, E-710, E-816 (one `db/*.template` + one driver module per model) | `iocs/motor/pi-ioc/st.cmd` | motor record only (per model) |
| `pijena-ioc` | `motor-pijena` | PI Jena PIJEDS (serial ASCII) | `iocs/motor/pijena-ioc/st.cmd` | motor record only |
| `pm500-ioc` | `motor-newport` (`pm500.rs`, ports from `drvPM500.cc`+`devPM500.cc`) | Newport PM500 precision motor controller (serial/GPIB ASCII) | `iocs/motor/pm500-ioc/st.cmd` | motor record only |
| `pmac-ioc` | `motor-pmac` | Delta Tau Turbo PMAC / Geobrick, [ports from](https://github.com/epics-modules/tpmac) `epics-modules/tpmac` (ASCII over asyn octet: serial, TCP, or PMAC ethernet framing) | `iocs/motor/pmac-ioc/st.cmd` | motor record only |
| `smaract-ioc` | `motor-smaract` | SmarAct MCS / MCS2 (ASCII SCPI) / SCU | `iocs/motor/smaract-ioc/st.cmd` (+ `st.mcs.cmd`, `st.scu.cmd`) | motor record only |
| `smartmotor-ioc` | `motor-smartmotor` | Animatics SmartMotor servo controller | `iocs/motor/smartmotor-ioc/st.cmd` | motor record only |
| `smc100-ioc` | `motor-newport` (`smc100.rs`, ports from `SMC100Driver.cpp`) | Newport SMC100 single-axis controller (serial ASCII) | `iocs/motor/smc100-ioc/st.cmd` | motor record only |
| `thorlabs-ioc` | `motor-thorlabs` | ThorLabs MDT695 (serial ASCII) | `iocs/motor/thorlabs-ioc/st.cmd` | motor record only |
| `xps-ioc` | `motor-newport` (`xps/`, ports from `XPSAxis.cpp`/`XPS_C8_drivers.cpp`) | Newport XPS-C8 multi-axis motion controller (TCP RPC socket) | `iocs/motor/xps-ioc/st.cmd` | motor record only |

All 27 vendor driver crates are confirmed (each crate's `Cargo.toml` header comment or its
per-model `src/*.rs` module doc-comment names the upstream C source) to port from
[epics-modules/motor](https://github.com/epics-modules/motor), **except** `motor-pmac`, which
ports from the separate [epics-modules/tpmac](https://github.com/epics-modules/tpmac) module
(`pmacApp/pmacAsynMotorPortSrc`, `pmacAsynIPPortSrc`, `pmacAsynCoordSrc`). `motor-newport` is one
crate covering 11 distinct Newport/New Focus controller families, each in its own source module
(`esp300.rs`, `mm3000.rs`, `mm4000.rs`, `pm500.rs`, `smc100.rs`, `conex.rs`, `agap.rs`,
`agilis.rs`, `pmnc.rs`, `hxp/`, `xps/`), which is why 11 IOC crates all depend on it. The shared
`motor-common` crate (not itself a vendor driver) factors out the vendor-independent iocsh
plumbing — `MotorHolder`, create-command argument parsing, octet-port connect helpers, and
C-runtime numeric helpers — and is a direct dependency of every vendor driver crate except
`motor-newport`, which implements its own iocsh/holder plumbing instead (confirmed: `newport/Cargo.toml`
has no `motor-common` dependency line, while all other 26 vendor `Cargo.toml` files do).

---

## AreaDetector drivers (part 1)

### bruker — Bruker BIS detector

Rust port of `BISDetector.cpp` from [epics-modules/ADBruker](https://github.com/epics-modules/ADBruker). BIS is a server, not a detector: the driver sends bracketed ASCII commands over one TCP socket (`BIS_COMMAND`), listens to BIS's broadcast status on a second socket (`BIS_STATUS`), and reads acquired frames as SFRM files off a shared filesystem. `drivers/ad/bruker/src/`: `connection.rs` (BIS sockets), `protocol.rs` (command framing), `sfrm.rs` (SFRM file decode), `filename.rs`, `task.rs`, `driver.rs`.

Records (`db/BIS.template`, includes `ADBase.template` + `NDFile.template`):
- `ReadSFRMTimeout` (ao) — timeout waiting for the SFRM file beyond the exposure time
- `BISStatus` (waveform, I/O Intr) — last status line BIS broadcast
- `NumDarks` / `NumDarks_RBV` — number of dark frames
- `FileFormat` redefined to just `SFRM`/`Invalid`
- `FrameType` redefined: `Normal`/`Dark`/`Raw`/`DblCorrelation`
- `BISAsyn` (asyn record) — interactive access to the BIS command port

Build/run: `cargo run -p bruker-ioc --release -- iocs/ad/bruker-ioc/st.cmd`

Deviation: the C driver's `mar345`-style "file" port on 49154 (never actually used by the C driver either) is not created.

---

### csimdetector — ADCSimDetector (simulated ADC)

Rust port of [epics-modules/ADCSimDetector](https://github.com/epics-modules/ADCSimDetector) (driver version 2.5.0). Not a real detector — it synthesizes one 2-D NDArray (`[MAX_SIGNALS, numTimePoints]`) per frame plus eight 1-D per-signal waveforms (sine, cosine, square, sawtooth, noise, sums), and derives `asynNDArrayDriver` rather than `ADDriver`. `drivers/ad/csimdetector/src/`: `signals.rs` (the eight pure waveform generators, unit-tested against the C expressions), `rng.rs`, `driver.rs`, `task.rs`.

Records:
- `db/ADCSimDetector.template` (includes `NDArrayBase.template`, once per detector): `TimeStep`/`_RBV`, `NumTimePoints`/`_RBV`, `AcquireTime`/`_RBV`, `ElapsedTime`
- `db/ADCSimDetectorN.template` (loaded 8×, ADDR=0..7, one per signal): `Name`, `Amplitude`, `Offset`, `Period`, `Frequency`, `Phase`, `Noise`

Build/run: `cargo run -p ad-csimdetector-ioc --release -- iocs/ad/csimdetector-ioc/st.cmd`

Deviation (documented in st.cmd): upstream configures `dataType=7` meaning `NDFloat64` under the *old* `NDDataType_t` enum (pre-NDInt64/NDUInt64 insertion); this port passes `9` so the data type still resolves to `NDFloat64` under `ad-core-rs`'s modern enum, matching the `TYPE=Float64,FTVL=DOUBLE` waveform.

---

### eiger — Dectris Eiger (SIMPLON REST + ZeroMQ stream)

Rust port of [epics-modules/ADEiger](https://github.com/epics-modules/ADEiger) (`eigerApp/src/{eigerDetector,restApi,eigerParam,streamApi}.cpp`). Drives Dectris Eiger detectors over the SIMPLON REST API (HTTP) for control plus a ZeroMQ stream for images. `drivers/ad/eiger/src/`: `rest.rs` (SIMPLON REST client via `ureq`), `stream.rs` (ZeroMQ via the `zeromq` crate), `bslz4.rs` + `h5.rs` (bslz4-compressed HDF5 stream decode, replacing `libhdf5`), `tiff.rs`, `param.rs`/`params.rs`, `tasks.rs`.

Records — `db/eiger2.template` includes the driver's own `eigerBase.template` (1253 lines; the Eiger analog of `ADBase.template`, itself full of driver-specific SIMPLON parameters: `PhotonEnergy`, `ThresholdEnergy`, `BeamX`/`BeamY`, `DetDist`, filewriter `FWNamePattern`/`FWNImagesPerFile`/`FWState_RBV`, `MonitorState_RBV`, goniometer `Omega`/`Phi`/`Chi`/`Kappa` + increments, `Armed`, `Initialize`, `Error_RBV`, `CountCutoff_RBV`, `DeadTime_RBV`). `eiger2.template` itself adds the Eiger2-specific layer:
- `HVResetTime`/`_RBV`, `HVReset`, `HVState_RBV` — high-voltage control
- `CountingMode`/`_RBV` — Normal/Retrigger
- `Threshold1Enable`, `Threshold2Energy`, `Threshold2Enable`, `ThresholdDiffEnable` (+ `_RBV`s)
- `TriggerStartDelay`/`_RBV`; `TriggerMode`/`ExtGateMode` extended with `External Gate`/HDR/`Pump & Probe`
- `CompressionAlgo` extended with a `None` choice
- `StreamVersion` (Stream/Stream2), `StreamHdrDetail`, `StreamAsTSSource`
- `FWHDF5Format` — Legacy vs v2024.2

st.cmd wires three `NDStdArrays` outputs to three distinct named asyn ports the driver publishes (`$(PORT)` = every frame, `$(PORT)_TH1` = threshold 1, `$(PORT)_MON` = monitor image) since epics-rs routes NDArrays by port name rather than C's asyn-address convention.

Build/run: `cargo run -p eiger-ioc --release -- iocs/ad/eiger-ioc/st.cmd`

---

### mar345 — MAR 345 online image-plate detector

Rust port of `ADmar345/mar345App/src/mar345.cpp` ([epics-modules/ADmar345](https://github.com/epics-modules/ADmar345)). Controlled over a TCP ASCII socket (`marServer`) to the `mar345dtb` program; uppercase `COMMAND …` lines, LF-terminated in both directions, progress reported as free-form status lines the driver matches against an `"… Ended o.k."` substring. Frames are read back from `mar345dtb`'s `.mar<size>` files: a `CCP4 packed image, X: …, Y: …` header followed by the CCP4 "pck" run-length/delta packed pixel stream. `drivers/ad/mar345/src/`: `pck.rs` (CCP4 pck decoder), `server.rs`, `protocol.rs`, `file_name.rs`, `task.rs`.

Records (`iocs/ad/mar345-ioc/db/mar345.template`, includes `ADBase.template` + `NDFile.template`):
- `Abort`/`Abort_RBV`; `Erase` (busy)/`Erase_RBV`; `NumErase`/`_RBV`; `NumErased_RBV`
- `EraseMode`/`_RBV` — None/Before expose/After scan
- `ChangeMode` (busy)/`_RBV` — applies ScanSize/ScanResolution
- `ScanSize`/`_RBV` — 180/240/300/345 mm; `ScanResolution`/`_RBV` — 0.10/0.15 mm
- `FileFormat` redefined to `MAR345`/`Invalid`
- `marServerAsyn` (asyn record)
- `DetectorState_RBV` redefined: Idle/Exposing/Scanning/Erasing/Changing mode/Aborting/Error/Waiting

Build/run: `cargo run -p mar345-ioc --release -- iocs/ad/mar345-ioc/st.cmd`

Deviation: server I/O runs on a dedicated worker thread (a `PortDriver` method can't block on a second asyn port from inside its own port actor), so `writeInt32` only sets `mode` and signals an event while a `task` worker owns the `Server` and performs every socket round-trip — command order and the wire bytes are unchanged. Boot limitation: on the published `ad-plugins-rs` 0.22.1 baseline, `drvAsynIPPortConfigure`/`asynOctetSetInputEos`/`asynOctetSetOutputEos` iocsh commands aren't registered, so this st.cmd cannot actually create the marServer port unmodified (see st.cmd header comment).

---

### marccd — MAR marCCD

Rust port of `ADmarCCD/marCCDApp/src/marCCD.cpp` (driver version 2.3.0, [epics-modules/ADmarCCD](https://github.com/epics-modules/ADmarCCD)). Controlled over a `marccd_server` TCP ASCII socket (LF-terminated both directions); images read back as 16-bit TIFFs the server writes to a shared filesystem. The server exposes a packed state word (`get_state`) whose nibbles carry per-task (acquire/readout/correct/write/dezinger/series) queued/executing/error status, polled by the acquisition state machine. `drivers/ad/marccd/src/`: `server.rs`, `protocol.rs`, `image.rs`, `file_name.rs`, `task.rs`.

Records (`iocs/ad/marccd-ioc/db/marCCD.template`, includes `ADBase.template` + `NDFile.template`):
- `FrameType` redefined: Normal/Background/Raw/DblCorrelation; `FileFormat` redefined to `TIFF`/`Invalid`
- `GateMode`/`_RBV`, `ReadoutMode`/`_RBV` — enum choices depend on server mode, set at runtime
- `ServerMode_RBV`
- `SeriesFileTemplate`/`_RBV`, `SeriesFileDigits`/`_RBV`, `SeriesFileFirst`/`_RBV`
- `MarState_RBV` (raw state word) plus decoded per-task mbbi records: `MarStatus_RBV`, `MarAcquireStatus_RBV`, `MarReadoutStatus_RBV`, `MarCorrectStatus_RBV`, `MarWritingStatus_RBV`, `MarDezingerStatus_RBV`, `MarSeriesStatus_RBV` (each Idle/Queued/Executing/Error/Reserved)
- `ReadTiffTimeout`; `OverlapMode`/`_RBV`; `FrameShift`/`_RBV`; `Stability`/`_RBV`
- `marServerAsyn` (asyn record)
- Crystallography metadata: `DetectorDistance`, `BeamX`, `BeamY`, `StartPhi`, `RotationAxis`, `RotationRange`, `TwoTheta`, `Wavelength`, `FileComments`, `DatasetComments`

Build/run: `cargo run -p marccd-ioc --release -- iocs/ad/marccd-ioc/st.cmd`

Deviation: the upstream template's duplicate `MarState_RBV` definition was dropped as a retro-fixed upstream defect (`doc/upstream-c-defects.md` #15). Server I/O runs on three worker threads sharing one `Server` behind a `tokio::sync::Mutex` (the analog of the C driver lock), since a `PortDriver` method can't block on a second port from its own actor; command order and the wire bytes are unchanged, only the moment a record's write callback returns differs.

---

### merlin — Quantum Detectors Merlin (Medipix)

Port of `areaDetector/ADMerlin/merlinApp/src` ([epics-modules/ADMerlin](https://github.com/epics-modules/ADMerlin)). A Labview server speaks the MPX protocol over two separate TCP asyn octet ports (created by `drvAsynIPPortConfigure` in st.cmd): a request/response command channel (`$(PORT)cmd`, LF-terminated) and a push-only binary data channel (`$(PORT)data`, no EOS — MPX frames are length-delimited). `drivers/ad/merlin/src/`: `protocol.rs` (MPX framing/codec, pure + unit-tested), `image.rs` (pixel decode + Y flip), `connection.rs`.

Records (`db/merlin.template`, includes only `ADBase.template` — no file plugin):
- `TriggerMode`/`_RBV` — Internal/Trigger Enable/Trigger start rising/falling/Trigger both rising/Software
- `ImageMode`/`_RBV` extended with Threshold and Background
- `LabviewAsynCmd`, `LabviewAsynData` (asyn records for the two sockets)
- `ThresholdEnergy0`..`ThresholdEnergy7` (+ `_RBV`) — 8 energy thresholds
- `OperatingEnergy`/`_RBV`
- `StartThresholdScan`, `StopThresholdScan`, `StepThresholdScan` (+ `_RBV`), `ThresholdScan`/`_RBV`
- `SoftwareTrigger` (busy)/`_RBV`
- `Reset` (busy, restarts the Labview server — IOC exits with it)/`_RBV`
- `CounterDepth`/`_RBV` — 12/24 bit
- `EnableCounter1`, `ContinuousRW` (+ `_RBV`)
- XBPM (University of Manchester extension): `ProfileControl`/`_RBV`, `ProfileAverageX_RBV`, `ProfileAverageY_RBV`, `EnableBackgroundCorrection`, `EnableSumAverage`
- Merlin Quad: `QuadMerlinMode`/`_RBV` (12 bit/24 bit/Two Threshold/Continuous RW/Colour/Charge Summing), `SelectGui_RBV`
- `DataType`, `ColorMode`, `BinX`, `BinY`, `ReverseX`, `ReverseY` from `ADBase.template` are disabled (`DISA=1`) — Merlin doesn't implement them

Build/run: `cargo run -p merlin-ioc --release -- iocs/ad/merlin-ioc/st.cmd`

Deviation: the shared workspace `merlin.template` drops `FileFormat`/`FileFormat_RBV` (present in the upstream template but pointing at a file plugin/`NDFile.template` this driver never loads, so upstream's copies were dead records with no device support).

---

### mythen — Dectris Mythen strip detector

Rust port of `ADMythen/mythenApp/src/mythen.cpp` ([epics-modules/ADMythen](https://github.com/epics-modules/ADMythen)). Driven over an asyn octet IP port using the M1K ASCII/binary command set; the socket itself belongs to asyn exactly as in C (`drvAsynIPPortConfigure` + CR output EOS, `noProcessEos=1` since detector replies are binary with no line-oriented input EOS). `drivers/ad/mythen/src/`: `detector.rs`, `protocol.rs`, `transport.rs`, `task.rs`.

Records (`drivers/ad/mythen/db/mythen.template`, includes only `ADBase.template`):
- `Setting`/`_RBV` — Cu/Mo/Ag/Cr
- `DelayTime`/`_RBV` — delay after trigger
- `ThresholdEnergy`/`_RBV`, `BeamEnergy`/`_RBV`
- `UseFlatField`/`_RBV`, `UseCountRate`/`_RBV`, `Tau`/`_RBV` (deadtime constant), `UseBadChanIntrpl`/`_RBV`
- `BitDepth`/`_RBV` — 24/16/8/4
- `UseGates`/`_RBV`, `NumGates`/`_RBV`, `NumFrames`/`_RBV`
- `TriggerMode`/`_RBV` — None/Single/Continuous
- `NumModules_RBV`, `FirmwareVersion`
- `ReadMode`/`_RBV` — Raw/Corrected
- `ImageMode` (only Single/Multiple; no Continuous choice for Mythen)

Build/run: `cargo run -p mythen-ioc --release -- iocs/ad/mythen-ioc/st.cmd`

---

### photonii — Bruker PhotonII

Port of `areaDetector/ADPhotonII` (`PhotonIIApp/src/PhotonII.cpp`, [epics-modules/ADPhotonII](https://github.com/epics-modules/ADPhotonII)). Driven by Bruker's `p2util` program reached over a TCP socket (a procServ port, CRLF input EOS / LF output EOS): the driver sends command lines (`set --exposure-time 1.0`, `grab --dstdir ... --count 5`, `abort`) and `p2util` answers each and, during acquisition, announces every frame it writes by naming the `.raw` file, which the driver then reads off the filesystem. `drivers/ad/photonii/src/`: `protocol.rs` (command language + frame-message parse, pure functions), `raw.rs` (`.raw` readiness test + decode), `connection.rs`, `task.rs`.

Records (`db/photonII.template`, includes `ADBase.template` + `NDFile.template`):
- `FileFormat` redefined to `Raw`/`Invalid`
- `FrameType`/`_RBV` — Normal/Dark/ADC0
- `NumDarks`/`_RBV` — frame count when FrameType is Dark or ADC0
- `TriggerType`/`_RBV` — Step/Continuous; `TriggerEdge`/`_RBV` — Rising/Falling
- `DRSumEnable`/`_RBV` — DR summation mode
- `NumSubFrames`/`_RBV`
- `PIIAsyn` (asyn record) — interactive access to the p2util socket

Build/run: `cargo run -p photonii-ioc --release -- iocs/ad/photonii-ioc/st.cmd`

---

### pilatus — Dectris Pilatus

Rust port of `ADPilatus/pilatusApp/src/pilatusDetector.cpp` (driver version 2.9.0, [epics-modules/ADPilatus](https://github.com/epics-modules/ADPilatus)). Controlled over a `camserver` TCP ASCII socket (LF-terminated commands, replies framed on camserver's 0x18/CAN byte); images read back from TIFF files camserver writes to a shared filesystem. `drivers/ad/pilatus/src/`: `camserver.rs`, `protocol.rs`, `image.rs`, `file_name.rs`, `task.rs`.

Records (`iocs/ad/pilatus-ioc/db/pilatus.template`, includes `ADBase.template` + `NDFile.template`):
- `TriggerMode`/`_RBV` redefined — Internal/Ext. Enable/Ext. Trigger/Mult. Trigger/Alignment
- `FileFormat` redefined to `TIFF`/`Invalid`
- `Armed` — ready for external triggers
- `ResetPower`, `ResetPowerTime`/`_RBV`
- `DelayTime`/`_RBV` — external-trigger delay
- `ThresholdEnergy`/`_RBV`, `ThresholdApply` (busy), `ThresholdAutoApply`/`_RBV`
- `Energy`/`_RBV`, `GainMenu` (writes through to `Gain.VAL`)
- `ImageFileTmot`
- `BadPixelFile`, `NumBadPixels`, `FlatFieldFile`, `MinFlatField`/`_RBV`, `FlatFieldValid`, `GapFill`/`_RBV`
- `CamserverAsyn` (asyn record)
- Crystallography/beamline metadata: `Wavelength`, `EnergyLow`, `EnergyHigh`, `DetDist`, `DetVOffset`, `BeamX`, `BeamY`, `Flux`, `FilterTransm`, `StartAngle`, `AngleIncr`, `Det2theta`, `Polarization`, `Alpha`, `Kappa`, `Phi`/`PhiIncr`, `Chi`/`ChiIncr`, `Omega`/`OmegaIncr`, `OscillAxis`, `NumOscill`
- `PixelCutOff_RBV`, `Temp0_RBV`..`Temp2_RBV`, `Humid0_RBV`..`Humid2_RBV`, `TVXVersion_RBV`, `CbfTemplateFile`, `HeaderString`
- `DataType`, `ColorMode`, `BinX`/`BinY`, `MinX`/`MinY`, `SizeX`/`SizeY`, `ReverseX`/`ReverseY` from `ADBase.template` disabled (`DISA=1`) — not applicable to Pilatus

Build/run: `cargo run -p pilatus-ioc --release -- iocs/ad/pilatus-ioc/st.cmd`

Deviations (per crate doc comments): CBF file format is not supported (C links CBFlib; this port logs an error and fails the read on `.cbf`). Camserver I/O is queued to a `PilatusCmdTask` worker thread rather than run inline in the write callback (same actor-blocking constraint as mar345/marccd). `createFileName`/`checkPath` are reimplemented locally because `ad-core-rs` 0.22.1 doesn't expose them. Multi-strip TIFFs decode correctly here (the `tiff` crate reads every strip), whereas C's `readTiff` always passes strip index 0 — the two agree because camserver only ever writes single-strip files. Bad-pixel indices are bounds-checked (C does not).

---

## AreaDetector drivers (part 2)

### pixirad — `drivers/ad/pixirad`, `iocs/ad/pixirad-ioc`

Pixirad CdTe photon-counting detector. ASCII command/response over TCP, image
and environment data delivered on separate UDP broadcast streams (no request/
reply on the data path).

- **Ports from:** `ADPixirad` (`pixiradApp/Db/pixirad.template`, per the db
  template's own header comment). No explicit upstream GitHub URL found in
  source, so no link is given here.
- **Build/run:** `cargo run -p pixirad-ioc --release -- iocs/ad/pixirad-ioc/st.cmd`
- **Records** (`db/pixirad.template`, macros `P,R,PORT,ADDR,TIMEOUT`):
  - `Threshold1`..`Threshold4` / `_RBV`, `ThresholdActual1`..`4_RBV`,
    `HitThreshold`/`_RBV`, `HitThresholdActual_RBV` — per-colour energy
    discriminator thresholds (keV) and what the hardware actually applied.
  - `CountMode`/`_RBV` (mbbo/mbbi: Normal/NPI/NPISUM), `FrameType`/`_RBV`
    (1/2/4-colour + DTF layouts), `TriggerMode`/`_RBV` (Internal/External/
    Bulb — redeclares the ADBase mbbo/mbbi with detector-specific states).
  - `HVValue`/`_RBV`, `HVState`/`_RBV`, `HVMode`/`_RBV`, `HVActual_RBV`,
    `HVCurrent_RBV` — bias high-voltage control/readback.
  - `SyncInPolarity`/`_RBV`, `SyncOutPolarity`/`_RBV`, `SyncOutFunction`/`_RBV`
    — sync I/O configuration.
  - `CoolingState`/`_RBV`, `CoolingStatus_RBV`, `HotTemperature_RBV`,
    `BoxTemperature_RBV`, `BoxHumidity_RBV`, `DewPoint_RBV`,
    `PeltierPower_RBV`, `Temperature`/`_RBV`/`TemperatureActual` — Peltier
    cooling loop and environment telemetry.
  - `AutoCalibrate`, `SystemReset` (busy), `SystemInfo` (waveform) —
    calibration/reset commands and ASCII system-info readout.
  - `ColorsCollected_RBV`, `UDPBuffersRead_RBV`/`Max_RBV`/`Free_RBV`,
    `UDPSpeed_RBV` — UDP image-stream health counters.
  - ADBase fields the detector doesn't implement are disabled (`DISA="1"`):
    `DataType`, `ColorMode`, `BinX/Y`, `MinX/Y`, `SizeX/Y`, `ReverseX/Y`.

### psl — `drivers/ad/psl`, `iocs/ad/psl-ioc`

Photonic Sciences Ltd. CCD detector, driven indirectly through the vendor's
own PSLViewer program (a `Command;argument` TCP protocol); the driver polls
`HasNewData` during acquisition and pulls frames with `GetImage`.

- **Ports from:** `ADPSL` (`pslApp/Db/PSL.template`, `pslApp/src/PSL.cpp`).
  No explicit upstream GitHub URL found in source.
- **Build/run:** `cargo run -p psl-ioc --release -- iocs/ad/psl-ioc/st.cmd`
- **Records** (`db/PSL.template`, macros `P,R,PORT,ADDR,TIMEOUT,PSL_SERVER_PORT`):
  - `CameraName`/`_RBV` (mbbo/mbbi) — the choice set is populated at runtime
    from PSLViewer, so `ZRST`/`ONST`/... are deliberately left empty.
  - `TIFFComment`/`_RBV` (waveform, `CHAR[256]`) — comment string embedded in
    saved TIFFs.
  - `PSLServer` (`asyn` record, `IMAX/OMAX=4096`) — raw command channel to
    PSLViewer for interactive debugging.
- FileFormat and TriggerMode enums referenced in the module header are
  likewise server-populated but are not separate PVs beyond `CameraName`.

### pva-driver — `drivers/ad/pva-driver`, `iocs/ad/pva-driver-ioc`

Ingests an upstream `epics:nt/NTNDArray:1.0` pvAccess PV (e.g. from another
areaDetector IOC's own pva plugin) and republishes it as a local NDArray
stream — a receive-only "detector" whose data source is a PVA channel, not
hardware.

- **Ports from:** `pvaDriver` (`areaDetector pvaDriver`, per the IOC's
  `st.cmd` header, which cites `iocs/pvaDriverIOC/iocBoot/iocPvaDriver/`; the
  decode logic mirrors ADCore's `NTNDArrayConverter`/`ntndArrayConverter.cpp`).
  https://github.com/areaDetector/pvaDriver
- **Build/run:** `cargo run -p pva-driver-ioc --release -- iocs/ad/pva-driver-ioc/st.cmd`
- **Records** (`db/pva.template`, macros `P,R,PORT,ADDR,TIMEOUT`):
  - `PvName`/`_RBV` (waveform, `CHAR[256]`) — name of the upstream NTNDArray
    PV to monitor (`PINI=YES`).
  - `PvConnection_RBV` (bi, Up/Down) — channel connection state.
  - `OverrunCounter`/`_RBV` (longout/longin) — count of server-side squashed
    (overrun) monitor updates.
- Uses `epics-pva-rs` as its pvAccess client; a `tokio::select!` races the
  driver's command channel against the PVA client's connect/monitor bridge.

### simdetector — `drivers/ad/simdetector`, `iocs/ad/simdetector-ioc`

The areaDetector simulated-camera driver: four pattern generators (linear
ramp, peaks, sine, offset+noise) with no real hardware.

- **Ports from:** `ADSimDetector`, driver version 2.11.0 —
  https://github.com/areaDetector/ADSimDetector
  (`simDetectorApp/src/simDetector.cpp`, followed line-for-line; see the
  driver crate's `image.rs` for the four pattern generators).
- **Build/run:** `cargo run -p ad-simdetector-ioc --release -- iocs/ad/simdetector-ioc/st.cmd`
- **Records** (`db/simDetector.template`, macros `P,R,PORT,ADDR,TIMEOUT`):
  - `GainX`/`_RBV`, `GainY`/`_RBV`, `GainRed`/`_RBV`, `GainGreen`/`_RBV`,
    `GainBlue`/`_RBV`, `Offset`/`_RBV`, `Noise`/`_RBV` — per-axis/per-channel
    gain and the offset+noise mode parameters.
  - `SimMode`/`_RBV` (mbbo/mbbi: LinearRamp/Peaks/Sine/Offset&Noise).
  - `Reset`/`_RBV` (longout/longin) — force-reset the simulated image buffer.
  - Peaks mode: `PeakStartX/Y`, `PeakNumX/Y`, `PeakStepX/Y`, `PeakWidthX/Y`,
    `PeakVariation` (each with `_RBV`).
  - Sine mode: `XSineOperation`/`YSineOperation` (Add/Multiply) and
    `XSine1/2`, `YSine1/2` `Amplitude`/`Frequency`/`Phase` (each `_RBV`).
  - `ColorMode`/`_RBV` intentionally reuse the ADBase.template menu rather
    than upstream's narrower Mono/RGB1/RGB2/RGB3 redeclaration — a duplicate
    record name across `dbLoadRecords` isn't permitted by `epics-base-rs`;
    the driver treats every unsupported mode as Mono.

### specs-analyser — `drivers/ad/specs-analyser`, `iocs/ad/specs-analyser-ioc`

SPECS Phoibos electron energy analyser. ASCII/TCP command-response protocol
over an asyn octet port; produces spectrum/image `Float64` array data rather
than camera frames.

- **Ports from:** `epics-modules specsAnalyser`
  (`specsAnalyserApp/src/specsAnalyser.cpp`), per the driver crate's
  `Cargo.toml` header comment. https://github.com/epics-modules/specsAnalyser
- **Build/run:** `cargo run -p specs-analyser-ioc --release -- iocs/ad/specs-analyser-ioc/st.cmd`
- **Records** (`db/specsAnalyser.template`, macros `P,R,PORT,ADDR,TIMEOUT`):
  - Acquisition control/status: `CONNECT`, `CONNECTED_RBV`, `COUNTER_RBV`,
    `SAFE_STATE`/`_RBV`, `PAUSE`/`_RBV`, `DEFINE_SPECTRUM`,
    `VALIDATE_SPECTRUM`, `SERVER_NAME_RBV`, `PROTOCOL_VERSION_RBV`.
  - Spectrum definition: `PASS_ENERGY`/`_RBV`, `LOW_ENERGY`/`_RBV`,
    `HIGH_ENERGY`/`_RBV`, `ENERGY_WIDTH_RBV` (calc, `B-A` of low/high),
    `KINETIC_ENERGY`/`_RBV`, `RETARDING_RATIO`/`_RBV`, `STEP_SIZE`/`_RBV`,
    `LENS_MODE`/`_RBV`, `SCAN_RANGE`/`_RBV`, `ACQ_MODE`/`_RBV`,
    `SLICES`/`_RBV`, `SAMPLES`, `VALUES`/`_RBV`.
  - Progress/readback: `TOTAL_POINTS_RBV`, `TOTAL_POINTS_ITERATION_RBV`,
    `CURRENT_POINT_RBV`, `CURRENT_CHANNEL_RBV`, `REGION_TIME_LEFT_RBV`,
    `TOTAL_TIME_LEFT_RBV`, `REGION_PROGRESS_RBV`, `PROGRESS_RBV`,
    `Y_UNITS_RBV`, `Y_MIN_RBV`, `Y_MAX_RBV`.
  - Data: `INT_SPECTRUM` (waveform, `DOUBLE[100000]`), `IMAGE` (waveform,
    `DOUBLE[2000000]`) — both `asynFloat64ArrayIn`, `SCAN=I/O Intr`.
  - ADBase fields the analyser doesn't implement are disabled (`DISA="1"`):
    `BinX/Y`, `MinX/Y`, `MaxSizeX_RBV`, `MaxSizeY_RBV`, `SizeX/Y`, `Gain`,
    `ReverseX/Y`. (The port fixes an upstream template bug where
    `MaxSizeY_RBV`'s disable block was a copy-pasted duplicate of
    `MaxSizeX_RBV`'s.)

### std-arrays-driver — `drivers/ad/std-arrays-driver`, `iocs/ad/std-arrays-driver-ioc`

`NDDriverStdArrays`: not a detector — it turns plain EPICS waveform-record
writes into NDArrays, letting any Channel Access client inject image data
into an areaDetector plugin chain. No acquisition thread.

- **Ports from:** `NDDriverStdArrays`, driver version 1.3.0 —
  https://github.com/areaDetector/NDDriverStdArrays
- **Build/run:** `cargo run -p ad-std-arrays-driver-ioc --release -- iocs/ad/std-arrays-driver-ioc/st.cmd`
- **Records** (`db/NDDriverStdArrays.template`, macros
  `P,R,PORT,ADDR,TIMEOUT,NELEMENTS,TYPE,FTVL`):
  - `CallbackMode`/`_RBV` (mbbo/mbbi: On update/On complete/On command).
  - `DoCallbacksScan` (bo, `SDIS`-gated on `Acquire`) / `DoCallbacks` (bo) —
    trigger publish of the assembled array.
  - `NewArray`, `ArrayComplete` (bo) — mark a fresh array / mark it finished.
  - `AppendMode`/`_RBV` (bo/bi) — append vs. overwrite into the working
    buffer.
  - `NumElements_RBV`, `NextElement`/`_RBV`, `Stride`/`_RBV`, `FillValue`/`_RBV`
    — buffer geometry: where the next write lands, its stride, and the fill
    value for untouched elements.
  - `ArrayIn` (waveform, `DTYP="asyn$(TYPE)ArrayOut"`, `FTVL=$(FTVL)`,
    `NELM=$(NELEMENTS)`) — the actual data-injection record; a client's
    `caput` here is what becomes the published NDArray.

### timepix3 — `drivers/ad/timepix3`, `iocs/ad/timepix3-ioc`

ASI/Amsterdam Scientific TimePix3 detector behind an ASI Serval HTTP server
(JSON control + raw-TCP `JSON header \n binary payload` preview streams).
By far the largest driver in this set: 9 templates, ~250 driver-specific
records, plus per-chip (0-7) and per-power-rail (0-5) record replication via
`ADDR`.

- **Ports from:** `ADTimePix3` — https://github.com/areaDetector/ADTimePix3
  (`tpx3App/src/`: `ADTimePix.cpp`, `serval_http.cpp`, `serval_stream.cpp`,
  `histogram_io.cpp`, `mask_io.cpp`, `acquire.cpp`, `network_client.cpp`,
  `img_accumulation.cpp`).
- **Build/run:** `cargo run -p timepix3-ioc --release -- iocs/ad/timepix3-ioc/st.cmd`
- **Records** — grouped by template (all under `drivers/ad/timepix3/db/`,
  macros `P,R,PORT,ADDR,TIMEOUT` unless noted; full field lists are large
  enough that this is a categorized sample, not exhaustive — see the
  templates for the complete set):
  - `TimePix3Base.template` — redeclares ADBase's `TriggerMode`/`_RBV` (8
    Serval trigger modes), `DataType`/`_RBV`, `ColorMode`/`_RBV` with
    detector-valid states.
  - `ADTimePix3.template` — detector identity/health (`ServerURL_RBV`,
    `DetType_RBV`, `SW_ver_RBV`, `FW_ver_RBV`, `LocalTemp_RBV`,
    `FPGATemp_RBV`, `Fan1/2Speed_RBV`, `BiasVoltage_RBV`, `Humidity_RBV`,
    `Health`), chip/geometry readback (`PixCount_RBV`, `RowLen_RBV`,
    `NChips_RBV`, `NRows_RBV`, `MpxType_RBV`, `Chip1_RBV`..`Chip8_RBV`),
    trigger/timing config (`TriggerIn`/`_RBV`, `TriggerOut`/`_RBV`,
    `TriggerDelay`/`_RBV`, `nTriggers_RBV`, `ExposureTime_RBV`,
    `TriggerPeriod_RBV`, `Tdc0`/`Tdc1` + `_RBV`), bias control
    (`BiasVolt`/`_RBV`, `BiasEnbl`/`_RBV`), and `DetOrient`/`_RBV`.
  - `File.template` — BPC/DACS pixel-config file paths and status
    (`BPCFilePath`/`_RBV`, `BPCFilePathExists_RBV`, `DACSFilePath`/`_RBV`,
    `WriteBPCFile`, `WriteDACSFile`, `MaskedPelsJson_RBV`,
    `MaskedPelsCount_RBV`).
  - `Server.template` (largest file) — per-stream (`Raw`/`Raw1`/`Img`/`Img1`/
    `PrvImg`/`PrvImg1`/`PrvHst`) write-enable, file path/template, format,
    integration mode, queue size and rate/frame-count readback, e.g.
    `WriteData`, `WriteImg`/`_RBV`, `ImgFileFmt`/`_RBV`, `ImgQueueSize`/`_RBV`,
    `ImgAcqRate_RBV`, `PrvHstBinWidth`/`_RBV`, `PrvHstTotalCounts_RBV`.
  - `Measurement.template` — live acquisition counters: `PelEvtRate_RBV`,
    `Tdc1EvtRate_RBV`/`Tdc2EvtRate_RBV`, `ElapsedTime_RBV`, `TimeLeft_RBV`,
    `FrameCnt_RBV`, `DroppedFrames_RBV`, `Status_RBV`, plus STEM-scan
    (`StemScanWidth/Height`, `StemDwellTime`, `StemRadiusOuter/Inner`) and
    TOF-TDC (`TofTdcReference`, `TofMin`/`TofMax`) parameters.
  - `Dashboard.template` (loaded with `S=Stats5:`) — `ServalConnected_RBV`,
    `DetConnected_RBV`, `RefreshConnection`, `RefreshPixelConfig`,
    `ApplyConfig`, `FreeSpace_RBV`, `DiskLimReach_RBV`, `WriteSpeed_RBV`,
    plus `calcout`/`calc` chains (`FileWriterCalc*`, `HotPixelCalc`,
    `TdcCalc`, `Alarm`) that derive dashboard alarm state.
  - `MaskBPC.template` — pixel mask editing: `BPC`/`BPCmasked`, `MaskBPC`,
    `MaskOnOff`, `MaskReset`, `MaskMinX/Y`, `MaskSizeX/Y`, `MaskRadius`,
    `MaskRectangle`, `MaskCircle`, `MaskPel`, `MaskWrite`, plus `sseq`
    sequence records that drive multi-step mask-apply operations.
  - `Chips.template` (loaded 8x, `ADDR=0..7`, `C=CHIP0..CHIP7`) — per-chip
    DAC readback/write: `Vth_coarse`/`Vth_fine` (+`_RBV`, writable), and
    read-only `CP_PLL_RBV`, `Ikrum_RBV`, `PixelDAC_RBV`, `Preamp_ON/OFF_RBV`,
    `Temp_RBV`, `PixelConfigMatchBPC_RBV`, etc.
  - `OperatingVoltage.template` (loaded 6x, `ADDR=0..5`) — `VDD_RBV`,
    `AVDD_RBV` per SPIDR power rail.

### url — `drivers/ad/url`, `iocs/ad/url-ioc`

`ADURL`: fetches a still image from a file path or HTTP(S) URL on each
acquire, using `image` for decode (PNG/JPEG/GIF/BMP/TIFF/WebP) in place of
upstream's GraphicsMagick.

- **Ports from:** `ADURL` (`urlApp/Db/URLDriver.template`; the IOC `st.cmd`
  header cites `areaDetector ADURL iocs/urlIOC/iocBoot/iocURLDriver/`).
  https://github.com/areaDetector/ADURL
- **Build/run:** `cargo run -p url-ioc --release -- iocs/ad/url-ioc/st.cmd`
- **Records** (`db/url.template`, macros `P,R,PORT,ADDR,TIMEOUT`):
  - `URL1`..`URL10` (waveform, `CHAR[256]`, `asynOctetWrite`) — 10 selectable
    source URLs/paths, all bound to the same `URL_NAME` asyn parameter.
  - `URLSelect` (mbbo, `URL1`..`URL10`) — chooses which `URLn` record's
    value is pushed to the driver via `URLSeq` (a `seq` record fanning out
    `LNK1`..`LNKA`).
  - `URL_RBV` (waveform, `CHAR[256]`) — currently-active URL/path readback.
  - Fixes an upstream defect (retro-fixed here, doc'd inline): `URLSelect`'s
    `EIST`/`NIST` (`URL9`/`URL10`) states had unset/colliding `EIVL`/`NIVL`
    values in the original template, so selecting `URL9` or `URL10` didn't
    reliably drive the right `seq` link; `EIVL="9"`/`NIVL="10"` are assigned
    here to continue the `ZRVL..SVVL` progression.

---

## quadEM, MCA & Scaler

Three independent record families, each porting one `epics-modules` repository. Unlike the motor family, quadEM's 8 IOCs share one big base template (`quadEM.template`) that every model-specific template `include`s and extends; MCA's 3 IOCs instead share a single generic `mcaRecord` binding (`mca.db`) with the interesting fields living inside the record type itself; scaler974 is a single IOC around one `scalerRecord` instance.

### quadEM (`drivers/quadem`)

Ports from [`epics-modules/quadEM`](https://github.com/epics-modules/quadEM) — confirmed by `drivers/quadem/src/lib.rs`'s module doc, which maps each Rust module to its upstream `.cpp` source (`caenSrc/drvTetrAMM.cpp`, `caenSrc/drvAHxxx.cpp`, `nslsSrc/drvNSLS_EM.cpp`, `sensicSrc/drvPCR4.cpp`, `sydorSrc/drvT4U_EM.cpp` / `drvT4UDirect_EM.cpp`, `FX4Src/drvFX4.cpp`). `drv_quad_em` is the shared base (`drvQuadEM`): parameter library, sample ring buffer, averaging/trigger semantics, per-address NDArray callbacks. `nslsSrc/drvNSLS2_EM`/`drvNSLS2_IC` (memory-mapped FPGA/I²C) are out of scope — only devices reachable over TCP/UDP/serial/WebSocket are ported.

#### Shared quadEM records

Every IOC's model template starts with `include "quadEM.template"` (source: `iocs/quadem/db/quadEM.template`, which itself `include`s `NDArrayBase.template`). Selected records (full list is ~150 records incl. per-channel offset/scale/precision fanouts and `*Ave`/`FastAverage*` records — see the template for the complete set):

| PV suffix | Record type | Purpose |
|---|---|---|
| `Acquire` | bo | Start/stop acquisition (`PINI=YES`) |
| `Model` | mbbi | Model readback (`QE_MODEL`) — enumerates every quadEM device incl. unported ones |
| `Firmware` | waveform | Firmware version string |
| `AcquireMode` / `_RBV` | mbbo/mbbi | Continuous / Multiple / Single |
| `Range`, `Range1`-`Range4` / `_RBV` | mbbo/mbbi | Per-channel gain range (`QE_RANGE`, asyn addr 0-4) |
| `PingPong` / `_RBV` | mbbo/mbbi | Ping-pong buffering mode |
| `IntegrationTime` / `_RBV`, `SampleTime_RBV` | ao/ai | Conversion time / sampling time |
| `TriggerMode` / `_RBV`, `TriggerPolarity` / `_RBV` | mbbo/mbbi, bo/bi | Trigger config (each model redefines the choice set) |
| `NumChannels` / `_RBV` | mbbo/mbbi | 1/2/4 input channels |
| `Resolution` / `_RBV` | mbbo/mbbi | 16/24-bit |
| `BiasState`/`_RBV`, `BiasVoltage`/`_RBV`, `BiasInterlock`/`_RBV` | bo/bi, ao/ai | HV bias control |
| `HVSReadback`, `HVVReadback`, `HVIReadback` | bi, ai, ai | Bias state/voltage/current readback |
| `Temperature` | ai | Device temperature |
| `ValuesPerRead`/`_RBV`, `NumAcquire`/`_RBV`, `NumAcquired` | longout/longin | Acquisition counters |
| `ReadFormat`/`_RBV` | bo/bi | Binary vs ASCII wire format |
| `AveragingTime`/`_RBV`, `NumAverage_RBV`, `NumAveraged_RBV` | ao/ai, longin | Sample averaging |
| `Reset`, `ReadStatus` | bo | One-shot commands |
| `Geometry`/`_RBV` | mbbo/mbbi | Diamond/Square/SquareCC/Custom position geometry |
| `CurrentOffset1`-`4`, `CurrentScale1`-`4`, `CurrentPrec1`-`4` (+ `*Fanout*`) | ao, mbbo, dfanout | Per-channel current calibration |
| `WeightXsum*`, `WeightYsum*`, `WeightXdelta*`, `WeightYdelta*` | ao | Position-computation weights |
| `PositionOffsetX`/`Y`, `PositionScaleX`/`Y`, `PositionPrecX`/`Y` | ao, dfanout | Position calibration |
| `RingOverflows` | longin | Ring-buffer overflow count |
| `ReadData` | busy | Manual ring-buffer read trigger |
| `NDAttributesFile` | waveform | AD attributes file path (asynOctetWrite) |
| `Asyn` | asyn | Debug asyn record |
| `Current1Ave`-`Current4Ave`, `SumXAve`, `SumYAve`, `SumAllAve`, `DiffXAve`, `DiffYAve`, `PositionXAve`, `PositionYAve` | ai (`asynFloat64Average`) | Fast-averaged channel data (addr 0-10 on `QE_DOUBLE_DATA`) |
| `FastAveragingTime`/`_RBV`, `NumFastAverage`, `ProcessAve`/`2`, `FastScanCalc`, `FastScanSeq`/`2` | ao, transform, longin, seq, calcout | Fast-average scan-rate plumbing |

A companion `quadEM_TimeSeries.template` (`EraseAll`, `StartAll`, `StopAll`, `PresetReal`, `Dwell`/`_RBV`, `Current1TS`-`Current4TS`, `SumXTS`/`SumYTS`/`SumAllTS`, `DiffXTS`/`DiffYTS`, `PositionXTS`/`PositionYTS`, `*FFT` waveforms, `NuseAll`, `CurrentChannel`, `MaxChannels`) exists in `iocs/quadem/db/` for time-series/FFT MCA-style acquisition, but none of the 8 st.cmd files load it — it is not wired into any of the current IOCs.

#### The 8 IOCs

| IOC crate | Model / protocol | st.cmd | Extras (own template, beyond shared) |
|---|---|---|---|
| `ah401-ioc` | Elettra/CaenEls **AH401B**/AH401D picoammeter, TCP (`drvAsynIPPortConfigure` + `drvAHxxxConfigure`, EOS `\r\n`/`\r`) | `iocs/quadem/ah401-ioc/st.cmd` | `AH401B.template`: `Range`/`_RBV`, `IntegrationTime`/`_RBV`, `TriggerMode`/`_RBV` |
| `ah501-ioc` | Elettra/CaenEls **AH501BE** (AH501 series), TCP (same `drvAHxxxConfigure`) | `iocs/quadem/ah501-ioc/st.cmd` | `AH501.template`: `Range`/`_RBV`, `TriggerMode`/`_RBV` |
| `nsls-em-ioc` | **NSLS Precision Integrator**, UDP broadcast discovery (port 37747) then driver-opened TCP command (4747)/data (5757) (`drvNSLS_EMConfigure`) | `iocs/quadem/nsls-em-ioc/st.cmd` | `NSLS_EM.template`: `Range`/`_RBV`, `IntegrationTime`/`_RBV`, `PingPong`/`_RBV` |
| `pcr4-ioc` | SenSiC **PCR4** 4-channel picoammeter, TCP (`drvPCR4Configure`, EOS `\r\n`/`\r`) | `iocs/quadem/pcr4-ioc/st.cmd` | `PCR4.template`: `TriggerMode`/`_RBV`, `TriggerPolarity`/`_RBV`, `ReadFormat` |
| `t4u-direct-em-ioc` | Sydor **T4U**, direct to meter — telnet command (23) + driver-bound UDP data port, calibration file `DBPM_Settings.ini` (`drvT4UDirect_EMConfigure`) | `iocs/quadem/t4u-direct-em-ioc/st.cmd` | `T4UDirect_EM.template` (`include`s `T4U_EM.template`): `WaitStateMode`/`_RBV`, `ReadsPerPacket`/`_RBV` |
| `t4u-em-ioc` | Sydor **T4U**, through the Qt middle layer — driver-opened TCP command/data ports on `QTHOST:QTBASEPORT`/`+1` (`drvT4U_EMConfigure`) | `iocs/quadem/t4u-em-ioc/st.cmd` | `T4U_EM.template` (`include`s `gc_t4u.db`): bias P/N/pulse controls, `DACMode`, `PIDEn`/`PIDCtrlPol`/`PIDCtrlEx`/`PIDCuEn`/`PIDHystEn`, `Updater`, `PosTrackMode`, `SampleFreq`, plus model-local `IntegrationTime`/`_RBV`, `Geometry`, `Resolution`, `Range`/`Range1`-`4` (44 records total) |
| `tetramm-ioc` | CaenEls **TetrAMM** electrometer, TCP (`drvTetrAMMConfigure`, EOS `\r\n`/`\r`) | `iocs/quadem/tetramm-ioc/st.cmd` | `TetrAMM.template`: `Range`/`Range1`-`4` + `_RBV`, `TriggerMode`/`_RBV`, `InterlockStatus_RBV`, `InterlockCheck` |
| `fx4-ioc` | Pyramid **FX4** 4-channel picoammeter, JSON over **WebSocket** (`ws://`, `drvFX4Configure` — no asyn IP port; the driver opens the socket itself) | `iocs/quadem/fx4-ioc/st.cmd` | `FX4.template`: `Range`/`_RBV`, `CurrentUnits`/`SetCurrentUnits`/`_RBV`, `SetRange`/`GetRange`, `SetValuesPerRead`/`GetValuesPerRead`, `GetSampleTime`, bias set/get shadow records, `GetFirmware`, `TriggerMode`/`_RBV` (CA links to the FX4's own PV server via `$(FXP)`) |

Build/run, e.g.:
```
cargo run -p ah401-ioc --release -- iocs/quadem/ah401-ioc/st.cmd
cargo run -p fx4-ioc --release -- iocs/quadem/fx4-ioc/st.cmd
```
(same pattern for the other 6 — see each `st.cmd`'s header comment.)

### MCA (`drivers/mca`, `drivers/mca-amptek`, `drivers/mca-rontec`)

Ports from [`epics-modules/mca`](https://github.com/epics-modules/mca):
- `drivers/mca` — shared core of `mcaApp/mcaSrc`: the asyn MCA interface contract (`mca.h`/`drvMca.h`), `dev_mca_asyn::DevMcaAsyn` (ported from `devMcaAsyn.c`, binds the `mcaRecord` to any conforming asyn MCA driver), and `fastsweep` (ported from `drvFastSweep.cpp`, a software MCA sweeping an upstream asyn signal into channels).
- `drivers/mca-rontec` — Rontec detector driver, ported from `mcaApp/RontecSrc/drvMcaRontec.c`.
- `drivers/mca-amptek` — Amptek DP5/PX5/DP5G/MCA8000D/TB5/DP5-X driver, ported from `mcaApp/AmptekSrc/drvAmptek.cpp`. USB (`DppLibUsb.cpp`) is feasibility-gated out (no USB crate in the workspace); serial is unported because it's an empty no-op in the upstream C driver too.

The actual `mcaRecord` type (channel array, ROIs, presets, elapsed-time fields, `.S1`-`.S16`-style equivalents) is not defined in this repo — it comes from the standalone `mca-rs` crate (workspace dependency, `mca-rs = "0.24.3"` — pinned because "no `epics-rs` \"mca\" feature exists yet"). `drivers/mca`'s own `mcaSum.c` (ROI summing) equivalent lives in `mca_rs::record::roi::sum_rois`, called from `McaRecord::process()`.

#### Shared MCA record set

Every IOC loads the same `db/mca.db` (byte-identical across all three IOC dirs except header comments — verified by diff), which instantiates exactly one `mcaRecord`:

```
grecord(mca,"$(P)$(R)") {
    field(DTYP,"asynMCA")
    field(INP,"@asyn($(PORT),$(ADDR=0))")
    field(NMAX,"$(NCHAN)")
    field(NUSE,"$(NCHAN)")
}
```
Macros: `P`,`R` (name prefix/suffix), `PORT` (asyn MCA port), `ADDR` (signal/channel select, default 0), `NCHAN` (NMAX/NUSE — must match the driver's channel capacity).

#### Per-backend IOCs

| IOC crate | Backend | st.cmd | Extras |
|---|---|---|---|
| `mca-ioc` | Software MCA over a synthetic signal source: `DemoSourceConfig` (this IOC's own stand-in, not an upstream driver) → `initFastSweep` (`FastSweepConfig`) sweeping 1 signal into 10 channels | `iocs/mca-ioc/st.cmd` | none — one `mca.db` instance (`P=mca:,R=spectrum1,PORT=FS0,NCHAN=10`) |
| `mca-rontec-ioc` | **Rontec** detector over a serial octet port (`drvAsynSerialPortConfigure` + `RontecConfig`); EOS `\r\n` is this IOC's own placeholder, since upstream `RontecConfig` never sets one itself | `iocs/mca-rontec-ioc/st.cmd` | none — one `mca.db` instance (`PORT=RONTEC0,NCHAN=4096`) |
| `mca-amptek-ioc` | **Amptek DP5** over Ethernet/UDP (`drvAmptekConfigure(portName, interface=0, addressInfo, directMode)`) | `iocs/mca-amptek-ioc/st.cmd` | `mca.db` (`PORT=Amptek1,NCHAN=8192`) **+** `Amptek.db` (polarity, clock, gain, gate, peaking/flat-top time, thresholds, PUR, MCA/MCS source, MCS channel range, config file load/save, status: temps/HV/model/firmware/FPGA/build/serial, aux out 1/2/34, connectors, SCA output width, `CopyROIsSCAs`) **+** `Amptek_SCAn.db` loaded 8× (`N=0..7`, one per SCA channel — `dbLoadTemplate` isn't an iocsh command in epics-rs, so the upstream `Amptek_SCAs.substitutions` is mechanically expanded into 8 explicit `dbLoadRecords` calls in `st.cmd`) |

Build/run:
```
cargo run -p mca-ioc --release -- iocs/mca-ioc/st.cmd
cargo run -p mca-rontec-ioc --release -- iocs/mca-rontec-ioc/st.cmd
cargo run -p mca-amptek-ioc --release -- iocs/mca-amptek-ioc/st.cmd
```

### scaler974 (`drivers/scaler974`)

Ports from [`epics-modules/scaler`](https://github.com/epics-modules/scaler) — `drivers/scaler974/src/lib.rs`'s module doc: "Ortec 974 counter/timer asyn port driver, ported from `epics-modules/scaler` (`drvScaler974.cpp`)". `scaler974::driver::Scaler974Driver` implements `scaler-rs`'s `ScalerDriver` trait directly (no `asynInt32`/`PortDriver` layer needed — `ScalerAsynDeviceSupport` calls the Rust methods directly, collapsing what C's `devScalerAsyn.c` plus `Scaler974`'s own `asynPortDriver` did). `registry` hands the driver constructed by `initScaler974` off to a `register_dynamic_device_support` closure at `iocInit`; because `ScalerRecord` declares its own private `"OUT"` field (mirroring real `scalerRecord.dbd`), `DeviceSupportContext.out` is always empty for it, so this registry is deliberately **single-instance-only** (one `initScaler974`/board per process).

#### scaler974 records

Not vendored in this repo: `scaler974-ioc/main.rs` points `$(SCALER)` at `epics_rs::scaler::SCALER_DB_DIR` (the `scaler-rs` crate's own bundled `db/`, re-exported through `epics-rs`'s `scaler` feature, pinned to `scaler-rs 0.24.0` per `Cargo.lock`) and loads `$(SCALER)/scaler.db`. That file instantiates one real `scalerRecord`:

```
grecord(scaler,"$(P)$(S)") {
    field(DTYP,"$(DTYP)")
    field(PRIO,"HIGH")
    field(FLNK,"$(P)$(S)_cts1.PROC  PP MS")
    field(FREQ,"$(FREQ)")
    field(OUT,"$(OUT)")
    field(PREC,"3")
}
```
plus 8 chained `calc`/`transform` helper records (`_calcEnable`, `_calc_ctrl`, `_calc1`-`_calc8`, `_cts1`-`_cts4`) that derive count-rates from the record's own `.S1`-`.S16` (16-channel counts) and `.T` (elapsed-time preset) fields — confirming a 16-channel `scalerRecord`, consistent with `scaler-rs`'s sibling `scaler16.db`/`scaler32.db`/`scaler16m.db` files also present in that crate's `db/` (not loaded by this IOC).

`iocs/scaler974-ioc/st.cmd` configures a serial port (`drvAsynSerialPortConfigure`, 9600/8/N/1, EOS `\r\n`/`\r` — not set by `drvScaler974` itself, per `connect.rs`'s doc, so this is the IOC's own choice pending the Ortec 974 manual), then `initScaler974("SCL1","S0",0,100)` (100 ms poll), then `dbLoadRecords("$(SCALER)/scaler.db", "P=scaler974:,S=scaler1,OUT=@asyn(SCL1 0 0),FREQ=1000000")` followed by a `dbpf(...DTYP,"Asyn Scaler")` — DTYP is set via `dbpf` rather than a `dbLoadRecords` macro because macro-based `DTYP=` would force-overwrite every record's DTYP field in `scaler.db`, corrupting the `_calcEnable`/`_calc_ctrl` helper records' `"Soft Channel"` DTYP.

Build/run:
```
cargo run -p scaler974-ioc --release -- iocs/scaler974-ioc/st.cmd
```

### Notes on record-layout coverage
- The full `mcaRecord` field layout (ROI fields, preset fields, etc.) — defined in the external `mca-rs` crate (not in this repo); only the 4 macro-substituted fields (`DTYP`, `INP`, `NMAX`, `NUSE`) that `mca.db` sets are shown above.
- The full `scalerRecord` field layout beyond `DTYP`/`PRIO`/`FLNK`/`FREQ`/`OUT`/`PREC` and the `.S1`-`.S16`/`.T` fields referenced by `scaler.db`'s helper records — defined in the external `scaler-rs` crate.
- `quadEM_TimeSeries.template` exists in `iocs/quadem/db/` but is not loaded by any of the 8 st.cmd files; documented above as present-but-unused rather than part of any IOC's live record set.

---

## Vacuum, fieldbus & miscellaneous drivers

This section covers the workspace's device-specific drivers that don't fit
the motor/areaDetector/scaler families: vacuum controllers, fieldbus/PLC
protocols, an OPC-UA client, a robot arm, and a handful of serial
instruments (PID controller, delay generators, syringe pumps, a
displacement sensor, two Yokogawa data-acquisition units).

---

### vac — vacuum-gauge and ion-pump controllers

**Ports from:** [epics-modules/vac](https://github.com/epics-modules/vac)
(`devVacSen.c`, `vsRecord.c`, `devDigitelPump.c`)

`drivers/vac` implements two independent custom EPICS record types plus
their asyn-octet device support, exactly as upstream does: the `vs` record
+ `devVacSen` for vacuum-gauge controllers (Granville-Phillips GP307/GP350,
Televac MM200/MX200/CC10), and the `digitel` record + `devDigitelPump` for
ion-pump controllers (Perkin-Elmer Digitel 500/1500, Gamma Vacuum
MPC/QPC). Both families read/write raw asyn octet frames whose EOS is
owned by the startup script, not the device support — matching the C
module, which never calls `setInputEos`/`setOutputEos` itself.

Two separate IOC binaries wire these up:

| IOC crate | Build/run | Device family |
|---|---|---|
| `digitel-ioc` | `cargo run -p digitel-ioc --release -- iocs/vac/digitel-ioc/st.cmd` | Digitel 500/1500, MPC, QPC |
| `vacsen-ioc` | `cargo run -p vacsen-ioc --release -- iocs/vac/vacsen-ioc/st.cmd` | GP307, GP350, MM200, MX200, CC10 |

Each IOC loads exactly one record instance of its custom type (`drivers/vac/db/digitelPump.db`, `drivers/vac/db/vs.db`). All device data lives as
*fields* of that one record, not as separate PVs:

**`digitel` record** (`drivers/vac/src/records/digitel.rs`, DTYP `asyn DigitelPump`) — representative fields:

| Field group | Fields |
|---|---|
| Main value / alarms | `VAL`, `LVAL`, `HIHI`, `LOLO`, `HIGH`, `LOW`, `HHSV`, `LLSV`, `HSV`, `LSV`, `HYST`, `LALM` |
| Mode / bakeout | `MODS`, `MODR`, `BAKS`, `BAKR`, `COOL`, `CMOR` |
| Setpoints (×4) | `SET[]`, `SPS[]`, `SPR[]`, `SHS[]`, `SHR[]`, `SMS[]`, `SMR[]`, `SVS[]`, `SVR[]` |
| Ion current/voltage | `CRNT`, `VOLT`, `TONL`, `FLGS`, `SPFG`, `BKIN` |
| Display ranges | `HOPR`/`LOPR`, `HCTR`/`LCTR`, `HVTR`/`LVTR`, `HLPR`/`LLPR` |
| Simulation | `SIML`, `SIMM`, `SLMO`, `SVMO`, `SLCR`, `SVCR` |
| Status | `CYCL`, `ERR` |

(full 90-odd field list at `drivers/vac/src/records/digitel.rs:65`)

**`vs` record** (`drivers/vac/src/records/vs.rs`, DTYP `asyn VacSen`) — representative fields:

| Field group | Fields |
|---|---|
| Main value | `VAL`, `PRES`, `CGAP`, `CGBP`, `LPRS`, `LCAP`, `LCBP`, `CHGC` |
| Peak-hold shadow | `PVAL`, `PPRE`, `PCGA`, `PCGB`, `PLPE`, `PLCA`, `PLCB` |
| Alarms | `HIHI`, `LOLO`, `HIGH`, `LOW`, `HHSV`, `LLSV`, `HSV`, `LSV`, `HYST`, `LALM` |
| Setpoints | `SP[]`, `SPS[]`, `SPR[]` (and their peak-shadow `PSP[]`/`PSS[]`/`PSR[]`) |
| Display ranges | `HOPR`/`LOPR`, `HLPR`/`LLPR`, `HAPR`/`LAPR`, `HALR`/`LALR`, `HBPR`/`LBPR`, `HBLR`/`LBLR` |
| Config | `TIPE`, `DGSS`, `DGSR`, `FLTR`, `PREC`, `ERR` |

(full ~70-field list at `drivers/vac/src/records/vs.rs:35`)

---

### ether-ip — Allen-Bradley ControlLogix / PLC-5 (EtherNet/IP + CIP)

**Ports from:** [epics-modules/ether_ip](https://github.com/epics-modules/ether_ip)
(`ether_ip.c`, `drvEtherIP.c`, `devEtherIP.c`)

`drivers/ether-ip` speaks the CIP message-router protocol (tag-path
encoding, ReadData/WriteData/MultiRequest, CM_Unconnected_Send) over the
EtherNet/IP encapsulation layer (`ListServices`/`RegisterSession`/
`SendRRData`), maintaining one TCP session per PLC and a scan thread that
packs tags into MultiRequests. Standard EPICS record types bind through
DTYP `EtherIP` (`EtherIPReset` for the stats-reset `bo`) via a
`@PLC tag [B bit] [S secs] [E]` link.

**Build/run:** `cargo run -p ether-ip-ioc --release -- iocs/ether-ip/ether-ip-ioc/st.cmd`

`db/eip_stat.db` — driver/PLC statistics (real, fixed PVs):

| Record | Type | DTYP link |
|---|---|---|
| `$(IOC):PLC_ERRORS` | ai | `@$(PLC) $(TAG) PLC_ERRORS` |
| `$(IOC):PLC_TASK_SLOW` | ai | `@$(PLC) $(TAG) PLC_TASK_SLOW` |
| `$(IOC):LIST_TICKS` | ai | `@$(PLC) $(TAG) LIST_TICKS` |
| `$(IOC):LIST_SCAN_TIME` | ai | `@$(PLC) $(TAG) LIST_SCAN_TIME` |
| `$(IOC):LIST_MIN_SCAN_TIME` | ai | `@$(PLC) $(TAG) LIST_MIN_SCAN_TIME` |
| `$(IOC):LIST_MAX_SCAN_TIME` | ai | `@$(PLC) $(TAG) LIST_MAX_SCAN_TIME` |
| `$(IOC):TAG_TRANSFER_TIME` | ai | `@$(PLC) $(TAG) TAG_TRANSFER_TIME` |
| `$(IOC):RESET_PLC_STATS` | bo | DTYP `EtherIPReset` |

`db/test.db` is a demo database (not fixed production PVs) exercising the
link grammar against arbitrary PLC tags — `ai`/`ao`/`bi`/`bo`/`mbbi`/
`mbbo`/`mbbiDirect`/`stringin`/`waveform` records addressing scalar tags,
array elements (`REALs[5]`), single-element-only reads (`E` flag), explicit
bit addressing (`B <bit>`), and I/O-Intr driven records.

---

### ip — serial vacuum/temperature instruments (epics-modules `ip`)

**Ports from:** [epics-modules/ip](https://github.com/epics-modules/ip)
(`ipApp/src`: `devMPC.c`, `devTPG261.c`, `devTelevac.c`, `devAiMKS.c`,
`devAiHeidND261.c`, `devXxEurotherm.c`)

`drivers/ip` (crate `ip-devices`) re-implements each C device-support file
as an asyn **port driver**: the record link binds to a port parameter
instead of embedding the raw command text, and each port drives its device
over a pre-configured serial/IP octet port on its own worker thread.

**Build/run:** `cargo run -p ip-ioc --release -- iocs/ip/ip-ioc/st.cmd`

| Device | db template | Records |
|---|---|---|
| Gamma Vacuum MPC / Digitel ion pump | `mpc.db` | `STAT`, `PRES`, `PRESEGU`, `CUR`, `VOLT`, `SIZE`, `SP1V`-`SP4V`, `SP1S`-`SP4S`, `GAUTOS`, `TSPSTAT`, `DIS`, `UNIT`, `SSIZE`, `SET1`-`SET4`, `STOP`, `ULCK`, `SAUTOS` (26 records) |
| Titanium sublimation pump (same controller) | `tsp.db` | `TSP_STAT`, `TSP_REMAINING`, `TSP_MIN1`-`MIN4`, `SEND_TIMED`, `TSP_OFF`, `TSP_FIL`, `TSP_CLEAR`, `TSP_AUTO_ADV`, `TSP_CONTINUOUS`, `SEND_SUBLIMATION`, `TSP_DEGAS`, `TSP_MODE`, `TSP_SUBLIMATION`, `TSP_SUB2`, `TSP_SETMODE`, `TSP_TIMED`, `TSP_VALUE`, `TSP_UNITS`, `TSP_SECONDS`, `TSP_MINUTES`, `TSP_NCYCLES`, `TSP_MIN_PRESS` (25 records) |
| Pfeiffer TPG261/TPG262 gauge controller | `tpg261.db` | `ID`, `UNIT`, `PRES`, `GSTATUS`, `STATUS`, `SP1S`, `SP2S`, `SP1V`, `SP2V`, `START`, `SUNIT`, `SET1`, `SET2` (13 records) |
| Televac vacuum gauge | `televac.db` | `PRES` (1 record) |
| Televac relay outputs | `televac_relay.db` | `ON`, `OFF`, `STATE` (3 records) |
| MKS / HPS SensaVac 937 gauge | `mks.db` | `PRES`, `STATUS`, `UNITS`, `TYPE`, `LOW`, `HIGH` (6 records) |
| Heidenhain ND261 display unit | `nd261.db` | `POS`, `STATUS` (2 records) |
| Eurotherm 800/2000 temperature controller | `eurotherm.db` | `Setpoint`, `RampRate`, `ReadRequest` (3 records) |

---

### twincat-ads — Beckhoff TwinCAT PLC (ADS/AMS protocol)

**Ports from:** [epics-modules/twincat-ads](https://github.com/epics-modules/twincat-ads)
(`adsAsynPortDriver`)

`drivers/twincat-ads` speaks the ADS/AMS wire protocol directly (AMS/TCP
framing, the nine ADS commands, notifications, symbol lookup) rather than
linking Beckhoff's `AdsLib`, so nothing C/C++ is built. Records bind via
`@asyn(PORT,addr,timeout)<options>/<plc-address><?|=>`, with options for
AMS sub-port, PLC-side sample/buffer timing, timestamp source, and bulk
polling vs. subscription.

**Build/run:** `cargo run -p twincat-ads-ioc --release -- iocs/twincat-ads-ioc/st.cmd`

`db/adsTestAsyn.db` — port of upstream `adsExApp/Db/adsTestAsyn.db`, one
group of records per PLC test variable:

| PLC variable | Type | Records |
|---|---|---|
| `Main.fAmplitude` (LREAL) | ao/ai | `SetFAmplitudeRB`, `SetFAmplitudeRBPoll`, `SetFAmplitude`, `GetFAmplitude` |
| `Main.fTest` (LREAL) | ai | `GetFTestPLCTime`, `GetFTestEpicsTime` (+ `:T` Soft-Timestamp stringins) |
| `Main.bEnableUpdateSine` (BOOL) | bo/bi | `SetBEnableUpdateSineRB`, `SetBEnableUpdateSineRBPoll`, `SetBEnableUpdateSine`, `GetBEnableUpdateSine` |
| `Main.iCycleCounter` (DINT) | ao/ai | `SetICycleCounter`, `SetICycleCounterRB`, `GetICycleCounter`, `GetICycleCounterSCAN` |
| `Main.fTestArray` (ARRAY OF LREAL) | waveform | `GetFTestArray` |
| `Main.fTestArray2` (ARRAY OF LREAL) | waveform | `SetFTestArray2RB`, `SetFTestArray2`, `GetFTestArray2` |
| `Main.bArray` (ARRAY OF BOOL, as CHAR) | waveform | `SetBArrayRB`, `SetBArray`, `GetBArray` |
| `Main.sTest` (STRING) | waveform/stringin/stringout/lso/lsi | `SetSTestRB`, `SetSTest`, `GetSTest`, `SetSTestStringRB`, `SetSTestString`, `GetSTestString`, `SetSTestLsoRB`, `SetSTestLso`, `GetSTestLsi` |
| `Main.Int8Array` (ARRAY OF SINT) | waveform | `SetInt8ArrayRB`, `SetInt8Array`, `GetInt8Array` |
| `Main.Int32Array` (ARRAY OF DINT) | waveform | `SetInt32ArrayRB`, `GetInt32Array` |
| AMS port state (driver-internal, not a PLC symbol) | ai/ao | `GetAmsPort851State`, `SetAmsPort851State` |

---

### opcua — OPC-UA client device support

**Ports from:** [epics-modules/opcua](https://github.com/epics-modules/opcua)
(`devOpcuaSup/`: link grammar, device support, item/element tree, session
and subscription lifecycle)

`drivers/opcua` re-implements the client-library-agnostic layer of the C
module on top of the pure-Rust `async-opcua` client (crate aliased from
`async-opcua` to avoid a name collision), replacing both C backends
(Unified Automation SDK, open62541 — the latter is the behavioural
reference). Records bind via `@<session-or-subscription> <node> [key=value ...]`;
naming a subscription makes the item monitored, naming a session makes it
read-on-demand.

**Build/run:** `cargo run -p opcua-ioc --release -- iocs/opcua-ioc/st.cmd`

`db/opcuaExample.db` — modelled on the C module's `Demo.Static.Scalar.db`
and `Demo.WorkOrderVariable.template`:

| Record | Type | Node |
|---|---|---|
| `bibool` / `bobool` | bi / bo | `Demo.Static.Scalar.Boolean` |
| `liint32` / `loint32` | longin / longout | `Demo.Static.Scalar.Int32` |
| `i64iint64` | int64in | `Demo.Static.Scalar.Int64` |
| `aidouble` / `aodouble` | ai / ao | `Demo.Static.Scalar.Double` |
| `aidoublex` | ai (unmonitored, `monitor=n`) | `Demo.Static.Scalar.Double` |
| `sistring` | stringin | `Demo.Static.Scalar.String` |
| `mbbienum` | mbbi (2-bit mask) | `Demo.Static.Scalar.Byte` |
| `wfdouble` | waveform (10 elem) | `Demo.Static.Arrays.Double` |
| `item` | `opcuaItem` | `Demo.Static.UserScalar.WorkOrder` |
| `assetid` / `assetidRBK` | stringout / stringin | item element `AssetID` |
| `priority` | longout | item element `Priority` |

---

### ur-robot — Universal Robots arm (RTDE + script + dashboard + gripper)

**Ports from:** [epics-modules/urRobot](https://github.com/epics-modules/urRobot)
(plus the vendored `ur_rtde` C++ client, pin `68ac4e18`, re-implemented
from scratch rather than linked)

`drivers/ur-robot` re-implements all four TCP interfaces urRobot drives an
arm through, each as its own asyn port driver mirroring urRobot's
`asynPortDriver` subclasses and PV surface:

| Interface | Port | Rust modules |
|---|---|---|
| RTDE (binary, big-endian) | 30004 | `rtde`, `session`, `stream`, `receive`, `control`, `io` |
| Script server (URScript text) | 30003 | `script` |
| Dashboard server (line text) | 29999 | `dashboard` |
| Robotiq gripper URCap (text) | 63352 | `gripper` |

**Build/run:** `cargo run -p ur-robot-ioc --release -- iocs/ur-robot/ur-robot-ioc/st.cmd`

Records, by db file (all under `$(P)` prefix):

| db file | Record count | PV pattern / examples |
|---|---|---|
| `dashboard.db` | 25 | `Dashboard:Connected/Running/IsProgramSaved/IsInRemoteControl` (bi); `Dashboard:Play/Stop/Pause/Connect/Disconnect/Shutdown/PowerOn/PowerOff/BrakeRelease/...` (bo); `Dashboard:PolyscopeVersion/SerialNumber/ProgramState/RobotMode/RobotModel/LoadedProgram/SafetyStatus` (stringin); `Dashboard:Popup/LoadURP` (stringout) |
| `robotiq_gripper.db` | 24 | `RobotiqGripper:Connected/Calibrated/IsActive/IsOpen/IsClosed/IsStoppedInner/IsStoppedOuter` (bi); `RobotiqGripper:Connect/Activate/AutoCalibrate/Open/Close` (bo); `RobotiqGripper:MinPosition/MaxPosition` (longout); `RobotiqGripper:MoveStatus` (mbbi); `RobotiqGripper:CurrentPosition/OpenPosition/ClosedPosition` (ai); `RobotiqGripper:SetSpeed/SetForce` (ao) |
| `rtde_control.db` | ~110 | `Control:Connected/Steady/Moving/AsyncMoveDone` (bi); `Control:moveJ/moveL/stopJ/stopL/TeachMode/TriggerProtectiveStop` (bo); per-joint `Control:J1Cmd`…`J6Cmd` + `J*TweakVal/TweakFwd/TweakRev` (ao/bo/calcout); per-pose `Control:PoseXCmd`…`PoseYawCmd` + tweaks; `Control:TCPOffset_X/Y/Z/Roll/Pitch/Yaw`; `Control:JointSpeed/JointAcceleration/JointBlend`, `LinearSpeed/LinearAcceleration/LinearBlend`; `Control:CustomScriptFile/CustomInlineScript/RunCustomScriptFile/CustomScriptTimeout` |
| `rtde_control_jog.db` (optional, not loaded by default st.cmd) | 19 | `Control:Jogging/JogStart/JogStop/JogAccel/JogSpeedX..Yaw`, `Control:jog_watchdog:*` |
| `rtde_io.db` | 22 | `IO:SpeedSlider` (ao); `IO:SetStandardDO0-7/SetConfigurableDO0-7/SetToolDO0-1` (bo); `IO:SetVoltageAO0-1/SetCurrentAO0-1` (ao) |
| `rtde_receive.db` | ~43 | `Receive:Connected/Disconnect/Reconnect`; scalar readbacks `Receive:ControllerTimestamp/SafetyStatusBits/DigitalInputBits/DigitalOutputBits/RuntimeState/RobotMode/SafetyMode/AnalogInput0-1/AnalogOutput0-1/SpeedScaling/...`; per-joint `Receive:Joint1`…`Joint6`; per-pose `Receive:PoseX..Yaw`; waveforms `ActualJointPositions/Velocities/Currents`, `ActualTCPPose/Speed/Force`, `TargetJoint*`, `TargetTCPPose/Speed`, `JointTemperatures`, `ActualJointVoltages`, `JointModes`, `ActualToolAccelerometer` |

---

### love — Love Controls PID controller (RS-485)

**Ports from:** [epics-modules/love](https://github.com/epics-modules/love)
(`drvLove.c`)

`drivers/love` is an asyn port driver for Love 1600/16A PID controllers on
a shared RS-485 multi-drop line, using the checksummed ASCII frame format
and per-address model configuration (`LoveInit` for the port, `LoveConfig`
per bus address) that the C driver uses. EOS (ACK `0x06` in / ETX `0x03`
out) is hardcoded by the driver itself, not the startup script.

**Build/run:** `cargo run -p love-ioc --release -- iocs/love-ioc/st.cmd`

| db template | Records |
|---|---|
| `LoveController.template` | `Disable` (bo); `getValue/getSP1/getSP2/getAlLo/getAlHi/getPeak/getValley/getDecpts` (longin, DTYP `asynInt32`); `getAlMode/getInpType` (mbbi, `asynUInt32Digital`); `getCommStatus`, `AlarmEnable` (bi, `asynUInt32Digital`); `Value/SetPt1/SetPt2/AlarmLo/AlarmHi/Peak/Valley` (calc, decimal-point-scaled); `FastFanout`, `SlowFanout` |
| `LoveControllerControl.template` | `putSP1/putSP2/putAlLo/putAlHi` (longout, `asynInt32`); `PutSetPt1/PutSetPt2/PutAlarmLo/PutAlarmHi` (calcout, decimal-point-scaled writes) |

---

### delaygen — SRS DG645 / Colby PDL-100A / Coherent SDG delay generators

**Ports from:** [epics-modules/delaygen](https://github.com/epics-modules/delaygen)
(`drvAsynDG645.cpp`, `drvAsynColby.cpp`, `drvAsynCoherentSDG.cpp`)

`drivers/delaygen` implements three unrelated ASCII serial protocols as
three asyn port drivers sharing one crate. The shipped `st.cmd` wires one
shared serial port (19200 8N1) and enables exactly one device block at a
time (matching upstream's own `st.cmd` + per-device `.cmd` split); the
other two device blocks are commented out.

**Build/run:** `cargo run -p delaygen-ioc --release -- iocs/delaygen-ioc/st.cmd`

| Device | db template | Records |
|---|---|---|
| SRS DG645 (active by default in the shipped st.cmd) | `dg645.template` | 242 records: identity/status/reset (`Label`, `IdentSI`, `StatusLI/SI`, `StatusClearBO`, `ResetBO`, `GotoLocal/RemoteBO`, `EventStatusLI`/`MBBID`); trigger config (`TriggerRate/Level/Source/Inhibit/Delay/AdvancedMode/Holdoff`, per-pair `Trigger{AB,CD,EF,GH}Prescale`/`Phase`); burst config (`BurstMode/Count/Config/Delay/Period`); interface config (Serial/GPIB/LAN/DHCP/AutoIp/StaticIp/BareSocket/Telnet/Vxi state + reset, `IfaceIpAddr/NetMask/Gateway/MacAddr`); 8 channel delays A–H (`{X}Reference`, `{X}Delay`, `{X}DelayTweak{Val,IncBO,DecBO}`); 5 output pairs T0/AB/CD/EF/GH (`{Y}OutputAmp/Offset/Polarity`, `{Y}OutputModeTtlSS/NimSS`, tweak inc/dec) |
| Colby PDL-100A | `colbyPDL100A.template` | `delay`, `delay_rbk`, `inc`, `dec`, `step`, `step_rbk`, `units`, `ip`, `gw`, `nm`, `tcp`, `dhcp`, `mac`, `update`, `init` (15 records) |
| Coherent SDG | `coherentSDG.template` | `disableScanBI`, `identSI`, `trigRateMI/MO`, `bwdSwitchBI`, `bwdPD1BI`, `bwdPD2BI`, `bwdVdcIntlckBI`, `bwdResetBO`, `bwdFO`, `rfSyncXF/BI/BO`, `trigModXF`, `trigModeBI/BO`, `manTrigBO`, and per-channel (Ch1–Ch3) `{X}outDelayXF/BI/BO/AI/AO/IncCO/DecCO/StepAO` (41 records) |

---

### syringepump — Teledyne ISCO, ISCO (Modbus), Vindum syringe pumps

**Ports from:** [epics-modules/SyringePump](https://github.com/epics-modules/SyringePump)
(`teled_d.proto`/`teled_h.proto` for the native driver; ISCO/Vindum are
Modbus-only upstream)

`drivers/syringepump` is a native asyn port driver for the Teledyne ISCO
D/H-series pumps, translating their StreamDevice `.proto` command tables
(no StreamDevice engine exists in this framework) into a native command
table, transcribed byte-for-byte including the shared `%0<nsum>` frame
checksum. The module's other two families, ISCO and Vindum, are
Modbus-only upstream (`drvModbusAsynConfigure` + generic templates, no
StreamDevice) and are wired directly in `iocs/syringepump-ioc` against
`epics-modbus-rs` — no driver code of their own in this crate.

**Build/run:** `cargo run -p syringepump-ioc --release -- iocs/syringepump-ioc/st.cmd`

Native-driver templates (`drivers/syringepump`-backed):

| db template | Records |
|---|---|
| `teledynePumpD.template` | `PressureTweak(+Down/Up)`, `Run`, `Stop`, `Remote`, `Local`, `PressureSP`, `PressSeq`, `PressSend`, `MaxFlow`, `MaxFlowSeq`, `MaxFlowSend`, `Scanner`, `Vol`, `Pressure`, `PressSet_RBV`, `Flow`, `MaxFlow_RBV`, `Status`, `Command`, `CmdReply`, `AlarmI` (22 records) |
| `teledynePumpH.template` | Same core set as D-series plus `VolPARSE/Format`, `PressurePARSE/Format`, `PressSetPARSE/Format`, `FlowPARSE/Format`, `MaxFlow_RBVPARSE/Format` (`aSub` parsers, since the H-series reply is line text, not binary), `Units`, `Mode`, `ID`, `Refill`, `GetRefillLimit`, `SetRefillRate(+Calc)`, `SetDigitalOut`, `SetDigitalCalc1`, `AutoFillSeq1/2`, `ControlVentSeq/2` (44 records) |

Modbus-wired templates (loaded directly by `iocs/syringepump-ioc`'s
st.cmd against `drvModbusAsynConfigure` ports, not the `syringepump`
driver crate):

| db template | Records |
|---|---|
| `ISCOController.template` | `PressUnits`, `SetAtm/Bar/kPa/PSI`, `FlowUnits`, `SetmlPerMin/mlPerHr/ulPerMin/ulPerHr` (10 records) |
| `ISCOPumpN.template` | `PressureTweak/FlowRateTweak/RefillRateTweak` (+Down/Up), `Mode`, `SetConstPress/SetConstFlow`, `CopyMaxVolume`, `Description` (14 records) |
| `VindumController.template` | `PumpModeRBVCalc` (×3), `PumpMode_RBV`, `TotalVolumeSP(+Calc)`, `ResetVolume` (A/B/total, +Calc), `PumpMode`, `PumpModeSeq`, `PumpModeCalcA/B` (16 records) |
| `VindumPumpN.template` | `PressureTweak/FlowRateTweak` (+Down/Up), `PumpMode`, `PumpMode_RBV`, `AutofillMode`, `VolumeMode`, `Run`, `Start{Normal,Volume,Autofill,AutoVolume}Calc`, `VolRemaining`, `Description`, `PressureLOPR/HOPR` (18 records) |
| generic Modbus glue (`bi_bit`, `bo_bit`, `ai`, `aiFloat64`, `ao`/`aoFloat64`, `longinInt32`, `longoutInt32`, `vindumMbbi`, `vindumMbbo` templates) | one instance per Modbus bit/register offset — 142 input bits + 109 output bits + 208 holding registers for ISCO alone; see `iocs/syringepump-ioc/st.cmd` for the full per-offset expansion |

---

### microepsilon — capaNCDT6200 capacitive displacement sensor

**Ports from:** [epics-modules/microEpsilon](https://github.com/epics-modules/microEpsilon)
(`capaNCDT6200Sup.c` for the data port; `capaNCDT6200.proto` StreamDevice
protocol for the config port)

`drivers/microepsilon` ports two independent asyn ports per physical unit:
**L0**, a config port whose ~30-command ASCII protocol (sample rate,
averaging, trigger mode, per-channel status/info/linearization, data-port
number, analog filter) is translated from StreamDevice `.proto` into a
native command table; and **L1**, a fully custom native driver with its
own background reader thread ingesting a raw binary TCP measurement
stream (fixed-format packets, up to 4 channels), ported byte-for-byte
including duplicate/missed-packet detection and averaging/throttle.

**Build/run:** `cargo run -p microepsilon-ioc --release -- iocs/microepsilon-ioc/st.cmd`

| Port | db template | Records |
|---|---|---|
| L0 (config, `xxCapaNCDT6200.template`) | 68 records | Identity: `version1M/2M`, `welcome`, `deviceID`, `serialNum`. Averaging: `avgTypeModeM/C`, `avgNumModeM/C`. Trigger: `trigModeC/M`. Channel status: `chan1-4StatM`, `chanStatusM`. Linearization: `linModeM`, `chan1-4LinModeM/C`, `CH1-4:setLinPointC`. Sampling: `sampleTimeC/M`. Data routing: `dataPortM/C`. Filter: `analogFilterC/M`. Math-function reset: `ch1-4ClearMathFuncC`. Measurement: `measDataMonitor`, `measDataM`. Per-channel info: `chan1-4InfoM`, `chan1-4NAM/ANO/OFS/SNO/RNG/UNT`. |
| L1 (data, `xxCapaMeas.template`) | 20 records | `dispChan1-4M` (measured displacement); `measRangeChan1-4C`; `pvThrottleC`; `numMeasChans`; connection-health counters `dataPacket:{goodCount,badReadCount,badCount,timeoutCount,outSequenceCount,duplicateCount,missedCount,measuredCount}`; `dataPacket:Fanout`, `dataPacket:Fanout2` |

---

### yokogawa-gm10 / yokogawa-mw100 — Yokogawa data-acquisition units

**Ports from:** [epics-modules/Yokogawa_DAS](https://github.com/epics-modules/Yokogawa_DAS)
(GM10: `drvGM10.c` + `devGM10_{ai,ao,bi,bo,mbbi,mbbo,longin,stringin}.c`;
MW100: `drvMW100.c` + `devMW100_{ai,ao,bi,bo,mbbi,mbbo,longin,stringin}.c`)

Both drivers are the workspace's first (and only) drivers built on a
dynamic `DeviceSupport` factory instead of asyn — the wire protocol is a
proprietary TCP ASCII/binary command set, not Modbus or a generic bus.
Each implements all 8 upstream record-type dsets from a single factory
that mirrors every `devGM10_*.c`/`devMW100_*.c::init_record`'s
command/address grammar. GM10 addresses channels **module-relative**
(module *N*'s channels occupy Signal addresses `N*100+1 .. N*100+k`);
MW100 addresses them **sequentially** across installed modules in slot
order — a structural difference between the two protocols, not a port
inconsistency.

**Build/run:**
- `cargo run -p yokogawa-gm10-ioc --release -- iocs/yokogawa-gm10-ioc/st.cmd`
- `cargo run -p yokogawa-mw100-ioc --release -- iocs/yokogawa-mw100-ioc/st.cmd`

**GM10** (`iocs/yokogawa-gm10-ioc/db/`):

| db template | Records |
|---|---|
| `gm10_system.template` | `IPAddress`; `Module0-9` + `Module0-9:Presence`; `ErrorFlag`, `ErrorMessage1-3`, `ErrorClear`; `AlarmFlag`, `AlarmAck`; pollers `ChannelPoll`, `MiscPoll`, `StatusPoll`, `InfoPoll`; `Settings`; `Recording`/`RecordingSet`; `Computation`/`ComputationSet` (~30 records) |
| `gm10_analog_input.template` | `$(ADDRESS)` (ai), `:Unit`, `:ChStatus`, `:ValStatus`, `:Alarm1-4`, `:AlarmStatus`, `:Label` (10 records) |
| `gm10_analog_output.template` | as above plus `:Set` (ao), `:Mode`, `:TweakVal/Fwd/Bkwd` (16 records) |
| `gm10_digital_input.template` | `$(ADDRESS)` (bi) + status/alarm/label group (9 records) |
| `gm10_digital_output.template` | as digital input plus `:Mode`, `:Set` (bo) (11 records) |
| `gm10_pulse_input.template` | `$(ADDRESS)` (longin) + status/alarm/label group (9 records) |
| `gm10_calculation.template` | `$(ADDRESS)` (ai), `:Unit`, `:Expr`, status/alarm/label group (11 records) |
| `gm10_communication.template` | `$(ADDRESS)` (ai), `:Set` (ao), `:Unit`, status/alarm/label group (11 records) |
| `gm10_constant.template` | `$(ADDRESS)` (ai), `:Set` (ao), `:Label` (3 records) |
| `gm10_varconstant.template` | `$(ADDRESS)` (ai), `:Set` (ao), `:Label` (3 records) |

**MW100** (`iocs/yokogawa-mw100-ioc/db/`):

| db template | Records |
|---|---|
| `mw100_system.template` | `IPAddress`; `Module0-5` + `:Presence/:Model/:Code/:Speed/:Number`; `ErrorFlag`, `ErrorMessage1-3`, `ErrorClear`; `AlarmFlag`, `AlarmAck`; pollers `InputPoll`, `OutputPoll`, `StatusPoll`, `InfoPoll`; `OpMode`/`OpModeSet`; `MeasureMode`; `Computation`/`ComputationSet` (~40 records) |
| `mw100_mx110_channel.template` / `mx112_channel.template` (analog input) | `:ADC` (ai), `:Unit`, status/alarm/label group (10 records) |
| `mw100_mx114_channel.template` (counter) | `:Counter` (longin) + status/alarm/label group (9 records) |
| `mw100_mx115_channel.template` (digital input) | `:DI` (bi) + status/alarm/label group (9 records) |
| `mw100_mx120_channel.template` (analog output) | `:DAC` (ai), `:DAC_Set` (ao), `:Unit`, `:ChStatus`, `:DAC_Mode`, `:DAC_TweakVal/Fwd/Bkwd`, `:Label` (9 records) |
| `mw100_mx125_channel.template` (relay) | `:Relay` (bi), `:ChStatus`, `:RelayMode`, `:RelaySet` (bo), `:Label` (5 records) |
| `mw100_calculation_channel.template` | `$(ADDRESS)` (ai), `:Unit`, `:Expr`, status/alarm/label group (11 records) |
| `mw100_communication_channel.template` | `$(ADDRESS)` (ai), `:Set` (ao), `:Label` (3 records) |
| `mw100_constant_channel.template` | `$(ADDRESS)` (ai), `:Set` (ao), `:Label` (3 records) |
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

An epics-rs 0.24.3 based areaDetector IOC for the Intel RealSense D435i
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
