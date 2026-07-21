#============================================================
# st.cmd — Ortec 974 counter/timer IOC (serial/GPIB octet port)
#
# Usage:
#   cargo run -p scaler974-ioc -- st.cmd
#
# Mirrors upstream epics-modules/scaler's iocBoot/iocScalerTest/st.cmd
# (serial port setup) plus iocsh/softScaler.iocsh's dbLoadRecords
# convention (this driver reuses scaler-rs's own bundled db/scaler.db —
# see main.rs's `SCALER` env var, set to `epics_rs::scaler::SCALER_DB_DIR`).
#
# Like delaygen/love, EOS is not set by drvScaler974 itself (see
# connect.rs's module doc) -- consult the Ortec 974 manual for the actual
# terminator and set it explicitly below before initScaler974 runs.
#============================================================

# ---- underlying serial octet port ----
drvAsynSerialPortConfigure("S0", "/dev/ttyS0", 0, 0, 0)

asynSetOption(S0, 0, "baud",    "9600")
asynSetOption(S0, 0, "bits",    "8")
asynSetOption(S0, 0, "parity",  "none")
asynSetOption(S0, 0, "stop",    "1")
asynSetOption(S0, 0, "clocal",  "Y")
asynSetOption(S0, 0, "crtscts", "N")

asynOctetSetInputEos("S0", 0, "\r\n")
asynOctetSetOutputEos("S0", 0, "\r")

# ---- Scaler974 driver ----
# initScaler974(portName,serialPort,serialAddr,poll) -- C
# initScaler974(portName,serialPort,serialAddr,poll); poll is milliseconds
# between SHOW_COUNTS polls while armed (0 defaults to 100, per
# drvScaler974.cpp).
initScaler974("SCL1", "S0", 0, 100)

# ---- scalerRecord ----
#
# NOTE 1: device support binds by DTYP alone, not by matching this
# record's OUT link back to a specific initScaler974 call -- scalerRecord
# declares its own private "OUT" field (mirroring real scalerRecord.dbd),
# so dbLoadRecords never populates the generic RecordCommon.out that
# register_dynamic_device_support's context exposes (see
# scaler974::registry's module doc for the full explanation). This IOC
# therefore supports exactly one scaler974 instance/board per process --
# a second initScaler974 call before this record binds is a startup
# error, by design.
#
# NOTE 2: DTYP is set below via dbpf, NOT as a dbLoadRecords macro. epics-
# base-rs 0.22.1's dbLoadRecords applies a passed DTYP=... macro via
# db_loader::override_dtyp, which force-overwrites *every* record's DTYP
# field in the loaded file -- not just fields that reference $(DTYP) in
# the text (real EPICS macLib only ever does textual $(...) substitution
# and never touches a hardcoded literal). scaler.db's own helper records
# (scaler1_calcEnable/_calc_ctrl, DTYP="Soft Channel" literal) would be
# corrupted to DTYP="Asyn Scaler" too, breaking their soft-channel
# classification. Loading without the DTYP macro and patching just the
# scaler record's DTYP field via dbpf afterward (still pre-iocInit, so
# wire_device_support's dtyp read sees the corrected value) avoids the
# blast radius entirely without editing the vendored db file.
dbLoadRecords("$(SCALER)/scaler.db", "P=scaler974:,S=scaler1,OUT=@asyn(SCL1 0 0),FREQ=1000000")
dbpf("scaler974:scaler1.DTYP", "Asyn Scaler")

#------------------------------------------------------------------------------
iocInit()
