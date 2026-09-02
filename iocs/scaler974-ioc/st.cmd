#============================================================
# st.cmd — Ortec 974 counter/timer IOC (serial/GPIB octet port)
#
# Usage:
#   cargo run -p scaler974-ioc -- st.cmd
#
# Mirrors upstream epics-modules/scaler's iocBoot/iocScalerTest/st.cmd
# (serial port setup) plus iocsh/softScaler.iocsh's dbLoadRecords
# convention (this driver reuses scaler-rs's own bundled db/scaler.db —
# see main.rs's `SCALER` env var, set to the scaler-rs crate dir).
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
dbLoadRecords("$(SCALER)/db/scaler.db", "P=scaler974:,S=scaler1,OUT=@asyn(SCL1 0 0),DTYP=Asyn Scaler,FREQ=1000000")

#------------------------------------------------------------------------------
iocInit()
