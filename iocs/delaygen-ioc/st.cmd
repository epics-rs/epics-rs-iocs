#============================================================
# st.cmd — delay/pulse generator IOC (SRS DG645 / Colby PDL-100A /
# Coherent SDG)
#
# Usage:
#   cargo run -p delaygen-ioc -- st.cmd
#
# Mirrors the upstream epics-modules/delaygen st.cmd: one shared "serial1"
# octet port at 19200/8N1, with exactly one device-specific fragment active
# below (PICK ONE) — the other two are commented out, matching the C
# driver's own st.cmd + {dg645,colby,coherentSDG}.cmd split.
#============================================================

# DELAYGEN is set by main.rs (epics_rs::base::runtime::env::set_default) to
# this IOC crate's CARGO_MANIFEST_DIR.

# ---- shared serial octet port ----
drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)

asynSetOption(serial1, 0, "baud",   "19200")
asynSetOption(serial1, 0, "bits",   "8")
asynSetOption(serial1, 0, "parity", "none")
asynSetOption(serial1, 0, "stop",   "1")

## For IP asyn support instead, comment the drvAsynSerialPortConfigure call
## above and use one of:
# drvAsynIPPortConfigure("serial1","x.x.x.x:5025",0,0,0)   # DG645
# drvAsynIPPortConfigure("serial1","x.x.x.x:7000",0,0,0)   # Colby

#------------------------------------------------------------------------------
## Device specific configuration -- PICK ONE

# ---- SRS DG645 ----
# EOS is driver-owned via the port (drvAsynDG645.cpp never appends a
# terminator itself); the device uses hardware handshaking.
asynOctetSetInputEos("serial1", 0, "\r\n")
asynOctetSetOutputEos("serial1", 0, "\n")
asynSetOption(serial1, 0, "crtscts", "Y")

# DG645Config(myport,ioport,ioaddr)
DG645Config("DG1", "serial1", -1)

dbLoadRecords("$(DELAYGEN)/db/dg645.template", "P=delaygen:,R=DG1:,PORT=DG1")

#------------------------------------------------------------------------------
iocInit()
