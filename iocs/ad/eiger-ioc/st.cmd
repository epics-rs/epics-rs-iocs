#!../../target/debug/eiger-ioc
#============================================================
# st.cmd — Dectris Eiger areaDetector IOC startup script
#
# Usage:
#   cargo run -p eiger-ioc -- iocs/ad/eiger-ioc/st.cmd
#
# Port of ADEiger/iocs/eigerIOC/iocBoot/iocEiger2/st.cmd.
#============================================================

epicsEnvSet("PREFIX",  "13EIG2:")
epicsEnvSet("PORT",    "EIG")
epicsEnvSet("QSIZE",   "20")
epicsEnvSet("XSIZE",   "1030")
epicsEnvSet("YSIZE",   "1065")
epicsEnvSet("NCHANS",  "2048")
epicsEnvSet("CBUFFS",  "500")
epicsEnvSet("EIGERIP", "10.54.160.198")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "5000000")

# $(ADEIGER) is set to this crate's root (iocs/ad/eiger-ioc) by ioc_support at
# IOC startup; $(ADCORE) is exported by ad-core-rs.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(ADEIGER)/../../../drivers/ad/eiger/db")

eigerDetectorConfig("$(PORT)", "$(EIGERIP)", 0)

# The template must match the detector family: eiger1, eiger2 or pilatus4.
dbLoadRecords("eiger2.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Standard-arrays plugins.
#
# C wires these to asyn *addresses* of the detector port: address 0 is every
# frame, address 1 is threshold 1. epics-rs routes NDArrays by port name, so
# the driver exposes those as the named outputs $(PORT) and $(PORT)_TH1
# (also _TH2.._TH4, and _MON for the monitor image).
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=1096950,NDARRAY_PORT=$(PORT)")

NDStdArraysConfigure("Image2", 5, 0, "$(PORT)_TH1", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=Image2,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=1096950,NDARRAY_PORT=$(PORT)_TH1")

# The monitor interface (C asyn address 10).
NDStdArraysConfigure("ImageMon", 5, 0, "$(PORT)_MON", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=monitor1:,PORT=ImageMon,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=1096950,NDARRAY_PORT=$(PORT)_MON")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                 # List all PVs
#   dbpf 13EIG2:cam1:Acquire 1          # Start acquisition
#   dbgf 13EIG2:cam1:ArrayCounter_RBV   # Frame counter
#   asynReport                          # Show port/plugin status
