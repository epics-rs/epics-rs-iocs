#!../../target/debug/mythen-ioc
#============================================================
# st.cmd — Dectris Mythen areaDetector IOC startup script
#
# Usage:
#   cargo run -p mythen-ioc -- iocs/ad/mythen-ioc/st.cmd
#
# Port of ADMythen/iocs/mythenIOC/iocBoot/iocMythen/st.cmd.
#============================================================

epicsEnvSet("PREFIX", "dp_mythen1K:")
epicsEnvSet("PORT",   "SD1")
epicsEnvSet("XSIZE",  "1280")
epicsEnvSet("YSIZE",  "1")
epicsEnvSet("NCHANS", "1280")
epicsEnvSet("MYTHENIP", "192.168.0.90:1030 UDP")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "64008")

# The socket to the detector. noProcessEos=1 and an output EOS of CR: the
# detector's replies are binary (no input EOS to look for), and every command
# is terminated with a CR.
drvAsynIPPortConfigure("IP_M1K", "$(MYTHENIP)", 0, 0, 1)
asynOctetSetOutputEos("IP_M1K", 0, "\r")

# $(ADMYTHEN) is set to this crate's root (iocs/ad/mythen-ioc) by ioc_support at
# IOC startup; $(ADCORE) is exported by ad-core-rs.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(ADMYTHEN)/../../../drivers/ad/mythen/db")

mythenConfig("$(PORT)", "IP_M1K", 0)

dbLoadRecords("mythen.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Standard-arrays plugin. The detector's readout is UInt32 (one word per
# channel), so the waveform is LONG, and it is as wide as all the modules
# together.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,TYPE=Int32,FTVL=LONG,NELEMENTS=2560")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                      # List all PVs
#   dbpf dp_mythen1K:cam1:Acquire 1          # Start acquisition
#   dbgf dp_mythen1K:cam1:ArrayCounter_RBV   # Frame counter
#   asynReport                               # Show port/plugin status
