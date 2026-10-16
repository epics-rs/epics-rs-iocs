#!../../../target/debug/pva-driver-ioc
#============================================================
# st.cmd — pvaDriver (NTNDArray-over-pvAccess) areaDetector IOC startup script
#
# Ported from areaDetector pvaDriver iocs/pvaDriverIOC/iocBoot/iocPvaDriver/{st.cmd,st_base.cmd}.
#
# Usage:
#   cargo run -p pva-driver-ioc -- iocs/ad/pva-driver-ioc/st.cmd
#============================================================

# Environment (C st_base.cmd: PREFIX/PORT/QSIZE/XSIZE/YSIZE/NCHANS/CBUFFS)
epicsEnvSet("PREFIX", "13PVA1:")
epicsEnvSet("PORT",   "PVA")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "1024")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")

# The name of the upstream NTNDArray pvAccess PV to monitor.
epicsEnvSet("PVNAME", "13SIM1:Pva1:Image")

# $(PVADRIVERIOC) is set to this crate's root (iocs/ad/pva-driver-ioc) by
# ioc_support at IOC startup. The templates live in its db/ subdir.

# Create a pvaDriver.
# pvaDriverConfig(portName, pvName, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers/priority/stackSize are accepted for st.cmd drop-in compatibility
# with upstream but unused by this port -- see ioc_support.rs doc comment.
pvaDriverConfig("$(PORT)", "$(PVNAME)", 0, 0, 0, 0)
# Template-internal `include` lines (ADBase.template, NDArrayBase.template,
# ...) resolve through the db search path; direct dbLoad paths stay explicit.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

dbLoadRecords("$(PVADRIVERIOC)/db/pva.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from the pvaDriver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)

# This creates a waveform large enough for 2000x2000x3 (e.g. RGB color) arrays.
# This waveform only allows transporting 8-bit images
#dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=12000000")
# This waveform only allows transporting 16-bit images
#dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=12000000")
# This waveform allows transporting 32-bit images
#dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=12000000")
# This waveform allows transporting 64-bit float images
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=12000000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/ioc/commonPlugins.cmd

set_requestfile_path("$(PVADRIVERIOC)/db")

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")

#
# Example interactive commands:
#   dbl                                   # List all PVs
#   dbpf 13PVA1:cam1:PvName "13SIM1:Pva1:Image"
#   dbgf 13PVA1:cam1:PvConnection_RBV     # Channel connection status
#   dbpf 13PVA1:cam1:Acquire 1            # Start acquisition
#   dbgf 13PVA1:cam1:ArrayCounter_RBV     # Frame counter
#   dbgf 13PVA1:cam1:OverrunCounter_RBV   # Server-side squashed-update count
