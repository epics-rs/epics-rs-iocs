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
# ioc_support at IOC startup. The shared workspace db/ lives three levels up
# from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(PVADRIVERIOC)/../../../db:$(ADCORE)/db")

# Create a pvaDriver.
# pvaDriverConfig(portName, pvName, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers/priority/stackSize are accepted for st.cmd drop-in compatibility
# with upstream but unused by this port -- see ioc_support.rs doc comment.
pvaDriverConfig("$(PORT)", "$(PVNAME)", 0, 0, 0, 0)
dbLoadRecords("pva.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Load the NDStdArrays image-output plugin and the rest of the standard
# plugin chain via the framework's commonPlugins.cmd. Unlike upstream's own
# st.cmd, this does NOT also call NDStdArraysConfigure("Image1",...) itself
# first: this framework's commonPlugins.cmd already creates the Image1/
# IMAGE1 NDStdArrays plugin (record prefix "image1:"), and duplicating it
# here would collide on that same record prefix. TYPE/FTVL/NELEMENTS below
# select the upstream st.cmd's active (64-bit float) NDStdArrays.template line.
epicsEnvSet("TYPE",      "Float64")
epicsEnvSet("FTVL",      "DOUBLE")
epicsEnvSet("NELEMENTS", "12000000")

< $(ADCORE)/ioc/commonPlugins.cmd

# Upstream also calls asynSetTraceMask/asynSetTraceInfoMask here. Omitted:
# AdIoc (ad-plugins-rs) never calls asyn-rs's register_asyn_commands_on_shell,
# so no asyn* iocsh command (asynSetTraceMask, asynReport, ...) is
# registered on an AdIoc's shell at all -- a pre-existing framework gap
# affecting every AdIoc-based IOC in this workspace, not specific to this
# port. An unknown-command line here is a fatal script error (aborts before
# iocInit()), so it cannot be ported even as a faithful no-op.

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                   # List all PVs
#   dbpf 13PVA1:cam1:PvName "13SIM1:Pva1:Image"
#   dbgf 13PVA1:cam1:PvConnection_RBV     # Channel connection status
#   dbpf 13PVA1:cam1:Acquire 1            # Start acquisition
#   dbgf 13PVA1:cam1:ArrayCounter_RBV     # Frame counter
#   dbgf 13PVA1:cam1:OverrunCounter_RBV   # Server-side squashed-update count
