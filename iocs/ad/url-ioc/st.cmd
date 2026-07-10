#!../../../target/debug/url-ioc
#============================================================
# st.cmd — ADURL (image-over-HTTP/file) areaDetector IOC startup script
#
# Ported from areaDetector ADURL iocs/urlIOC/iocBoot/iocURLDriver/{st.cmd,st_base.cmd}.
#
# Usage:
#   cargo run -p url-ioc -- iocs/ad/url-ioc/st.cmd
#============================================================

# Environment (C st_base.cmd: PREFIX/PORT/QSIZE/XSIZE/YSIZE/NCHANS/CBUFFS)
epicsEnvSet("PREFIX", "13URL1:")
epicsEnvSet("PORT",   "URL1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "640")
epicsEnvSet("YSIZE",  "480")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")

# $(ADURLIOC) is set to this crate's root (iocs/ad/url-ioc) by ioc_support at
# IOC startup. The shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADURLIOC)/../../../db:$(ADCORE)/db")

# Create a URL driver.
# URLDriverConfig(portName, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers/priority/stackSize are accepted for st.cmd drop-in compatibility
# with upstream but unused by this port -- see ioc_support.rs doc comment.
URLDriverConfig("$(PORT)", 0, 0)
dbLoadRecords("url.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Load the NDStdArrays image-output plugin and the rest of the standard
# plugin chain via the framework's commonPlugins.cmd. Unlike upstream's own
# st.cmd, this does NOT also call NDStdArraysConfigure("Image1",...) itself
# first: this framework's commonPlugins.cmd already creates the Image1/
# IMAGE1 NDStdArrays plugin (record prefix "image1:"), and duplicating it
# here would collide on that same record prefix. TYPE/FTVL/NELEMENTS below
# select the upstream st.cmd's active (16-bit) NDStdArrays.template line.
epicsEnvSet("TYPE",      "Int16")
epicsEnvSet("FTVL",      "SHORT")
epicsEnvSet("NELEMENTS", "12582912")

< $(ADCORE)/ioc/commonPlugins.cmd

# Upstream also calls asynSetTraceIOMask/asynSetTraceMask here. Omitted:
# AdIoc (ad-plugins-rs) never calls asyn-rs's register_asyn_commands_on_shell,
# so no asyn* iocsh command (asynSetTraceIOMask, asynReport, ...) is
# registered on an AdIoc's shell at all -- a pre-existing framework gap
# affecting every AdIoc-based IOC in this workspace, not specific to this
# port. An unknown-command line here is a fatal script error (aborts before
# iocInit()), so it cannot be ported even as a faithful no-op.

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                # List all PVs
#   dbpf 13URL1:cam1:URL1 "file:///path/to/image.png"
#   dbpf 13URL1:cam1:URLSelect 0       # Select URL1
#   dbpf 13URL1:cam1:Acquire 1         # Start acquisition
#   dbgf 13URL1:cam1:ArrayCounter_RBV  # Frame counter
