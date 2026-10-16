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
# IOC startup. The templates live in its db/ subdir.

# Create a URL driver.
# URLDriverConfig(portName, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers/priority/stackSize are accepted for st.cmd drop-in compatibility
# with upstream but unused by this port -- see ioc_support.rs doc comment.
URLDriverConfig("$(PORT)", 0, 0)
# Template-internal `include` lines (ADBase.template, NDArrayBase.template,
# ...) resolve through the db search path; direct dbLoad paths stay explicit.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

dbLoadRecords("$(ADURLIOC)/db/url.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)

# This creates a waveform large enough for 2048x2048x3 (e.g. RGB color) arrays.
# This waveform only allows transporting 8-bit images
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=12582912")
# This waveform allows transporting 16-bit images
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=12582912")

# Load all other plugins using commonPlugins.cmd
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
