#!../../target/debug/pilatus-ioc
#============================================================
# st.cmd — Dectris Pilatus areaDetector IOC startup script
# Ported from ADPilatus/iocs/pilatusIOC/iocBoot/iocPilatus/st.cmd
#
# Usage:
#   cargo run -p pilatus-ioc -- iocs/ad/pilatus-ioc/st.cmd
#
#------------------------------------------------------------
# BOOT LIMITATION (published epics-rs 0.22.1 baseline)
#
# The published ad-plugins-rs 0.22.1 `AdIoc` does NOT register the asyn
# port/EOS/trace iocsh commands (`drvAsynIPPortConfigure`,
# `asynOctetSetInputEos`, `asynOctetSetOutputEos`, `asynSetTraceMask`,
# `asynSetTraceIOMask`), and it sets `$(ADCORE)` to a crates.io registry
# path that does not exist on disk. As a result this script does NOT boot
# unchanged on the published baseline: the camserver port cannot be
# created, its EOS cannot be set, and the `$(ADCORE)/db` /
# `$(ADCORE)/iocBoot` includes below cannot be resolved. The lines are
# kept (some as comments) so they run verbatim once the framework
# provides those commands. See the crate UNFIXED notes.
#============================================================

# Prefix for all records
epicsEnvSet("PREFIX", "13PIL1:")
# The port name for the detector
epicsEnvSet("PORT",   "PIL")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "487")
# The maximum image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "195")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# NDStdArrays waveform sizing: XSIZE * YSIZE = 487 * 195 = 94965
epicsEnvSet("NELEMENTS", "94965")

# $(ADPILATUS) is set to this crate's root (iocs/ad/pilatus-ioc) by
# ioc_support at startup; pilatus.template lives in its db/ subdir.
# ADBase.template / NDFile.template resolve from $(ADCORE)/db.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADPILATUS)/db:$(ADCORE)/db")

###
# Create the asyn port to talk to the Pilatus camserver on TCP port 41234.
# (Requires drvAsynIPPortConfigure — see BOOT LIMITATION above.)
drvAsynIPPortConfigure("camserver", "gse-pilatus1:41234")

# camserver framing (reproduced from the C st.cmd): camserver terminates
# every asynchronous reply with 0x18 (CAN) and expects commands terminated
# with 0x0A (LF). The C boot sets these with:
#   asynOctetSetInputEos("camserver", 0, "\x18")
#   asynOctetSetOutputEos("camserver", 0, "\n")
# Omitted here because the published AdIoc does not register the
# asynOctetSetInputEos / asynOctetSetOutputEos iocsh commands (BOOT
# LIMITATION). The driver's read loop consumes camserver replies framed on
# the 0x18 input terminator, so this EOS must be set for correct operation.

pilatusDetectorConfig("$(PORT)", "camserver", $(XSIZE), $(YSIZE), 0, 0)
dbLoadRecords("pilatus.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,CAMSERVER_PORT=camserver")

# Create a standard arrays plugin (image data for clients)
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd (resolves under $(ADCORE),
# see BOOT LIMITATION).
< $(ADCORE)/iocBoot/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                   # List all PVs
#   dbpf 13PIL1:cam1:Acquire 1            # Start acquisition
#   dbgf 13PIL1:cam1:ArrayCounter_RBV     # Frame counter
#   asynReport                            # Show port/plugin status
