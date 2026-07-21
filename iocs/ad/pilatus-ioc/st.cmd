#!../../target/debug/pilatus-ioc
#============================================================
# st.cmd — Dectris Pilatus areaDetector IOC startup script
# Ported from ADPilatus/iocs/pilatusIOC/iocBoot/iocPilatus/st.cmd
#
# Usage:
#   cargo run -p pilatus-ioc -- iocs/ad/pilatus-ioc/st.cmd
#
#------------------------------------------------------------
# Boots clean on the pinned ad-plugins-rs / ad-core-rs 0.24.3.
#
# History: the published 0.22.1 baseline did NOT register the asyn
# port/EOS/trace iocsh commands (`drvAsynIPPortConfigure`,
# `asynOctetSetInputEos`, `asynOctetSetOutputEos`, `asynSetTraceMask`,
# `asynSetTraceIOMask`) and set `$(ADCORE)` to a crates.io registry path
# that did not exist, so this script could not boot unchanged. As of
# 0.24.3 `AdIoc` registers those commands and `$(ADCORE)` resolves to
# ad-core-rs's real crate dir: the camserver port is created, the EOS
# commands exist, and `$(ADCORE)/db` + `$(ADCORE)/ioc/commonPlugins.cmd`
# all resolve (verified live to iocInit). The EOS-setting lines below are
# still commented pending a decision on whether to set them (the commands
# are now registered, so uncommenting should work; not separately tested).
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
# (drvAsynIPPortConfigure is registered by AdIoc as of ad-plugins-rs 0.24.3.)
drvAsynIPPortConfigure("camserver", "gse-pilatus1:41234")

# camserver framing (reproduced from the C st.cmd): camserver terminates
# every asynchronous reply with 0x18 (CAN) and expects commands terminated
# with 0x0A (LF). The C boot sets these with:
#   asynOctetSetInputEos("camserver", 0, "\x18")
#   asynOctetSetOutputEos("camserver", 0, "\n")
# Left commented pending verification: as of ad-plugins-rs 0.24.3 AdIoc
# registers the asynOctetSetInputEos / asynOctetSetOutputEos iocsh commands,
# so these lines can be enabled. The driver's read loop consumes camserver
# replies framed on the 0x18 input terminator, so this EOS must be set for
# correct operation.

pilatusDetectorConfig("$(PORT)", "camserver", $(XSIZE), $(YSIZE), 0, 0)
dbLoadRecords("pilatus.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,CAMSERVER_PORT=camserver")

# Create a standard arrays plugin (image data for clients)
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd (resolves under $(ADCORE)).
< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                   # List all PVs
#   dbpf 13PIL1:cam1:Acquire 1            # Start acquisition
#   dbgf 13PIL1:cam1:ArrayCounter_RBV     # Frame counter
#   asynReport                            # Show port/plugin status
