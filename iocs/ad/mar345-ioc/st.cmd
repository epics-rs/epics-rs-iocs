#!../../target/debug/mar345-ioc
#============================================================
# st.cmd — MAR 345 areaDetector IOC startup script
# Ported from ADmar345/iocs/mar345IOC/iocBoot/iocMAR345/st.cmd
#
# Usage:
#   cargo run -p mar345-ioc -- iocs/ad/mar345-ioc/st.cmd
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
# ad-core-rs's real crate dir: the marServer port is created, the EOS
# commands exist, and `$(ADCORE)/db` + `$(ADCORE)/ioc/commonPlugins.cmd`
# all resolve (verified live to iocInit). The EOS-setting lines below are
# still commented pending a decision on whether to set them (the commands
# are now registered, so uncommenting should work; not separately tested).
#============================================================

# Prefix for all records
epicsEnvSet("PREFIX", "13MAR345_1:")
# The port name for the detector
epicsEnvSet("PORT",   "MAR")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "3450")
# The maximum image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "3450")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# NDStdArrays waveform sizing (C st.cmd uses 12000000; 3450 * 3450 = 11902500)
epicsEnvSet("NELEMENTS", "12000000")

# $(ADMAR345) is set to this crate's root (iocs/ad/mar345-ioc) by
# ioc_support at startup; mar345.template lives in its db/ subdir.
# ADBase.template / NDFile.template resolve from $(ADCORE)/db.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADMAR345)/db:$(ADCORE)/db")

###
# Create the asyn port to talk to the MAR on TCP port 5001.
# (Requires drvAsynIPPortConfigure — see BOOT LIMITATION above.)
drvAsynIPPortConfigure("marServer", "gse-marip2.cars.aps.anl.gov:5001")

# marServer framing (reproduced from the C st.cmd): commands and replies are
# both terminated with 0x0A (LF). The C boot sets these with:
#   asynOctetSetInputEos("marServer", 0, "\n")
#   asynOctetSetOutputEos("marServer", 0, "\n")
# Omitted here because the published AdIoc does not register the
# asynOctetSetInputEos / asynOctetSetOutputEos iocsh commands (BOOT
# LIMITATION). The driver sends commands and matches replies with no embedded
# terminator, so this EOS must be set for correct operation.

mar345Config("$(PORT)", "marServer", 0, 0)
dbLoadRecords("mar345.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,MARSERVER_PORT=marServer")

# Create a standard arrays plugin (image data for clients). The published
# arrays are NDUInt16; the C example loads the record as Int16/SHORT.
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd (resolves under $(ADCORE),
# see BOOT LIMITATION).
< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                       # List all PVs
#   dbpf 13MAR345_1:cam1:Acquire 1            # Start acquisition
#   dbpf 13MAR345_1:cam1:Erase 1              # Erase the plate
#   dbpf 13MAR345_1:cam1:ChangeMode 1         # Apply ScanSize/ScanResolution
#   dbgf 13MAR345_1:cam1:ArrayCounter_RBV     # Frame counter
#   dbgf 13MAR345_1:cam1:DetectorState_RBV    # Idle/Exposing/Scanning/...
#   asynReport                                # Show port/plugin status
