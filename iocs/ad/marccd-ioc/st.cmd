#!../../target/debug/marccd-ioc
#============================================================
# st.cmd — MAR marCCD areaDetector IOC startup script
# Ported from ADmarCCD/iocs/marCCDIOC/iocBoot/iocMARCCD/st.cmd
#
# Usage:
#   cargo run -p marccd-ioc -- iocs/ad/marccd-ioc/st.cmd
#
#------------------------------------------------------------
# BOOT LIMITATION (published epics-rs 0.22.1 baseline)
#
# The published ad-plugins-rs 0.22.1 `AdIoc` does NOT register the asyn
# port/EOS/trace iocsh commands (`drvAsynIPPortConfigure`,
# `asynOctetSetInputEos`, `asynOctetSetOutputEos`, `asynSetTraceMask`,
# `asynSetTraceIOMask`), and it sets `$(ADCORE)` to a crates.io registry
# path that does not exist on disk. As a result this script does NOT boot
# unchanged on the published baseline: the marServer port cannot be
# created, its EOS cannot be set, and the `$(ADCORE)/db` /
# `$(ADCORE)/iocBoot` includes below cannot be resolved. The lines are
# kept (some as comments) so they run verbatim once the framework
# provides those commands. See the crate UNFIXED notes.
#============================================================

# Prefix for all records
epicsEnvSet("PREFIX", "13MARCCD1:")
# The port name for the detector
epicsEnvSet("PORT",   "MAR")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "2048")
# The maximum image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "2048")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# NDStdArrays waveform sizing: a little bigger than 2048 * 2048
epicsEnvSet("NELEMENTS", "4200000")

# $(ADMARCCD) is set to this crate's root (iocs/ad/marccd-ioc) by
# ioc_support at startup; marCCD.template lives in its db/ subdir.
# ADBase.template / NDFile.template resolve from $(ADCORE)/db.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADMARCCD)/db:$(ADCORE)/db")

###
# Create the asyn port to talk to the MAR on TCP port 2222.
# (Requires drvAsynIPPortConfigure — see BOOT LIMITATION above.)
drvAsynIPPortConfigure("marServer", "gse-marccd1.cars.aps.anl.gov:2222")

# marServer framing (reproduced from the C st.cmd): commands and replies are
# both terminated with 0x0A (LF). The C boot sets these with:
#   asynOctetSetInputEos("marServer", 0, "\n")
#   asynOctetSetOutputEos("marServer", 0, "\n")
# Omitted here because the published AdIoc does not register the
# asynOctetSetInputEos / asynOctetSetOutputEos iocsh commands (BOOT
# LIMITATION). The driver sends commands and parses replies with no embedded
# terminator, so this EOS must be set for correct operation.

marCCDConfig("$(PORT)", "marServer", 0, 0)
dbLoadRecords("marCCD.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,MARSERVER_PORT=marServer")

# Create a standard arrays plugin (image data for clients). The published
# arrays are NDUInt16; the C example loads the record as Int16/SHORT.
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd (resolves under $(ADCORE),
# see BOOT LIMITATION).
< $(ADCORE)/iocBoot/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                   # List all PVs
#   dbpf 13MARCCD1:cam1:Acquire 1         # Start acquisition
#   dbgf 13MARCCD1:cam1:ArrayCounter_RBV  # Frame counter
#   dbgf 13MARCCD1:cam1:MarState_RBV      # marccd_server state word
#   asynReport                            # Show port/plugin status
