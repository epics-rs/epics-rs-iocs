#!../../../target/debug/ad-simdetector-ioc
#============================================================
# st.cmd — ADSimDetector areaDetector IOC startup script
#
# Mirrors iocs/simDetectorIOC/iocBoot/iocSimDetector/st_base.cmd
# from upstream ADSimDetector.
#
# Usage:
#   cargo run -p ad-simdetector-ioc -- iocs/ad/simdetector-ioc/st.cmd
#============================================================

# Environment
epicsEnvSet("PREFIX", "13SIM1:")
epicsEnvSet("CAM",    "cam1:")
epicsEnvSet("PORT",   "SIM1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "1024")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")

# NELEMENTS for the NDStdArrays output waveform: XSIZE * YSIZE * 3, so an RGB
# frame at the full detector size fits.
epicsEnvSet("NELEMENTS", "3145728")

# $(ADSIMDETECTOR) is set to this crate's root by ioc_support at IOC startup.
# The shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADSIMDETECTOR)/../../../db:$(ADCORE)/db")

# simDetectorConfig(portName, maxSizeX, maxSizeY, dataType, maxBuffers, maxMemory)
#   dataType 1 = NDUInt8; maxBuffers 0 and maxMemory 0 mean unlimited.
simDetectorConfig("$(PORT)", $(XSIZE), $(YSIZE), 1, 0, 0)

dbLoadRecords("simDetector.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Plugin chain
< $(ADSIMDETECTOR)/simDetectorPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                  # List all PVs
#   dbpf 13SIM1:cam1:SimMode 1           # Peaks pattern
#   dbpf 13SIM1:cam1:Acquire 1           # Start acquisition
#   dbgf 13SIM1:cam1:ArrayCounter_RBV    # Frame counter
#   asynReport                           # Show port/plugin status
