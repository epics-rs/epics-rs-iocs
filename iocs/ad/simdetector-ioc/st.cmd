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
epicsEnvSet("MAX_THREADS", "8")

# $(ADSIMDETECTOR) is set to this crate's root by ioc_support at IOC startup.
# The templates live in its db/ subdir.

# simDetectorConfig(portName, maxSizeX, maxSizeY, dataType, maxBuffers, maxMemory)
#   dataType 1 = NDUInt8; maxBuffers 0 and maxMemory 0 mean unlimited.
simDetectorConfig("$(PORT)", $(XSIZE), $(YSIZE), 1, 0, 0)

# Template-internal `include` lines (ADBase.template, NDArrayBase.template,
# ...) resolve through the db search path; direct dbLoad paths stay explicit.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

dbLoadRecords("$(ADSIMDETECTOR)/db/simDetector.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from first simDetector driver.
NDStdArraysConfigure("Image1", 20, 0, "$(PORT)", 0, 0, 0, 0, 0, 5)

# This creates a waveform large enough for 2000x2000x3 (e.g. RGB color) arrays.
# This waveform only allows transporting 8-bit images
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=12000000")

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
