#!../../../target/debug/merlin-ioc
#============================================================
# st.cmd — Merlin (Medipix) areaDetector IOC startup script
#
# Usage:
#   cargo run -p merlin-ioc -- iocs/ad/merlin-ioc/st.cmd
#============================================================

epicsEnvSet("PREFIX", "13ML1:")
epicsEnvSet("PORT",   "ML")
epicsEnvSet("QSIZE",  "20")
# Merlin Quad is 512x512; a single-chip Merlin is 256x256.
epicsEnvSet("XSIZE",  "512")
epicsEnvSet("YSIZE",  "512")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("NELEMENTS", "262144")

# The two Labview sockets: commands and data.
epicsEnvSet("COMMAND_PORT",   "$(PORT)cmd")
epicsEnvSet("DATA_PORT",      "$(PORT)data")
epicsEnvSet("MERLIN_IP",      "164.54.160.214")
epicsEnvSet("COMMAND_IPPORT", "6341")
epicsEnvSet("DATA_IPPORT",    "6342")
# 0=Merlin, 1=MedipixXBPM, 2=UomXBPM, 3=MerlinQuad
epicsEnvSet("MODEL", "3")

# $(ADMERLIN) is set to this crate's root by ioc_support at startup; the
# shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADMERLIN)/../../../db:$(ADCORE)/db")

# The command channel is line-terminated; the data channel is not — MPX data
# frames are length-delimited and carry binary pixels, so an input EOS would
# cut them at the first 0x0A byte.
drvAsynIPPortConfigure("$(COMMAND_PORT)", "$(MERLIN_IP):$(COMMAND_IPPORT)", 0, 0, 0)
asynOctetSetOutputEos("$(COMMAND_PORT)", 0, "\n")
asynOctetSetInputEos("$(COMMAND_PORT)", 0, "\n")

drvAsynIPPortConfigure("$(DATA_PORT)", "$(MERLIN_IP):$(DATA_IPPORT)", 0, 0, 0)

# merlinDetectorConfig(portName, cmdPort, dataPort, maxSizeX, maxSizeY,
#                      detectorType, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers, priority and stackSize are accepted and ignored.
merlinDetectorConfig("$(PORT)", "$(COMMAND_PORT)", "$(DATA_PORT)", $(XSIZE), $(YSIZE), $(MODEL), 0, 0, 0, 0)

dbLoadRecords("merlin.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,XSIZE=$(XSIZE),YSIZE=$(YSIZE)")

# Standard arrays plugin fed from the Merlin port.
NDStdArraysConfigure("Image1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(NELEMENTS)")

< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                 # List all PVs
#   dbpf 13ML1:cam1:Acquire 1           # Start acquisition
#   dbgf 13ML1:cam1:ArrayCounter_RBV    # Frame counter
#   asynReport                          # Port status
