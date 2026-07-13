#!../../../target/debug/photonii-ioc
#============================================================
# st.cmd — Bruker PhotonII areaDetector IOC startup script
#
# Usage:
#   cargo run -p photonii-ioc -- iocs/ad/photonii-ioc/st.cmd
#============================================================

epicsEnvSet("PREFIX", "13PII_1:")
epicsEnvSet("PORT",   "PII")
epicsEnvSet("QSIZE",  "20")
# The detector is 768 x 1024, fixed.
epicsEnvSet("XSIZE",  "768")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("NELEMENTS", "786432")

# The asyn port that reaches p2util, which runs under procServ.
epicsEnvSet("PII_SERVER",         "PIIServer")
epicsEnvSet("PII_SERVER_ADDR",    "localhost:20000")
epicsEnvSet("PII_STARTUP_SCRIPT", "/home/bruker/p2util/scripts/prep_collection.cmd")

# $(ADPHOTONII) is set to this crate's root by ioc_support at startup; the
# shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADPHOTONII)/../../../db:$(ADCORE)/db")

drvAsynIPPortConfigure("$(PII_SERVER)", "$(PII_SERVER_ADDR)", 0, 0, 0)
asynOctetSetInputEos("$(PII_SERVER)", 0, "\r\n")
asynOctetSetOutputEos("$(PII_SERVER)", 0, "\n")

# PhotonIIConfig(portName, commandPort, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers, priority and stackSize are accepted and ignored.
PhotonIIConfig("$(PORT)", "$(PII_SERVER)", 0, 0, 0, 0)

# Load and run the detector preparation command file inside p2util.
p2util("$(PORT)", "load --commands --filename $(PII_STARTUP_SCRIPT)")

dbLoadRecords("photonII.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,PII_SERVER_PORT=$(PII_SERVER)")

# Standard arrays plugin fed from the PhotonII port.
NDStdArraysConfigure("Image1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(NELEMENTS)")

< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                    # List all PVs
#   dbpf 13PII_1:cam1:Acquire 1            # Start acquisition
#   dbgf 13PII_1:cam1:ArrayCounter_RBV     # Frame counter
#   asynReport                             # Port status
