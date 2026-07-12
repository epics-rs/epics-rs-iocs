#!../../../target/debug/bruker-ioc
#============================================================
# st.cmd — Bruker BIS areaDetector IOC startup script
#
# Usage:
#   cargo run -p bruker-ioc -- iocs/ad/bruker-ioc/st.cmd
#
# BIS must be running on the instrument PC with its socket server enabled.
# The frames BIS writes must be reachable from this host under the same path
# the driver hands BIS in the Scan command.
#============================================================

epicsEnvSet("PREFIX", "BIS:")
epicsEnvSet("PORT",   "APX")
epicsEnvSet("QSIZE",  "20")
# The largest frame BIS reports; the driver publishes whatever the frame file
# it reads actually holds.
epicsEnvSet("XSIZE",  "4096")
epicsEnvSet("YSIZE",  "4096")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
# The largest frame BIS reports; the driver sizes each array from the frame
# file it actually reads.
epicsEnvSet("NELEMENTS", "16777216")

epicsEnvSet("BIS_HOST",       "chemmat21")
epicsEnvSet("COMMAND_PORT",   "BIS_COMMAND")
epicsEnvSet("STATUS_PORT",    "BIS_STATUS")

# $(ADBRUKER) is set to this crate's root by ioc_support at startup; the shared
# workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADBRUKER)/../../../db:$(ADCORE)/db")

# The command socket. BIS takes a newline-terminated command and answers with a
# bracketed message that ends with ']' — the BIS documentation says the answers
# end with a newline, but they do not.
drvAsynIPPortConfigure("$(COMMAND_PORT)", "$(BIS_HOST):49153", 0, 0, 0)
asynOctetSetOutputEos("$(COMMAND_PORT)", 0, "\n")
asynOctetSetInputEos("$(COMMAND_PORT)", 0, "]")

# The socket BIS broadcasts its status on, one newline-terminated message at a
# time.
drvAsynIPPortConfigure("$(STATUS_PORT)", "$(BIS_HOST):49155", 0, 0, 0)
asynOctetSetInputEos("$(STATUS_PORT)", 0, "\n")

# ADBruker's own st.cmd also created a "file" port on 49154 that the driver
# never connected to; it is not created here.

# BISDetectorConfig(portName, BISPortName, statusPortName, maxBuffers,
#                   maxMemory, priority, stackSize)
# maxBuffers, priority and stackSize are accepted and ignored.
BISDetectorConfig("$(PORT)", "$(COMMAND_PORT)", "$(STATUS_PORT)", 0, 0, 0, 0)

dbLoadRecords("BIS.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,BIS_PORT=$(COMMAND_PORT)")

# Standard arrays plugin fed from the BIS port. The driver publishes 32-bit
# unsigned pixels, so TYPE/FTVL are Int32/LONG.
NDStdArraysConfigure("Image1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(NELEMENTS)")

< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                # List all PVs
#   dbpf BIS:cam1:FilePath /data/      # Where BIS writes the frames
#   dbpf BIS:cam1:FileName test
#   dbpf BIS:cam1:AcquireTime 10
#   dbpf BIS:cam1:Acquire 1            # Start a scan
#   dbgf BIS:cam1:BISStatus            # What BIS last broadcast
#   asynReport                         # Port status
