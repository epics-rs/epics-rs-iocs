#!../../../target/debug/pixirad-ioc
#============================================================
# st.cmd — Pixirad CdTe detector areaDetector IOC startup script
#
# Usage:
#   cargo run -p pixirad-ioc -- iocs/ad/pixirad-ioc/st.cmd
#
# The box answers ASCII commands on TCP 2222 and broadcasts UDP: the
# image data on 2223 (Pixirad-1) or 9999 (Pixirad-2), the environment
# on 2224.
#============================================================

epicsEnvSet("PREFIX", "13PR1:")
epicsEnvSet("PORT",   "PIXI")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")

epicsEnvSet("COMMAND_PORT", "PIXI_CMD")
epicsEnvSet("COMMAND_ADDR", "192.168.0.1:2222 HTTP")
# Pixirad-1 broadcasts its data on 2223, Pixirad-2 on 9999.
epicsEnvSet("DATA_PORT",   "9999")
epicsEnvSet("STATUS_PORT", "2224")
epicsEnvSet("DATA_PORT_BUFFERS", "1500")

# The sensor: X is 476 on a Pixie-II chip and 402 on a Pixie-III;
# Y is 512 per module (Pixirad-1: 512, Pixirad-2: 1024, Pixirad-8: 4096).
epicsEnvSet("XSIZE", "402")
epicsEnvSet("YSIZE", "1024")
# 402 * 1024.
epicsEnvSet("NELEMENTS", "411648")

# $(ADPIXIRAD) is set to this crate's root by ioc_support at startup; the
# shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADPIXIRAD)/../../../db:$(ADCORE)/db")

# The command socket. The box terminates what it sends; only an output EOS
# is needed.
drvAsynIPPortConfigure("$(COMMAND_PORT)", "$(COMMAND_ADDR)", 0, 0, 0)
asynOctetSetOutputEos("$(COMMAND_PORT)", 0, "\n")

# pixiradConfig(portName, commandPort, dataPortNumber, statusPortNumber,
#               maxDataPortBuffers, maxSizeX, maxSizeY,
#               [maxBuffers] [maxMemory] [priority] [stackSize])
# maxBuffers, priority and stackSize are accepted and ignored.
pixiradConfig("$(PORT)", "$(COMMAND_PORT)", $(DATA_PORT), $(STATUS_PORT), $(DATA_PORT_BUFFERS), $(XSIZE), $(YSIZE), 0, 0, 0, 0)

# A Pixie-III detector needs this. The values are specific to the detector and
# ship with it; these are the ones from the upstream startup script.
pixiradAutoCal("$(PORT)", 0, 0, 7, 7, 3, 7, 1850)

dbLoadRecords("pixirad.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Standard arrays plugin. The detector always sends 16-bit counts; a
# multi-colour frame type makes the array 3-dimensional, so NELEMENTS has to
# be multiplied by the number of colours to see one.
NDStdArraysConfigure("Image1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                     # List all PVs
#   dbpf 13PR1:cam1:AutoCalibrate 1         # Calibrate the chip
#   dbpf 13PR1:cam1:Acquire 1               # Start acquisition
#   dbgf 13PR1:cam1:CoolingStatus_RBV       # Dew point / temperature alarms
#   dbgf 13PR1:cam1:UDPSpeed_RBV            # Data rate off the detector
#   asynReport                              # Port status
