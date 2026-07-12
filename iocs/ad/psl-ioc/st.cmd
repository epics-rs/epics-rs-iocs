#!../../../target/debug/psl-ioc
#============================================================
# st.cmd — Photonic Sciences (PSL) areaDetector IOC startup script
#
# Usage:
#   cargo run -p psl-ioc -- iocs/ad/psl-ioc/st.cmd
#
# PSLViewer must be running on the camera PC with its socket server
# started (PSL Software menu → Connexion).
#============================================================

epicsEnvSet("PREFIX", "13PSL1:")
epicsEnvSet("PORT",   "PSL")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "4007")
epicsEnvSet("YSIZE",  "2670")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
# A little bigger than 4007 * 2670.
epicsEnvSet("NELEMENTS", "11000000")

epicsEnvSet("PSL_SERVER",      "PSLServer")
epicsEnvSet("PSL_SERVER_ADDR", "localhost:50000")

# $(ADPSL) is set to this crate's root by ioc_support at startup; the shared
# workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADPSL)/../../../db:$(ADCORE)/db")

# The PSLViewer socket. The server answers a command with one message and no
# terminator, and the image payload is binary, so only an output EOS is set.
drvAsynIPPortConfigure("$(PSL_SERVER)", "$(PSL_SERVER_ADDR)", 0, 0, 0)
asynOctetSetOutputEos("$(PSL_SERVER)", 0, "\n")

# PSLConfig(portName, serverPort, maxBuffers, maxMemory, priority, stackSize)
# maxBuffers, priority and stackSize are accepted and ignored.
PSLConfig("$(PORT)", "$(PSL_SERVER)", 0, 0, 0, 0)

dbLoadRecords("PSL.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,PSL_SERVER_PORT=$(PSL_SERVER)")

# Standard arrays plugin fed from the PSL port. TYPE/FTVL follow the server's
# default 16-bit mode; a camera running in 8-, 32-bit or RGB mode needs the
# matching TYPE/FTVL here.
NDStdArraysConfigure("Image1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                     # List all PVs
#   dbpf 13PSL1:cam1:Acquire 1              # Start acquisition
#   dbgf 13PSL1:cam1:ArrayCounter_RBV       # Frame counter
#   dbpf 13PSL1:cam1:CameraName 1           # Open another camera
#   asynReport                              # Port status
