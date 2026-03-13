#!../../target/debug/d435i_ioc
#============================================================
# st.cmd — D435i RealSense areaDetector IOC startup script
#
# Matches C++ areaDetector IOC startup structure with
# commonPlugins.cmd include for plugin configuration.
#
# Usage:
#   cargo run --bin d435i_ioc --features ioc -- ioc/st.cmd
#============================================================

# Environment
epicsEnvSet("PREFIX", "RS1:")
epicsEnvSet("CAM",    "cam1:")
epicsEnvSet("PORT",   "RS1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "1920")
epicsEnvSet("YSIZE",  "1080")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# Depth port name (auto-created by d435iConfig)
epicsEnvSet("DEPTH_PORT", "$(PORT)_DEPTH")

# Create D435i detector
# d435iConfig(portName, serial, maxSizeX, maxSizeY, maxMemory)
# Color port = $(PORT), Depth port = $(DEPTH_PORT) (auto-created)
d435iConfig("$(PORT)", "", $(XSIZE), $(YSIZE), 100000000)

# Load Color port records
dbLoadRecords("db/d435i_color.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Load Depth port records
dbLoadRecords("db/d435i_depth.template", "P=$(PREFIX),R=depth1:,PORT=$(DEPTH_PORT),ADDR=0,TIMEOUT=1")

# ===== StdArrays plugins for image display =====

# Color image (RGB8: XSIZE * YSIZE * 3)
NDStdArraysConfigure("IMAGE1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=IMAGE1,DTYP=asynIMAGE1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=6220800")

# Depth image (Z16: XSIZE * YSIZE)
NDStdArraysConfigure("IMAGE2", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=IMAGE2,DTYP=asynIMAGE2,NDARRAY_PORT=$(DEPTH_PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=2073600")

# ===== Load all common plugins (wired to color port) =====
< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# After init, the interactive iocsh shell starts.
#
# Example interactive commands:
#   dbl                                # List all PVs
#   dbpf RS1:cam1:Acquire 1            # Start acquisition
#   dbgf RS1:cam1:ArrayCounter_RBV     # Read frame counter
#   asynReport                         # Show port/plugin status
