#!../../target/debug/d435i_ioc
#============================================================
# st.cmd — D435i RealSense areaDetector IOC startup script
#
# Usage:
#   cargo run --bin d435i_ioc --features ioc -- ioc/st.cmd
#============================================================

# Environment
epicsEnvSet("PREFIX",     "RS1:")
epicsEnvSet("CAM",        "cam1:")
epicsEnvSet("PORT",       "RS1")
epicsEnvSet("DEPTH_PORT", "$(PORT)_DEPTH")
epicsEnvSet("PC_PORT",    "$(PORT)_PC")
epicsEnvSet("QSIZE",      "20")
epicsEnvSet("XSIZE",      "640")
epicsEnvSet("YSIZE",      "480")
epicsEnvSet("NCHANS",     "2048")
epicsEnvSet("CBUFFS",     "500")

# NELEMENTS sizing for the three NDStdArrays output waveforms.
# Sized for the D435i's maximum 1280x720 stream so smaller modes also fit:
#   COLOR =  1280 * 720 * 3   (RGB8 interleaved)
#   DEPTH =  1280 * 720       (Z16 mono)
#   PC    =  1280 * 720 * 3   (Float32 XYZ vertex)
epicsEnvSet("NELEMENTS_COLOR", "2764800")
epicsEnvSet("NELEMENTS_DEPTH", "921600")
epicsEnvSet("NELEMENTS_PC",    "2764800")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADD435I)/db:$(ADCORE)/db")

# $(ADD435I) is set to this crate's root by the d435i module at IOC startup,
# so db/ and ioc/ paths below resolve regardless of the shell's cwd.
d435iConfig("$(PORT)", "", $(XSIZE), $(YSIZE), 100000000)

# Load per-port record databases
dbLoadRecords("d435i_color.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("d435i_depth.template", "P=$(PREFIX),R=depth1:,PORT=$(DEPTH_PORT),ADDR=0,TIMEOUT=1")

# Load plugin chains per port
< $(ADD435I)/ioc/d435iColorPlugins.cmd
< $(ADD435I)/ioc/d435iDepthPlugins.cmd
< $(ADD435I)/ioc/d435iPCPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                # List all PVs
#   dbpf RS1:cam1:Acquire 1            # Start acquisition
#   dbgf RS1:cam1:ArrayCounter_RBV     # Color frame counter
#   dbgf RS1:depth1:ArrayCounter_RBV   # Depth frame counter
#   asynReport                         # Show port/plugin status
