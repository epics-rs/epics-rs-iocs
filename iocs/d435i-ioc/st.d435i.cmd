#!../../target/debug/d435i-ioc
#============================================================
# st.d435i.cmd — RealSense D435i areaDetector IOC startup script
#
# The instance-specific variant of st.cmd: it pins SERIAL to the D435i on
# this host, so it stays bound to that camera no matter what else is
# plugged in or in what order librealsense enumerates.
#
# Usage:
#   cargo run -p d435i-ioc -- iocs/d435i-ioc/st.d435i.cmd
#
# Runs alongside st.d405.cmd — the two use different PREFIX/PORT names and
# different PVA server ports.
#============================================================

# Environment
epicsEnvSet("PREFIX",     "RS435:")
epicsEnvSet("CAM",        "cam1:")
epicsEnvSet("PORT",       "RS435")
# The D435i on this host, as `rs-enumerate-devices -s` reports it. Note this
# is NOT the iSerial in the USB descriptor -- `lsusb -v` shows 253443063633
# for this camera, which librealsense will not match.
# Left empty this would take whatever librealsense enumerates first, which
# with the D405 also plugged in is not a stable choice.
epicsEnvSet("SERIAL",     "348122070162")
epicsEnvSet("DEPTH_PORT", "$(PORT)_DEPTH")
epicsEnvSet("PC_PORT",    "$(PORT)_PC")
epicsEnvSet("QSIZE",      "20")
# Largest frame the camera can deliver. This is the driver's MaxSizeX/MaxSizeY
# and the size the plugins' profile arrays are allocated for -- not the stream
# mode in use, which RSStreamMode selects at runtime (default 640x480).
epicsEnvSet("XSIZE",      "1280")
epicsEnvSet("YSIZE",      "720")
epicsEnvSet("NCHANS",     "2048")
# Bins in the NDStats histogram array.
epicsEnvSet("HIST_SIZE",  "256")
epicsEnvSet("CBUFFS",     "500")

# Two IOCs on one host cannot both take the default PVA TCP port 5075. This
# one keeps it; st.d405.cmd moves. CA needs no such split -- its TCP port is
# ephemeral and the 5064 name-search socket is shared.
epicsEnvSet("EPICS_PVAS_SERVER_PORT", "5075")

# NELEMENTS sizing for the three NDStdArrays output waveforms.
# Sized for the D435i's maximum 1280x720 stream so smaller modes also fit:
#   COLOR =  1280 * 720 * 3   (RGB8 interleaved)
#   DEPTH =  1280 * 720       (Z16 mono)
#   PC    =  1280 * 720 * 3   (Float32 XYZ vertex)
epicsEnvSet("NELEMENTS_COLOR", "2764800")
epicsEnvSet("NELEMENTS_DEPTH", "921600")
epicsEnvSet("NELEMENTS_PC",    "2764800")

# $(ADD435I) is set to this crate's root (iocs/d435i-ioc) by ioc_support at
# IOC startup. The shared workspace db/ lives two levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADD435I)/../../db:$(ADCORE)/db")

d435iConfig("$(PORT)", "$(SERIAL)", $(XSIZE), $(YSIZE), 100000000)

# Load per-port record databases
dbLoadRecords("d435i_color.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("d435i_depth.template", "P=$(PREFIX),R=depth1:,PORT=$(DEPTH_PORT),ADDR=0,TIMEOUT=1")

# Load plugin chains per port (plugin scripts live next to this st.cmd)
< $(ADD435I)/d435iColorPlugins.cmd
< $(ADD435I)/d435iDepthPlugins.cmd
< $(ADD435I)/d435iPCPlugins.cmd

# Autosave: request file lives next to this script, saved state under
# ./autosave/<prefix>. set_pass1_restoreFile is a no-op until the first save
# has run, so a fresh checkout starts on the driver defaults.
# The save file is per-prefix because the D405 instance shares this request
# file -- one shared .sav would have the two cameras overwrite each other.
set_requestfile_path("$(ADD435I)")
set_savefile_path("$(ADD435I)/autosave/RS435")
set_pass1_restoreFile("auto_settings.sav", "P=$(PREFIX)")

# Spelled out so create_monitor_set below runs after record init. epics-rs
# calls iocInit itself once the script finishes and the command is idempotent,
# so this is not a second initialisation.
iocInit()

create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")

# NOTE on restoring RSStreamMode.
#
# pass1 restore puts the saved VAL into the record but does not process it, so
# a value that only reaches the hardware through device support does not get
# there: RSStreamMode reads back the restored mode while RSStreamMode_RBV --
# served from the driver's own parameter -- still shows the driver default, and
# the camera streams at that default. The plugin bo records restore fine; it is
# specifically this path that needs a process.
#
# It cannot be fixed from here: the framework runs pass1 restore AFTER this
# script finishes, so any dbpf written here executes before the restore and
# processes the default. run-d435i-ioc.sh forces the process once the IOC is up.

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                  # List all PVs
#   dbpf RS435:cam1:Acquire 1            # Start acquisition
#   dbgf RS435:cam1:ArrayCounter_RBV     # Color frame counter
#   dbgf RS435:depth1:ArrayCounter_RBV   # Depth frame counter
#   dbgf RS435:cam1:RSHasIMU_RBV         # 1 on the D435i -- RSAccel*/RSGyro* are live
#   asynReport                           # Show port/plugin status
