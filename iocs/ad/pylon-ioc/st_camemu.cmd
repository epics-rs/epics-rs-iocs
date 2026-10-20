#!../../../target/debug/ad-pylon-ioc
#============================================================
# st_camemu.cmd — ADPylon IOC on pylon's camera emulation transport layer
# (no hardware).
#
# Usage:
#   cargo run -p ad-pylon-ioc -- iocs/ad/pylon-ioc/st_camemu.cmd
#============================================================

# Create 1 Pylon emulation camera
epicsEnvSet("PYLON_CAMEMU", "1")

epicsEnvSet("PREFIX", "PYLON1:")
epicsEnvSet("PORT",   "PYLON1")
epicsEnvSet("XSIZE",  "4096")
epicsEnvSet("YSIZE",  "4096")
# Define NELEMENTS to be enough for a 4096x4096x3 (color) image
epicsEnvSet("NELEMENTS", "50331648")

# The CAMERA_ID can be either of the following:
#  Camera serial number
#  Camera index number, starting from 0
epicsEnvSet("CAMERA_ID", "0815-0000")

# The emulator exposes the generic GenICam feature set only, so no
# per-model template is loaded here.
< $(ADPYLON)/st_base.cmd
