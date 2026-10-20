#============================================================
# st_base.cmd — common part of every ADPylon IOC
#
# Expects PREFIX, PORT, CAMERA_ID, XSIZE, YSIZE and NELEMENTS to be set by
# the including script. The camera model's own GenICam database is loaded by
# that script too, after this one: its filename is per-model, and this
# workspace's db-path convention (tools/ioc-conventions) wants every
# dbLoadRecords path spelled $(MACRO)/db/<file> in the file that issues it.
#============================================================

epicsEnvSet("CAM",    "cam1:")
epicsEnvSet("QSIZE",  "200")
epicsEnvSet("NCHANS", "128")
epicsEnvSet("CBUFFS", "50")

# Template-internal `include` lines resolve through the db search path:
# pylon.template includes ADGenICam.template, which includes ad-core-rs's
# ADBase.template.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(ADPYLON)/db")

# Autosave configuration
set_requestfile_path("$(ADPYLON)/db")
set_requestfile_path("$(ADCORE)/db")
set_savefile_path("$(ADPYLON)/autosave")
save_restoreSet_status_prefix("$(PREFIX)")
set_pass0_restoreFile("pylon_settings.req", "P=$(PREFIX),R=$(CAM)")
set_pass1_restoreFile("pylon_settings.req", "P=$(PREFIX),R=$(CAM)")

# ADPylonConfig(portName, cameraId, maxMemory, priority, stackSize)
ADPylonConfig("$(PORT)", "$(CAMERA_ID)", 0, 0, 0)

# Main database
dbLoadRecords("$(ADPYLON)/db/pylon.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT)")

# Load all common plugins (includes image1 StdArrays)
< $(ADCORE)/ioc/commonPlugins.cmd

create_monitor_set("pylon_settings.req", 30, "P=$(PREFIX),R=$(CAM)")
