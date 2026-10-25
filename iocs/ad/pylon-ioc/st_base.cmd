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

# ADPylonConfig(portName, cameraId, maxMemory, priority, stackSize)
ADPylonConfig("$(PORT)", "$(CAMERA_ID)", 0, 0, 0)

# Main database
dbLoadRecords("$(ADPYLON)/db/pylon.template", "P=$(PREFIX),R=$(CAM),PORT=$(PORT)")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
# Use this line for 8-bit or 16-bit data
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/ioc/commonPlugins.cmd

# commonPlugins.cmd sets up autosave with ./autosave as the save path; these
# keep the request and save files under $(ADPYLON) whatever the start dir.
set_requestfile_path("$(ADPYLON)")
set_requestfile_path("$(ADPYLON)/db")
set_savefile_path("$(ADPYLON)/autosave")
