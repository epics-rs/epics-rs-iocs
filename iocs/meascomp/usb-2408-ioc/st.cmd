# USB-2408-2AO IOC startup script, ported from iocBoot/iocUSB2408/st.cmd
#
# MEASCOMP is set by main.rs to iocs/meascomp; its db/ holds measComp's
# templates, substitutions and _settings.req files verbatim.
#
# Not ported from save_restore.cmd: the save_restoreSet_* settings, which
# epics-rs autosave does not provide.

epicsEnvSet("PREFIX",        "USB2408:")
epicsEnvSet("PORT",          "USB2408_1")
epicsEnvSet("WDIG_POINTS",   "2048")
epicsEnvSet("WGEN_POINTS",   "2048")
epicsEnvSet("UNIQUE_ID",     "01DA523D")

## Configure port driver
# MultiFunctionConfig((portName,        # The name to give to this asyn port driver
#                      uniqueID,        # For USB the serial number.  For Ethernet the MAC address or IP address.
#                      maxInputPoints,  # Maximum number of input points for waveform digitizer
#                      maxOutputPoints) # Maximum number of output points for waveform generator
MultiFunctionConfig("$(PORT)", "$(UNIQUE_ID)", $(WDIG_POINTS), $(WGEN_POINTS))

dbLoadTemplate("$(MEASCOMP)/db/USB2408.substitutions", "P=$(PREFIX),PORT=$(PORT),WDIG_POINTS=$(WDIG_POINTS),WGEN_POINTS=$(WGEN_POINTS)")

set_savefile_path("$(MEASCOMP)/usb-2408-ioc", "autosave")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
set_requestfile_path("$(MEASCOMP)/usb-2408-ioc", "")
set_requestfile_path("$(MEASCOMP)", "db")

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX)")

# Need to force the time arrays to process because the records are scan=I/O Intr
# but asynPortDriver does not do array callbacks before iocInit.
dbpf $(PREFIX)WaveDigDwell.PROC 1
dbpf $(PREFIX)WaveGenUserDwell.PROC 1
