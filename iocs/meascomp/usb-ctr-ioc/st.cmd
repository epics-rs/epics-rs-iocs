# USB-CTR08 IOC startup script, ported from iocBoot/iocUSBCTR/st.cmd
#
# main.rs sets MEASCOMP to iocs/meascomp, whose db/ holds measComp's
# templates, substitutions and _settings.req files verbatim, SCALER to
# scaler-rs and MCA to drivers/mca.
#
# Not ported from save_restore.cmd: the save_restoreSet_* settings, which
# epics-rs autosave does not provide.

epicsEnvSet("PREFIX",             "USBCTR:")
epicsEnvSet("PORT",               "USBCTR_1")
epicsEnvSet("UNIQUE_ID",          "01DAB0FB")
epicsEnvSet("MCS_PREFIX",         "$(PREFIX)MCS:")
epicsEnvSet("SCALER_PREFIX",      "$(PREFIX)")
epicsEnvSet("SCALER_NAME",        "scaler1")
epicsEnvSet("RNAME",              "mca")
epicsEnvSet("MAX_COUNTERS",       "9")
epicsEnvSet("MAX_POINTS",         "2048")
# For MCA records FIELD=READ, for waveform records FIELD=PROC
epicsEnvSet("FIELD",              "PROC")

## Set the minimum sleep time to 1 ms
asynSetMinTimerPeriod(0.001)

## Configure port driver
# USBCTRConfig(portName,       # The name to give to this asyn port driver
#              uniqueID,       # Device serial number.
#              maxTimePoints)  # Maximum number of time points for MCS
USBCTRConfig("$(PORT)", "$(UNIQUE_ID)", $(MAX_POINTS))

#asynSetTraceMask($(PORT), -1, ERROR|FLOW|DRIVER)

dbLoadTemplate("$(MEASCOMP)/db/USBCTR.substitutions", "P=$(PREFIX), PORT=$(PORT)")

# This loads the scaler record and supporting records
dbLoadRecords("$(SCALER)/db/scaler.db", "P=$(SCALER_PREFIX), S=$(SCALER_NAME), DTYP=Asyn Scaler, OUT=@asyn($(PORT)), FREQ=10000000")

# This database provides the support for the MCS functions
dbLoadRecords("$(MEASCOMP)/db/measCompMCS.template", "P=$(MCS_PREFIX), PORT=$(PORT), MAX_POINTS=$(MAX_POINTS)")

# Load either MCA or waveform records below
# The number of records loaded must be the same as MAX_COUNTERS defined above

# This loads the waveform records
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)1,  INP=@asyn($(PORT) 0),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)2,  INP=@asyn($(PORT) 1),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)3,  INP=@asyn($(PORT) 2),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)4,  INP=@asyn($(PORT) 3),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)5,  INP=@asyn($(PORT) 4),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)6,  INP=@asyn($(PORT) 5),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)7,  INP=@asyn($(PORT) 6),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)8,  INP=@asyn($(PORT) 7),  CHANS=$(MAX_POINTS)")
dbLoadRecords("$(MCA)/db/SIS38XX_waveform.template", "P=$(MCS_PREFIX), R=$(RNAME)9,  INP=@asyn($(PORT) 8),  CHANS=$(MAX_POINTS)")

set_savefile_path("$(MEASCOMP)/usb-ctr-ioc", "autosave")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
set_requestfile_path("$(MEASCOMP)/usb-ctr-ioc", "")
set_requestfile_path("$(SCALER)", "db")
set_requestfile_path("$(MEASCOMP)", "db")

iocInit

seq(USBCTR_SNL, "P=$(MCS_PREFIX), R=$(RNAME), NUM_COUNTERS=$(MAX_COUNTERS), FIELD=$(FIELD)")

create_monitor_set("auto_settings.req",30,"P=$(PREFIX), MP=$(MCS_PREFIX)")
