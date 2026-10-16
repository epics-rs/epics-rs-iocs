#!../../../target/debug/fx4-ioc
#============================================================
# st.cmd — Pyramid FX4 4-channel picoammeter IOC
#
# Usage (from this directory, as C runs from iocBoot/iocFX4):
#   ../../../target/debug/fx4-ioc st.cmd
#
# Port of quadEM iocBoot/iocFX4/st.cmd + FX4.cmd. dbLoadDatabase and
# registerRecordDeviceDriver have no epics-rs counterpart; $(QUADEM) is set
# to iocs/quadem by ioc_support.
#============================================================

errlogInit(5000)

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "FX4:")
epicsEnvSet("PORT",      "FX4")
epicsEnvSet("TEMPLATE",  "FX4")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "4096")
epicsEnvSet("IP",        "164.54.161.10")

drvFX4Configure("$(PORT)", "$(IP)", $(RING_SIZE))
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), FXP=$(IP):, ADDR=0, TIMEOUT=1")

< $(QUADEM)/quadEM_Plugins.cmd

asynSetTraceIOMask("$(PORT)",0,2)

asynSetTraceMask("$(PORT)",  0, ERROR | DRIVER)

< $(QUADEM)/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
