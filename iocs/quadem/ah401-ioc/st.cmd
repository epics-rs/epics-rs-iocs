#!../../../target/debug/ah401-ioc
#============================================================
# st.cmd — Elettra/CaenEls AH401B / AH401D picoammeter IOC
#
# Usage (from this directory, as C runs from iocBoot/iocAH401B):
#   ../../../target/debug/ah401-ioc st.cmd
#
# Port of quadEM iocBoot/iocAH401B/st.cmd + AH401B.cmd. dbLoadDatabase and
# registerRecordDeviceDriver have no epics-rs counterpart; $(QUADEM) is set
# to iocs/quadem by ioc_support.
#============================================================

errlogInit(5000)

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "AH401B:")
epicsEnvSet("PORT",      "AH401B")
epicsEnvSet("TEMPLATE",  "AH401B")
epicsEnvSet("MODEL",     "AH401B")
#epicsEnvSet("MODEL",     "AH401D")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "1000")
epicsEnvSet("IP",        "164.54.160.242:10001")

< $(QUADEM)/AHxxx.cmd

< $(QUADEM)/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
