#!../../../target/debug/ah501-ioc
#============================================================
# st.cmd — Elettra/CaenEls AH501 series picoammeter IOC
#
# Usage (from this directory, as C runs from iocBoot/iocAH501):
#   ../../../target/debug/ah501-ioc st.cmd
#
# Port of quadEM iocBoot/iocAH501/st.cmd + AH501.cmd. dbLoadDatabase and
# registerRecordDeviceDriver have no epics-rs counterpart; $(QUADEM) is set
# to iocs/quadem by ioc_support.
#============================================================

errlogInit(5000)

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "AH501:")
epicsEnvSet("PORT",      "AH501")
epicsEnvSet("TEMPLATE",  "AH501")
#epicsEnvSet("MODEL",     "AH501D")
epicsEnvSet("MODEL",     "AH501BE")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "1000")
epicsEnvSet("IP",        "164.54.160.206:10001")

< $(QUADEM)/AHxxx.cmd

< $(QUADEM)/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
