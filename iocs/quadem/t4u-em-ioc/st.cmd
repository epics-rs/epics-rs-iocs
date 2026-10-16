#!../../../target/debug/t4u-em-ioc
#============================================================
# st.cmd — Sydor T4U electrometer IOC, through the Qt middle layer
#
# Usage (from this directory, as C runs from iocBoot/iocT4U_EM):
#   ../../../target/debug/t4u-em-ioc st.cmd
#
# Port of quadEM iocBoot/iocT4U_EM/st.cmd + T4U_EM.cmd. dbLoadDatabase and
# registerRecordDeviceDriver have no epics-rs counterpart; $(QUADEM) is set
# to iocs/quadem by ioc_support.
#============================================================

errlogInit(5000)

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

epicsEnvSet("PREFIX",    "QE1_")
epicsEnvSet("RECORD",    "T4U_EM_")
epicsEnvSet("PORT",      "T4U_EM")
epicsEnvSet("TEMPLATE",  "T4U_EM")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "1000")
epicsEnvSet("QTHOST", "127.0.0.1")
epicsEnvSet("QTBASEPORT", "15001")

# Load asynRecord record
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX), R=asyn1,PORT=TCP_Command_$(PORT),ADDR=0,OMAX=256,IMAX=256")

drvT4U_EMConfigure("$(PORT)", "$(QTHOST)", $(RING_SIZE), $(QTBASEPORT))

#asynSetTraceIOMask("UDP_$(PORT)", 0, 2)
#asynSetTraceMask("UDP_$(PORT)", 0, 9)
asynSetTraceIOMask("TCP_Command_$(PORT)", 0, 2)
asynSetTraceMask("TCP_Command_$(PORT)", 0, 9)
#asynSetTraceIOMask("TCP_Data_$(PORT)", 0, 2)
#asynSetTraceMask("TCP_Data_$(PORT)", 0, 9)

dbLoadRecords("$(QUADEM)/db/$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT),ADDR=0,TIMEOUT=1")

< $(QUADEM)/quadEM_Plugins.cmd

asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",  0,0x29)

< $(QUADEM)/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
