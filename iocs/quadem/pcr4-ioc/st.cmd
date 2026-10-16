#!../../../target/debug/pcr4-ioc
#============================================================
# st.cmd — SenSiC PCR4 4-channel picoammeter IOC
#
# Usage (from this directory, as C runs from iocBoot/iocPCR4):
#   ../../../target/debug/pcr4-ioc st.cmd
#
# Port of quadEM iocBoot/iocPCR4/st.cmd + PCR4.cmd. dbLoadDatabase and
# registerRecordDeviceDriver have no epics-rs counterpart; $(QUADEM) is set
# to iocs/quadem by ioc_support.
#============================================================

errlogInit(5000)

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "PCR4:")
epicsEnvSet("PORT",      "PCR4")
epicsEnvSet("TEMPLATE",  "PCR4")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "2048")
epicsEnvSet("IP",        "164.54.160.165:3000")

#drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,
#                        noProcessEos)
drvAsynIPPortConfigure("IP_$(PORT)", "$(IP)", 0, 0, 0)
asynOctetSetInputEos("IP_$(PORT)",  0, "\r\n")
asynOctetSetOutputEos("IP_$(PORT)", 0, "\r")

# Set both TRACE_IO_ESCAPE (for ASCII command/response) and TRACE_IO_HEX (for binary data)
asynSetTraceIOMask("IP_$(PORT)", 0, 6)
asynSetTraceIOTruncateSize("IP_$(PORT)", 0, 4000)

# Load asynRecord record
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX), R=asyn1,PORT=IP_$(PORT),ADDR=0,OMAX=256,IMAX=256")

drvPCR4Configure("$(PORT)", "IP_$(PORT)", $(RING_SIZE))
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

< $(QUADEM)/quadEM_Plugins.cmd

asynSetTraceIOMask("$(PORT)",0,2)
# Enable ASYN_TRACE_ERROR and ASYN_TRACE_WARNING
#asynSetTraceMask("$(PORT)",  0, 0x21)

< $(QUADEM)/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
