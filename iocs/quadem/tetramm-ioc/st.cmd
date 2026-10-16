#!../../../target/debug/tetramm-ioc
#============================================================
# st.cmd — CaenEls TetrAMM electrometer IOC
#
# Usage (from this directory, as C runs from iocBoot/iocTetrAMM):
#   ../../../target/debug/tetramm-ioc st.cmd
#
# Port of quadEM iocBoot/iocTetrAMM/st.cmd + TetrAMM.cmd. dbLoadDatabase and
# registerRecordDeviceDriver have no epics-rs counterpart; $(QUADEM) is set
# to iocs/quadem by ioc_support.
#============================================================

errlogInit(5000)

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "TetrAMM:")
epicsEnvSet("PORT",      "TetrAMM")
epicsEnvSet("TEMPLATE",  "TetrAMM")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "2048")
epicsEnvSet("IP",        "10.54.160.186:10001")

#drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,
#                        noProcessEos)
drvAsynIPPortConfigure("IP_$(PORT)", "$(IP)", 0, 0, 0)
asynOctetSetInputEos("IP_$(PORT)",  0, "\r\n")
asynOctetSetOutputEos("IP_$(PORT)", 0, "\r")

# Set both TRACE_IO_ESCAPE (for ASCII command/response) and TRACE_IO_HEX (for binary data)
asynSetTraceIOMask("IP_$(PORT)", 0, 6)
#asynSetTraceFile("IP_$(PORT)",   0, "AHxxx.out")
#asynSetTraceMask("IP_$(PORT)",   0,  9)
asynSetTraceIOTruncateSize("IP_$(PORT)", 0, 4000)

# Load asynRecord record
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX), R=asyn1,PORT=IP_$(PORT),ADDR=0,OMAX=256,IMAX=256")

drvTetrAMMConfigure("$(PORT)", "IP_$(PORT)", $(RING_SIZE))
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

< $(QUADEM)/quadEM_Plugins.cmd

asynSetTraceIOMask("$(PORT)",0,2)
# Enable ASYN_TRACE_ERROR and ASYN_TRACE_WARNING
#asynSetTraceMask("$(PORT)",  0, 0x21)

< $(QUADEM)/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
