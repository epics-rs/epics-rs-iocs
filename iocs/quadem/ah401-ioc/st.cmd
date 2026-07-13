#!../../../target/debug/ah401-ioc
#============================================================
# st.cmd — Elettra/CaenEls AH401B / AH401D picoammeter IOC
#
# Usage:
#   cargo run -p ah401-ioc -- iocs/quadem/ah401-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocAH401B/AH401B.cmd + iocBoot/AHxxx.cmd.
#============================================================

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "AH401B:")
epicsEnvSet("PORT",      "AH401B")
epicsEnvSet("TEMPLATE",  "AH401B")
epicsEnvSet("MODEL",     "AH401B")
#epicsEnvSet("MODEL",     "AH401D")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "1000")
epicsEnvSet("IP",        "164.54.160.242:10001")

# $(QUADEM) is set to this crate's root (iocs/quadem/ah401-ioc) by ioc_support
# at IOC startup; the shared quadEM db lives one level up.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(QUADEM)/db:$(QUADEM)/../db:$(ADCORE)/db")

# drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,
#                        noProcessEos)
drvAsynIPPortConfigure("IP_$(PORT)", "$(IP)", 0, 0, 0)
asynOctetSetInputEos("IP_$(PORT)",  0, "\r\n")
asynOctetSetOutputEos("IP_$(PORT)", 0, "\r")

drvAHxxxConfigure("$(PORT)", "IP_$(PORT)", $(RING_SIZE), "$(MODEL)")

dbLoadRecords("$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                            # List all PVs
#   dbpf QE1:AH401B:Acquire 1      # Start acquisition
#   dbgf QE1:AH401B:Current1:MeanValue_RBV
#   asynReport                     # Show port/plugin status
