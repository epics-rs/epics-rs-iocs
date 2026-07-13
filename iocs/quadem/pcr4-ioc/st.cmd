#!../../../target/debug/pcr4-ioc
#============================================================
# st.cmd — SenSiC PCR4 4-channel picoammeter IOC
#
# Usage:
#   cargo run -p pcr4-ioc -- iocs/quadem/pcr4-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocPCR4/PCR4.cmd.
#============================================================

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "PCR4:")
epicsEnvSet("PORT",      "PCR4")
epicsEnvSet("TEMPLATE",  "PCR4")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "2048")
epicsEnvSet("IP",        "164.54.160.165:3000")

# $(QUADEM) is set to this crate's root (iocs/quadem/pcr4-ioc) by ioc_support
# at IOC startup; the shared quadEM db lives one level up.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(QUADEM)/db:$(QUADEM)/../db:$(ADCORE)/db")

# drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,
#                        noProcessEos)
drvAsynIPPortConfigure("IP_$(PORT)", "$(IP)", 0, 0, 0)
asynOctetSetInputEos("IP_$(PORT)",  0, "\r\n")
asynOctetSetOutputEos("IP_$(PORT)", 0, "\r")

drvPCR4Configure("$(PORT)", "IP_$(PORT)", $(RING_SIZE))

dbLoadRecords("$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                          # List all PVs
#   dbpf QE1:PCR4:Acquire 1      # Start acquisition
#   dbgf QE1:PCR4:Current1:MeanValue_RBV
#   asynReport                   # Show port/plugin status
