#!../../../target/debug/nsls-em-ioc
#============================================================
# st.cmd — NSLS Precision Integrator (NSLS_EM) IOC
#
# Usage:
#   cargo run -p nsls-em-ioc -- iocs/quadem/nsls-em-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocNSLS_EM/NSLS_EM.cmd.
#============================================================

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "NSLS_EM:")
epicsEnvSet("PORT",      "NSLS_EM")
epicsEnvSet("TEMPLATE",  "NSLS_EM")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "1000")
epicsEnvSet("BROADCAST", "164.54.160.255")
epicsEnvSet("MODULE_ID", "0")

# $(QUADEM) is set to this crate's root (iocs/quadem/nsls-em-ioc) by
# ioc_support at IOC startup; the shared quadEM db lives one level up.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(QUADEM)/db:$(QUADEM)/../db:$(ADCORE)/db")

# The driver broadcasts on $(BROADCAST):37747 to find the module with the
# given ID, then opens the TCP command (4747) and data (5757) ports itself.
drvNSLS_EMConfigure("$(PORT)", "$(BROADCAST)", $(MODULE_ID), $(RING_SIZE))

dbLoadRecords("$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                             # List all PVs
#   dbpf QE1:NSLS_EM:Acquire 1      # Start acquisition
#   dbgf QE1:NSLS_EM:Current1:MeanValue_RBV
#   asynReport                      # Show port/plugin status
