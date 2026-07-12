#!../../../target/debug/t4u-em-ioc
#============================================================
# st.cmd — Sydor T4U electrometer IOC, through the Qt middle layer
#
# Usage:
#   cargo run -p t4u-em-ioc -- iocs/quadem/t4u-em-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocT4U_EM/T4U_EM.cmd.
#============================================================

epicsEnvSet("PREFIX",     "QE1_")
epicsEnvSet("RECORD",     "T4U_EM_")
epicsEnvSet("PORT",       "T4U_EM")
epicsEnvSet("TEMPLATE",   "T4U_EM")
epicsEnvSet("RING_SIZE",  "10000")
epicsEnvSet("TSPOINTS",   "1000")
epicsEnvSet("QTHOST",     "127.0.0.1")
epicsEnvSet("QTBASEPORT", "15001")

# $(QUADEM) is set to this crate's root (iocs/quadem/t4u-em-ioc) by ioc_support
# at IOC startup; the shared quadEM db lives one level up.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(QUADEM)/../db:$(ADCORE)/db")

# The driver opens the command port ($(QTBASEPORT)) and the data port
# ($(QTBASEPORT) + 1) on the middle-layer host itself, as TCP_Command_$(PORT)
# and TCP_Data_$(PORT).
drvT4U_EMConfigure("$(PORT)", "$(QTHOST)", $(RING_SIZE), $(QTBASEPORT))

dbLoadRecords("$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                              # List all PVs
#   dbpf QE1_T4U_EM_Range 2          # Select the 47 ohm range
#   dbpf QE1_T4U_EM_Updater 1        # Dump registers 100-107
#   dbgf QE1_T4U_EM_Current1:MeanValue_RBV
#   asynReport                       # Show port/plugin status
