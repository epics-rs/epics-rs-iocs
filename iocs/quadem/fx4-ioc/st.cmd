#!../../../target/debug/fx4-ioc
#============================================================
# st.cmd — Pyramid FX4 4-channel picoammeter IOC
#
# Usage:
#   cargo run -p fx4-ioc -- iocs/quadem/fx4-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocFX4/FX4.cmd.
#============================================================

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "FX4:")
epicsEnvSet("PORT",      "FX4")
epicsEnvSet("TEMPLATE",  "FX4")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "4096")
epicsEnvSet("IP",        "164.54.161.10")

# $(QUADEM) is set to this crate's root (iocs/quadem/fx4-ioc) by ioc_support at
# IOC startup; the shared quadEM db lives one level up.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(QUADEM)/../db:$(ADCORE)/db")

# The driver opens ws://$(IP) itself, so there is no asyn IP port to configure.
drvFX4Configure("$(PORT)", "$(IP)", $(RING_SIZE))

# $(FXP) is the prefix of the PV server the FX4 itself publishes; the template's
# range/bias/units records are Channel Access links to it.
dbLoadRecords("$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), FXP=$(IP):, ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                          # List all PVs
#   dbpf QE1:FX4:Acquire 1       # Start acquisition
#   dbgf QE1:FX4:Current1:MeanValue_RBV
#   asynReport                   # Show port/plugin status
