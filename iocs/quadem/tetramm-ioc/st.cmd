#!../../../target/debug/tetramm-ioc
#============================================================
# st.cmd — CaenEls TetrAMM electrometer IOC
#
# Usage:
#   cargo run -p tetramm-ioc -- iocs/quadem/tetramm-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocTetrAMM/TetrAMM.cmd.
#============================================================

epicsEnvSet("PREFIX",    "QE1:")
epicsEnvSet("RECORD",    "TetrAMM:")
epicsEnvSet("PORT",      "TetrAMM")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "2048")
epicsEnvSet("IP",        "10.54.160.186:10001")

# $(QUADEM) is set to the quadem family root (iocs/quadem) by ioc_support
# at IOC startup; every quadEM template lives in its db/ subdir.

# drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,
#                        noProcessEos)
drvAsynIPPortConfigure("IP_$(PORT)", "$(IP)", 0, 0, 0)
asynOctetSetInputEos("IP_$(PORT)",  0, "\r\n")
asynOctetSetOutputEos("IP_$(PORT)", 0, "\r")

drvTetrAMMConfigure("$(PORT)", "IP_$(PORT)", $(RING_SIZE))

# Template-internal `include` lines (ADBase.template, NDArrayBase.template,
# ...) resolve through the db search path; direct dbLoad paths stay explicit.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

dbLoadRecords("$(QUADEM)/db/TetrAMM.template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                              # List all PVs
#   dbpf QE1:TetrAMM:Acquire 1       # Start acquisition
#   dbgf QE1:TetrAMM:Current1:MeanValue_RBV
#   asynReport                       # Show port/plugin status
