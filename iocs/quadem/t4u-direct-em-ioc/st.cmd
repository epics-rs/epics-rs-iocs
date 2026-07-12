#!../../../target/debug/t4u-direct-em-ioc
#============================================================
# st.cmd — Sydor T4U electrometer IOC, straight to the meter
#
# Usage:
#   cargo run -p t4u-direct-em-ioc -- iocs/quadem/t4u-direct-em-ioc/st.cmd
#
# Mirrors quadEM's iocBoot/iocT4UDirect_EM/T4UDirect_EM.cmd.
#============================================================

epicsEnvSet("PREFIX",    "QE1_")
epicsEnvSet("RECORD",    "T4U_EM_")
epicsEnvSet("PORT",      "T4U_EM")
epicsEnvSet("TEMPLATE",  "T4UDirect_EM")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "5000")
epicsEnvSet("T4U_ADDR",  "192.168.11.90")
epicsEnvSet("DATA_PORT", "10101")

# $(QUADEM) is set to this crate's root (iocs/quadem/t4u-direct-em-ioc) by
# ioc_support at IOC startup; the shared quadEM db lives one level up.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(QUADEM)/../db:$(ADCORE)/db")
epicsEnvSet("CALFILE", "$(QUADEM)/DBPM_Settings.ini")

# Commands go to the meter's telnet port (23); data arrives as UDP datagrams on
# $(DATA_PORT), which the driver binds. The calibration file supplies the
# per-range slope/offset pairs written to registers 100-107 on a range change.
drvT4UDirect_EMConfigure("$(PORT)", "$(T4U_ADDR)", $(RING_SIZE), $(DATA_PORT), "$(CALFILE)")

dbLoadRecords("$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                  # List all PVs
#   dbpf QE1_T4U_EM_WaitStateMode 2      # Triggered mode (pulsed calibration)
#   dbpf QE1_T4U_EM_ReadsPerPacket 50
#   dbgf QE1_T4U_EM_Current1:MeanValue_RBV
#   asynReport                           # Show port/plugin status
