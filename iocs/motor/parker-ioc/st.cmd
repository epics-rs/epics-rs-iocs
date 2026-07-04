#============================================================
# st.cmd — Parker OEM750 motor IOC startup script
#
# Usage:
#   cargo run -p parker-ioc -- st.cmd
#
# Requires a Parker OEM750 controller reachable over TCP at the host:port below.
#============================================================

epicsEnvSet("P",      "OEM:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "OEM")
epicsEnvSet("HOST",   "192.168.1.220:5000")
epicsEnvSet("NAXES",  "1")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# OEM framing: the driver appends the CR command terminator, so only the input
# EOS is configured here. Do not set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\r")

# ---- OEM750 controller ----
# OEMCreateController(card, oemPort, numAxes, [movingPollMs], [idlePollMs]).
OEMCreateController(0, "$(IPPORT)", $(NAXES), 100, 1000)

# One motor record per axis (DTYP OEM_$(CARD)_0, _1, ...). The driver reports
# positions in controller counts, so MRES = 1.
dbLoadRecords("db/oem.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor OEM:m1 OEM:m1.RBV
#   caput OEM:m1 10000
