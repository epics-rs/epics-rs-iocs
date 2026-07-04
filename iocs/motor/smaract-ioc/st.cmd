#============================================================
# st.cmd — SmarAct MCS2 motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.cmd
#
# Requires a SmarAct MCS2 controller reachable over TCP at the host:port below.
# MCS2CreateController reads the controller serial number, so it must be
# reachable when the command runs.
#
# The positioner type (PTYP) and calibration must be configured on the
# controller (or with the SmarAct tools) beforehand — those auxiliary parameters
# are not exposed through the motor record.
#============================================================

epicsEnvSet("P",      "MCS2:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "MCS2")
epicsEnvSet("HOST",   "192.168.1.200:55551")
epicsEnvSet("NAXES",  "3")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# MCS2 framing: the driver appends the LF command terminator, so only the input
# EOS is configured here (replies are CRLF terminated). Do not set an output EOS
# — the port would append it a second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")

# ---- MCS2 controller ----
# MCS2CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]).
MCS2CreateController(0, "$(IPPORT)", $(NAXES), 100, 1000)

# One motor record per channel (DTYP MCS2_$(CARD)_0, _1, _2). The driver reports
# positions in nanometres (linear) / micro-degrees (rotary), so MRES = 1 and
# EGU = nm.
dbLoadRecords("db/mcs2.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/mcs2.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/mcs2.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor MCS2:m1 MCS2:m1.RBV
#   caput MCS2:m1 1000000
