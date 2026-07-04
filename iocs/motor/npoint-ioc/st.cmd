#============================================================
# st.cmd — nPoint C300 motor IOC startup script
#
# Usage:
#   cargo run -p npoint-ioc -- st.cmd
#
# Requires an nPoint C300 controller reachable over TCP at the host:port below.
# C300CreateController unlocks the controller and probes each axis (stage range,
# DI factor), so it must be reachable when the command runs.
#
# The C300 behaves like a setpoint controller: a move sets the target position,
# there is no speed/acceleration, and done is simulated from the encoder error.
#============================================================

epicsEnvSet("P",      "NPOINT:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "C300")
epicsEnvSet("HOST",   "192.168.0.100:23")
epicsEnvSet("NAXES",  "3")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# C300 framing: the driver appends the LF command terminator, so only the input
# EOS is configured here. Do not set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\n")

# ---- C300 controller ----
# C300CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]).
C300CreateController(0, "$(IPPORT)", $(NAXES), 100, 1000)

# One motor record per axis (DTYP C300_$(CARD)_0, _1, _2). Positions cross the
# driver boundary in the controller's native units, so MRES = 1.
dbLoadRecords("db/c300.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/c300.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/c300.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor NPOINT:m1 NPOINT:m1.RBV
#   caput NPOINT:m1 100000
