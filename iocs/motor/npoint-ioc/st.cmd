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
epicsEnvSet("CTRL",   "C300_1")
epicsEnvSet("IPPORT", "C300")
epicsEnvSet("HOST",   "192.168.0.100:23")
epicsEnvSet("NAXES",  "3")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# Port EOS as C needs it: C300MotorDriver sets no EOS and sends bare commands.
asynOctetSetInputEos("$(IPPORT)", 0, "\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\n")

# ---- C300 controller ----
# C300CreateController(portName, C300PortName, numAxes, movingPollPeriod, idlePollPeriod).
C300CreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 1000)

# Each axis's asyn_motor.db, as C's C300.cmd loads them.
dbLoadTemplate("$(NPOINT_IOC)/db/C300.substitutions", "P=$(P),PORT=$(CTRL)")

# motorUtil's records, as C's st.cmd loads them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor NPOINT:m1 NPOINT:m1.RBV
#   caput NPOINT:m1 100000
