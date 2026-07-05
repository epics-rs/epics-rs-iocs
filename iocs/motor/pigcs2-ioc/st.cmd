#============================================================
# st.cmd — PI GCS2 stage-controller motor IOC startup script (motorPIGCS2)
#
# Usage:
#   cargo run -p pigcs2-ioc -- st.cmd
#
# Requires a PI GCS2 controller (C-863/C-867/C-663/C-884/E-861/E-871/E-873
# family) reachable over serial or TCP. One controller connection discovers
# every attached axis via SAI? and drives numAxes of them (in that order).
#============================================================

epicsEnvSet("P",       "pi:")
epicsEnvSet("CTRL",    "0")
epicsEnvSet("PORT",    "piAsyn")
epicsEnvSet("TTY",     "/dev/ttyUSB0")
epicsEnvSet("NUMAXES", "1")

# ---- asyn octet port ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
#drvAsynIPPortConfigure("$(PORT)", "192.168.1.100:50000", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "115200")

# GCS2 framing: replies are terminated by "\n". The driver owns the output
# terminator, so only the input EOS is set here. Do NOT set an output EOS (it
# would double-terminate).
asynOctetSetInputEos("$(PORT)", 0, "\n")

# ---- PI GCS2 controller ----
# PIGCS2CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]).
# Axes are auto-discovered via SAI? — the first numAxes of them (in
# controller-reported order) are attached.
PIGCS2CreateController("$(CTRL)", "$(PORT)", $(NUMAXES), 100, 1000)

# One motor record for axis 0. DTYP = PIGCS2_$(CTRL)_$(AXIS), where AXIS is
# the GCS axis name (e.g. "1"), not a bare index.
dbLoadRecords("db/pigcs2.template", "P=$(P),M=m1,CTRL=$(CTRL),AXIS=1")

iocInit()

# Example:
#   dbl
#   camonitor pi:m1 pi:m1.RBV
#   caput pi:m1 10.0
