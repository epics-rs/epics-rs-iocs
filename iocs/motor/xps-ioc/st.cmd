#============================================================
# st.cmd — Newport XPS motion controller IOC startup script
#
# Usage:
#   cargo run -p xps-ioc -- st.cmd
#
# Requires a Newport XPS (C8/Q8/RL) reachable over TCP on port 5001.
# The XPS speaks a text RPC protocol: each socket is one TCP connection.
# This driver uses one shared POLL socket (all reads) plus one MOVE socket
# per axis (fire-and-forget motion). XPSCreateAxis reads the positioner's
# S-gamma jerk times at startup, so the controller must be reachable when it
# runs. This example configures a single axis (GROUP1.POSITIONER).
#============================================================

epicsEnvSet("P",     "XPS:")
epicsEnvSet("M0",    "m0")
epicsEnvSet("PORT",  "MOTOR1")
epicsEnvSet("POLL",  "XPSPOLL")
epicsEnvSet("MOVE0", "XPSMOVE0")
epicsEnvSet("HOST",  "192.168.0.254:5001")

# ---- TCP octet sockets to the XPS (one connection each) ----
# drvAsynIPPortConfigure(portName, hostInfo, [priority], [noAutoConnect], [noProcessEos])
# noProcessEos=1: the driver frames replies on ",EndOfAPI" itself, matching the
# C asynOctetSocket which sets noProcessEos.
drvAsynIPPortConfigure("$(POLL)",  "$(HOST)", 0, 0, 1)
drvAsynIPPortConfigure("$(MOVE0)", "$(HOST)", 0, 0, 1)

# ---- XPS controller + axes ----
# XPSCreateController(motorPort, pollPort, numAxes, [movingPollMs], [idlePollMs],
#                     [enableSetPosition], [setPositionSettlingMs])
XPSCreateController("$(PORT)", "$(POLL)", 1, 100, 1000, 0, 0)
# XPSCreateAxis(motorPort, movePort, axis, positionerName, stepsPerUnit)
XPSCreateAxis("$(PORT)", "$(MOVE0)", 0, "GROUP1.POSITIONER", 1)

dbLoadRecords("db/xps.template", "P=$(P),M0=$(M0),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor XPS:m0 XPS:m0.RBV
#   caput XPS:m0 10.0
