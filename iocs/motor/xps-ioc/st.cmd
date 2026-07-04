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

# Run a TCL script file stored on the XPS (task name + parameters default "0"):
#   XPSTclScriptExecute("$(PORT)", "MyScript.tcl")

# Position-compare output (PCO). The four modes and settling time live on the
# motor record's PCO_* fields; drive them with caput, then arm with PCO_ENABLE.
# PCO_MODE: 0=Disable 1=Pulse 2=AquadB-windowed 3=AquadB-always.
# PCO_PW / PCO_SETTLE are microseconds (valid PW {0.2,1,2.5,10};
# valid SETTLE {0.075,1,4,12}). Example (Pulse mode over 0..10 EGU, step 0.5):
#   caput $(P)$(M0).PCO_MODE 1
#   caput $(P)$(M0).PCO_START 0
#   caput $(P)$(M0).PCO_END 10
#   caput $(P)$(M0).PCO_INC 0.5
#   caput $(P)$(M0).PCO_PW 1.0
#   caput $(P)$(M0).PCO_SETTLE 0.075
#   caput $(P)$(M0).PCO_ENABLE 1
# NOTE: a helper .db with DB output links to these fields is NOT possible — the
# base DB-link parser caps target field names at 4 chars, and these are longer.
# Use caput (CA/PVA), which resolves full field names.

# PVT trajectory profiles (driver-private). A profile is defined from a CSV
# points file, built (trajectory-file generation + FTP upload to the XPS +
# verification against dynamics and soft limits), then executed on a background
# thread over a dedicated socket so polling continues.
#
# The CSV has one row per point: "time, pos_0, pos_1, ..." with one position
# column per positioner in the group, in the order the axes were created
# (device/positioner units and seconds). '#' comments and blank lines are
# skipped. Example one-axis file db/scan.csv:
#     # time,  GROUP1.POSITIONER
#     0.5, 0.0
#     0.5, 1.0
#     0.5, 2.0
#     0.5, 3.0
#
# Execution needs its own TCP socket (MultipleAxesPVTExecution blocks until the
# trajectory finishes). Configure a dedicated exec port alongside the move port:
#     drvAsynIPPortConfigure("XPSEXEC0", "$(HOST)", 0, 0, 1)
#
# Then, after iocInit:
#     XPSDefineProfileFromFile("$(PORT)", "GROUP1", "db/scan.csv", "absolute")
#     XPSBuildProfile("$(PORT)", "TrajectoryScan.trj", "192.168.0.254")
#     XPSExecuteProfile("$(PORT)", "XPSEXEC0", 1)
# XPSBuildProfile FTP credentials/dir default to the XPS factory Administrator
# account and /Admin/Public/Trajectories; override with extra args if changed.
# Only plain FTP (XPS-C/Q) is supported; XPS-D SFTP is not implemented.

# Example:
#   dbl
#   camonitor XPS:m0 XPS:m0.RBV
#   caput XPS:m0 10.0
