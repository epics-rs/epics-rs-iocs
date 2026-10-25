#============================================================
# st.cmd — Newport XPS motion controller IOC startup script
#
# Usage:
#   cargo run -p xps-ioc -- st.cmd
#
# Requires a Newport XPS (C8/Q8/RL) reachable over TCP on port 5001.
# The XPS speaks a text RPC protocol: each socket is one TCP connection.
# XPSCreateController opens the poll socket, XPSCreateAxis one move socket
# per axis, each its own asyn IP port as C's ConnectToServer makes it.
# XPSCreateAxis reads the positioner's
# S-gamma jerk times and corrector at startup and, as C does, ignores a failed
# read. This example configures a single axis (GROUP1.POSITIONER).
#============================================================

epicsEnvSet("P",     "XPS:")
epicsEnvSet("M0",    "m0")
epicsEnvSet("PORT",  "MOTOR1")
epicsEnvSet("IP",    "192.168.0.254")

# ---- XPS controller + axes ----
# XPSCreateController(portName, IPAddress, IPPort, numAxes, movingPollPeriod,
#                     idlePollPeriod, enableSetPosition, setPositionSettlingTime)
XPSCreateController("$(PORT)", "$(IP)", 5001, 1, 100, 1000, 0, 0)
# XPSCreateAxis(portName, axis, positionerName, stepsPerUnit)
# stepsPerUnit is 1/MRES of the axis record.
XPSCreateAxis("$(PORT)", 0, "GROUP1.POSITIONER", "20480")
# XPSCreateProfile(portName, maxPoints, ftpUsername, ftpPassword)
XPSCreateProfile("$(PORT)", 2000, "Administrator", "Administrator")

dbLoadRecords("$(XPS_IOC)/db/xps.template", "P=$(P),M0=$(M0),PORT=$(PORT),ADDR=0")

# xpsSlave.st's records, for the seq line below:
#   dbLoadRecords("$(XPS_IOC)/db/xpsSlave.db", "P=$(P),R=slave")

iocInit()

# xpsSlave.st gears group S to positioner M on its own XPS socket:
#   seq &xpsSlave, "P=$(P),R=slave,IP=$(IP),M=GROUP1.POSITIONER,S=GROUP2"

# Run a TCL script file stored on the XPS (task name + parameters default "0"):
#   XPSTclScriptExecute("$(PORT)", "MyScript.tcl")

# Position-compare output (PCO), driver-private iocsh command (the C
# base-class PCO API is an open, unmerged motor PR #248, so PCO is not a
# motor-record feature here):
#   XPSPositionCompare(motorPort, positionerName, mode, [minPosition],
#                      [maxPosition], [positionStep], [pulseWidth], [settlingTime])
# mode: 0=Disable 1=Pulse 2=AquadB-windowed 3=AquadB-always.
# Positions/step are device units; pulseWidth/settlingTime are microseconds
# (valid widths {0.2,1,2.5,10}; valid settling {0.075,1,4,12}), defaulting to
# 0.2/0.075. Example (Pulse mode over 0..10, step 0.5, 1 us pulses):
#   XPSPositionCompare("$(PORT)", "GROUP1.POSITIONER", 1, 0, 10, 0.5, 1.0, 0.075)
# Disable:
#   XPSPositionCompare("$(PORT)", "GROUP1.POSITIONER", 0)

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
# Execution runs on the profile socket XPSCreateController opened
# (MultipleAxesPVTExecution blocks until the trajectory finishes). After iocInit:
#     XPSDefineProfileFromFile("$(PORT)", "GROUP1", "db/scan.csv", "absolute")
#     XPSBuildProfile("$(PORT)", "TrajectoryScan.trj", "$(IP)")
#     XPSExecuteProfile("$(PORT)", 1)
# XPSBuildProfile FTP credentials default to the XPSCreateProfile login and the
# directory to /Admin/Public/Trajectories; override with extra args if changed.
# Only plain FTP (XPS-C/Q) is supported; XPS-D SFTP is not implemented.
#
# Gathering: every execution samples SetpointPosition + CurrentPosition of all
# registered positioners on each trajectory pulse. The optional trailing args
#     XPSExecuteProfile(motorPort, [executions],
#                       [startPulses], [endPulses], [numPulses])
# window the pulse train over 1-based trajectory elements (defaults: the whole
# profile with one pulse per element). After the run completes, read the
# samples back into a CSV file (actual position + following error per
# positioner, device units):
#     XPSReadbackProfile("$(PORT)", "readback.csv")

# Example:
#   dbl
#   camonitor XPS:m0 XPS:m0.RBV
#   caput XPS:m0 10.0
