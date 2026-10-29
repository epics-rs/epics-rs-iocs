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
# read. This configures C's st.cmd.xps5 controller: six axes of group GROUP
# and its XPSAuxConfig I/O port.
#============================================================

epicsEnvSet("P",     "XPS:")
epicsEnvSet("PORT",  "XPS1")
epicsEnvSet("IP",    "192.168.0.254")

# ---- XPS controller + axes ----
# XPSCreateController(portName, IPAddress, IPPort, numAxes, movingPollPeriod,
#                     idlePollPeriod, enableSetPosition, setPositionSettlingTime)
XPSCreateController("$(PORT)", "$(IP)", 5001, 6, 10, 500, 0, 500)
# XPSAuxConfig(portName, IPAddress, IPPort, pollPeriod)
XPSAuxConfig("XPS_AUX1", "$(IP)", 5001, 50)
# XPSCreateAxis(portName, axis, positionerName, stepsPerUnit)
# stepsPerUnit is 1/MRES of the axis record.
XPSCreateAxis("$(PORT)", 0, "GROUP.PHI",   "1000")
XPSCreateAxis("$(PORT)", 1, "GROUP.KAPPA", "10000")
XPSCreateAxis("$(PORT)", 2, "GROUP.OMEGA", "10000")
XPSCreateAxis("$(PORT)", 3, "GROUP.PSI",   "4000")
XPSCreateAxis("$(PORT)", 4, "GROUP.THETA", "10000")
XPSCreateAxis("$(PORT)", 5, "GROUP.NU",    "4000")
# XPSCreateProfile(portName, maxPoints, ftpUsername, ftpPassword)
XPSCreateProfile("$(PORT)", 2000, "Administrator", "Administrator")

# C's eight basic_asyn_motor.db rows, as C's st.cmd.xps5 loads them; m7 and
# m8 sit at addresses 6 and 7, past the six axes, and fail init as C's do. C's other
# XPSCreateController example, st.cmd.xps6, loads a motor.substitutions.xps6
# that C does not ship, so it boots with no motor records.
dbLoadTemplate("$(XPS_IOC)/db/motor.substitutions.xps5", "P=$(P),PORT=$(PORT)")

# xpsSlave.st's records, for the seq line below:
#   dbLoadRecords("$(XPS_IOC)/db/xpsSlave.db", "P=$(P),R=slave")

# XPS_trajectoryScan.st's records, for the seq line below:
#   dbLoadRecords("$(XPS_IOC)/db/trajectoryScan.db", "P=$(P),R=traj1:,NAXES=1,NELM=2000,NPULSE=2000")

iocInit()

# This IOC does not use save/restore, so set values of some PVs
dbpf("$(P)m1.RTRY", "0")
dbpf("$(P)m1.TWV", "0.1")
dbpf("$(P)m2.RTRY", "0")
dbpf("$(P)m2.TWV", "0.1")
dbpf("$(P)m3.RTRY", "0")
dbpf("$(P)m3.TWV", "0.1")
dbpf("$(P)m4.RTRY", "0")
dbpf("$(P)m4.TWV", "0.1")
dbpf("$(P)m5.RTRY", "0")
dbpf("$(P)m5.TWV", "0.1")
dbpf("$(P)m6.RTRY", "0")
dbpf("$(P)m6.TWV", "0.1")

# xpsSlave.st gears group S to positioner M on its own XPS socket:
#   seq &xpsSlave, "P=$(P),R=slave,IP=$(IP),M=GROUP.PHI,S=GROUP2"

# XPS_trajectoryScan.st runs trajectoryScan.db's PVT trajectories on group
# GROUP over its own XPS sockets, the file sent by FTP as USERNAME:
#   seq &XPS_trajectoryScan, "P=$(P),R=traj1:,M1=m1,IPADDR=$(IP),PORT=5001,GROUP=GROUP,P1=PHI,USERNAME=Administrator,PASSWORD=Administrator"

# Run a TCL script file stored on the XPS (task name + parameters default "0"):
#   XPSTclScriptExecute("$(PORT)", "MyScript.tcl")

# Position compare output of axis 0; writing a PositionCompare* record
# applies the six settings to the positioner and reads them back:
#   dbLoadRecords("$(XPS_IOC)/db/XPSPositionCompare.db", "P=$(P),R=m1:,PORT=$(PORT),ADDR=0,PREC=4")

# Example:
#   dbl
#   camonitor XPS:m1 XPS:m1.RBV
#   caput XPS:m1 10.0
