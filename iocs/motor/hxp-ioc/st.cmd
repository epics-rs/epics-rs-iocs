#============================================================
# st.cmd — Newport HXP hexapod controller IOC startup script
#
# Usage:
#   cargo run -p hxp-ioc -- st.cmd
#
# Requires a Newport HXP hexapod reachable over TCP on port 5001. The HXP
# speaks the same text RPC protocol as the XPS. HXPCreateController opens the
# poll socket and one move socket per axis, each its own asyn IP port as C's
# ConnectToServer makes it, and creates all six axes X, Y, Z, U, V, W
# (addresses 0 .. 5) in one step.
#============================================================

epicsEnvSet("P",    "HXP:")
epicsEnvSet("PORT", "HXP1")
epicsEnvSet("IP",   "192.168.0.254")

# ---- HXP controller + its six axes ----
# HXPCreateController(portName, IPAddress, IPPort, movingPollPeriod, idlePollPeriod)
HXPCreateController("$(PORT)", "$(IP)", 5001, 200, 1000)

dbLoadRecords("$(HXP_IOC)/db/hxp.template", "P=$(P),MX=mX,MY=mY,MZ=mZ,MU=mU,MV=mV,MW=mW,PORT=$(PORT)")
# Coordinate system for moves, group status/error, move-all targets.
dbLoadRecords("$(HXP_IOC)/db/HXP_extra.db", "P=$(P),R=,PORT=$(PORT)")
# Coordinate-system definitions: read back, set, then sync the motors.
dbLoadRecords("$(HXP_IOC)/db/HXP_coords.db", "P=$(P),R=,PORT=$(PORT),M1=mX,M2=mY,M3=mZ,M4=mU,M5=mV,M6=mW")

iocInit()

# Example:
#   dbl
#   camonitor HXP:mZ HXP:mZ.RBV
#   caput HXP:mZ 1.0
#   caput HXP:T3 1.5; caput HXP:MOVE_ALL 1
