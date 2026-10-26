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

# motorUtil's records, then each axis's asyn_motor.db and the HXP's
# HXP_extra.db and HXP_coords.db, as C's st.cmd and motor.cmd.hxp load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(HXP_IOC)/db/motor.substitutions.hxp", "P=$(P),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor HXP:m3 HXP:m3.RBV
#   caput HXP:m3 1.0
#   caput HXP:T3 1.5; caput HXP:MOVE_ALL 1
