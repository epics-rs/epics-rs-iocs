#============================================================
# st.cmd — Newport HXP hexapod controller IOC startup script
#
# Usage:
#   cargo run -p hxp-ioc -- st.cmd
#
# Requires a Newport HXP hexapod reachable over TCP on port 5001. The HXP
# speaks the same text RPC protocol as the XPS. This driver uses one shared
# POLL socket (all reads, group-wide status/position) and one shared MOVE
# socket (fire-and-forget hexapod moves). HXPCreateController creates all six
# axes X, Y, Z, U, V, W (DTYP HXP_$(PORT)_0 .. _5) in one step.
#============================================================

epicsEnvSet("P",    "HXP:")
epicsEnvSet("PORT", "HXP1")
epicsEnvSet("POLL", "HXPPOLL")
epicsEnvSet("MOVE", "HXPMOVE")
epicsEnvSet("HOST", "192.168.0.10:5001")

# ---- TCP octet sockets to the hexapod (one connection each) ----
# noProcessEos=1: the driver frames replies on ",EndOfAPI" itself.
drvAsynIPPortConfigure("$(POLL)", "$(HOST)", 0, 0, 1)
drvAsynIPPortConfigure("$(MOVE)", "$(HOST)", 0, 0, 1)

# ---- HXP controller + its six axes ----
# HXPCreateController(motorPort, pollPort, movePort, [movingPollMs], [idlePollMs])
HXPCreateController("$(PORT)", "$(POLL)", "$(MOVE)", 100, 1000)

dbLoadRecords("db/hxp.template", "P=$(P),MX=mX,MY=mY,MZ=mZ,MU=mU,MV=mV,MW=mW,PORT=$(PORT)")

iocInit()

# Motor-record moves default to Work coordinates; switch to Tool with:
#   HXPSetMoveCoordSys("$(PORT)", "Tool")
#
# Move all six axes in one motion (absolute Work coordinates, device units):
#   HXPMoveAll("$(PORT)", 0.0, 0.0, 1.5, 0.0, 0.0, 0.0)
#
# Read or redefine the coordinate-system origins (Work/Tool/Base):
#   HXPCoordSysRead("$(PORT)")
#   HXPCoordSysSet("$(PORT)", "Work", 0.0, 0.0, 10.0, 0.0, 0.0, 0.0)

# Example:
#   dbl
#   camonitor HXP:mZ HXP:mZ.RBV
#   caput HXP:mZ 1.0
