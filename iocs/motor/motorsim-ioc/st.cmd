#============================================================
# st.cmd — Simulated motor IOC startup script
#
# Usage:
#   cargo run -p motorsim-ioc -- st.cmd
#
# No hardware required: each axis integrates its own trapezoidal trajectory.
#============================================================

epicsEnvSet("P",     "SIM:")
epicsEnvSet("PORT",  "MOTOR1")

# ---- simulated controller: 2 axes ----
# motorSimCreateController(portName, numAxes, priority, stackSize)
motorSimCreateController("$(PORT)", 2)

# Optional per-axis reconfigure of hard limits / home / start:
# motorSimConfigAxis(portName, axis, hiHardLimit, lowHardLimit, home, start)
motorSimConfigAxis("$(PORT)", 0, 100, -100, 0, 0)
motorSimConfigAxis("$(PORT)", 1, 100, -100, 0, 0)

# Each axis's basic_asyn_motor.db, as C's motorSim.cmd loads them.
dbLoadTemplate("$(MOTORSIM_IOC)/db/motorSim.substitutions", "P=$(P),PORT=$(PORT)")

# motorUtil's records. C's st.cmd runs motorUtilInit("IOC:") without loading
# motorUtil.db, so the allstop and alldone it serves never exist.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor SIM:m1 SIM:m1.RBV
#   caput SIM:m1 5.0
