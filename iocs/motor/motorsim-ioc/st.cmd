#============================================================
# st.cmd — Simulated motor IOC startup script
#
# Usage:
#   cargo run -p motorsim-ioc -- st.cmd
#
# No hardware required: each axis integrates its own trapezoidal trajectory.
#============================================================

epicsEnvSet("P",     "SIM:")
epicsEnvSet("PORT",  "motorSim1")

# ---- simulated controller: 8 axes ----
# motorSimCreateController(portName, numAxes, priority, stackSize)
motorSimCreateController("$(PORT)", 8)

# Per-axis hard limits, home and start, in steps, as C's motorSim.cmd sets
# them:
# motorSimConfigAxis(portName, axis, hiHardLimit, lowHardLimit, home, start)
motorSimConfigAxis("$(PORT)", 0, 20000, -20000,  500, 0)
motorSimConfigAxis("$(PORT)", 1, 20000, -20000, 1500, 0)
motorSimConfigAxis("$(PORT)", 2, 20000, -20000, 2500, 0)
motorSimConfigAxis("$(PORT)", 3, 20000, -20000, 3000, 0)
motorSimConfigAxis("$(PORT)", 4, 20000, -20000,  500, 0)
motorSimConfigAxis("$(PORT)", 5, 20000, -20000, 1500, 0)
motorSimConfigAxis("$(PORT)", 6, 20000, -20000, 2500, 0)
motorSimConfigAxis("$(PORT)", 7, 20000, -20000, 3000, 0)

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
#   caput SIM:m1 1.0
