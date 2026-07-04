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
# motorSimCreateController(motorPort, numAxes, [movingPollMs], [idlePollMs])
motorSimCreateController("$(PORT)", 2, 100, 1000)

# Optional per-axis reconfigure of hard limits / home / start:
# motorSimConfigAxis(motorPort, axis, hiHardLimit, lowHardLimit, home, start)
motorSimConfigAxis("$(PORT)", 0, 100, -100, 0, 0)
motorSimConfigAxis("$(PORT)", 1, 100, -100, 0, 0)

# One motor record per axis (DTYP motorSim_$(PORT)_0, _1, ...).
dbLoadRecords("db/motorsim.template", "P=$(P),M=m1,N=0,PORT=$(PORT)")
dbLoadRecords("db/motorsim.template", "P=$(P),M=m2,N=1,PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor SIM:m1 SIM:m1.RBV
#   caput SIM:m1 5.0
