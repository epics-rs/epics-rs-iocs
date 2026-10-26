#============================================================
# st.cmd — PI GCS2 stage-controller motor IOC startup script (motorPIGCS2)
#
# Usage:
#   cargo run -p pigcs2-ioc -- st.cmd
#
# Requires a PI GCS2 controller (C-863/C-867/C-663/C-884/E-861/E-871/E-873
# family) reachable over serial or TCP. One controller connection discovers
# every attached axis via SAI? and drives numAxes of them (in that order).
#============================================================

epicsEnvSet("P",       "pi:")
epicsEnvSet("CTRL",    "C867")
epicsEnvSet("PORT",    "C867_ETH")
epicsEnvSet("TTY",     "/dev/ttyUSB0")
epicsEnvSet("NUMAXES", "1")

# ---- asyn octet port ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
#drvAsynIPPortConfigure("$(PORT)", "192.168.1.100:50000", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "115200")

# ---- PI GCS2 controller ----
# PI_GCS2_CreateController(portName, asynPort, numAxes, priority, stackSize,
#                          movingPollingRate, idlePollingRate), rates in ms.
# Axes are auto-discovered via SAI? — the first numAxes of them (in
# controller-reported order) are attached.
PI_GCS2_CreateController("$(CTRL)", "$(PORT)", $(NUMAXES), 0, 0, 100, 1000)

# The asyn_motor.db of axis 0 (the first discovered GCS axis), as C's
# PI_GCS2.cmd loads it.
dbLoadTemplate("$(PIGCS2_IOC)/db/PI_GCS2.substitutions", "P=$(P),PORT=$(CTRL)")

# motorUtil's records, as C's st.cmd loads them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor pi:m1 pi:m1.RBV
#   caput pi:m1 10.0
