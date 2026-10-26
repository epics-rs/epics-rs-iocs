#============================================================
# st.a3200.cmd — Aerotech A3200 motor IOC startup script
#
# Usage:
#   cargo run -p aerotech-ioc -- st.a3200.cmd
#
# Requires an Aerotech A3200 controller reachable over TCP (or swap in the
# serial port line below). A3200AsynConfig discovers the name of each axis
# 0..numAxes; records address axis n as @asyn($(CTRL),n).
#============================================================

epicsEnvSet("P",      "a3200:")
epicsEnvSet("CTRL",   "L1")
epicsEnvSet("IPPORT", "a3200Asyn")
epicsEnvSet("HOST",   "192.168.1.50:8000")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# ---- A3200 controller ----
A3200AsynSetup(1)

# A3200AsynConfig(card, portName, asynAddress, numAxes, movingPollPeriod,
#                 idlePollPeriod, taskNumber, linear), periods in ms.
A3200AsynConfig(0, "$(IPPORT)", 0, 1, 100, 1000, 1, 1)
drvAsynMotorConfigure("$(CTRL)", "motorA3200", 0, 1)

# The axis's basic_asyn_motor.db and motorUtil's records, as C's
# a3.asyn.st.cmd.linux-x86 loads them.
dbLoadTemplate("$(AEROTECH_IOC)/db/a3.asyn.motor.substitutions", "P=$(P),PORT=$(CTRL)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor a3200:m1 a3200:m1.RBV
#   caput a3200:m1 10
