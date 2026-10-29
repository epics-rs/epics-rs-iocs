#============================================================
# st.cmd — Pro-Dex OMS MAXnet motor IOC startup script
#
# Usage:
#   cargo run -p oms-asyn-ioc -- st.cmd
#
# Requires an OMS MAXnet controller reachable over TCP (MAXnet is Ethernet;
# swap in the serial line for an RS-232 unit).
#============================================================

epicsEnvSet("P",      "oms:")
epicsEnvSet("PORT",   "MAXnet1")
epicsEnvSet("IPPORT", "MAXNET")
epicsEnvSet("HOST",   "192.168.1.30:5001")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "38400")

# omsMAXnetConfig defaults the EOS to "\n\r" in, "\n" out where it is unset;
# set either here to override it.

# ---- MAXnet controller ----
# omsMAXnetConfig(portName, numAxes, serialPortName, movingPollPeriod,
#                 idlePollPeriod, initString). The axes are those the
# controller reports, up to numAxes; initString is sent verbatim at boot.
omsMAXnetConfig("$(PORT)", 8, "$(IPPORT)", 200, 2000, "AX LH PSO;")

# The axis's asyn_motor.db, as C's MAXnet.cmd loads it.
dbLoadTemplate("$(OMS_ASYN_IOC)/db/MAXnet.substitutions", "P=$(P),PORT=$(PORT)")

# motorUtil's records, as C's st.cmd loads them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor oms:m2 oms:m2.RBV
#   caput oms:m2 1
