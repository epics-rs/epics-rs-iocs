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
epicsEnvSet("IPPORT", "omsAsyn")
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
omsMAXnetConfig("$(PORT)", 8, "$(IPPORT)", 200, 2000, "")

dbLoadRecords("$(OMS_ASYN_IOC)/db/oms.template", "P=$(P),M=m1,PORT=$(PORT),ADDR=0")

iocInit()

# Example:
#   dbl
#   camonitor oms:m1 oms:m1.RBV
#   caput oms:m1 1000
