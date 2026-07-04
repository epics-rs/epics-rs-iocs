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
epicsEnvSet("CTRL",   "omsPort")
epicsEnvSet("IPPORT", "omsAsyn")
epicsEnvSet("HOST",   "192.168.1.30:5001")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "38400")

# OMS framing: the controller terminates replies with "\n\r"; the driver owns
# the "\n" output terminator, so only the input EOS is set here. Do NOT set an
# output EOS.
asynOctetSetInputEos("$(IPPORT)", 0, "\n\r")

# ---- MAXnet controller ----
# omsMAXnetConfig(controllerName, asynPort, [initString], [movingPollMs],
#                 [idlePollMs], [timeoutMs]). initString is sent verbatim at
# boot (empty here); poll periods default to 100/1000 ms.
omsMAXnetConfig("$(CTRL)", "$(IPPORT)", "", 100, 1000, 2000)

# One motor record per axis: omsCreateAxis(controllerName, axis), axis 0-based.
omsCreateAxis("$(CTRL)", 0)
dbLoadRecords("db/oms.template", "P=$(P),M=m1,CTRL=$(CTRL),AXIS=0")

iocInit()

# Example:
#   dbl
#   camonitor oms:m1 oms:m1.RBV
#   caput oms:m1 1000
