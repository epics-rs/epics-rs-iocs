#============================================================
# st.cmd — Parker OEM750 motor IOC startup script
#
# Usage:
#   cargo run -p parker-ioc -- st.cmd
#
# Requires a Parker OEM750 controller reachable over TCP at the host:port below.
#============================================================

epicsEnvSet("P",      "OEM:")
epicsEnvSet("CTRL",   "PARKER1")
epicsEnvSet("IPPORT", "OEM")
epicsEnvSet("HOST",   "192.168.1.220:5000")
epicsEnvSet("NAXES",  "1")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ---- OEM750 controller ----
# OEMCreateController(portName, OEMPortName, numAxes, movingPollPeriod, idlePollPeriod).
OEMCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 1000)

# One motor record per axis (OUT @asyn($(CTRL),axis)) and its OEM_RESOLUTION.
dbLoadRecords("$(PARKER_IOC)/db/oem.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")

iocInit()

# Example:
#   dbl
#   camonitor OEM:m1 OEM:m1.RBV
#   caput OEM:m1 10000
