#============================================================
# st.cmd — Parker OEM750 motor IOC startup script
#
# Usage:
#   cargo run -p parker-ioc -- st.cmd
#
# Requires a Parker OEM750 controller reachable over TCP at the host:port below.
#============================================================

epicsEnvSet("P",      "OEM:")
epicsEnvSet("CTRL",   "OEM")
epicsEnvSet("IPPORT", "PORT")
epicsEnvSet("HOST",   "192.168.1.220:5000")
epicsEnvSet("NAXES",  "1")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ---- OEM750 controller ----
# OEMCreateController(portName, OEMPortName, numAxes, movingPollPeriod, idlePollPeriod).
OEMCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 20, 1000)

# The axis's basic_asyn_motor.db and OEMSupport.template, as C's st.cmd.oem
# loads them.
dbLoadTemplate("$(PARKER_IOC)/db/motor.substitutions.oem", "P=$(P),PORT=$(CTRL)")

# The octet port's asyn record, as C's st.cmd.oem loads it.
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=asyn1,PORT=$(IPPORT),ADDR=0,OMAX=256,IMAX=256")

iocInit()

# Example:
#   dbl
#   camonitor OEM:m1 OEM:m1.RBV
#   caput OEM:m1 10000
