#============================================================
# st.cmd — Aerotech Ensemble motor IOC startup script
#
# Usage:
#   cargo run -p aerotech-ioc -- st.cmd
#
# Requires an Aerotech Ensemble controller reachable over TCP (or swap in the
# serial port line below). Homing uses the vendor HomeAsync.bcx program (task
# 5), which must be loaded on the controller.
#============================================================

epicsEnvSet("P",      "aerotech:")
epicsEnvSet("CTRL",   "AeroE1")
epicsEnvSet("IPPORT", "ensAsyn")
epicsEnvSet("HOST",   "192.168.1.40:8000")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# ---- Ensemble controller ----
EnsembleAsynSetup(1)

# EnsembleAsynConfig(card, portName, asynAddress, numAxes, movingPollPeriod,
#                    idlePollPeriod), periods in ms. Probes axes 0.. and keeps
# the first numAxes that exist; drvAsynMotorConfigure makes them port $(CTRL),
# addresses 0.. in the order found.
EnsembleAsynConfig(0, "$(IPPORT)", 0, 1, 100, 1000)
drvAsynMotorConfigure("$(CTRL)", "motorEnsemble", 0, 1)

dbLoadRecords("$(AEROTECH_IOC)/db/aerotech.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")

iocInit()

# Example:
#   dbl
#   camonitor aerotech:m1 aerotech:m1.RBV
#   caput aerotech:m1 10
