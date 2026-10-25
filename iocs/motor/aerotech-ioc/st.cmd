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

# EnsembleTrajectoryScan.st's records, for the seq line below (NELM at most
# 1000, NPULSE at most 8000):
#   dbLoadRecords("$(AEROTECH_IOC)/db/trajectoryScan.db", "P=$(P),R=traj1:,NAXES=1,NELM=1000,NPULSE=1000")

iocInit()

# EnsembleTrajectoryScan.st runs PVT trajectories through doCommand.bcx
# (doCommand.ab 1.2 or later), which must be loaded on the controller:
#   seq &EnsembleTrajectoryScan, "P=$(P),R=traj1:,M1=m1,PORT=$(IPPORT)"

# Example:
#   dbl
#   camonitor aerotech:m1 aerotech:m1.RBV
#   caput aerotech:m1 10
