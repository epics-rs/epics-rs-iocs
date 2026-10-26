#============================================================
# st.cmd — Aerotech Ensemble and Soloist motor IOC startup script
#
# Usage:
#   cargo run -p aerotech-ioc -- st.cmd
#
# Requires an Aerotech Ensemble and a Soloist reachable over TCP (or swap in
# the serial port lines below). Ensemble homing uses the vendor HomeAsync.bcx
# program (task 5), which must be loaded on the controller.
#============================================================

epicsEnvSet("P",      "aerotech:")
epicsEnvSet("CTRL",   "AeroE1")
epicsEnvSet("IPPORT", "EnsemblePort")
epicsEnvSet("HOST",   "192.168.1.40:8000")
epicsEnvSet("SPORT",  "SoloistPort")
epicsEnvSet("SHOST",  "192.168.1.41:8000")

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

# ---- Soloist controller (model 1) ----
drvAsynIPPortConfigure("$(SPORT)", "$(SHOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(SPORT)", "/dev/ttyUSB1", 0, 0, 0)
# The driver sets no EOS; the Soloist's AsciiCmdEOSChar is "\n".
asynOctetSetInputEos("$(SPORT)", 0, "\n")
asynOctetSetOutputEos("$(SPORT)", 0, "\n")

# SoloistSetup(num_cards, scan_rate), SoloistConfig(card, asyn port, addr).
SoloistSetup(1, 10)
SoloistConfig(0, "$(SPORT)", 0)

# The Ensemble's asyn_motor.db, the Soloist's motor.db and motorUtil's
# records, as C's st.cmd, Ensemble.cmd and Soloist.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(AEROTECH_IOC)/db/Ensemble.substitutions", "P=$(P),PORT=$(CTRL)")
dbLoadTemplate("$(AEROTECH_IOC)/db/Soloist.substitutions", "P=$(P)")

# EnsembleTrajectoryScan.st's records, for the seq line below (NELM at most
# 1000, NPULSE at most 8000):
#   dbLoadRecords("$(AEROTECH_IOC)/db/trajectoryScan.db", "P=$(P),R=traj1:,NAXES=1,NELM=1000,NPULSE=1000")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# EnsembleTrajectoryScan.st runs PVT trajectories through doCommand.bcx
# (doCommand.ab 1.2 or later), which must be loaded on the controller:
#   seq &EnsembleTrajectoryScan, "P=$(P),R=traj1:,M1=m1,PORT=$(IPPORT)"

# Example:
#   dbl
#   camonitor aerotech:m1 aerotech:m1.RBV
#   caput aerotech:m1 .1
