#============================================================
# st-m1.cmd — drvMotorSim.c ("Motor Simulation") startup script
#
# Usage:
#   cargo run -p motorsim-ioc -- st-m1.cmd
#============================================================

epicsEnvSet("P", "motorSim:")

# motorSimCreate(card, signal, lowLimit, hiLimit, home, nCards, nSignals, startPosn)
motorSimCreate(0, 0, -22000, 42000, 10000, 1, 3, 5000)

dbLoadRecords("$(MOTORSIM_IOC)/db/motorsim-m1.template", "P=$(P),M=test0,C=0,S=0")
dbLoadRecords("$(MOTORSIM_IOC)/db/motorsim-m1.template", "P=$(P),M=test1,C=0,S=1")

iocInit()
