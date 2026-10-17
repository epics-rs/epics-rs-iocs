#============================================================
# st-mvp2001.cmd — MicroMo MVP 2001 model-1 (drvMVP2001.cc) startup script
#
# Usage:
#   cargo run -p micromo-ioc -- st-mvp2001.cmd
#============================================================

epicsEnvSet("P",    "MVP:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# MicroMoSetup(num_chains, scan_rate), MicroMoConfig(chain, asyn port).
# Config probes the chain for controllers at addresses 1, 2, ...
MicroMoSetup(1, 10)
MicroMoConfig(0, "$(PORT)")

dbLoadRecords("$(MICROMO_IOC)/db/mvp2001-m1.template", "P=$(P),M=m1,C=0,S=0")

iocInit()
