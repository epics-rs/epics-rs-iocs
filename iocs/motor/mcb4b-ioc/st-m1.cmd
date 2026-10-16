#============================================================
# st-m1.cmd — ACS MCB-4B model-1 (drvMCB4B.cc) startup script
#
# Usage:
#   cargo run -p mcb4b-ioc -- st-m1.cmd
#============================================================

epicsEnvSet("P",    "acs:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "19200")
asynSetOption("$(PORT)", -1, "bits", "8")
asynSetOption("$(PORT)", -1, "stop", "1")
asynSetOption("$(PORT)", -1, "parity", "none")
asynOctetSetInputEos( "$(PORT)", -1, "\r")
asynOctetSetOutputEos("$(PORT)", -1, "\r")

# MCB4BSetup(num_cards, scan_rate), MCB4BConfig(card, asyn port)
MCB4BSetup(1, 10)
MCB4BConfig(0, "$(PORT)")

dbLoadRecords("$(MCB4B_IOC)/db/mcb4b-m1.template", "P=$(P),M=m0,C=0,S=0")
dbLoadRecords("$(MCB4B_IOC)/db/mcb4b-m1.template", "P=$(P),M=m1,C=0,S=1")
dbLoadRecords("$(MCB4B_IOC)/db/mcb4b-m1.template", "P=$(P),M=m2,C=0,S=2")
dbLoadRecords("$(MCB4B_IOC)/db/mcb4b-m1.template", "P=$(P),M=m3,C=0,S=3")

iocInit()
