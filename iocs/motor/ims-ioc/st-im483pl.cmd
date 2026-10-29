#============================================================
# st-im483pl.cmd — IMS IM483PL model-1 (drvIM483PL.cc) startup script
#
# Usage:
#   cargo run -p ims-ioc -- st-im483pl.cmd
#============================================================

epicsEnvSet("P",    "ims:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# IM483PLSetup(num_cards, scan_rate), IM483PLConfig(card, asyn port)
IM483PLSetup(1, 10)
IM483PLConfig(0, "$(PORT)")

dbLoadRecords("$(IMS_IOC)/db/im483-m1.template", "P=$(P),M=m0,DTYP=IM483PL,C=0,S=0")
dbLoadRecords("$(IMS_IOC)/db/im483-m1.template", "P=$(P),M=m1,DTYP=IM483PL,C=0,S=1")

iocInit()
