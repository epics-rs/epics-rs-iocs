#============================================================
# st-im483sm.cmd — IMS IM483SM model-1 (drvIM483SM.cc) startup script
#
# Usage:
#   cargo run -p ims-ioc -- st-im483sm.cmd
#============================================================

epicsEnvSet("P",    "ims:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# IM483SMSetup(num_cards, scan_rate), IM483SMConfig(card, asyn port);
# one axis per card.
IM483SMSetup(1, 10)
IM483SMConfig(0, "$(PORT)")

dbLoadRecords("$(IMS_IOC)/db/im483-m1.template", "P=$(P),M=m0,DTYP=IM483SM,C=0,S=0")

iocInit()
