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

# motorUtil's records and the axis's motor.db, as C's st.cmd and IM483.cmd
# load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(IMS_IOC)/db/IM483.substitutions", "P=$(P),C=0")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")
