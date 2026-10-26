#============================================================
# st.moco.cmd — Micos MoCo dc model-1 (drvMicos.cc) startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.moco.cmd
#
# C's iocMicos st.cmd with MoCo.cmd alone (see st.cmd).
#============================================================

epicsEnvSet("P",    "micos:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

# motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "19200")

dbLoadTemplate("$(MICOS_IOC)/db/MoCo.substitutions", "P=$(P)")

# MicosSetup(num_cards, num_channels, scan_rate), MicosConfig(card, asyn port);
# every one of the num_channels MoCo addresses must answer.
MicosSetup(1, 2, 10)
MicosConfig(0, "$(PORT)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")
