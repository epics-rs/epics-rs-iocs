#============================================================
# st.moco.cmd — Micos MoCo dc model-1 (drvMicos.cc) startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.moco.cmd
#============================================================

epicsEnvSet("P",    "micos:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "19200")

# MicosSetup(num_cards, num_channels, scan_rate), MicosConfig(card, asyn port);
# every one of the num_channels MoCo addresses must answer.
MicosSetup(1, 2, 10)
MicosConfig(0, "$(PORT)")

dbLoadRecords("$(MICOS_IOC)/db/moco.template", "P=$(P),M=m0,C=0,S=0")
dbLoadRecords("$(MICOS_IOC)/db/moco.template", "P=$(P),M=m1,C=0,S=1")

iocInit()
