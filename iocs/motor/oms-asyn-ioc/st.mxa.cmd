#============================================================
# st.mxa.cmd — Pro-Dex OMS MXA motor IOC startup script
#
# Usage:
#   cargo run -p oms-asyn-ioc -- st.mxa.cmd
#
# The MXA speaks the identical OMS ASCII protocol as the MAXnet; only the config
# command label (and firmware minimum) differ.
#============================================================

epicsEnvSet("P",      "oms:")
epicsEnvSet("PORT",   "MXA1")
epicsEnvSet("IPPORT", "mxaAsyn")
epicsEnvSet("HOST",   "192.168.1.31:5001")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "38400")

# OMS framing: replies end with "\n\r"; the driver owns the "\n" output
# terminator, so only the input EOS is set here. Do NOT set an output EOS.
asynOctetSetInputEos("$(IPPORT)", 0, "\n\r")

# ---- MXA controller ----
# omsMXAConfig(portName, numAxes, serialPortName, movingPollPeriod,
#              idlePollPeriod, initString).
omsMXAConfig("$(PORT)", 8, "$(IPPORT)", 200, 2000, "")

# Each axis's asyn_motor_model2.db, as C's MXA.cmd loads them.
dbLoadTemplate("$(OMS_ASYN_IOC)/db/MXA.substitutions", "P=$(P),PORT=$(PORT)")

# motorUtil's records, as C's st.cmd loads them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()
