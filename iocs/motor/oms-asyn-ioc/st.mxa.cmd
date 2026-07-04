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
epicsEnvSet("CTRL",   "mxaPort")
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
# omsMXAConfig(controllerName, asynPort, [initString], [movingPollMs],
#              [idlePollMs], [timeoutMs]).
omsMXAConfig("$(CTRL)", "$(IPPORT)", "", 100, 1000, 2000)

omsCreateAxis("$(CTRL)", 0)
dbLoadRecords("db/oms.template", "P=$(P),M=m1,CTRL=$(CTRL),AXIS=0")

iocInit()
