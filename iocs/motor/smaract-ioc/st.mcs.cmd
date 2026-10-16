#============================================================
# st.mcs.cmd — SmarAct MCS (RS-232) motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.mcs.cmd
#
# The MCS is commonly reached over a terminal server (drvAsynIPPort). Use a RAW
# connection, not TELNET — the driver does not slurp telnet negotiation bytes.
# The controller is created first, then one axis per channel.
#============================================================

epicsEnvSet("P",       "MCS:")
epicsEnvSet("CTRL",    "MCS_1")
epicsEnvSet("IPPORT",  "MCS")
epicsEnvSet("HOST",    "192.168.1.210:5000")

# ---- asyn IP octet port (RAW terminal server) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ---- MCS controller + axes ----
# smarActMCSCreateController(motorPortName, ioPortName, numAxes,
#                            movingPollPeriod, idlePollPeriod, disableSpeed),
# periods in seconds.
smarActMCSCreateController("$(CTRL)", "$(IPPORT)", 3, 0.05, 1.0, 0)

# smarActMCSCreateAxis(controllerPortName, axisNumber, channel): records reach
# axisNumber as @asyn($(CTRL),axisNumber); channel is the controller channel.
smarActMCSCreateAxis("$(CTRL)", 0, 0)
smarActMCSCreateAxis("$(CTRL)", 1, 1)
smarActMCSCreateAxis("$(CTRL)", 2, 2)

# One motor record per axis. The driver reports the controller's raw position in
# nanometres (linear) / micro-degrees (rotary) with no scaling, so MRES = 1e-6
# reads the record in millimetres (linear) / degrees (rotary).
dbLoadRecords("$(SMARACT_IOC)/db/mcs.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
dbLoadRecords("$(SMARACT_IOC)/db/mcs.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=1")
dbLoadRecords("$(SMARACT_IOC)/db/mcs.template", "P=$(P),M=m3,PORT=$(CTRL),ADDR=2")

iocInit()

# Example:
#   dbl
#   camonitor MCS:m1 MCS:m1.RBV
#   caput MCS:m1 1.0
