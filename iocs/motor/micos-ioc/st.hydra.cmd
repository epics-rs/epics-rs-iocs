#============================================================
# st.hydra.cmd — Micos SMC hydra motor IOC startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.hydra.cmd
#
# C's iocMicos st.cmd with motor.cmd.SMChydra alone (see st.cmd). Requires
# an SMC hydra controller reachable over serial (or swap in the IP port
# line below).
#============================================================

epicsEnvSet("P",      "micos:")
epicsEnvSet("CTRL",   "Hydra1")
epicsEnvSet("IPPORT", "hydraPort")
epicsEnvSet("NAXES",  "2")

# motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- asyn octet port (serial; use the IP line for Ethernet) ----
drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)
#drvAsynIPPortConfigure("$(IPPORT)", "192.168.1.17:4001", 0, 0, 0)

# Port EOS as the C example sets it (motor.cmd.SMChydra:14-15);
# the driver sends bare commands.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r\n")

dbLoadTemplate("$(MICOS_IOC)/db/motor.substitutions.SMChydra", "P=$(P),PORT=$(CTRL)")

# ---- hydra controller ----
# SMChydraCreateController(portName, hydraPort, numAxes, movingPollPeriod, idlePollPeriod).
SMChydraCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 500)

# Include an asyn record for each motor to make it easy to enable diagnostic messages
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=hydraComm,PORT=$(IPPORT),ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=hydraAsyn1,PORT=$(CTRL),ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=hydraAsyn2,PORT=$(CTRL),ADDR=1,OMAX=256,IMAX=256")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor micos:m3 micos:m3.RBV
#   caput micos:m3 5.0
