#============================================================
# st.cmd — Micos SMC corvus motor IOC startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.cmd
#
# C's iocMicos st.cmd with motor.cmd.SMCcorvus. Requires an SMC corvus
# controller reachable over TCP (or swap in the serial port line below).
#
# C's st.cmd loads MoCo.cmd, motor.cmd.SMChydra and motor.cmd.SMCcorvus
# together, but MoCo's m1-m2 and hydra's m3 are corvus's record names, so
# corvus's rows silently redefine those motors; here each controller has its
# own script (st.moco.cmd, st.hydra.cmd).
#============================================================

epicsEnvSet("P",      "micos:")
epicsEnvSet("CTRL",   "Corvus1")
epicsEnvSet("IPPORT", "corvusPort")
epicsEnvSet("HOST",   "192.168.1.170:2103")
epicsEnvSet("NAXES",  "3")

# motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- asyn octet port (Ethernet; use the serial line for RS-232) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)

# Port EOS as the C example sets it (motor.cmd.SMCcorvus:14-15);
# the driver sends bare commands.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r\n")

dbLoadTemplate("$(MICOS_IOC)/db/motor.substitutions.SMCcorvus", "P=$(P),PORT=$(CTRL)")

# ---- corvus controller ----
# SMCcorvusCreateController(portName, corvusPort, numAxes, movingPollPeriod, idlePollPeriod).
SMCcorvusCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 500)

# Include an asyn record for each motor to make it easy to enable diagnostic messages
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=corvusComm,PORT=$(IPPORT),ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=corvusAsyn1,PORT=$(CTRL),ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=corvusAsyn2,PORT=$(CTRL),ADDR=1,OMAX=256,IMAX=256")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
