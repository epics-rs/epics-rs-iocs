#============================================================
# st.taurus.cmd — Micos SMC Taurus motor IOC startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.taurus.cmd
#
# Requires an SMC Taurus controller reachable over TCP (or swap in the serial
# port line below).
#
# motorMicos ships no Taurus example; this is st.cmd's corvus layout for one
# axis.
#============================================================

epicsEnvSet("P",      "micos:")
epicsEnvSet("CTRL",   "MICOS1")
epicsEnvSet("IPPORT", "taurusPort")
epicsEnvSet("HOST",   "192.168.1.18:4001")
epicsEnvSet("NAXES",  "1")

# motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- asyn octet port (Ethernet; use the serial line for RS-232) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)

# Port EOS as C needs it: SMCTaurusDriver sets no EOS and sends bare
# commands, as the hydra sibling (motor.cmd.SMChydra:14-15).
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r\n")

dbLoadTemplate("$(MICOS_IOC)/db/motor.substitutions.SMCTaurus", "P=$(P),PORT=$(CTRL)")

# ---- Taurus controller ----
# SMCTaurusCreateController(portName, taurusPort, numAxes, movingPollPeriod, idlePollPeriod).
SMCTaurusCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 500)

# An asyn record for the port and the axis, as motor.cmd.SMCcorvus loads them.
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=taurusComm,PORT=$(IPPORT),ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=taurusAsyn1,PORT=$(CTRL),ADDR=0,OMAX=256,IMAX=256")

iocInit()

# motorUtil (allstop & alldone), as C's iocMicos st.cmd starts it
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
