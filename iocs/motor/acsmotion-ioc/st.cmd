#============================================================
# st.cmd — ACS SPiiPlus motor IOC startup script (motorAcsMotion)
#
# Usage:
#   cargo run -p acsmotion-ioc -- st.cmd
#
# C's iocAcsMotion st.cmd with AcsMotion.cmd and AcsMotionAuxIO.cmd: one
# ACS SPiiPlus controller reachable over TCP (control port 701) drives
# numAxes axes (0-based), its auxiliary I/O read on a second connection.
#============================================================

epicsEnvSet("P",       "acs:")
epicsEnvSet("CTRL",    "ACS1")
epicsEnvSet("IPPORT",  "ACS1_ETH")
epicsEnvSet("AUXIO",   "ACS1_IO")
epicsEnvSet("AUXPORT", "ACS1_IO_ETH")
epicsEnvSet("HOST",    "192.168.1.100:701")
epicsEnvSet("NUMAXES", "8")

# motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- SPiiPlus controller, as C's ACS_Motion_tcp.iocsh ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
# AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod, idlePollPeriod,
#                 virtualAxisList). Poll periods in seconds; virtualAxisList is
#                 comma-separated 0-based indices.
AcsMotionConfig("$(CTRL)", "$(IPPORT)", $(NUMAXES), 0.1, 1.0, "")
# Profile moves: SPiiPlusCreateProfile(acsPort, maxPoints, maxPulses), as
# ACS_Motion.iocsh always calls it.
SPiiPlusCreateProfile("$(CTRL)", 2000, 2000)

# The motor records, profile move and SPiiPlus per-axis PVs, and an asyn
# record for debugging, as C's AcsMotion.cmd loads them.
dbLoadTemplate("$(ACSMOTION_IOC)/db/AcsMotion.substitutions", "P=$(P),PORT=$(CTRL)")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=asyn,PORT=$(IPPORT),ADDR=0,OMAX=256,IMAX=256")

# ---- auxiliary I/O, as C's ACS_Motion_AuxIO_tcp.iocsh ----
drvAsynIPPortConfigure("$(AUXPORT)", "$(HOST)", 0, 0, 0)
# AcsMotionAuxIOConfig(auxIOPortName, asynPortName, numChannels, pollPeriod)
AcsMotionAuxIOConfig("$(AUXIO)", "$(AUXPORT)", 32, 0.1)

# Its records and asyn record, as C's AcsMotionAuxIO.cmd loads them.
dbLoadTemplate("$(ACSMOTION_IOC)/db/AcsMotionAuxIO.substitutions", "P=$(P),PORT=$(AUXIO)")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=io:asyn,PORT=$(AUXPORT),ADDR=0,OMAX=256,IMAX=256")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor acs:m1 acs:m1.RBV
#   caput acs:m1 10.0
