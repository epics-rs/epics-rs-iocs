#============================================================
# st.cmd — ACS SPiiPlus motor IOC startup script (motorAcsMotion)
#
# Usage:
#   cargo run -p acsmotion-ioc -- st.cmd
#
# Requires an ACS SPiiPlus controller reachable over TCP (default control port
# 701) or serial. One controller drives numAxes axes (0-based).
#============================================================

epicsEnvSet("P",      "acs:")
epicsEnvSet("CTRL",   "ACS1")
epicsEnvSet("IPPORT", "acsAsyn")
epicsEnvSet("HOST",   "192.168.1.100:701")
epicsEnvSet("NUMAXES", "1")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# ---- SPiiPlus controller ----
# AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod, idlePollPeriod,
#                 virtualAxisList). Poll periods in seconds; virtualAxisList is
#                 comma-separated 0-based indices.
AcsMotionConfig("$(CTRL)", "$(IPPORT)", $(NUMAXES), 0.1, 1.0, "")
# Profile moves: SPiiPlusCreateProfile(acsPort, maxPoints, maxPulses), as
# ACS_Motion.iocsh always calls it.
SPiiPlusCreateProfile("$(CTRL)", 2000, 2000)

# One motor record for axis 0.
dbLoadRecords("$(ACSMOTION_IOC)/db/acsmotion.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
# Its homing parameters; homingMethod defaults to None, as in C.
dbLoadRecords("$(ACSMOTION_IOC)/db/SPiiPlusHoming.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0,TIMEOUT=1")

# The profile move PVs, as AcsMotion.substitutions loads them.
dbLoadRecords("$(ACSMOTION_IOC)/db/profileMoveController.template", "P=$(P),R=pm1:,PORT=$(CTRL),NAXES=$(NUMAXES),NPOINTS=2000,NPULSES=2000,TIMEOUT=2.0")
dbLoadRecords("$(ACSMOTION_IOC)/db/profileMoveAxis.template", "P=$(P),R=pm1:,M=1,NPOINTS=2000,NREADBACK=2000,PORT=$(CTRL),ADDR=0,TIMEOUT=2.0,PREC=4")
dbLoadRecords("$(ACSMOTION_IOC)/db/SPiiPlusProfileMoveController.db", "P=$(P),R=pm1:,PORT=$(CTRL),NPULSES=2000,TIMEOUT=2.0,PREC=4")
dbLoadRecords("$(ACSMOTION_IOC)/db/SPiiPlusPEG.db", "P=$(P),R=pm1:,PORT=$(CTRL),NAXES=$(NUMAXES),NPOINTS=2000,NPULSES=2000,TIMEOUT=2.0")

iocInit()

# Example:
#   dbl
#   camonitor acs:m1 acs:m1.RBV
#   caput acs:m1 10.0
