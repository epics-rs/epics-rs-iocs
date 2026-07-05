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
epicsEnvSet("CTRL",   "0")
epicsEnvSet("IPPORT", "acsAsyn")
epicsEnvSet("HOST",   "192.168.1.100:701")
epicsEnvSet("NUMAXES", "1")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# SPiiPlus framing: ACSPL+ replies are terminated by CR ("\r"). The driver owns
# the output terminator ("\r"), so only the input EOS is set here. Do NOT set an
# output EOS (it would double-terminate).
asynOctetSetInputEos("$(IPPORT)", 0, "\r")

# ---- SPiiPlus controller ----
# AcsMotionConfig(card, asynPort, numAxes, [virtualAxisList], [homingMethod],
#                 [movingPollMs], [idlePollMs]). homingMethod default 1
#                 (limit+index); virtualAxisList is comma/space-separated
#                 0-based indices.
AcsMotionConfig("$(CTRL)", "$(IPPORT)", $(NUMAXES), "", 1, 100, 1000)

# One motor record for axis 0. DTYP = ACSMOTION_$(CTRL)_$(AXIS).
dbLoadRecords("db/acsmotion.template", "P=$(P),M=m1,CTRL=$(CTRL),AXIS=0")

iocInit()

# Example:
#   dbl
#   camonitor acs:m1 acs:m1.RBV
#   caput acs:m1 10.0
