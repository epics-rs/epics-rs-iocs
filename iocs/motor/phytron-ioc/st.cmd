#============================================================
# st.cmd — Phytron phyMOTION (MCM) motor IOC startup script
#
# Usage:
#   cargo run -p phytron-ioc -- st.cmd
#
# Requires a Phytron phyMOTION controller reachable over TCP (or swap in the
# serial port line below).
#============================================================

epicsEnvSet("P",      "phytron:")
epicsEnvSet("CTRL",   "phyMotionPort")
epicsEnvSet("IPPORT", "testRemote")
epicsEnvSet("HOST",   "192.168.1.20:22222")

# ---- asyn octet port (Ethernet; use the serial line for USB/RS-232) ----
# noProcessEos=1 as in C's iocBoot: the driver frames STX..ETX both ways and
# needs the ETX in the reply, which an input EOS of "\x03" would strip.
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 1)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 1)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# ---- phyMOTION controller ----
# phytronCreateController(controllerName, asynPort, movingPollPeriod,
#                         idlePollPeriod, timeout, noResetAtBoot), all in ms.
# noResetAtBoot unset resets the controller at boot (a reset waits ~5 s, then
# polls the controller for up to 120 s); 1 skips the reset.
phytronCreateController("$(CTRL)", "$(IPPORT)", 10, 10, 1000)
asynSetOption("$(CTRL)", 0, "pollMethod", "parallel")
asynSetOption("$(CTRL)", 0, "fakeHomedEnable", "true")
asynSetOption("$(CTRL)", 0, "clearAxisStatus", "true")

# One axis per I1AM01 module.index: phytronCreateAxis(controllerName, module,
# index). Load one motor record per axis with ADDR = module*10 + index.
phytronCreateAxis("$(CTRL)", 1, 1)
phytronCreateAxis("$(CTRL)", 2, 1)
phytronCreateAxis("$(CTRL)", 3, 1)

# I/O channels as C's iocPhytron: an AIOM01 on card 1 and a DIOM01 on card 1.
# phytronCreateAnalog/phytronCreateDigital(ioName, controllerName, card,
#                                          channel, initCommands)
phytronCreateAnalog("phyA1", "$(CTRL)", 1, 1, "")
phytronCreateAnalog("phyA2", "$(CTRL)", 1, 2, "")
phytronCreateAnalog("phyA3", "$(CTRL)", 1, 3, "")
phytronCreateAnalog("phyA4", "$(CTRL)", 1, 4, "")
phytronCreateDigital("phyD1", "$(CTRL)", 1, 1, "")

# Phytron_motor.db with the m1 and m2 rows of the phyMOTION (MRES) block of
# C's motor.substitutions.phytron; axis 31 has no row there.
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_motor.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=11,DESC=motor 1,EGU=mm,MRES=0.000625,ERES=0.00005,UEIP=No,DIR=Pos,VELO=2.5,VBAS=1,VMAX=25,ACCL=.5,BDST=0,BVEL=25,BACC=.2,PREC=5,RDBD=0,RTRY=0,OFF=-0.49,DHLM=1440,DLLM=-1440,MDEL=0.001,INIT=")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_motor.db", "P=$(P),M=m2,PORT=$(CTRL),ADDR=21,DESC=motor 2,EGU=mm,MRES=0.000625,ERES=0.00005,UEIP=No,DIR=Pos,VELO=2.5,VBAS=1,VMAX=25,ACCL=.5,BDST=0,BVEL=25,BACC=.2,PREC=5,RDBD=0,RTRY=0,OFF=-0.49,DHLM=1440,DLLM=-1440,MDEL=0.001,INIT=")

# The modules' aux records, with C's motor.substitutions.phytron values. The
# axis records' P must be the controller records' P: an axis re-initialises
# when the controller's $(P)-RESET_ processes. C's st.cmd.phytron loads that
# file's active MCC-2 blocks, so its phyMOTION gets Phytron_MCC.db records
# and an m2 at ADDR 12, none of its axes 11, 21, 31; these are the file's
# commented phyMOTION blocks.
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_MCM01.db", "P=$(P),PORT=$(CTRL),ADDR=0,TIMEOUT=10")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_I1AM01.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=11,SCAN=1 second,TIMEOUT=1,INIT=YES,HOMING=0,MODE=1,POS_OFFSET=0,NEG_OFFSET=0,INIT_TIME=20,POS_TIME=20,BOOST=0,SWITCH_TYP=0,PWR_STAGE=1,ENC_TYP=1,ENC_SFI=0,ENC_DIR=1,STOP_CURR=20,RUN_CURR=80,BOOST_CURR=100,CURRENT_DELAY=20,STEP_RES=0,PS_MON=1")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_I1AM01.db", "P=$(P),M=m2,PORT=$(CTRL),ADDR=21,SCAN=1 second,TIMEOUT=1,INIT=YES,HOMING=0,MODE=1,POS_OFFSET=0,NEG_OFFSET=0,INIT_TIME=20,POS_TIME=20,BOOST=0,SWITCH_TYP=0,PWR_STAGE=1,ENC_TYP=1,ENC_SFI=0,ENC_DIR=1,STOP_CURR=20,RUN_CURR=80,BOOST_CURR=100,CURRENT_DELAY=20,STEP_RES=0,PS_MON=1")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_AIO.db", "P=$(P)phyA1,PORT=phyA1")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_AIO.db", "P=$(P)phyA2,PORT=phyA2")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_AIO.db", "P=$(P)phyA3,PORT=phyA3")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_AIO.db", "P=$(P)phyA4,PORT=phyA4")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_DIO.db", "P=$(P)phyD1,PORT=phyD1")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_CMD.db", "P=$(P)phy,PORT=$(CTRL)")

iocInit()

# Example:
#   dbl
#   camonitor phytron:m1 phytron:m1.RBV
#   caput phytron:m1 1000
