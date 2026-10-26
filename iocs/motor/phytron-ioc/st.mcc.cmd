#============================================================
# st.mcc.cmd — Phytron MCC-1 / MCC-2 motor IOC startup script
#
# Usage:
#   cargo run -p phytron-ioc -- st.mcc.cmd
#
# Requires a Phytron MCC controller reachable over serial (or TCP).
#============================================================

epicsEnvSet("P",      "phytron:")
epicsEnvSet("CTRL",   "phyMotionPort")
epicsEnvSet("IPPORT", "testRemote")
epicsEnvSet("HOST",   "192.168.1.21:22222")
epicsEnvSet("ADDR",   "0")

# ---- asyn octet port (serial is typical for the MCC; TCP also works) ----
# noProcessEos=1 as in C's iocBoot: the driver frames STX..ETX both ways and
# needs the ETX in the reply, which an input EOS of "\x03" would strip.
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 1)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 1)
#asynSetOption("$(IPPORT)", -1, "baud", "57600")

# ---- MCC controller ----
# phytronCreateMCC(controllerName, asynPort, address, movingPollPeriod,
#                  idlePollPeriod, timeout, noResetAtBoot), all in ms.
phytronCreateMCC("$(CTRL)", "$(IPPORT)", $(ADDR), 1000, 1000, 1000, 1)
asynSetOption("$(CTRL)", 0, "fakeHomedEnable", "true")

# phytronCreateAxis(controllerName, module, index): module 1 as C's
# st.cmd.mcc; an MCC addresses the axis by the index digit alone.
phytronCreateAxis("$(CTRL)", 1, 1)
phytronCreateAxis("$(CTRL)", 1, 2)

# The MCC's fixed I/O as C's iocPhytron: two analog inputs and the digital
# channel. phytronCreateAnalog/phytronCreateDigital(ioName, controllerName,
# card, channel, initCommands)
phytronCreateAnalog("phyA1", "$(CTRL)", 1, 1, "")
phytronCreateAnalog("phyA2", "$(CTRL)", 1, 2, "")
phytronCreateDigital("phyD1", "$(CTRL)", 1, 1, "")

# Each axis's Phytron_motor.db, with the rows of the MCC-2 (SREV/UREV) block
# of C's motor.substitutions.phytron.
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_motor.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=11,DESC=motor 1,EGU=Deg,SREV=400,UREV=0.25,ERES=0.00005,UEIP=No,DIR=Pos,VELO=360,VBAS=180,VMAX=720,ACCL=.5,BDST=0,BVEL=180,BACC=.2,PREC=5,RDBD=0,RTRY=0,DHLM=1440,DLLM=-1440,OFF=-0.49,MDEL=0.001,INIT=")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_motor.db", "P=$(P),M=m2,PORT=$(CTRL),ADDR=12,DESC=motor 2,EGU=Deg,SREV=400,UREV=0.25,ERES=0.00005,UEIP=No,DIR=Pos,VELO=360,VBAS=180,VMAX=720,ACCL=.5,BDST=0,BVEL=180,BACC=.2,PREC=5,RDBD=0,RTRY=0,DHLM=1440,DLLM=-1440,OFF=-0.49,MDEL=0.001,INIT=")

# Each axis's aux records, with C's motor.substitutions.phytron values. Like
# C's MCC example this loads no Phytron_MCM01.db, so the axis link to
# $(P)-RESET_ stays unresolved. STEP_RES is STEP-RES_SET's state index: C's
# "1/16" makes INIT-AXIS-2's DOL7 a link to a PV of that name; 5 is 1/16.
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_MCC.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=11,SCAN=5 second,TIMEOUT=1,INIT=YES,HOMING=0,MODE=1,POS_OFFSET=0,NEG_OFFSET=0,INIT_TIME=20,POS_TIME=20,BOOST=0,SWITCH_TYP=0,PWR_STAGE=1,ENC_TYP=0,ENC_SFI=0,ENC_DIR=1,STOP_CURR=20,RUN_CURR=80,BOOST_CURR=100,CURRENT_DELAY=20,STEP_RES=5")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_MCC.db", "P=$(P),M=m2,PORT=$(CTRL),ADDR=12,SCAN=5 second,TIMEOUT=1,INIT=YES,HOMING=0,MODE=1,POS_OFFSET=0,NEG_OFFSET=0,INIT_TIME=20,POS_TIME=20,BOOST=0,SWITCH_TYP=0,PWR_STAGE=1,ENC_TYP=0,ENC_SFI=0,ENC_DIR=1,STOP_CURR=20,RUN_CURR=80,BOOST_CURR=100,CURRENT_DELAY=20,STEP_RES=5")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_AIO_MCC.db", "P=$(P)phyA1,PORT=phyA1")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_AIO_MCC.db", "P=$(P)phyA2,PORT=phyA2")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_DIO.db", "P=$(P)phyD1,PORT=phyD1")
# An MCC-1 also configures the digital directions:
#dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_DIO_MCC1.db", "P=$(P)phyD1,PORT=phyD1")
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_CMD.db", "P=$(P)phy,PORT=$(CTRL)")

iocInit()

# Example:
#   dbl
#   camonitor phytron:m1 phytron:m1.RBV
#   caput phytron:m1 1000
