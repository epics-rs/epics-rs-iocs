#============================================================
# st.mcc.cmd — Phytron MCC-1 / MCC-2 motor IOC startup script
#
# Usage:
#   cargo run -p phytron-ioc -- st.mcc.cmd
#
# Requires a Phytron MCC controller reachable over serial (or TCP).
#============================================================

epicsEnvSet("P",      "phytron:")
epicsEnvSet("CTRL",   "mccPort")
epicsEnvSet("IPPORT", "mccAsyn")
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
phytronCreateMCC("$(CTRL)", "$(IPPORT)", $(ADDR), 100, 1000, 1000, 1)

# MCC axes are single-digit 1..8: phytronCreateAxis(controllerName, module=0,
# index). The axis command prefix is the index digit.
phytronCreateAxis("$(CTRL)", 0, 1)

# The MCC's fixed I/O as C's iocPhytron: two analog inputs and the digital
# channel. phytronCreateAnalog/phytronCreateDigital(ioName, controllerName,
# card, channel, initCommands)
phytronCreateAnalog("phyA1", "$(CTRL)", 1, 1, "")
phytronCreateAnalog("phyA2", "$(CTRL)", 1, 2, "")
phytronCreateDigital("phyD1", "$(CTRL)", 1, 1, "")

# The axis's Phytron_motor.db, as C's motor.substitutions.phytron loads it;
# the values are the motor record this IOC had before, in controller steps.
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_motor.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=1,DESC=,DIR=Pos,VELO=1000,VBAS=0,VMAX=0,ACCL=0.2,BDST=0,BVEL=0,BACC=0.5,MRES=1,ERES=0,UEIP=No,PREC=0,EGU=steps,DHLM=100000,DLLM=-100000,INIT=")

# The module's aux records, with C's motor.substitutions.phytron values. Like
# C's MCC example this loads no Phytron_MCM01.db, so the axis link to
# $(P)-RESET_ stays unresolved. STEP_RES is STEP-RES_SET's state index: C's
# "1/16" makes INIT-AXIS-2's DOL7 a link to a PV of that name; 5 is 1/16.
dbLoadRecords("$(PHYTRON_IOC)/db/Phytron_MCC.db", "P=$(P),M=m1,PORT=$(CTRL),ADDR=1,SCAN=5 second,TIMEOUT=1,INIT=YES,HOMING=0,MODE=1,POS_OFFSET=0,NEG_OFFSET=0,INIT_TIME=20,POS_TIME=20,BOOST=0,SWITCH_TYP=0,PWR_STAGE=1,ENC_TYP=0,ENC_SFI=0,ENC_DIR=1,STOP_CURR=20,RUN_CURR=80,BOOST_CURR=100,CURRENT_DELAY=20,STEP_RES=5")
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
