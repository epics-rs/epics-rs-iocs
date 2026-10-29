#============================================================
# st.cmd — SmarAct MCS2 motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.cmd
#
# Requires a SmarAct MCS2 controller reachable over TCP at the host:port below.
# MCS2CreateController reads the controller serial number, so it must be
# reachable when the command runs.
#============================================================

epicsEnvSet("P",      "MCS2:")
epicsEnvSet("CTRL",   "MCS2_1")
epicsEnvSet("IPPORT", "MCS2")
epicsEnvSet("HOST",   "192.168.1.200:55551")
epicsEnvSet("NAXES",  "3")

# ---- motorUtil (allstop & alldone) ----
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P), R=asyn1, PORT=$(IPPORT), ADDR=0, OMAX=256, IMAX=256")

# ---- MCS2 controller ----
# MCS2CreateController(portName, MCS2PortName, numAxes, movingPollPeriod,
#                      idlePollPeriod, unusedMask), periods in ms.
MCS2CreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 100, 0)
asynSetTraceIOMask("$(CTRL)", -1, 0x8)

# One motor per channel, ADDR the channel, with the rows of C's
# motor.substitutions.smaractmcs2.
dbLoadRecords("$(MOTOR)/db/basic_asyn_motor.db", "P=$(P),M=m1,DTYP=asynMotor,PORT=$(CTRL),ADDR=0,DESC=motor 1,EGU=mm,DIR=Pos,VELO=.0,VBAS=.0,ACCL=.0,BDST=0,BVEL=1,BACC=.2,MRES=1.e-6,PREC=6,DHLM=0,DLLM=0,INIT=")
dbLoadRecords("$(SMARACT_IOC)/db/MCS2_Extra.db", "P=$(P),M=m1:,PORT=$(CTRL),ADDR=0,TIMEOUT=1.0")
dbLoadRecords("$(MOTOR)/db/basic_asyn_motor.db", "P=$(P),M=m2,DTYP=asynMotor,PORT=$(CTRL),ADDR=1,DESC=motor 2,EGU=mm,DIR=Pos,VELO=.0,VBAS=.0,ACCL=.0,BDST=0,BVEL=1,BACC=.2,MRES=1.e-6,PREC=6,DHLM=0,DLLM=0,INIT=")
dbLoadRecords("$(SMARACT_IOC)/db/MCS2_Extra.db", "P=$(P),M=m2:,PORT=$(CTRL),ADDR=1,TIMEOUT=1.0")
dbLoadRecords("$(MOTOR)/db/basic_asyn_motor.db", "P=$(P),M=m3,DTYP=asynMotor,PORT=$(CTRL),ADDR=2,DESC=motor 3,EGU=mm,DIR=Pos,VELO=.0,VBAS=.0,ACCL=.0,BDST=0,BVEL=1,BACC=.2,MRES=1.e-6,PREC=6,DHLM=0,DLLM=0,INIT=")
dbLoadRecords("$(SMARACT_IOC)/db/MCS2_Extra.db", "P=$(P),M=m3:,PORT=$(CTRL),ADDR=2,TIMEOUT=1.0")

# ---- autosave/restore machinery (C iocSmarAct AutoSave.cmd) ----
set_requestfile_path("$(SMARACT_IOC)", "")
set_requestfile_path("$(MOTOR)", "db")
set_savefile_path("$(SMARACT_IOC)", "autosave")
set_pass0_restoreFile("auto_positions.sav")
set_pass0_restoreFile("auto_settings_mcs2.sav")
set_pass1_restoreFile("auto_settings_mcs2.sav")
# C's AutoSave.cmd names $(PREFIX), which its st.cmd never sets
save_restoreSet_status_prefix("$(P)")
dbLoadRecords("$(AUTOSAVE)/db/save_restoreStatus.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# save motor positions every five seconds
create_monitor_set("auto_positions.req", 5, "P=$(P)")
# save settings every thirty seconds
create_monitor_set("auto_settings_mcs2.req", 30, "P=$(P)")

# Example:
#   dbl
#   camonitor MCS2:m1 MCS2:m1.RBV
#   caput MCS2:m1 1.0
