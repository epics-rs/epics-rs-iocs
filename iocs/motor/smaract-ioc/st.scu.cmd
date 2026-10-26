#============================================================
# st.scu.cmd — SmarAct SCU motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.scu.cmd
#
# Two SCUs, as C's smaractscu.iocsh has, each on an FTDI USB serial port
# (drvAsynSerialPort). Each controller is created first, then one axis per
# channel (the axis number and the controller channel are mapped explicitly,
# matching the C smarActSCUCreateAxis command).
#============================================================

epicsEnvSet("P",       "SCU:")
epicsEnvSet("CTRL",    "SCU_1")
epicsEnvSet("SERPORT", "serial1")
epicsEnvSet("DEVICE",  "/dev/ttyUSB0")
epicsEnvSet("CTRL2",    "SCU_2")
epicsEnvSet("SERPORT2", "serial2")
epicsEnvSet("DEVICE2",  "/dev/ttyUSB1")

# ---- motorUtil (allstop & alldone) ----
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- asyn serial octet ports (FTDI USB), opened at 9600 8-N-1 ----
# Port EOS as the C example sets it (smaractscu.iocsh:7-8, :12-13);
# the driver sends bare commands.
drvAsynSerialPortConfigure("$(SERPORT)", "$(DEVICE)", 0, 0, 0)
asynOctetSetInputEos("$(SERPORT)", 0, "\n")
asynOctetSetOutputEos("$(SERPORT)", 0, "\n")
asynSetTraceIOMask("$(SERPORT)", 0, 2)
drvAsynSerialPortConfigure("$(SERPORT2)", "$(DEVICE2)", 0, 0, 0)
asynOctetSetInputEos("$(SERPORT2)", 0, "\n")
asynOctetSetOutputEos("$(SERPORT2)", 0, "\n")
asynSetTraceIOMask("$(SERPORT2)", 0, 2)

# ---- SCU controllers + axes ----
# smarActSCUCreateController(motorPortName, ioPortName, numAxes,
#                            movingPollPeriod, idlePollPeriod), periods in s.
smarActSCUCreateController("$(CTRL)", "$(SERPORT)", 3, 0.05, 1.0)
smarActSCUCreateController("$(CTRL2)", "$(SERPORT2)", 3, 0.05, 1.0)

# smarActSCUCreateAxis(controllerPortName, axisNumber, channel): records reach
# axisNumber as @asyn($(CTRL),axisNumber); channel is the controller channel.
smarActSCUCreateAxis("$(CTRL)", 0, 0)
smarActSCUCreateAxis("$(CTRL)", 1, 1)
smarActSCUCreateAxis("$(CTRL)", 2, 2)
smarActSCUCreateAxis("$(CTRL2)", 0, 0)
smarActSCUCreateAxis("$(CTRL2)", 1, 1)
smarActSCUCreateAxis("$(CTRL2)", 2, 2)

# One motor per axis with the rows of C's motor.substitutions.smaractscu.
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m1,DTYP=asynMotor,PORT=$(CTRL),ADDR=0,DESC=motor 1,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-3,PREC=3,DHLM=20000,DLLM=-20000,INIT=")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m2,DTYP=asynMotor,PORT=$(CTRL),ADDR=1,DESC=motor 2,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-3,PREC=3,DHLM=20000,DLLM=-20000,INIT=")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m3,DTYP=asynMotor,PORT=$(CTRL),ADDR=2,DESC=motor 3,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-3,PREC=3,DHLM=20000,DLLM=-20000,INIT=")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m4,DTYP=asynMotor,PORT=$(CTRL2),ADDR=0,DESC=motor 4,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-3,PREC=3,DHLM=20000,DLLM=-20000,INIT=")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m5,DTYP=asynMotor,PORT=$(CTRL2),ADDR=1,DESC=motor 5,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-3,PREC=3,DHLM=20000,DLLM=-20000,INIT=")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m6,DTYP=asynMotor,PORT=$(CTRL2),ADDR=2,DESC=motor 6,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-3,PREC=3,DHLM=20000,DLLM=-20000,INIT=")

# ---- autosave/restore machinery (C iocSmarAct AutoSave.cmd) ----
set_requestfile_path("$(SMARACT_IOC)", "")
set_requestfile_path("$(MOTOR)", "db")
set_savefile_path("$(SMARACT_IOC)", "autosave")
set_pass0_restoreFile("auto_positions.sav")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
# C's AutoSave.cmd names $(PREFIX), which its st.cmd never sets
save_restoreSet_status_prefix("$(P)")
dbLoadRecords("$(AUTOSAVE)/db/save_restoreStatus.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# save motor positions every five seconds
create_monitor_set("auto_positions.req", 5, "P=$(P)")
# save settings every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(P)")

# Example:
#   dbl
#   camonitor SCU:m1 SCU:m1.RBV
#   caput SCU:m1 10.0
