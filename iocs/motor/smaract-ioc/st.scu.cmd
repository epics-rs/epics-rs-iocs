#============================================================
# st.scu.cmd — SmarAct SCU motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.scu.cmd
#
# The SCU communicates over an FTDI USB serial port (drvAsynSerialPort). The
# controller is created first, then one axis per channel (the axis number and
# the controller channel are mapped explicitly, matching the C
# smarActSCUCreateAxis command).
#============================================================

epicsEnvSet("P",       "SCU:")
epicsEnvSet("CTRL",    "SCU_1")
epicsEnvSet("SERPORT", "serial1")
epicsEnvSet("DEVICE",  "/dev/ttyUSB0")

# ---- asyn serial octet port (FTDI USB, 9600 8-N-1 typical) ----
drvAsynSerialPortConfigure("$(SERPORT)", "$(DEVICE)", 0, 0, 0)
asynSetOption("$(SERPORT)", 0, "baud", "9600")
asynSetOption("$(SERPORT)", 0, "bits", "8")
asynSetOption("$(SERPORT)", 0, "parity", "none")
asynSetOption("$(SERPORT)", 0, "stop", "1")

# Port EOS as the C example sets it (smaractscu.iocsh:7-8);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERPORT)", 0, "\n")
asynOctetSetOutputEos("$(SERPORT)", 0, "\n")

# ---- SCU controller + axes ----
# smarActSCUCreateController(motorPortName, ioPortName, numAxes,
#                            movingPollPeriod, idlePollPeriod), periods in s.
smarActSCUCreateController("$(CTRL)", "$(SERPORT)", 3, 0.05, 1.0)

# smarActSCUCreateAxis(controllerPortName, axisNumber, channel): records reach
# axisNumber as @asyn($(CTRL),axisNumber); channel is the controller channel.
smarActSCUCreateAxis("$(CTRL)", 0, 0)
smarActSCUCreateAxis("$(CTRL)", 1, 1)
smarActSCUCreateAxis("$(CTRL)", 2, 2)

# One motor record per axis. The driver reports positions in motor-record steps
# (1000 steps = 1 micron for linear, 1 millidegree for rotary), so MRES = 0.001
# makes the record read directly in microns / millidegrees.
dbLoadRecords("$(SMARACT_IOC)/db/scu.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
dbLoadRecords("$(SMARACT_IOC)/db/scu.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=1")
dbLoadRecords("$(SMARACT_IOC)/db/scu.template", "P=$(P),M=m3,PORT=$(CTRL),ADDR=2")

# ---- autosave/restore machinery (C iocSmarAct AutoSave.cmd) ----
set_requestfile_path("$(SMARACT_IOC)", "")
set_requestfile_path("$(MOTOR)", "ioc")
set_savefile_path("$(SMARACT_IOC)", "autosave")
set_pass0_restoreFile("auto_positions.sav")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
# C's AutoSave.cmd names $(PREFIX), which its st.cmd never sets
save_restoreSet_status_prefix("$(P)")
dbLoadRecords("$(AUTOSAVE)/db/save_restoreStatus.db", "P=$(P)")

iocInit()

# save motor positions every five seconds
create_monitor_set("auto_positions.req", 5, "P=$(P)")
# save settings every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(P)")

# Example:
#   dbl
#   camonitor SCU:m1 SCU:m1.RBV
#   caput SCU:m1 10.0
