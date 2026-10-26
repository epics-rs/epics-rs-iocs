#============================================================
# st.mcs.cmd — SmarAct MCS (RS-232) motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.mcs.cmd
#
# The MCS is commonly reached over a terminal server (drvAsynIPPort). Use a RAW
# connection, not TELNET — the driver does not slurp telnet negotiation bytes.
# The controller is created first, then one axis per channel.
#============================================================

epicsEnvSet("P",       "MCS:")
epicsEnvSet("CTRL",    "MCS_1")
epicsEnvSet("IPPORT",  "MCS")
epicsEnvSet("HOST",    "192.168.1.210:5000")

# ---- motorUtil (allstop & alldone) ----
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- asyn IP octet port (RAW terminal server) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ---- MCS controller + axes ----
# smarActMCSCreateController(motorPortName, ioPortName, numAxes,
#                            movingPollPeriod, idlePollPeriod, disableSpeed),
# periods in seconds.
smarActMCSCreateController("$(CTRL)", "$(IPPORT)", 1, 0.05, 1.0, 0)

# smarActMCSCreateAxis(controllerPortName, axisNumber, channel): records reach
# axisNumber as @asyn($(CTRL),axisNumber); channel is the controller channel.
smarActMCSCreateAxis("$(CTRL)", 0, 0)

# The motor of C's motor.substitutions.smaractmcs.
dbLoadRecords("$(MOTOR)/db/asyn_motor.db", "P=$(P),M=m1,DTYP=asynMotor,PORT=$(CTRL),ADDR=0,DESC=motor 1,EGU=mm,DIR=Pos,VELO=1,VBAS=0,ACCL=0.2,BDST=0,BVEL=1,BACC=0.2,MRES=1e-6,PREC=2,DHLM=3,DLLM=-3,INIT=")

# ---- autosave/restore machinery (C iocSmarAct AutoSave.cmd) ----
set_requestfile_path("$(SMARACT_IOC)", "")
set_requestfile_path("$(MOTOR)", "db")
set_savefile_path("$(SMARACT_IOC)", "autosave")
set_pass0_restoreFile("auto_positions_mcs.sav")
set_pass0_restoreFile("auto_settings_mcs.sav")
set_pass1_restoreFile("auto_settings_mcs.sav")
# C's AutoSave.cmd names $(PREFIX), which its st.cmd never sets
save_restoreSet_status_prefix("$(P)")
dbLoadRecords("$(AUTOSAVE)/db/save_restoreStatus.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# save motor positions every five seconds
create_monitor_set("auto_positions_mcs.req", 5, "P=$(P)")
# save settings every thirty seconds
create_monitor_set("auto_settings_mcs.req", 30, "P=$(P)")

# Example:
#   dbl
#   camonitor MCS:m1 MCS:m1.RBV
#   caput MCS:m1 1.0
