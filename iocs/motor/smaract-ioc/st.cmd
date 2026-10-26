#============================================================
# st.cmd — SmarAct MCS2 motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.cmd
#
# Requires a SmarAct MCS2 controller reachable over TCP at the host:port below.
# MCS2CreateController reads the controller serial number, so it must be
# reachable when the command runs.
#
# The positioner type (PTYP) and calibration must be configured on the
# controller (or with the SmarAct tools) beforehand — those auxiliary parameters
# are not exposed through the motor record.
#============================================================

epicsEnvSet("P",      "MCS2:")
epicsEnvSet("CTRL",   "MCS2_1")
epicsEnvSet("IPPORT", "MCS2")
epicsEnvSet("HOST",   "192.168.1.200:55551")
epicsEnvSet("NAXES",  "3")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ---- MCS2 controller ----
# MCS2CreateController(portName, MCS2PortName, numAxes, movingPollPeriod,
#                      idlePollPeriod, unusedMask), periods in ms.
MCS2CreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 1000, 0)

# One motor record per channel, ADDR the channel. The driver reports
# positions in nanometres (linear) / micro-degrees (rotary), so MRES = 1 and
# EGU = nm.
dbLoadRecords("$(SMARACT_IOC)/db/mcs2.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
dbLoadRecords("$(SMARACT_IOC)/db/mcs2.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=1")
dbLoadRecords("$(SMARACT_IOC)/db/mcs2.template", "P=$(P),M=m3,PORT=$(CTRL),ADDR=2")

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
#   camonitor MCS2:m1 MCS2:m1.RBV
#   caput MCS2:m1 1000000
