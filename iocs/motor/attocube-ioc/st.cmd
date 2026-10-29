#============================================================
# st.cmd — attocube ANC150 piezo stepper motor IOC startup script
#
# Usage:
#   cargo run -p attocube-ioc -- st.cmd
#
# Requires an attocube ANC150 controller on the serial line below.
# ANC150AsynConfig identifies the controller (ver) and reads each axis's step
# frequency (getf) at startup, so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "ANC150:")
epicsEnvSet("CTRL",   "ANC150")
epicsEnvSet("SERIAL", "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "38400")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# ---- ANC150 controller ----
ANC150AsynSetup(1)

# ANC150AsynConfig(card, portName, numAxes, movingPollPeriod, idlePollPeriod),
# periods in ms; then the axes of card 0 become asyn port $(CTRL).
ANC150AsynConfig(0, "$(SERIAL)", 3, 250, 2000)
# C's ANC150.cmd lacks this line, so its records have no motor port.
drvAsynMotorConfigure("$(CTRL)", "motorANC150", 0, 3)

# motorUtil's records and one asyn_motor.db per axis, ADDR the 0-based axis
# index, as C's st.cmd and ANC150.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(ATTOCUBE_IOC)/db/ANC150.substitutions", "P=$(P),PORT=$(CTRL)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor ANC150:m1 ANC150:m1.RBV
#   caput ANC150:m1 10
