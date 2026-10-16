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
epicsEnvSet("SERIAL", "SERIAL1")
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
ANC150AsynConfig(0, "$(SERIAL)", 3, 100, 1000)
drvAsynMotorConfigure("$(CTRL)", "motorANC150", 0, 3)

# One motor record per axis, ADDR the 0-based axis index.
# The ANC150 is an open-loop stepper: positions travel the driver boundary in
# steps (EGU = steps), so MRES is 1. The record's velocity/acceleration have no
# effect — the step rate is a controller setting read with 'getf'.
dbLoadRecords("$(ATTOCUBE_IOC)/db/anc150.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
dbLoadRecords("$(ATTOCUBE_IOC)/db/anc150.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=1")
dbLoadRecords("$(ATTOCUBE_IOC)/db/anc150.template", "P=$(P),M=m3,PORT=$(CTRL),ADDR=2")

iocInit()

# Example:
#   dbl
#   camonitor ANC150:m1 ANC150:m1.RBV
#   caput ANC150:m1 1000
