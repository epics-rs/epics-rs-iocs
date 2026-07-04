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
epicsEnvSet("CARD",   "0")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "38400")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# ANC150 framing: the driver appends the CR+LF terminator to each command, so
# only the input EOS is configured here (each reply blob ends at the "> "
# console prompt). Do not also set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(SERIAL)", 0, "> ")

# ---- ANC150 controller ----
# ANC150AsynSetup(maxControllers) is accepted for startup-script parity; the
# asyn-rs port allocates per ANC150AsynConfig call.
ANC150AsynSetup(1)

# ANC150AsynConfig(card, asynPort, numAxes, [movingPollMs], [idlePollMs])
ANC150AsynConfig(0, "$(SERIAL)", 3, 100, 1000)

# One motor record per axis (DTYP ANC150_$(CARD)_0, _1, _2).
# The ANC150 is an open-loop stepper: positions travel the driver boundary in
# steps (EGU = steps), so MRES is 1. The record's velocity/acceleration have no
# effect — the step rate is a controller setting read with 'getf'.
dbLoadRecords("db/anc150.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/anc150.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/anc150.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor ANC150:m1 ANC150:m1.RBV
#   caput ANC150:m1 1000
