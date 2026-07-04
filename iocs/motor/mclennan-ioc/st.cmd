#============================================================
# st.cmd — Mclennan PM304 / PM600 stepper motor IOC startup script
#
# Usage:
#   cargo run -p mclennan-ioc -- st.cmd
#
# Requires a Mclennan PM304 or PM600 controller on the serial line below.
# PM304Config identifies the controller (1ID) and auto-detects the model, so it
# must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "PM304:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# PM304/PM600 framing: the driver appends the CR terminator to each command, so
# only the input EOS is configured here (replies are CR+LF terminated; on the
# PM600 the command echo ends in a lone CR that the driver strips). Do not set
# an output EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")

# ---- PM304 controller ----
# PM304Setup(maxControllers, [scanRate]) is accepted for startup-script parity;
# the asyn-rs port allocates per PM304Config call.
PM304Setup(1, 10)

# PM304Config(card, asynPort, nAxes, [movingPollMs], [idlePollMs]) - the model
# (PM304 vs PM600) is auto-detected from the 1ID reply.
PM304Config(0, "$(SERIAL)", 2, 100, 1000)

# One motor record per axis (DTYP PM304_$(CARD)_0, _1, ...). The controller
# works in motor steps, so MRES = 1 and EGU = steps.
dbLoadRecords("db/pm304.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/pm304.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor PM304:m1 PM304:m1.RBV
#   caput PM304:m1 10000
