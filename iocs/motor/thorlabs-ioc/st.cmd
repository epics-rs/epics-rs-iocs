#============================================================
# st.cmd — ThorLabs MDT693/694/695 piezo motor IOC startup script
#
# Usage:
#   cargo run -p thorlabs-ioc -- st.cmd
#
# Requires a ThorLabs MDT693/694/695 piezo controller on the serial line below.
# MDT695Config probes the controller (device "D" command, expecting an "MDT"
# reply) and reads the axis count, so it must be connected when the command runs.
# The controller is open-loop: a move sets the channel output voltage.
#============================================================

epicsEnvSet("P",      "THORLABS:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "115200")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# MDT695 framing: the driver appends the CR command terminator, so only the
# input EOS is configured here (replies are CR terminated). Do not set an output
# EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- MDT695 controller ----
# MDT695Setup(maxControllers, [scanRate]) is accepted for startup-script parity;
# the asyn-rs port allocates per MDT695Config call.
MDT695Setup(1, 10)

# MDT695Config(card, asynPort, [movingPollMs], [idlePollMs]) - the axis count
# (1 for MDT694, else 3) is read from the controller.
MDT695Config(0, "$(SERIAL)", 100, 1000)

# One motor record per channel (DTYP MDT695_$(CARD)_0, _1, _2). The controller
# works in volts, so MRES = 1 and EGU = V. Load only as many channels as the
# controller has (3 for MDT693/695, 1 for MDT694).
dbLoadRecords("db/mdt695.template", "P=$(P),M=x,N=0,CARD=$(CARD)")
dbLoadRecords("db/mdt695.template", "P=$(P),M=y,N=1,CARD=$(CARD)")
dbLoadRecords("db/mdt695.template", "P=$(P),M=z,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor THORLABS:x THORLABS:x.RBV
#   caput THORLABS:x 40.0
