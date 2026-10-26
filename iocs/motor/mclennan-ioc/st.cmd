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
epicsEnvSet("SERIAL", "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# Port EOS as C needs it: drvPM304 sets no EOS and sends bare commands. Replies
# are CR+LF terminated; on the PM600 the command echo ends in a lone CR that the
# driver strips.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- PM304 controller ----
# PM304Setup(num_cards, scan_rate in Hz)
PM304Setup(1, 10)

# PM304Config(card, asyn port, n_axes) - the model
# (PM304 vs PM600) is auto-detected from the 1ID reply.
PM304Config(0, "$(SERIAL)", 1)

# motorUtil's records and the axis's motor.db, as C's st.cmd and
# PM304.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(MCLENNAN_IOC)/db/PM304.substitutions", "P=$(P),C=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor PM304:m1 PM304:m1.RBV
#   caput PM304:m1 10
