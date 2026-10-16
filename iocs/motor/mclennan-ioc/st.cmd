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
# PM304Setup(num_cards, scan_rate in Hz)
PM304Setup(1, 10)

# PM304Config(card, asyn port, n_axes) - the model
# (PM304 vs PM600) is auto-detected from the 1ID reply.
PM304Config(0, "$(SERIAL)", 2)

# One motor record per axis (DTYP "Mclennan PM304", OUT #C$(CARD) S0, S1, ...). The controller
# works in motor steps, so MRES = 1 and EGU = steps.
dbLoadRecords("$(MCLENNAN_IOC)/db/pm304.template", "P=$(P),M=m1,C=$(CARD),S=0")
dbLoadRecords("$(MCLENNAN_IOC)/db/pm304.template", "P=$(P),M=m2,C=$(CARD),S=1")

iocInit()

# Example:
#   dbl
#   camonitor PM304:m1 PM304:m1.RBV
#   caput PM304:m1 10000
