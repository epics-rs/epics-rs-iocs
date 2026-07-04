#============================================================
# st.cmd — MicroMo MVP 2001 motor IOC startup script
#
# Usage:
#   cargo run -p micromo-ioc -- st.cmd
#
# Requires a MicroMo MVP 2001 controller on the serial line below. Each
# MVP2001CreateAxis probes its axis (sets limit polarity, homes, reads the loop
# sample period), so the controller must be connected when those commands run.
#============================================================

epicsEnvSet("P",      "MVP:")
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

# MVP2001 framing: the driver appends the CR command terminator, so only the
# input EOS is configured here. Do not set an output EOS — the port would append
# it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- MVP2001 controller + axes ----
# MVP2001CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]).
MVP2001CreateController(0, "$(SERIAL)", 1, 100, 1000)

# MVP2001CreateAxis(card, axisNo, encLinesPerRev, maxCurrentMa, limitPolarity).
# axisNo is 0-based; limitPolarity 1 = normally-open, 0 = normally-closed.
MVP2001CreateAxis(0, 0, 512, 1000, 1)

# One motor record per axis (DTYP MVP2001_$(CARD)_0). Positions cross the driver
# boundary in the controller's native encoder counts, so MRES = 1.
dbLoadRecords("db/mvp2001.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor MVP:m1 MVP:m1.RBV
#   caput MVP:m1 10000
