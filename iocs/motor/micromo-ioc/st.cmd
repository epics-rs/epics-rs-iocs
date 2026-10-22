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
epicsEnvSet("CTRL",   "MVP1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# The C driver sends bare commands, so the port's output EOS ends them
# (MVP2001.cmd:4, commented there). Replies end in CR.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- MVP2001 controller + axes ----
# MVP2001CreateController(portName, MVP2001PortName, numAxes, movingPollPeriod, idlePollPeriod).
MVP2001CreateController("$(CTRL)", "$(SERIAL)", 1, 100, 1000)

# MVP2001CreateAxis(portName, axisNo, encLinesPerRev, maxCurrentMa, limitPolarity).
# axisNo is 0-based; limitPolarity 1 = normally-open, 0 = normally-closed.
MVP2001CreateAxis("$(CTRL)", 0, 512, 1000, 1)

# One motor record per axis (OUT @asyn($(CTRL),axis)). Positions cross the driver
# boundary in the controller's native encoder counts, so MRES = 1.
dbLoadRecords("$(MICROMO_IOC)/db/mvp2001.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")

iocInit()

# Example:
#   dbl
#   camonitor MVP:m1 MVP:m1.RBV
#   caput MVP:m1 10000
