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
epicsEnvSet("SERIAL", "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# The C driver sends bare commands, so the port's output EOS ends them
# (MVP2001.cmd:4, commented there). A reply is "\r\n0001 <hex>\r\n", which
# drvMVP2001.cc:597 reads with "\n"; this driver sets no EOS, and only
# MVP2001.cmd:5's "\r\n" leaves its second read the bare "0001 <hex>".
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- MVP2001 controller + axes ----
# MVP2001CreateController(portName, MVP2001PortName, numAxes, movingPollPeriod, idlePollPeriod).
MVP2001CreateController("$(CTRL)", "$(SERIAL)", 1, 250, 1000)

# MVP2001CreateAxis(portName, axisNo, encLinesPerRev, maxCurrentMa, limitPolarity).
# axisNo is 0-based; limitPolarity 1 = normally-open, 0 = normally-closed.
MVP2001CreateAxis("$(CTRL)", 0, 16, 1000, 0)

# motorUtil's records and one asyn_motor.db per axis, as C's st.cmd and
# MVP2001.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(MICROMO_IOC)/db/MVP2001.substitutions", "P=$(P),PORT=$(CTRL)")

iocInit()

# Example:
#   dbl
#   camonitor MVP:m1 MVP:m1.RBV
#   caput MVP:m1 1
