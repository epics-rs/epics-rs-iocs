#============================================================
# st.cmd — Newport MM4000/4005/4006 motor IOC startup script
#
# Usage:
#   cargo run -p mm4000-ioc -- st.cmd
#
# Requires a Newport MM4000-series controller on the serial line below.
# MM4000CreateController identifies the controller (VE) and checks the
# axis count (TP) at startup, so it must be connected when the command
# runs.
#============================================================

epicsEnvSet("P",      "MM4000:")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# MM4000 serial line: 9600 baud, 8 data bits, no parity, 1 stop bit
# (controller default; match the front-panel setting).
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# MM4000 replies end in CR (the C example st.cmd sets EOS "\r" for RS-232).
# The driver appends the CR command terminator itself, so no output EOS is
# needed.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- MM4000 controller ----
# MM4000CreateController(motorPort, serialPort, numAxes, [movingPollMs], [idlePollMs])
MM4000CreateController("$(PORT)", "$(SERIAL)", 1, 100, 1000)

# One motor record per axis (DTYP MM4000_$(PORT)_0, _1, ...).
# Positions travel the driver boundary in EGU (controller units, mm/deg)
# directly; MRES only sets the record's raw-count resolution.
dbLoadRecords("db/mm4000.template", "P=$(P),M=m1,N=0,PORT=$(PORT),MRES=0.0001,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor MM4000:m1 MM4000:m1.RBV
#   caput MM4000:m1 5.0
