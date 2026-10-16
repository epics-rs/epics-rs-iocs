#============================================================
# st.cmd — Newport MM4000/4005/4006 motor IOC startup script
#
# Usage:
#   cargo run -p mm4000-ioc -- st.cmd
#
# Requires a Newport MM4000-series controller on the serial line below.
# MM4000Config identifies the controller (VE) and reads the axis count (TP),
# so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "MM4000:")
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

# Port EOS as the C example sets it (Newport_MM4000.iocsh:23-24);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- MM4000 controller ----
# MM4000Setup(num_cards, scan_rate in Hz)
MM4000Setup(1, 10)
# MM4000Config(card, asyn port, GPIB address)
MM4000Config(0, "$(SERIAL)")
# The model-2 form instead (records DTYP asynMotor, OUT @asyn(MM4,ADDR)):
#   MM4000AsynSetup(1)
#   MM4000AsynConfig(0, "$(SERIAL)", 0, 1, 100, 250)
#   drvAsynMotorConfigure("MM4", "motorMM4000", 0, 4)

# One motor record per axis, signal S of card 0.
# Positions travel the driver boundary in EGU (controller units, mm/deg)
# directly; MRES only sets the record's raw-count resolution.
dbLoadRecords("$(MM4000_IOC)/db/mm4000.template", "P=$(P),M=m1,C=0,S=0,MRES=0.0001,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor MM4000:m1 MM4000:m1.RBV
#   caput MM4000:m1 5.0
