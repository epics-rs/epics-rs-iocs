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
epicsEnvSet("SERIAL", "serial2")
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

# motorUtil's records and one motor.db per axis, signal S of card 0, as C's
# st.cmd and MM4000.cmd load them. Positions travel the driver boundary in
# EGU (controller units, mm/deg) directly; MRES only sets the record's
# raw-count resolution.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(MM4000_IOC)/db/MM4000.substitutions", "P=$(P)")

# MM4005_trajectoryScan.st's records, for the seq line below:
#   dbLoadRecords("$(MM4000_IOC)/db/trajectoryScan.db", "P=$(P),R=traj1:,NAXES=1,NELM=2000,NPULSE=2000")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# MM4005_trajectoryScan.st runs trajectoryScan.db's trajectories on the
# MM4005 over the same serial port, motor Mn its axis n:
#   seq &MM4005_trajectoryScan, "P=$(P),R=traj1:,M1=m11,PORT=$(SERIAL)"

# Example:
#   dbl
#   camonitor MM4000:m11 MM4000:m11.RBV
#   caput MM4000:m11 5.0
