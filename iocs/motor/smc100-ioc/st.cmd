#============================================================
# st.cmd — Newport SMC100 motor IOC startup script
#
# Usage:
#   cargo run -p smc100-ioc -- st.cmd
#
# Requires a Newport SMC100 controller on the serial line below.
#============================================================

epicsEnvSet("P",      "SMC100:")
epicsEnvSet("M",      "m1")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# SMC100 serial line: 57600 baud, 8 data bits, no parity, 1 stop bit.
# Flow control (crtscts/XON-XOFF) is left at the driver default; tune here
# for a specific controller/cabling if needed.
asynSetOption("$(SERIAL)", 0, "baud",   "57600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# CR/LF both ways (C st.cmd.SMC100); the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r\n")

# ---- SMC100 controller ----
# SMC100CreateController(portName, SMC100PortName, numAxes, movingPollPeriod,
#                        idlePollPeriod, eguPerStep)
SMC100CreateController("$(PORT)", "$(SERIAL)", 1, 100, 1000, "1.0")

# MRES only sets the record's raw-count resolution (display/deadband
# granularity); positions travel the driver boundary in EGU directly.
dbLoadRecords("$(SMC100_IOC)/db/smc100.template", "P=$(P),M=$(M),PORT=$(PORT),ADDR=0,MRES=0.001,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor SMC100:m1 SMC100:m1.RBV
#   caput SMC100:m1 5.0
