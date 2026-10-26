#============================================================
# st.cmd — Newport SMC100 motor IOC startup script
#
# Usage:
#   cargo run -p smc100-ioc -- st.cmd
#
# Requires a Newport SMC100 controller on the serial line below.
#============================================================

epicsEnvSet("P",      "SMC100:")
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
SMC100CreateController("$(PORT)", "$(SERIAL)", 1, 100, 1000, "0.00005")

# The axis's basic_asyn_motor.db, as C's st.cmd.SMC100 loads it. The
# boundary is raw steps: its MRES must equal eguPerStep above.
dbLoadTemplate("$(SMC100_IOC)/db/motor.substitutions.SMC100", "P=$(P),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor SMC100:m1 SMC100:m1.RBV
#   caput SMC100:m1 5.0
