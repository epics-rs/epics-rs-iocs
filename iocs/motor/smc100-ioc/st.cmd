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

# NOTE (hardware path): the SMC100 driver appends the CR/LF command
# terminator itself, so no output EOS is required. Input-response framing
# relies on the serial read returning a complete reply; asyn-rs exposes no
# `asynOctetSetInputEos` iocsh command yet, so the input terminator cannot be
# set here. Verify response framing against real hardware / a serial
# simulator before production use.

# ---- SMC100 controller ----
# SMC100CreateController(motorPort, serialPort, wireScale, [movingPollMs], [idlePollMs])
# wireScale is controller-units-per-record-EGU: 1.0 when the record EGU is the
# controller's native unit (mm), which is the normal configuration. This is
# NOT the C driver's eguPerStep — the record boundary here is EGU, not steps.
SMC100CreateController("$(PORT)", "$(SERIAL)", 1.0, 100, 1000)

# MRES only sets the record's raw-count resolution (display/deadband
# granularity); positions travel the driver boundary in EGU directly.
dbLoadRecords("db/smc100.template", "P=$(P),M=$(M),PORT=$(PORT),MRES=0.001,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor SMC100:m1 SMC100:m1.RBV
#   caput SMC100:m1 5.0
