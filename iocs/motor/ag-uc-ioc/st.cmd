#============================================================
# st.cmd — Newport Agilis AG-UC motor IOC startup script
#
# Usage:
#   cargo run -p ag-uc-ioc -- st.cmd
#
# Requires a Newport Agilis AG-UC2/UC8 controller on the serial line below.
# The driver resets the controller (RS), enters remote mode (MR) and reads
# the firmware version at startup, so it must be reachable when
# AG_UCCreateController runs. This example configures a UC2 (2 axes).
#============================================================

epicsEnvSet("P",      "AGILIS:")
epicsEnvSet("M0",     "m0")
epicsEnvSet("M1",     "m1")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Agilis USB serial: 921600 baud, 8 data bits, no parity, 1 stop bit is the
# common default; adjust to match your controller/cabling.
asynSetOption("$(SERIAL)", 0, "baud",   "921600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# NOTE (hardware path): the Agilis driver appends the CR/LF command terminator
# itself and spaces writes by ~10 ms (the controller needs it). Input-response
# framing relies on the serial read returning a complete reply; asyn-rs
# (published 0.21.0) exposes no asynOctetSetInputEos iocsh command, so the
# input terminator cannot be set here. Verify framing against real hardware.

# ---- Agilis controller + axes ----
# AG_UCCreateController(motorPort, serialPort, numAxes, [movingPollMs], [idlePollMs])
AG_UCCreateController("$(PORT)", "$(SERIAL)", 2, 100, 1000)
# AG_UCCreateAxis(motorPort, axis, hasLimits, forwardAmplitude, reverseAmplitude)
AG_UCCreateAxis("$(PORT)", 0, 0, 50, -50)
AG_UCCreateAxis("$(PORT)", 1, 0, 50, -50)

dbLoadRecords("db/agilis.template", "P=$(P),M0=$(M0),M1=$(M1),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor AGILIS:m0 AGILIS:m0.RBV
#   caput AGILIS:m0 1000
