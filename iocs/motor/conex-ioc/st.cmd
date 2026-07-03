#============================================================
# st.cmd — Newport CONEX motor IOC startup script
#
# Usage:
#   cargo run -p conex-ioc -- st.cmd
#
# Requires a Newport CONEX controller (CONEX-AGP/CC/PP, DL, FCL200) on the
# serial line below. The driver identifies the model at startup, so the
# controller must be reachable when AG_CONEXCreateController runs.
#============================================================

epicsEnvSet("P",      "CONEX:")
epicsEnvSet("M",      "m1")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")
epicsEnvSet("CID",    "1")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# CONEX serial line: 115200 baud, 8 data bits, no parity, 1 stop bit.
# USB-connected CONEX-CC/AGP present a virtual COM port; adjust baud
# (e.g. 921600) to match your controller/cabling.
asynSetOption("$(SERIAL)", 0, "baud",   "115200")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# NOTE (hardware path): the CONEX driver appends the CR/LF command terminator
# itself, so no output EOS is required. Input-response framing relies on the
# serial read returning a complete reply; asyn-rs (published 0.21.0) exposes no
# asynOctetSetInputEos iocsh command, so the input terminator cannot be set
# here. Verify response framing against real hardware before production use.

# ---- CONEX controller ----
# AG_CONEXCreateController(motorPort, serialPort, controllerID, [movingPollMs], [idlePollMs])
AG_CONEXCreateController("$(PORT)", "$(SERIAL)", $(CID), 100, 1000)

dbLoadRecords("db/conex.template", "P=$(P),M=$(M),PORT=$(PORT),EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor CONEX:m1 CONEX:m1.RBV
#   caput CONEX:m1 5.0
