#============================================================
# st.cmd — Newport CONEX-AGAP two-axis motor IOC startup script
#
# Usage:
#   cargo run -p agap-conex-ioc -- st.cmd
#
# Requires a Newport CONEX-AGAP controller on the serial line below. The
# driver reads the firmware version at startup and verifies "CONEX-AGAP",
# so the controller must be reachable when AGAP_CONEXCreateController runs.
#============================================================

epicsEnvSet("P",      "AGAP:")
epicsEnvSet("MU",     "u")
epicsEnvSet("MV",     "v")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")
epicsEnvSet("CID",    "1")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# CONEX serial line: 115200 baud, 8 data bits, no parity, 1 stop bit.
# USB-connected CONEX controllers present a virtual COM port; adjust baud
# to match your controller/cabling.
asynSetOption("$(SERIAL)", 0, "baud",   "115200")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# NOTE (hardware path): the AGAP driver appends the CR/LF command terminator
# itself, so no output EOS is required. Input-response framing relies on the
# serial read returning a complete reply; asyn-rs (published 0.21.0) exposes no
# asynOctetSetInputEos iocsh command, so the input terminator cannot be set
# here. Verify response framing against real hardware before production use.

# ---- CONEX-AGAP controller (creates both U and V axes) ----
# AGAP_CONEXCreateController(motorPort, serialPort, controllerID, [movingPollMs], [idlePollMs])
AGAP_CONEXCreateController("$(PORT)", "$(SERIAL)", $(CID), 100, 1000)

dbLoadRecords("db/agap.template", "P=$(P),MU=$(MU),MV=$(MV),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor AGAP:u AGAP:u.RBV AGAP:v AGAP:v.RBV
#   caput AGAP:u 0.5
