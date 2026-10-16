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
# Port EOS as the C example sets it (st.cmd.AGAP_CONEX:15-16);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r\n")

# ---- CONEX-AGAP controller (creates both U and V axes) ----
# AGAP_CONEXCreateController(portName, serialPort, controllerID, movingPollPeriod, idlePollPeriod)
AGAP_CONEXCreateController("$(PORT)", "$(SERIAL)", $(CID), 100, 1000)

dbLoadRecords("$(AGAP_CONEX_IOC)/db/agap.template", "P=$(P),MU=$(MU),MV=$(MV),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor AGAP:u AGAP:u.RBV AGAP:v AGAP:v.RBV
#   caput AGAP:u 0.5
