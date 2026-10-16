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
# Port EOS as the C example sets it (st.cmd.CONEX-PP:15-16);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r\n")

# ---- CONEX controller ----
# AG_CONEXCreateController(portName, serialPort, controllerID, movingPollPeriod, idlePollPeriod)
AG_CONEXCreateController("$(PORT)", "$(SERIAL)", $(CID), 100, 1000)

dbLoadRecords("$(CONEX_IOC)/db/conex.template", "P=$(P),M=$(M),PORT=$(PORT),ADDR=0,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor CONEX:m1 CONEX:m1.RBV
#   caput CONEX:m1 5.0
