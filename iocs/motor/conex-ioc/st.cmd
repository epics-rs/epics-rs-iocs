#============================================================
# st.cmd — Newport CONEX motor IOC startup script
#
# Usage:
#   cargo run -p conex-ioc -- st.cmd
#
# Requires Newport CONEX controllers (CONEX-AGP/CC/PP, DL, FCL200) with IDs
# 1-3 on the serial line below. The driver identifies each model at startup,
# so a controller must be reachable when its AG_CONEXCreateController runs.
#============================================================

epicsEnvSet("P",      "CONEX:")
epicsEnvSet("SERIAL", "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

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

# ---- CONEX controllers ----
# Three controllers on the one serial line, as C's st.cmd.CONEX-PP:32-40.
# One whose ID does not answer keeps its port with no axis, as in C.
# AG_CONEXCreateController(portName, serialPort, controllerID, movingPollPeriod, idlePollPeriod)
AG_CONEXCreateController("CONEX1", "$(SERIAL)", 1, 50, 500)
AG_CONEXCreateController("CONEX2", "$(SERIAL)", 2, 50, 500)
AG_CONEXCreateController("CONEX3", "$(SERIAL)", 3, 50, 500)

# Each axis's basic_asyn_motor.db and the serial line's asyn record, as
# C's st.cmd.CONEX-PP loads them. MRES must equal the model's stepSize_
# (the boundary is raw steps).
dbLoadTemplate("$(CONEX_IOC)/db/motor.substitutions.CONEX-PP", "P=$(P)")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=serial1,PORT=$(SERIAL),ADDR=0,OMAX=256,IMAX=256")

iocInit()

dbpf("$(P)m1.RTRY", "0")
dbpf("$(P)m1.NTM", "0")

# Example:
#   dbl
#   camonitor CONEX:m1 CONEX:m1.RBV
#   caput CONEX:m1 5.0
