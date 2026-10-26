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
epicsEnvSet("PORT",   "Agilis1")
epicsEnvSet("SERIAL", "serial1")
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
AGAP_CONEXCreateController("$(PORT)", "$(SERIAL)", $(CID), 50, 500)

# Each axis's basic_asyn_motor.db and the serial line's asyn record, as
# C's st.cmd.AGAP_CONEX loads them.
dbLoadTemplate("$(AGAP_CONEX_IOC)/db/motor.substitutions.AGAP_CONEX", "P=$(P),PORT=$(PORT)")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=serial1,PORT=$(SERIAL),ADDR=0,OMAX=256,IMAX=256")

iocInit()

dbpf("$(P)m1.RTRY", "0")
dbpf("$(P)m1.NTM", "0")

# Example:
#   dbl
#   camonitor AGAP:m1 AGAP:m1.RBV AGAP:m2 AGAP:m2.RBV
#   caput AGAP:m1 0.5   (MRES 1e-4 = the driver stepSize_)
