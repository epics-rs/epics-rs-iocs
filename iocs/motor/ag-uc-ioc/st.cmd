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
epicsEnvSet("PORT",   "Agilis1")
epicsEnvSet("SERIAL", "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Agilis USB serial: 921600 baud, 8 data bits, no parity, 1 stop bit is the
# common default; adjust to match your controller/cabling.
asynSetOption("$(SERIAL)", 0, "baud",   "921600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")
# Port EOS as the C example sets it (st.cmd.AG_UC:15-16);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r\n")

# ---- Agilis controller + axes ----
# AG_UCCreateController(portName, serialPort, numAxes, movingPollPeriod, idlePollPeriod)
AG_UCCreateController("$(PORT)", "$(SERIAL)", 2, 50, 500)
# AG_UCCreateAxis(portName, axis, hasLimits, forwardAmplitude, reverseAmplitude)
AG_UCCreateAxis("$(PORT)", 0, 0, 50, -30)
AG_UCCreateAxis("$(PORT)", 1, 0, 40, -50)

# Each axis's basic_asyn_motor.db and the serial line's asyn record, as
# C's st.cmd.AG_UC loads them.
dbLoadTemplate("$(AG_UC_IOC)/db/motor.substitutions.AG_UC", "P=$(P),PORT=$(PORT)")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=serial1,PORT=$(SERIAL),ADDR=0,OMAX=256,IMAX=256")

iocInit()

# Example:
#   dbl
#   camonitor AGILIS:m1 AGILIS:m1.RBV
#   caput AGILIS:m1 1      # 1000 steps at MRES .001
