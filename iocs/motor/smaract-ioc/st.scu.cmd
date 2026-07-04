#============================================================
# st.scu.cmd — SmarAct SCU motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.scu.cmd
#
# The SCU communicates over an FTDI USB serial port (drvAsynSerialPort). The
# controller is created first, then one axis per channel (the axis number and
# the controller channel are mapped explicitly, matching the C
# smarActSCUCreateAxis command).
#============================================================

epicsEnvSet("P",       "SCU:")
epicsEnvSet("CARD",    "0")
epicsEnvSet("SERPORT", "serial1")
epicsEnvSet("DEVICE",  "/dev/ttyUSB0")

# ---- asyn serial octet port (FTDI USB, 9600 8-N-1 typical) ----
drvAsynSerialPortConfigure("$(SERPORT)", "$(DEVICE)", 0, 0, 0)
asynSetOption("$(SERPORT)", 0, "baud", "9600")
asynSetOption("$(SERPORT)", 0, "bits", "8")
asynSetOption("$(SERPORT)", 0, "parity", "none")
asynSetOption("$(SERPORT)", 0, "stop", "1")

# SCU framing: the driver appends the LF command terminator, so only the input
# EOS is configured here. Do not set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(SERPORT)", 0, "\n")

# ---- SCU controller + axes ----
# smarActSCUCreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]).
smarActSCUCreateController(0, "$(SERPORT)", 3, 50, 1000)

# smarActSCUCreateAxis(card, axisNo, channel): axisNo is 0-based (DTYP
# SCU_$(CARD)_$(N)), channel is the controller channel it drives.
smarActSCUCreateAxis(0, 0, 0)
smarActSCUCreateAxis(0, 1, 1)
smarActSCUCreateAxis(0, 2, 2)

# One motor record per axis. The driver reports positions in motor-record steps
# (1000 steps = 1 micron for linear, 1 millidegree for rotary), so MRES = 0.001
# makes the record read directly in microns / millidegrees.
dbLoadRecords("db/scu.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/scu.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/scu.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor SCU:m1 SCU:m1.RBV
#   caput SCU:m1 10.0
