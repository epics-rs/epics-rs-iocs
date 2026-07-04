#============================================================
# st.cmd — Newport ESP100/ESP300/ESP301 motor IOC startup script
#
# Usage:
#   cargo run -p esp300-ioc -- st.cmd
#
# Requires a Newport ESP-series controller on the serial line below.
# ESP300CreateController identifies the controller (VE?) and discovers the
# axis count at startup, so the controller must be connected when it runs.
#============================================================

epicsEnvSet("P",      "ESP300:")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# ESP300 serial line: 19200 baud, 8 data bits, no parity, 1 stop bit
# (controller default; match the front-panel setting).
asynSetOption("$(SERIAL)", 0, "baud",   "19200")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# ESP300 replies end in CR/LF: frame reads on LF (the driver strips the CR,
# as the C driver does). The driver appends the CR command terminator itself,
# so no output EOS is needed.
asynOctetSetInputEos("$(SERIAL)", 0, "\n")

# ---- ESP300 controller (axes discovered automatically) ----
# ESP300CreateController(motorPort, serialPort, [movingPollMs], [idlePollMs])
ESP300CreateController("$(PORT)", "$(SERIAL)", 100, 1000)

# One motor record per discovered axis (DTYP ESP300_$(PORT)_0, _1, ...).
# MRES must be set to the axis drive resolution (see motorNewport README):
# the driver reads positions back in drive-resolution steps.
dbLoadRecords("db/esp300.template", "P=$(P),M=m1,N=0,PORT=$(PORT),MRES=0.0001,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor ESP300:m1 ESP300:m1.RBV
#   caput ESP300:m1 5.0
