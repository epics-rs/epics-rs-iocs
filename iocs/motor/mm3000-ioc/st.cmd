#============================================================
# st.cmd — Newport MM3000 motor IOC startup script
#
# Usage:
#   cargo run -p mm3000-ioc -- st.cmd
#
# Requires a Newport MM3000 controller on the serial line below.
# MM3000CreateController probes the controller (VE) and reads the axis
# configuration (RC) at startup, so it must be connected when the
# command runs.
#============================================================

epicsEnvSet("P",      "MM3000:")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# MM3000 serial line: 9600 baud, 8 data bits, no parity, 1 stop bit
# (controller default; match the rear-panel setting).
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# MM3000 replies end in CR (the C example st.cmd sets EOS "\r" for RS-232).
# The driver appends the CR command terminator itself, so no output EOS is
# needed.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- MM3000 controller (axis configuration read automatically) ----
# MM3000CreateController(motorPort, serialPort, [movingPollMs], [idlePollMs])
MM3000CreateController("$(PORT)", "$(SERIAL)", 100, 1000)

# One motor record per configured axis (DTYP MM3000_$(PORT)_0, _1, ...).
# The MM3000 wire is step-native, so MRES=1 (EGU = motor steps / encoder
# counts) — like the AG-UC driver.
dbLoadRecords("db/mm3000.template", "P=$(P),M=m1,N=0,PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor MM3000:m1 MM3000:m1.RBV
#   caput MM3000:m1 5000
