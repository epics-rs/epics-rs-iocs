#============================================================
# st.cmd — Newport PM500 precision motor IOC startup script
#
# Usage:
#   cargo run -p pm500-ioc -- st.cmd
#
# Requires a Newport PM500 controller on the serial line below.
# PM500CreateController configures the controller (SCUM 1, SENAINT $AF),
# identifies it (SVN?), and autodiscovers axes (STAT? scan) at startup,
# so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "PM500:")
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# PM500 serial line: match the controller's configured settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# PM500 responses are CR-terminated (SENAINT $AF framing). The driver
# appends the CR command terminator itself, so no output EOS is needed.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- PM500 controller ----
# PM500CreateController(motorPort, serialPort, [movingPollMs], [idlePollMs])
# Autodiscovers axes; one motor axis per channel (X Y Z A B C ...).
PM500CreateController("$(PORT)", "$(SERIAL)", 100, 1000)

# One motor record per axis (DTYP PM500_$(PORT)_0, _1, ...).
# Positions travel the driver boundary in EGU (controller units: microns
# or arc-sec) directly; MRES only sets the record's raw-count resolution.
dbLoadRecords("db/pm500.template", "P=$(P),M=m1,N=0,PORT=$(PORT),MRES=0.01,EGU=um")

iocInit()

# Example:
#   dbl
#   camonitor PM500:m1 PM500:m1.RBV
#   caput PM500:m1 5.0
