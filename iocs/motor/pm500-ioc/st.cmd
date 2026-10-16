#============================================================
# st.cmd — Newport PM500 precision motor IOC startup script
#
# Usage:
#   cargo run -p pm500-ioc -- st.cmd
#
# Requires a Newport PM500 controller on the serial line below.
# PM500Config configures the controller (SCUM 1, SENAINT $AF), identifies it
# (SVN?), and autodiscovers axes (STAT? scan), so it must be connected when
# the command runs.
#============================================================

epicsEnvSet("P",      "PM500:")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# PM500 serial line: match the controller's configured settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# Port EOS as the C example sets it (Newport_PM500.iocsh:22-23);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- PM500 controller ----
# PM500Setup(num_cards, scan_rate in Hz)
PM500Setup(1, 10)
# PM500Config(card, asyn port, GPIB address): one axis per channel (X Y Z A ...).
PM500Config(0, "$(SERIAL)")

# One motor record per axis, signal S of card 0.
# Positions travel the driver boundary in EGU (controller units: microns
# or arc-sec) directly; MRES only sets the record's raw-count resolution.
dbLoadRecords("$(PM500_IOC)/db/pm500.template", "P=$(P),M=m1,C=0,S=0,MRES=0.01,EGU=um")

iocInit()

# Example:
#   dbl
#   camonitor PM500:m1 PM500:m1.RBV
#   caput PM500:m1 5.0
