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

# PM500 serial line as Newport_PM500.iocsh sets it: 9600 7E2, hardware
# handshake (setSerialParams.iocsh's clocal N, crtscts N).
asynSetOption("$(SERIAL)", 0, "baud",    "9600")
asynSetOption("$(SERIAL)", 0, "bits",    "7")
asynSetOption("$(SERIAL)", 0, "stop",    "2")
asynSetOption("$(SERIAL)", 0, "parity",  "even")
asynSetOption("$(SERIAL)", 0, "clocal",  "N")
asynSetOption("$(SERIAL)", 0, "crtscts", "N")

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
# The boundary is raw steps of 0.01 controller units (drive_resolution).
dbLoadRecords("$(PM500_IOC)/db/pm500.template", "P=$(P),M=m1,C=0,S=0,VELO=100,VBAS=0,ACCL=.5,HLM=10000,LLM=-10000,MRES=0.01,EGU=um,PREC=3")

iocInit()

# Example:
#   dbl
#   camonitor PM500:m1 PM500:m1.RBV
#   caput PM500:m1 5.0
