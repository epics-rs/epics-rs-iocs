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
epicsEnvSet("SERIAL", "serial3")
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

# The axes' motor.db rows, as C's PM500.cmd loads them, and motorUtil's
# records, as C's st.cmd loads them.
dbLoadTemplate("$(PM500_IOC)/db/PM500.substitutions", "P=$(P)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor PM500:m31 PM500:m31.RBV
#   caput PM500:m31 5.0
