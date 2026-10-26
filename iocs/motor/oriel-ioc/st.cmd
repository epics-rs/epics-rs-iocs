#============================================================
# st.cmd — Oriel Encoder Mike 18011 (EMC18011) motor IOC startup script
#
# Usage:
#   cargo run -p oriel-ioc -- st.cmd
#
# Requires an Oriel EMC18011 controller on the serial line below. EMC18011Config
# probes the controller (L/R, expecting "ON LINE"), so it must be connected when
# the command runs. The controller multiplexes three encoder-mike axes over one
# serial line and can only drive one at a time.
#============================================================

epicsEnvSet("P",      "ORIEL:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# ---- EMC18011 controller ----
# EMC18011Setup(num_cards, scan_rate in Hz)
EMC18011Setup(1, 10)

# EMC18011Config(card, asyn port) - fixed 3 axes.
EMC18011Config(0, "$(SERIAL)")

# Each axis's motor.db, as C's EMC18011.cmd loads it, and motorUtil's
# records, as C's st.cmd loads them. The controller works in millimetres
# (EGU = mm) at a 0.01 mm step (MRES).
dbLoadTemplate("$(ORIEL_IOC)/db/EMC18011.substitutions", "P=$(P),C=$(CARD)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor ORIEL:m1 ORIEL:m1.RBV
#   caput ORIEL:m1 5.0
