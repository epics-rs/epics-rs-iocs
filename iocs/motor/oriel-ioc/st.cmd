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

# EMC18011 framing: the driver appends the CR command terminator, so only the
# input EOS is configured here (replies are LF terminated). Do not set an output
# EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\n")

# ---- EMC18011 controller ----
# EMC18011Setup(maxControllers, [scanRate]) is accepted for startup-script
# parity; the asyn-rs port allocates per EMC18011Config call.
EMC18011Setup(1, 10)

# EMC18011Config(card, asynPort, [movingPollMs], [idlePollMs]) - fixed 3 axes.
EMC18011Config(0, "$(SERIAL)", 100, 1000)

# One motor record per axis (DTYP EMC18011_$(CARD)_0, _1, _2). The controller
# works in millimetres, so MRES = 1 and EGU = mm.
dbLoadRecords("db/emc18011.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/emc18011.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/emc18011.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor ORIEL:m1 ORIEL:m1.RBV
#   caput ORIEL:m1 5.0
