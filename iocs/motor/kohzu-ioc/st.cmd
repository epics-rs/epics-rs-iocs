#============================================================
# st.cmd — Kohzu SC-200/400/800 stepper motor IOC startup script
#
# Usage:
#   cargo run -p kohzu-ioc -- st.cmd
#
# Requires a Kohzu SC-200/400/800 controller on the serial line below.
# SC800Config identifies the controller (IDN) and derives the axis count from
# the model, so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "SC800:")
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

# SC-800 framing: the driver prefixes STX (0x02) and appends the CR+LF
# terminator to each command, so only the input EOS is configured here (replies
# are CR+LF terminated). Do not set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\r\n")

# ---- SC800 controller ----
# SC800Setup(maxControllers, [scanRate]) is accepted for startup-script parity;
# the asyn-rs port allocates per SC800Config call.
SC800Setup(1, 10)

# SC800Config(card, asynPort, [movingPollMs], [idlePollMs]) - axis count derived
# from the controller model (SC-800 -> 8, SC-400 -> 4, SC-200 -> 2).
SC800Config(0, "$(SERIAL)", 100, 1000)

# One motor record per axis (DTYP SC800_$(CARD)_0, _1, ...). The SC-800 works in
# motor steps, so MRES = 1 and EGU = steps. This example loads two axes; add
# more (up to the model's axis count) as needed.
dbLoadRecords("db/sc800.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/sc800.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor SC800:m1 SC800:m1.RBV
#   caput SC800:m1 10000
