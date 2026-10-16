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

# ---- SC800 controller ----
# SC800Setup(num_cards, scan_rate in Hz)
SC800Setup(1, 10)

# SC800Config(card, asyn port, GPIB address) - axis count derived from the
# controller model (SC-800 -> 8, SC-400 -> 4, SC-200 -> 2).
SC800Config(0, "$(SERIAL)", 0)

# One motor record per axis (DTYP SC-800, OUT #C$(CARD) S0, S1, ...). The SC-800 works in
# motor steps, so MRES = 1 and EGU = steps. This example loads two axes; add
# more (up to the model's axis count) as needed.
dbLoadRecords("$(KOHZU_IOC)/db/sc800.template", "P=$(P),M=m1,C=$(CARD),S=0")
dbLoadRecords("$(KOHZU_IOC)/db/sc800.template", "P=$(P),M=m2,C=$(CARD),S=1")

iocInit()

# Example:
#   dbl
#   camonitor SC800:m1 SC800:m1.RBV
#   caput SC800:m1 10000
