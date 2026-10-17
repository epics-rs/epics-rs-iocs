#============================================================
# st.cmd — Faulhaber MCDC2805 DC servo motor IOC startup script
#
# Usage:
#   cargo run -p faulhaber-ioc -- st.cmd
#
# Requires one or more Faulhaber MCDC2805 modules on the serial line below,
# addressed by node number. MCDC2805Config probes each node (VER) and runs the
# limit-switch/homing configuration, so they must be connected when it runs.
#============================================================

epicsEnvSet("P",      "MCDC:")
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

# Port EOS as the C example sets it (MCDC2805.cmd:4-5);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- MCDC2805 controller ----
# MCDC2805Setup(num_cards, scan_rate in Hz)
MCDC2805Setup(1, 10)

# MCDC2805Config(card, # modules on the port, asyn port)
MCDC2805Config(0, 2, "$(SERIAL)")

# One motor record per node (DTYP "MCDC2805", OUT #C$(CARD) S0, S1, ...).
# The MCDC2805 is a DC servo working in encoder counts: positions travel the
# driver boundary in counts, so MRES = 1 and EGU = counts. VELO/ACCL are
# converted to the controller's rev/min and rev/s^2 using the record's
# default SREV (200).
dbLoadRecords("$(FAULHABER_IOC)/db/mcdc2805.template", "P=$(P),M=m1,C=$(CARD),S=0")
dbLoadRecords("$(FAULHABER_IOC)/db/mcdc2805.template", "P=$(P),M=m2,C=$(CARD),S=1")

iocInit()

# Example:
#   dbl
#   camonitor MCDC:m1 MCDC:m1.RBV
#   caput MCDC:m1 10000
