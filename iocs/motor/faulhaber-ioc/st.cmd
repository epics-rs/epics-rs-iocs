#============================================================
# st.cmd — Faulhaber MCDC2805 DC servo motor IOC startup script
#
# Usage:
#   cargo run -p faulhaber-ioc -- st.cmd
#
# Requires a Faulhaber MCDC2805 module, node 0, on the serial line below.
# MCDC2805Config probes it (VER) and runs the limit-switch/homing
# configuration, so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "MCDC:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("SERIAL", "serial1")
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
MCDC2805Config(0, 1, "$(SERIAL)")

# motorUtil's records and the node's motor.db, as C's st.cmd and
# MCDC2805.cmd load them. VELO/ACCL are converted to the controller's rev/min
# and rev/s^2 using the record's SREV.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(FAULHABER_IOC)/db/MCDC2805.substitutions", "P=$(P),C=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor MCDC:m1 MCDC:m1.RBV
#   caput MCDC:m1 10
