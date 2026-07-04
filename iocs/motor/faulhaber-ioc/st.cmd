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
# Encoder counts per revolution (matches the motor's encoder; also programs the
# controller via ENCRES and scales velocity/acceleration).
epicsEnvSet("CPR",    "3000")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# MCDC2805 framing: the driver appends the CR terminator to each command, so
# only the input EOS is configured here (replies are CR-terminated). Do not also
# set an output EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- MCDC2805 controller ----
# MCDC2805Setup(maxControllers, [scanRate]) is accepted for startup-script
# parity; the asyn-rs port allocates per MCDC2805Config call.
MCDC2805Setup(1, 10)

# MCDC2805Config(card, numMotors, asynPort, countsPerRev, [movingPollMs], [idlePollMs])
MCDC2805Config(0, 2, "$(SERIAL)", $(CPR), 100, 1000)

# One motor record per node (DTYP MCDC2805_$(CARD)_0, _1, ...).
# The MCDC2805 is a DC servo working in encoder counts: positions travel the
# driver boundary in counts, so MRES = 1 and EGU = counts. VELO/ACCL are
# converted to the controller's rev/min and rev/s^2 using countsPerRev.
dbLoadRecords("db/mcdc2805.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/mcdc2805.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor MCDC:m1 MCDC:m1.RBV
#   caput MCDC:m1 10000
