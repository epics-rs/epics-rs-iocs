#============================================================
# st.cmd — piezosystem jena E-516 (PIJEDS) piezo motor IOC startup script
#
# Usage:
#   cargo run -p pijena-ioc -- st.cmd
#
# Requires a piezosystem jena E-516 controller on the serial line below.
# PIJEDSConfig brings the controller online (identity must contain "DSM") and
# auto-detects the present axes, so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "PIJEDS:")
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

# E-516 framing: the driver appends the CR terminator to each command, so only
# the input EOS is configured here (each reply is framed by an ETX (0x11) input
# EOS). Do not also set an output EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\021")

# ---- PIJEDS controller ----
# PIJEDSSetup(maxControllers, [scanRate]) is accepted for startup-script parity;
# the asyn-rs port allocates per PIJEDSConfig call.
PIJEDSSetup(1, 10)

# PIJEDSConfig(card, asynPort, [movingPollMs], [idlePollMs]) - axis count
# auto-detected.
PIJEDSConfig(0, "$(SERIAL)", 100, 1000)

# One motor record per detected axis (DTYP PIJEDS_$(CARD)_0, _1, ...).
# The E-516 is a closed-loop piezo positioner: positions travel the driver
# boundary in physical units (µm), so MRES = drive_resolution (1e-3) and
# EGU = um. Homing and hardware limit switches are not supported.
dbLoadRecords("db/pijeds.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/pijeds.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor PIJEDS:m1 PIJEDS:m1.RBV
#   caput PIJEDS:m1 50.0
