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

# ---- PIJEDS controller ----
# PIJEDSSetup(num_cards, scan_rate in Hz)
PIJEDSSetup(1, 10)

# PIJEDSConfig(card, asyn port, GPIB address) - axis count
# auto-detected.
PIJEDSConfig(0, "$(SERIAL)", 0)

# One motor record per detected axis (DTYP "PIJEDS", OUT #C$(CARD) S0, S1, ...).
# The E-516 is a closed-loop piezo positioner; the driver scales the record's
# steps by drive_resolution (1e-3 um), as C, so MRES = 1e-3 gives EGU = um.
# Homing and hardware limit switches are not supported.
dbLoadRecords("$(PIJENA_IOC)/db/pijeds.template", "P=$(P),M=m1,C=$(CARD),S=0")
dbLoadRecords("$(PIJENA_IOC)/db/pijeds.template", "P=$(P),M=m2,C=$(CARD),S=1")

iocInit()

# Example:
#   dbl
#   camonitor PIJEDS:m1 PIJEDS:m1.RBV
#   caput PIJEDS:m1 50.0
