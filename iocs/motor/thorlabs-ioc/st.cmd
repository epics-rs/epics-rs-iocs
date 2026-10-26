#============================================================
# st.cmd — ThorLabs MDT693/694/695 piezo motor IOC startup script
#
# Usage:
#   cargo run -p thorlabs-ioc -- st.cmd
#
# Requires a ThorLabs MDT693/694/695 piezo controller on the serial line below.
# MDT695Config probes the controller (device "D" command, expecting an "MDT"
# reply) and reads the axis count, so it must be connected when the command runs.
# The controller is open-loop: a move sets the channel output voltage.
#============================================================

epicsEnvSet("P",      "THORLABS:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "115200")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# ---- MDT695 controller ----
# MDT695Setup(num_cards, scan_rate in Hz)
MDT695Setup(1, 10)

# MDT695Config(card, asyn port) - the axis count
# (1 for MDT694, else 3) is read from the controller.
MDT695Config(0, "$(SERIAL)")

# Each channel's motor.db, as C's MDT695.cmd loads it, and motorUtil's
# records, as C's st.cmd loads them. The controller works in volts (EGU = V)
# at a 0.1 V step (MRES). Load only as many channels as the controller has
# (3 for MDT693/695, 1 for MDT694).
dbLoadTemplate("$(THORLABS_IOC)/db/MDT695.substitutions", "P=$(P),C=$(CARD)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# Example:
#   dbl
#   camonitor THORLABS:x THORLABS:x.RBV
#   caput THORLABS:x 40.0
