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
epicsEnvSet("SERIAL", "serial1")
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

# Channel X's motor.db, as C's MDT695.cmd loads it, and motorUtil's records,
# as C's st.cmd loads them.
dbLoadTemplate("$(THORLABS_IOC)/db/MDT695.substitutions", "P=$(P),C=$(CARD)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor THORLABS:m1 THORLABS:m1.RBV
#   caput THORLABS:m1 4.0
