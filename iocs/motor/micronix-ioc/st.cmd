#============================================================
# st.cmd — Micronix MMC-100/200 motor IOC startup script
#
# Usage:
#   cargo run -p micronix-ioc -- st.cmd
#
# Requires a Micronix MMC-100/103/110/200 controller on the serial line
# below. MMC200CreateController identifies each axis (VER?) and reads its
# max velocity (VMX?) at startup, so the controller must be connected when
# the command runs.
#============================================================

epicsEnvSet("P",      "MMC200:")
epicsEnvSet("PORT",   "mmc200_1")
epicsEnvSet("SERIAL", "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "38400")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# Port EOS with the C example's values (motor.cmd.mmc200:6-7, left
# commented there); the C driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\n\r")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- MMC-200 controller ----
# MMC200CreateController(portName, MMC200PortName, numAxes,
#                        movingPollPeriod, idlePollPeriod, ignoreLimits)
# An idle period of 0 polls only when a move wakes the poller, as C's.
MMC200CreateController("$(PORT)", "$(SERIAL)", 2, 500, 0, 1)

# motorUtil's records and one asyn_motor.db per axis, as C's st.cmd and
# motor.cmd.mmc200 load them. The driver scales the record's steps by the
# axis resolution read from the controller, as C; MRES in the rows is that
# resolution, for EGU = mm.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(MICRONIX_IOC)/db/motor.substitutions.mmc200", "P=$(P),PORT=$(PORT)")

iocInit()

# Example:
#   dbl
#   camonitor MMC200:m1 MMC200:m1.RBV
#   caput MMC200:m1 5.0
