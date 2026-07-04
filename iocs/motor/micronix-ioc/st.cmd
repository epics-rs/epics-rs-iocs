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
epicsEnvSet("PORT",   "MOTOR1")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# Match the controller's configured serial settings.
asynSetOption("$(SERIAL)", 0, "baud",   "38400")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# MMC framing: the driver appends the CR terminator to each command, so only
# the input EOS is configured here (replies are LF+CR terminated). Do not also
# set an output EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\n\r")

# ---- MMC-200 controller ----
# MMC200CreateController(motorPort, serialPort, numAxes,
#                        [movingPollMs], [idlePollMs], [ignoreLimits])
MMC200CreateController("$(PORT)", "$(SERIAL)", 2, 500, 2000, 1)

# One motor record per axis (DTYP MMC200_$(PORT)_0, _1, ...).
# Positions travel the driver boundary in EGU (controller units, mm or deg)
# directly; MRES only sets the record's raw-count resolution. Set MRES to the
# axis's physical resolution (the example matches a 2.44140625e-6 mm/microstep
# stage).
dbLoadRecords("db/mmc200.template", "P=$(P),M=m1,N=0,PORT=$(PORT),MRES=2.44140625e-6,EGU=mm")
dbLoadRecords("db/mmc200.template", "P=$(P),M=m2,N=1,PORT=$(PORT),MRES=2.44140625e-6,EGU=mm")

iocInit()

# Example:
#   dbl
#   camonitor MMC200:m1 MMC200:m1.RBV
#   caput MMC200:m1 5.0
