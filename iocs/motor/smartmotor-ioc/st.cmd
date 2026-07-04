#============================================================
# st.cmd — Animatics SmartMotor servo motor IOC startup script
#
# Usage:
#   cargo run -p smartmotor-ioc -- st.cmd
#
# Requires a single (non-daisy-chained) Animatics SmartMotor on the serial line
# below. SmartMotorConfig probes the motor (RBe), so it must be connected when
# the command runs. Daisy-chained controllers are not supported.
#============================================================

epicsEnvSet("P",      "SM:")
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

# SmartMotor framing: the driver appends the newline command terminator, so only
# the input EOS is configured here (status replies are CR terminated). Do not
# set an output EOS — the port would append it a second time.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")

# ---- SmartMotor controller ----
# SmartMotorSetup(maxControllers, [scanRate]) is accepted for startup-script
# parity; the asyn-rs port allocates per SmartMotorConfig call.
SmartMotorSetup(1, 10)

# SmartMotorConfig(card, asynPort, [movingPollMs], [idlePollMs]) - one motor.
SmartMotorConfig(0, "$(SERIAL)", 100, 1000)

# Motor record (DTYP SMARTMOTOR_$(CARD)_0). The SmartMotor works in encoder
# counts, so MRES = 1 and EGU = counts.
dbLoadRecords("db/smartmotor.template", "P=$(P),M=m1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor SM:m1 SM:m1.RBV
#   caput SM:m1 100000
