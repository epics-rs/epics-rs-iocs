#============================================================
# st.cmd — Animatics SmartMotor servo motor IOC startup script
#
# Usage:
#   cargo run -p smartmotor-ioc -- st.cmd
#
# Requires an Animatics SmartMotor, or a daisy chain of them, on the serial line
# below. SmartMotorConfig probes the motors (RBe), so they must be connected
# when the command runs.
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

# ---- SmartMotor controller ----
# SmartMotorSetup(maxControllers, [scanRate]) is accepted for startup-script
# parity; the asyn-rs port allocates per SmartMotorConfig call.
SmartMotorSetup(1, 10)

# SmartMotorConfig(card, asynPort, [movingPollMs], [idlePollMs]) - one DTYP
# per motor found, SMARTMOTOR_$(CARD)_0 up the chain.
SmartMotorConfig(0, "$(SERIAL)", 100, 1000)

# Motor record for chain axis 0; load one per motor with AXIS=n. The
# SmartMotor works in encoder counts, so MRES = 1 and EGU = counts.
dbLoadRecords("$(SMARTMOTOR_IOC)/db/smartmotor.template", "P=$(P),M=m1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor SM:m1 SM:m1.RBV
#   caput SM:m1 100000
