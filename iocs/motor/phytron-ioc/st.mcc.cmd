#============================================================
# st.mcc.cmd — Phytron MCC-1 / MCC-2 motor IOC startup script
#
# Usage:
#   cargo run -p phytron-ioc -- st.mcc.cmd
#
# Requires a Phytron MCC controller reachable over serial (or TCP).
#============================================================

epicsEnvSet("P",      "phytron:")
epicsEnvSet("CTRL",   "mccPort")
epicsEnvSet("IPPORT", "mccAsyn")
epicsEnvSet("HOST",   "192.168.1.21:22222")
epicsEnvSet("ADDR",   "0")

# ---- asyn octet port (serial is typical for the MCC; TCP also works) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "57600")

# The MCC replies end with ETX (0x03); the driver owns the STX..ETX output
# framing (no CRC on the MCC), so only the input EOS is set here.
asynOctetSetInputEos("$(IPPORT)", 0, "\x03")

# ---- MCC controller ----
# phytronCreateMCC(controllerName, asynPort, address, [movingPollMs],
#                  [idlePollMs], [timeoutMs], [noResetAtBoot]).
phytronCreateMCC("$(CTRL)", "$(IPPORT)", $(ADDR), 100, 1000, 1000, 1)

# MCC axes are single-digit 1..8: phytronCreateAxis(controllerName, module=0,
# index). The axis command prefix is the index digit.
phytronCreateAxis("$(CTRL)", 0, 1)
dbLoadRecords("db/phytron.template", "P=$(P),M=m1,CTRL=$(CTRL),MODULE=0,INDEX=1")

iocInit()

# Example:
#   dbl
#   camonitor phytron:m1 phytron:m1.RBV
#   caput phytron:m1 1000
