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
# noProcessEos=1 as in C's iocBoot: the driver frames STX..ETX both ways and
# needs the ETX in the reply, which an input EOS of "\x03" would strip.
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 1)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 1)
#asynSetOption("$(IPPORT)", -1, "baud", "57600")

# ---- MCC controller ----
# phytronCreateMCC(controllerName, asynPort, address, movingPollPeriod,
#                  idlePollPeriod, timeout, noResetAtBoot), all in ms.
phytronCreateMCC("$(CTRL)", "$(IPPORT)", $(ADDR), 100, 1000, 1000, 1)

# MCC axes are single-digit 1..8: phytronCreateAxis(controllerName, module=0,
# index). The axis command prefix is the index digit.
phytronCreateAxis("$(CTRL)", 0, 1)
dbLoadRecords("$(PHYTRON_IOC)/db/phytron.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=1")

iocInit()

# Example:
#   dbl
#   camonitor phytron:m1 phytron:m1.RBV
#   caput phytron:m1 1000
