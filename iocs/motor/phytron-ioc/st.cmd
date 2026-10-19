#============================================================
# st.cmd — Phytron phyMOTION (MCM) motor IOC startup script
#
# Usage:
#   cargo run -p phytron-ioc -- st.cmd
#
# Requires a Phytron phyMOTION controller reachable over TCP (or swap in the
# serial port line below).
#============================================================

epicsEnvSet("P",      "phytron:")
epicsEnvSet("CTRL",   "phyPort")
epicsEnvSet("IPPORT", "phyAsyn")
epicsEnvSet("HOST",   "192.168.1.20:22222")

# ---- asyn octet port (Ethernet; use the serial line for USB/RS-232) ----
# noProcessEos=1 as in C's iocBoot: the driver frames STX..ETX both ways and
# needs the ETX in the reply, which an input EOS of "\x03" would strip.
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 1)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 1)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# ---- phyMOTION controller ----
# phytronCreateController(controllerName, asynPort, movingPollPeriod,
#                         idlePollPeriod, timeout, noResetAtBoot), all in ms.
# noResetAtBoot=1 skips the boot reset (a reset waits ~5 s, then polls the
# controller for up to 120 s). Pass 0 to reset the controller at boot like C.
phytronCreateController("$(CTRL)", "$(IPPORT)", 100, 1000, 1000, 1)

# One axis per I1AM01 module.index: phytronCreateAxis(controllerName, module,
# index). Load one motor record per axis with ADDR = module*10 + index.
phytronCreateAxis("$(CTRL)", 1, 1)
dbLoadRecords("$(PHYTRON_IOC)/db/phytron.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=11")

iocInit()

# Example:
#   dbl
#   camonitor phytron:m1 phytron:m1.RBV
#   caput phytron:m1 1000
