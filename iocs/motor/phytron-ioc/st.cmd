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
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# Phytron framing: replies end with ETX (0x03); the driver owns the STX..ETX
# output framing, so only the input EOS is set here. Do NOT set an output EOS.
asynOctetSetInputEos("$(IPPORT)", 0, "\x03")

# ---- phyMOTION controller ----
# phytronCreatePhymotion(controllerName, asynPort, [movingPollMs], [idlePollMs],
#                        [timeoutMs], [noResetAtBoot]).
# noResetAtBoot=1 skips the boot reset (a reset waits ~5 s, then polls the
# controller for up to 120 s). Pass 0 to reset the controller at boot like C.
phytronCreatePhymotion("$(CTRL)", "$(IPPORT)", 100, 1000, 1000, 1)

# One axis per I1AM01 module.index: phytronCreateAxis(controllerName, module,
# index). Load one motor record per axis with matching MODULE/INDEX.
phytronCreateAxis("$(CTRL)", 1, 1)
dbLoadRecords("db/phytron.template", "P=$(P),M=m1,CTRL=$(CTRL),MODULE=1,INDEX=1")

iocInit()

# Example:
#   dbl
#   camonitor phytron:m1 phytron:m1.RBV
#   caput phytron:m1 1000
