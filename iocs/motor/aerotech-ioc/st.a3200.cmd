#============================================================
# st.a3200.cmd — Aerotech A3200 motor IOC startup script
#
# Usage:
#   cargo run -p aerotech-ioc -- st.a3200.cmd
#
# Requires an Aerotech A3200 controller reachable over TCP (or swap in the
# serial port line below). Axes are addressed by name string; A3200AsynConfig
# discovers the name of each axis 0..numAxes and installs it at
# DTYP A3200_{card}_{axisName}.
#============================================================

epicsEnvSet("P",      "a3200:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "a3200Asyn")
epicsEnvSet("HOST",   "192.168.1.50:8000")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# A3200 framing: replies end with "\n"; the driver owns the "\n" output
# terminator, so only the input EOS is set here. Do NOT set an output EOS.
asynOctetSetInputEos("$(IPPORT)", 0, "\n")

# ---- A3200 controller ----
# A3200AsynConfig(card, asynPort, numAxes, [taskNumber], [linear], [movingPollMs],
#                 [idlePollMs], [timeoutMs]). Discovers axis names 0..numAxes and
#                 attaches each; DTYP is A3200_{card}_{axisName}.
A3200AsynConfig("$(CARD)", "$(IPPORT)", 1, 1, 1, 100, 1000, 2000)

# One motor record per axis. AXISNAME is the controller's axis name (X here).
dbLoadRecords("db/a3200.template", "P=$(P),M=m1,CTRL=$(CARD),AXISNAME=X")

iocInit()

# Example:
#   dbl
#   camonitor a3200:m1 a3200:m1.RBV
#   caput a3200:m1 10
