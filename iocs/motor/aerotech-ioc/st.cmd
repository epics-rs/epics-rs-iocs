#============================================================
# st.cmd — Aerotech Ensemble motor IOC startup script
#
# Usage:
#   cargo run -p aerotech-ioc -- st.cmd
#
# Requires an Aerotech Ensemble controller reachable over TCP (or swap in the
# serial port line below). Homing uses the vendor HomeAsync.bcx program (task
# 5), which must be loaded on the controller.
#============================================================

epicsEnvSet("P",      "aerotech:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "ensAsyn")
epicsEnvSet("HOST",   "192.168.1.40:8000")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "115200")

# Ensemble framing: replies end with "\n"; the driver owns the "\n" output
# terminator, so only the input EOS is set here. Do NOT set an output EOS.
asynOctetSetInputEos("$(IPPORT)", 0, "\n")

# ---- Ensemble controller ----
# EnsembleAsynConfig(card, asynPort, numAxes, [movingPollMs], [idlePollMs],
#                    [timeoutMs]). Probes axes 0.. and attaches the first numAxes
# that exist; DTYP is ENSEMBLE_{card}_{axis} using the controller axis number.
EnsembleAsynConfig("$(CARD)", "$(IPPORT)", 1, 100, 1000, 2000)

# One motor record per axis. AXIS is the controller axis number (0 here).
dbLoadRecords("db/aerotech.template", "P=$(P),M=m1,CTRL=$(CARD),AXIS=0")

iocInit()

# Example:
#   dbl
#   camonitor aerotech:m1 aerotech:m1.RBV
#   caput aerotech:m1 10
