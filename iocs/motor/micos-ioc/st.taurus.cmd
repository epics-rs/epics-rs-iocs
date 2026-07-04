#============================================================
# st.taurus.cmd — Micos SMC Taurus motor IOC startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.taurus.cmd
#
# Requires an SMC Taurus controller reachable over TCP (or swap in the serial
# port line below).
#============================================================

epicsEnvSet("P",      "micos:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "taurusPort")
epicsEnvSet("HOST",   "192.168.1.18:4001")
epicsEnvSet("NAXES",  "1")

# ---- asyn octet port (Ethernet; use the serial line for RS-232) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)

# Taurus framing: the driver appends the CR/LF command terminator, so only the
# input EOS is configured here. Do not set an output EOS — the port would append
# it a second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")

# ---- Taurus controller ----
# SMCTaurusCreateController(card, taurusPort, numAxes, [movingPollMs], [idlePollMs]).
SMCTaurusCreateController(0, "$(IPPORT)", $(NAXES), 100, 500)

# One motor record per axis (DTYP TAURUS_$(CARD)_0, ...). The driver reports
# positions in controller-native engineering units, so MRES = 1.
dbLoadRecords("db/taurus.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
