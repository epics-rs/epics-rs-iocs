#============================================================
# st.hydra.cmd — Micos SMC hydra motor IOC startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.hydra.cmd
#
# Requires an SMC hydra controller reachable over serial (or swap in the IP
# port line below).
#============================================================

epicsEnvSet("P",      "micos:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "hydraPort")
epicsEnvSet("NAXES",  "2")

# ---- asyn octet port (serial; use the IP line for Ethernet) ----
drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)
#drvAsynIPPortConfigure("$(IPPORT)", "192.168.1.17:4001", 0, 0, 0)

# hydra framing: the driver appends the CR/LF command terminator, so only the
# input EOS is configured here. Do not set an output EOS — the port would append
# it a second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")

# ---- hydra controller ----
# SMChydraCreateController(card, hydraPort, numAxes, [movingPollMs], [idlePollMs]).
SMChydraCreateController(0, "$(IPPORT)", $(NAXES), 100, 500)

# One motor record per axis (DTYP HYDRA_$(CARD)_0, _1). The driver reports
# positions in controller-native engineering units, so MRES = 1.
dbLoadRecords("db/hydra.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/hydra.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
