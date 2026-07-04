#============================================================
# st.cmd — Micos SMC corvus motor IOC startup script
#
# Usage:
#   cargo run -p micos-ioc -- st.cmd
#
# Requires an SMC corvus controller reachable over TCP (or swap in the serial
# port line below).
#============================================================

epicsEnvSet("P",      "micos:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "corvusPort")
epicsEnvSet("HOST",   "192.168.1.170:2103")
epicsEnvSet("NAXES",  "3")

# ---- asyn octet port (Ethernet; use the serial line for RS-232) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)

# corvus framing: the driver appends the CR/LF command terminator, so only the
# input EOS is configured here. Do not set an output EOS — the port would append
# it a second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")

# ---- corvus controller ----
# SMCcorvusCreateController(card, corvusPort, numAxes, [movingPollMs], [idlePollMs]).
SMCcorvusCreateController(0, "$(IPPORT)", $(NAXES), 100, 500)

# One motor record per axis (DTYP CORVUS_$(CARD)_0, _1, _2). The driver reports
# positions in controller-native engineering units, so MRES = 1.
dbLoadRecords("db/corvus.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/corvus.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/corvus.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
