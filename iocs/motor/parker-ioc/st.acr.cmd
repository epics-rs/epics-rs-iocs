#============================================================
# st.acr.cmd — Parker ACR / Aries motor IOC startup script
#
# Usage:
#   cargo run -p parker-ioc -- st.acr.cmd
#
# Requires a Parker ACR-series controller (e.g. Aries) reachable over TCP.
#============================================================

epicsEnvSet("P",      "ACR:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("IPPORT", "ARIES1")
epicsEnvSet("HOST",   "gse-aries1:5002")
epicsEnvSet("NAXES",  "1")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ACR framing: the driver appends the CR command terminator, so only the input
# EOS is configured here. Do not set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\r")

# ---- ACR controller ----
# ACRCreateController(card, acrPort, numAxes, [movingPollMs], [idlePollMs]).
ACRCreateController(0, "$(IPPORT)", $(NAXES), 20, 1000)

# One motor record per axis (DTYP ACR_$(CARD)_0, _1, ...). The driver reports
# positions in controller counts (PPU divides commands, not readback), so
# MRES = 1.
dbLoadRecords("db/acr.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor ACR:m1 ACR:m1.RBV
#   caput ACR:m1 10000
