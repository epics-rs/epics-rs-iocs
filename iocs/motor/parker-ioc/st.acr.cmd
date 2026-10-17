#============================================================
# st.acr.cmd — Parker ACR / Aries motor IOC startup script
#
# Usage:
#   cargo run -p parker-ioc -- st.acr.cmd
#
# Requires a Parker ACR-series controller (e.g. Aries) reachable over TCP.
#============================================================

epicsEnvSet("P",      "ACR:")
epicsEnvSet("CTRL",   "PARKER1")
epicsEnvSet("IPPORT", "ARIES1")
epicsEnvSet("HOST",   "gse-aries1:5002")
epicsEnvSet("NAXES",  "1")

# ---- asyn IP octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# ACR framing: CR both ways, set on the port as C's st.cmd.acr does.
asynOctetSetInputEos("$(IPPORT)", 0, "\r")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r")

# ---- ACR controller ----
# ACRCreateController(portName, ACRPortName, numAxes, movingPollPeriod, idlePollPeriod).
ACRCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 20, 1000)

# One motor record per axis (OUT @asyn($(CTRL),axis)).
dbLoadRecords("$(PARKER_IOC)/db/acr.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")

iocInit()

# Example:
#   dbl
#   camonitor ACR:m1 ACR:m1.RBV
#   caput ACR:m1 10000
