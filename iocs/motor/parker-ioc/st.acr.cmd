#============================================================
# st.acr.cmd — Parker ACR / Aries motor IOC startup script
#
# Usage:
#   cargo run -p parker-ioc -- st.acr.cmd
#
# Requires two Parker ACR-series controllers (e.g. Aries) reachable over TCP.
#============================================================

epicsEnvSet("P",       "ACR:")
epicsEnvSet("CTRL1",   "ACR1")
epicsEnvSet("IPPORT1", "ARIES1")
epicsEnvSet("HOST1",   "gse-aries1:5002")
epicsEnvSet("CTRL2",   "ACR2")
epicsEnvSet("IPPORT2", "ARIES2")
epicsEnvSet("HOST2",   "gse-aries2:5002")
epicsEnvSet("NAXES",   "1")

# ---- asyn IP octet ports ----
drvAsynIPPortConfigure("$(IPPORT1)", "$(HOST1)", 0, 0, 0)
drvAsynIPPortConfigure("$(IPPORT2)", "$(HOST2)", 0, 0, 0)

# ACR framing: CR both ways, set on each port as C's st.cmd.acr does.
asynOctetSetInputEos("$(IPPORT1)", 0, "\r")
asynOctetSetOutputEos("$(IPPORT1)", 0, "\r")
asynOctetSetInputEos("$(IPPORT2)", 0, "\r")
asynOctetSetOutputEos("$(IPPORT2)", 0, "\r")

# ---- ACR controllers ----
# ACRCreateController(portName, ACRPortName, numAxes, movingPollPeriod, idlePollPeriod).
ACRCreateController("$(CTRL1)", "$(IPPORT1)", $(NAXES), 20, 1000)
ACRCreateController("$(CTRL2)", "$(IPPORT2)", $(NAXES), 20, 1000)

# Each controller's basic_asyn_motor.db and ACRAux*.template records, and
# each octet port's asynRecord, as C's st.cmd.acr loads them. C binds
# asyn2 to ARIES1 beside asyn1, leaving ARIES2 without one; it is bound to
# ARIES2 here.
dbLoadTemplate("$(PARKER_IOC)/db/motor.substitutions.acr", "P=$(P),PORT=$(CTRL1),N=1")
dbLoadTemplate("$(PARKER_IOC)/db/motor.substitutions.acr", "P=$(P),PORT=$(CTRL2),N=2")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=asyn1,PORT=$(IPPORT1),ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=asyn2,PORT=$(IPPORT2),ADDR=0,OMAX=256,IMAX=256")

iocInit()

# As C's st.cmd.acr, which uses no save/restore.
dbpf("$(P)m1.RTRY", "0")
dbpf("$(P)m1.TWV", "0.1")
dbpf("$(P)m2.RTRY", "0")
dbpf("$(P)m2.TWV", "0.1")

# Example:
#   dbl
#   camonitor ACR:m1 ACR:m1.RBV
#   caput ACR:m1 10000
