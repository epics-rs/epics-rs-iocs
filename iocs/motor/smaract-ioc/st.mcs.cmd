#============================================================
# st.mcs.cmd — SmarAct MCS (RS-232) motor IOC startup script
#
# Usage:
#   cargo run -p smaract-ioc -- st.mcs.cmd
#
# The MCS is commonly reached over a terminal server (drvAsynIPPort). Use a RAW
# connection, not TELNET — the driver does not slurp telnet negotiation bytes.
# The controller is created first, then one axis per channel.
#============================================================

epicsEnvSet("P",       "MCS:")
epicsEnvSet("CARD",    "0")
epicsEnvSet("IPPORT",  "MCS")
epicsEnvSet("HOST",    "192.168.1.210:5000")

# ---- asyn IP octet port (RAW terminal server) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)

# MCS framing: the driver appends the LF command terminator, so only the input
# EOS is configured here. Do not set an output EOS — the port would append it a
# second time.
asynOctetSetInputEos("$(IPPORT)", 0, "\n")

# ---- MCS controller + axes ----
# smarActMCSCreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs],
#                           [disableSpeed]). disableSpeed non-zero suppresses SCLS.
smarActMCSCreateController(0, "$(IPPORT)", 3, 50, 1000, 0)

# smarActMCSCreateAxis(card, axisNo, channel): axisNo is 0-based (DTYP
# MCS_$(CARD)_$(N)), channel is the controller channel it drives.
smarActMCSCreateAxis(0, 0, 0)
smarActMCSCreateAxis(0, 1, 1)
smarActMCSCreateAxis(0, 2, 2)

# One motor record per axis. The driver reports the controller's raw position in
# nanometres (linear) / micro-degrees (rotary) with no scaling, so MRES = 1e-6
# reads the record in millimetres (linear) / degrees (rotary).
dbLoadRecords("db/mcs.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/mcs.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")
dbLoadRecords("db/mcs.template", "P=$(P),M=m3,N=2,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor MCS:m1 MCS:m1.RBV
#   caput MCS:m1 1.0
