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
epicsEnvSet("CTRL",   "MICOS1")
epicsEnvSet("IPPORT", "taurusPort")
epicsEnvSet("HOST",   "192.168.1.18:4001")
epicsEnvSet("NAXES",  "1")

# ---- asyn octet port (Ethernet; use the serial line for RS-232) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)

# Port EOS as C needs it: SMCTaurusDriver sets no EOS and sends bare
# commands, as the hydra sibling (motor.cmd.SMChydra:14-15).
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r\n")

# ---- Taurus controller ----
# SMCTaurusCreateController(portName, taurusPort, numAxes, movingPollPeriod, idlePollPeriod).
SMCTaurusCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 500)

# One motor record per axis (OUT @asyn($(CTRL),axis)). The driver scales the
# record's steps by the axis axisRes, as C.
dbLoadRecords("$(MICOS_IOC)/db/taurus.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")

iocInit()

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
