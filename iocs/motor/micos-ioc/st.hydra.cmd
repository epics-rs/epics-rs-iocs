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
epicsEnvSet("CTRL",   "MICOS1")
epicsEnvSet("IPPORT", "hydraPort")
epicsEnvSet("NAXES",  "2")

# ---- asyn octet port (serial; use the IP line for Ethernet) ----
drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)
#drvAsynIPPortConfigure("$(IPPORT)", "192.168.1.17:4001", 0, 0, 0)

# Port EOS as the C example sets it (motor.cmd.SMChydra:14-15);
# the driver sends bare commands.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r\n")

# ---- hydra controller ----
# SMChydraCreateController(portName, hydraPort, numAxes, movingPollPeriod, idlePollPeriod).
SMChydraCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 500)

# One motor record per axis (OUT @asyn($(CTRL),axis)). The driver scales the
# record's steps by the axis axisRes, as C.
dbLoadRecords("$(MICOS_IOC)/db/hydra.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
dbLoadRecords("$(MICOS_IOC)/db/hydra.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=1")

iocInit()

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
