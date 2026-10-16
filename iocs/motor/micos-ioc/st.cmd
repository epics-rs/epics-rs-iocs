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
epicsEnvSet("CTRL",   "MICOS1")
epicsEnvSet("IPPORT", "corvusPort")
epicsEnvSet("HOST",   "192.168.1.170:2103")
epicsEnvSet("NAXES",  "3")

# ---- asyn octet port (Ethernet; use the serial line for RS-232) ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyS1", 0, 0, 0)

# Port EOS as the C example sets it (motor.cmd.SMCcorvus:14-15);
# the driver sends bare commands.
asynOctetSetInputEos("$(IPPORT)", 0, "\r\n")
asynOctetSetOutputEos("$(IPPORT)", 0, "\r\n")

# ---- corvus controller ----
# SMCcorvusCreateController(portName, corvusPort, numAxes, movingPollPeriod, idlePollPeriod).
SMCcorvusCreateController("$(CTRL)", "$(IPPORT)", $(NAXES), 100, 500)

# One motor record per axis (OUT @asyn($(CTRL),axis)). The driver reports
# positions in controller-native engineering units; the template's MRES is the
# record step.
dbLoadRecords("$(MICOS_IOC)/db/corvus.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=0")
dbLoadRecords("$(MICOS_IOC)/db/corvus.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=1")
dbLoadRecords("$(MICOS_IOC)/db/corvus.template", "P=$(P),M=m3,PORT=$(CTRL),ADDR=2")

iocInit()

# Example:
#   dbl
#   camonitor micos:m1 micos:m1.RBV
#   caput micos:m1 5.0
