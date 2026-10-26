#============================================================
# st.cmd — IMS MDrivePlus motor IOC startup script
#
# Usage:
#   cargo run -p ims-ioc -- st.cmd
#
# Requires an IMS MDrivePlus / MForce / Lexium controller reachable over TCP (or
# swap in the serial port line below). One controller drives one axis. For a
# party-mode multidrop bus, set a non-empty device name.
#============================================================

epicsEnvSet("P",      "ims:")
epicsEnvSet("MPORT",  "M1")
epicsEnvSet("IPPORT", "P1")
epicsEnvSet("HOST",   "192.168.1.60:2101")
epicsEnvSet("DEVNAME", "")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "9600")

# ---- IMS controller ----
# ImsMDrivePlusCreateController(motorPortName, IOPortName, devName,
#                              movingPollPeriod, idlePollPeriod), periods in ms.
#                              One axis; empty devName = no party mode.
ImsMDrivePlusCreateController("$(MPORT)", "$(IPPORT)", "$(DEVNAME)", 100, 1000)

# motorUtil's records, the single axis's basic_asyn_motor.db, IMS_extra.db
# and the IP port's asyn record, as C's st.cmd and MDrive-Eth.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(IMS_IOC)/db/motor.substitutions.ims", "P=$(P),PORT=$(MPORT)")
dbLoadRecords("$(IMS_IOC)/db/IMS_extra.db", "P=$(P),R={Slt:1-Ax:T},PORT=$(MPORT),ADDR=0,TIMEOUT=1.0")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R={MC:1}Asyn,PORT=$(IPPORT),ADDR=80,OMAX=100,IMAX=100")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor ims:1 ims:1.RBV
#   caput ims:1 1
