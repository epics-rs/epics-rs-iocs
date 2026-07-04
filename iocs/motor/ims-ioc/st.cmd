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
epicsEnvSet("MPORT",  "M06")
epicsEnvSet("IPPORT", "imsAsyn")
epicsEnvSet("HOST",   "192.168.1.60:2101")
epicsEnvSet("DEVNAME", "")

# ---- asyn octet port ----
drvAsynIPPortConfigure("$(IPPORT)", "$(HOST)", 0, 0, 0)
#drvAsynSerialPortConfigure("$(IPPORT)", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("$(IPPORT)", -1, "baud", "9600")

# IMS framing: queries reply terminated by "\n"; the driver owns the output
# terminator (\r\n, or \r/\n for a Lexium MDrive), so only the input EOS is set
# here. Do NOT set an output EOS.
asynOctetSetInputEos("$(IPPORT)", 0, "\n")

# ---- IMS controller ----
# ImsMDrivePlusCreateController(motorPort, ioPort, [deviceName], [movingPollMs],
#                              [idlePollMs], [timeoutMs]). One axis; DTYP is the
#                              motorPort string. Empty deviceName = no party mode.
ImsMDrivePlusCreateController("$(MPORT)", "$(IPPORT)", "$(DEVNAME)", 100, 1000, 2000)

# The single motor record. DTYP matches the motorPort.
dbLoadRecords("db/ims.template", "P=$(P),M=m1,MPORT=$(MPORT)")

iocInit()

# Example:
#   dbl
#   camonitor ims:m1 ims:m1.RBV
#   caput ims:m1 10000
