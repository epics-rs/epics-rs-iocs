#============================================================
# st-mdrive.cmd — IMS MDrive model-1 (drvMDrive.cc) startup script
#
# Usage:
#   cargo run -p ims-ioc -- st-mdrive.cmd
#============================================================

epicsEnvSet("P",    "ims:")
epicsEnvSet("PORT", "serial1")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# MDriveSetup(num_chains, scan_rate), MDriveConfig(chain, asyn port);
# drives answer to party names 1..8, signal 0 is drive 1.
MDriveSetup(1, 10)
MDriveConfig(0, "$(PORT)")

dbLoadRecords("$(IMS_IOC)/db/im483-m1.template", "P=$(P),M=m0,DTYP=MDrive,C=0,S=0")
dbLoadRecords("$(IMS_IOC)/db/im483-m1.template", "P=$(P),M=m1,DTYP=MDrive,C=0,S=1")

iocInit()
