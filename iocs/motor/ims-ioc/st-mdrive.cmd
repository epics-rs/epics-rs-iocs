#============================================================
# st-mdrive.cmd — IMS MDrive model-1 (drvMDrive.cc) startup script
#
# Usage:
#   cargo run -p ims-ioc -- st-mdrive.cmd
#============================================================

epicsEnvSet("P",    "ims:")
epicsEnvSet("PORT", "serial2")
epicsEnvSet("TTY",  "/dev/ttyUSB0")

drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# MDriveSetup(num_chains, scan_rate), MDriveConfig(chain, asyn port);
# drives answer to party names 1..8, signal 0 is drive 1.
MDriveSetup(1, 10)
MDriveConfig(0, "$(PORT)")

# motorUtil's records and the drive's motor.db, as C's st.cmd and
# MDrive.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(IMS_IOC)/db/MDrive.substitutions", "P=$(P),C=0")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")
