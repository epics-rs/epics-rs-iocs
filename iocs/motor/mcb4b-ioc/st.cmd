#============================================================
# st.cmd — ACS MCB-4B stepper motor IOC startup script (motorAcs)
#
# Usage:
#   cargo run -p mcb4b-ioc -- st.cmd
#
# Requires an ACS MCB-4B controller reachable over serial (19200 8N1). The
# MCB-4B answers every command, so it must be powered on and wired when
# MCB4BCreateController runs its first poll.
#============================================================

epicsEnvSet("P",        "acs:")
epicsEnvSet("MCB4B",    "MCB4B1")
epicsEnvSet("PORT",     "serial1")
epicsEnvSet("TTY",      "/dev/ttyUSB0")
epicsEnvSet("NUM_AXES", "4")

# ---- asyn octet port ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
#drvAsynIPPortConfigure("$(PORT)", "192.168.1.100:4001", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "19200")
asynSetOption("$(PORT)", -1, "bits", "8")
asynSetOption("$(PORT)", -1, "stop", "1")
asynSetOption("$(PORT)", -1, "parity", "none")

# MCB-4B framing: the PORT owns it, not the driver. The C driver never embeds a
# terminator in its command strings, and the reference ACS_MCB4B.iocsh sets BOTH
# input and output EOS to a single CR. The asyn-rs EosInterpose layer appends
# the output EOS to every write, so the driver sends bare command bytes and must
# NOT append \r itself.
asynOctetSetInputEos( "$(PORT)", -1, "\r")
asynOctetSetOutputEos("$(PORT)", -1, "\r")

# ---- ACS MCB-4B controller ----
# MCB4BCreateController(portName, MCB4BPortName, numAxes, movingPollPeriod,
# idlePollPeriod), poll periods in ms. Axes are 0-based, matching the C "#%02d"
# wire prefix.
MCB4BCreateController("$(MCB4B)", "$(PORT)", $(NUM_AXES), 100, 5000)

# Each axis's basic_asyn_motor.db and the serial line's asyn record, as
# C's st.cmd.mcb4b loads them.
dbLoadTemplate("$(MCB4B_IOC)/db/motor.substitutions.mcb4b", "P=$(P),PORT=$(MCB4B)")
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(P),R=serial1,PORT=$(PORT),ADDR=0,OMAX=80,IMAX=80")

iocInit()

# This IOC does not use save/restore, so set values of some PVs
dbpf("$(P)m1.RTRY", "0")
dbpf("$(P)m1.TWV", "0.1")
dbpf("$(P)m2.RTRY", "0")
dbpf("$(P)m2.TWV", "0.1")
dbpf("$(P)m3.RTRY", "0")
dbpf("$(P)m3.TWV", "0.1")
dbpf("$(P)m4.RTRY", "0")
dbpf("$(P)m4.TWV", "0.1")

# Example:
#   dbl
#   camonitor acs:m1 acs:m1.RBV
#   caput acs:m1 1
