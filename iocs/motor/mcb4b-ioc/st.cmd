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
epicsEnvSet("CARD",     "0")
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
# MCB4BCreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]).
# card is the DTYP prefix (MCB4B_{card}_{index}); axes are 0-based, matching the
# C "#%02d" wire prefix.
MCB4BCreateController($(CARD), "$(PORT)", $(NUM_AXES), 100, 1000)

# One motor record per axis (the MCB-4B works in raw controller steps, MRES=1).
dbLoadRecords("db/mcb4b.template", "P=$(P),M=m0,CARD=$(CARD),AXIS=0")
dbLoadRecords("db/mcb4b.template", "P=$(P),M=m1,CARD=$(CARD),AXIS=1")
dbLoadRecords("db/mcb4b.template", "P=$(P),M=m2,CARD=$(CARD),AXIS=2")
dbLoadRecords("db/mcb4b.template", "P=$(P),M=m3,CARD=$(CARD),AXIS=3")

iocInit()

# Example:
#   dbl
#   camonitor acs:m0 acs:m0.RBV
#   caput acs:m0 1000
