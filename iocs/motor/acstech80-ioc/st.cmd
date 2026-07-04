#============================================================
# st.cmd — ACS Tech80 SPiiPlus motion controller IOC startup script
#
# Usage:
#   cargo run -p acstech80-ioc -- st.cmd
#
# Requires a SPiiPlus controller reachable on the port below. SPiiPlusConfig
# identifies the controller (?VR) and auto-detects the axis count (?APOS), so it
# must be connected when the command runs.
#
# The SPiiPlus is commonly reached over TCP; switch the port section below to
# drvAsynIPPortConfigure if using Ethernet.
#============================================================

epicsEnvSet("P",      "ACS:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("PORT",   "ACS1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port (RS-232) ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)")
asynSetOption("$(PORT)", 0, "baud",   "115200")
asynSetOption("$(PORT)", 0, "bits",   "8")
asynSetOption("$(PORT)", 0, "parity", "none")
asynSetOption("$(PORT)", 0, "stop",   "1")

# ---- or TCP (uncomment for Ethernet; comment out the serial block above) ----
# drvAsynIPPortConfigure("$(PORT)", "192.168.0.10:701", 0, 0, 0)

# SPiiPlus framing: the driver appends the CR command terminator, so only the
# input EOS is configured here (replies are CR terminated). Do not set an output
# EOS — the port would append it a second time.
asynOctetSetInputEos("$(PORT)", 0, "\r")

# ---- SPiiPlus controller ----
# SPiiPlusSetup(maxControllers, [scanRate]) is accepted for startup-script
# parity; the asyn-rs port allocates per SPiiPlusConfig call.
SPiiPlusSetup(1, 10)

# SPiiPlusConfig(card, asynPort, [mode], [movingPollMs], [idlePollMs]) - mode is
# BUF (default; ACSPL program buffers), DIR (direct command interpreter), or CON
# (kinematic CONNECT). Axis count is auto-detected.
SPiiPlusConfig(0, "$(PORT)", "DIR", 100, 1000)

# One motor record per axis (DTYP SPIIPLUS_$(CARD)_0, _1, ...). The SPiiPlus
# works in controller counts, so MRES = 1 and EGU = counts.
dbLoadRecords("db/spiiplus.template", "P=$(P),M=m1,N=0,CARD=$(CARD)")
dbLoadRecords("db/spiiplus.template", "P=$(P),M=m2,N=1,CARD=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor ACS:m1 ACS:m1.RBV
#   caput ACS:m1 100000
