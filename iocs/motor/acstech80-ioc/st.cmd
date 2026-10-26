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
epicsEnvSet("PORT",   "serial61")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port (RS-232) ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)")
asynSetOption("$(PORT)", 0, "baud",   "115200")
asynSetOption("$(PORT)", 0, "bits",   "8")
asynSetOption("$(PORT)", 0, "parity", "none")
asynSetOption("$(PORT)", 0, "stop",   "1")

# ---- or TCP (uncomment for Ethernet; comment out the serial block above) ----
# drvAsynIPPortConfigure("$(PORT)", "192.168.0.10:701", 0, 0, 0)

# ---- SPiiPlus controller ----
# SPiiPlusSetup(num_cards, scan_rate in Hz)
SPiiPlusSetup(1, 60)

# SPiiPlusConfig(card, asyn port, mode) - mode is
# BUF (default; ACSPL program buffers), DIR (direct command interpreter), or CON
# (kinematic CONNECT). Axis count is auto-detected.
SPiiPlusConfig(0, "$(PORT)", "BUF")

# motorUtil's records and one motor.db per axis (DTYP "SPiiPlus", OUT
# #C$(CARD) S0, S1, ...), as C's st.cmd and SPiiPlus.cmd load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(ACSTECH80_IOC)/db/SPiiPlus.substitutions", "P=$(P),C=$(CARD)")

iocInit()

# Example:
#   dbl
#   camonitor ACS:m1 ACS:m1.RBV
#   caput ACS:m1 10
