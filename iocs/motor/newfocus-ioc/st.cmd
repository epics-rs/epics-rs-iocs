#============================================================
# st.cmd — Newport New Focus PMNC 8750/8752 picomotor IOC startup script
#
# Usage:
#   cargo run -p newfocus-ioc -- st.cmd
#
# Requires a New Focus PMNC 8750/8752 network controller reachable at the
# HOST below. PMNC87xxConfig identifies the controller (VER) and autodiscovers
# its driver modules and channels (MPV/DRT), so it must be reachable when the
# command runs. The 8742 at NF_HOST is created as C's newfocus8742.cmd
# creates it.
#============================================================

epicsEnvSet("P",     "NF:")
epicsEnvSet("ASYN",  "serial1")
epicsEnvSet("HOST",  "192.168.0.100:23")
epicsEnvSet("NF_PORT", "NF1")
epicsEnvSet("NF_HOST", "192.168.0.101:23")

# ---- TCP octet port ----
drvAsynIPPortConfigure("$(ASYN)", "$(HOST)", 0, 0, 0)

# ---- PMNC controller ----
# PMNC87xxSetup(num_cards, num_drivers, scan_rate in Hz)
PMNC87xxSetup(1, 2, 10)
# PMNC87xxConfig(card, asyn port): one motor axis per discovered channel.
PMNC87xxConfig(0, "$(ASYN)")

# ---- 8742 controller ----
drvAsynIPPortConfigure("$(NF_PORT)_ETH", "$(NF_HOST)")
asynOctetSetInputEos("$(NF_PORT)_ETH", 0, "\r\n")
asynOctetSetOutputEos("$(NF_PORT)_ETH", 0, "\r")
# nf874xCreateController(portName, nf874xPortName, numAxes, movingPollPeriod,
#                        idlePollPeriod), periods in ms. C's newfocus8742.cmd
# loads its records with P=$(PREFIX), which C's st.cmd never sets, so it
# loads none; neither does this script.
nf874xCreateController("$(NF_PORT)", "$(NF_PORT)_ETH", 3, 200, 1000)

# The PMNC's motor.db, as C's PMNC87xx.cmd loads it, and motorUtil's
# records, as C's st.cmd loads them.
dbLoadTemplate("$(NEWFOCUS_IOC)/db/PMNC87xx.substitutions", "P=$(P)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor NF:m1 NF:m1.RBV
#   caput NF:m1 1.5
