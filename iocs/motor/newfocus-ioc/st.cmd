#============================================================
# st.cmd — Newport New Focus PMNC 8750/8752 picomotor IOC startup script
#
# Usage:
#   cargo run -p newfocus-ioc -- st.cmd
#
# Requires a New Focus PMNC 8750/8752 network controller reachable at the
# HOST below. PMNC87xxConfig identifies the controller (VER) and autodiscovers
# its driver modules and channels (MPV/DRT), so it must be reachable when the
# command runs.
#============================================================

epicsEnvSet("P",     "NF:")
epicsEnvSet("ASYN",  "PMNC1")
epicsEnvSet("HOST",  "192.168.0.100:23")

# ---- TCP octet port ----
drvAsynIPPortConfigure("$(ASYN)", "$(HOST)", 0, 0, 0)

# ---- PMNC controller ----
# PMNC87xxSetup(num_cards, num_drivers, scan_rate in Hz)
PMNC87xxSetup(1, 2, 10)
# PMNC87xxConfig(card, asyn port): one motor axis per discovered channel.
PMNC87xxConfig(0, "$(ASYN)")

# One motor record per discovered channel, signal S of card 0.
# The template takes C's PMNC87xx.substitutions values (MRES 2.5e-4).
dbLoadRecords("$(NEWFOCUS_IOC)/db/newfocus.template", "P=$(P),M=m1,C=0,S=0")

iocInit()

# Example:
#   dbl
#   camonitor NF:m1 NF:m1.RBV
#   caput NF:m1 1.5
