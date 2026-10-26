#============================================================
# st-pc6k.cmd — Parker 6K model-1 (drvPC6K.cc) startup script
#
# Usage:
#   cargo run -p parker-ioc -- st-pc6k.cmd
#============================================================

epicsEnvSet("P",    "PC6K:")
epicsEnvSet("PORT", "serial1")

# The 6K's command port (README_PC6K: 192.168.10.30, port 5002).
drvAsynIPPortConfigure("$(PORT)", "192.168.10.30:5002", 0, 0, 0)

# PC6KSetup(num_cards, scan_rate), PC6KConfig(card, asyn port).
PC6KSetup(1, 60)
PC6KConfig(0, "$(PORT)")

# The axis's motor.db, as C's PC6K.cmd loads it, and motorUtil's records,
# as C's st.cmd loads them.
dbLoadTemplate("$(PARKER_IOC)/db/PC6K.substitutions", "P=$(P)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")
