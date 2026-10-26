#============================================================
# st-pc6k.cmd — Parker 6K model-1 (drvPC6K.cc) startup script
#
# Usage:
#   cargo run -p parker-ioc -- st-pc6k.cmd
#============================================================

epicsEnvSet("P",     "PC6K:")
epicsEnvSet("PORT0", "serial1")
epicsEnvSet("PORT1", "serial3")

# The 6Ks' command ports (README_PC6K: 192.168.10.30, port 5002; C's
# PC6K.cmd: 192.168.10.29, port 5002).
drvAsynIPPortConfigure("$(PORT0)", "192.168.10.30:5002", 0, 0, 0)
drvAsynIPPortConfigure("$(PORT1)", "192.168.10.29:5002", 0, 0, 0)

# PC6KSetup(num_cards, scan_rate), PC6KConfig(card, asyn port).
PC6KSetup(2, 60)
PC6KConfig(0, "$(PORT0)")
PC6KConfig(1, "$(PORT1)")

# Each axis's motor.db, as C's PC6K.cmd loads it, and motorUtil's records,
# as C's st.cmd loads them.
dbLoadTemplate("$(PARKER_IOC)/db/PC6K.substitutions", "P=$(P)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")
