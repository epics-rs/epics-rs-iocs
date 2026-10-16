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

dbLoadRecords("$(PARKER_IOC)/db/pc6k.template", "P=$(P),M=m1,C=0,S=0")

iocInit()
