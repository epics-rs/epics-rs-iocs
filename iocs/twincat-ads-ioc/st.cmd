# TwinCAT ADS IOC startup script.
#
# Before this works, the PLC has to know us: in TwinCAT, System → Routes → Add
# route, with this machine's IP followed by ".1.1" as the AMS Net Id.

epicsEnvSet("PREFIX",                "ADS_IOC:ASYN:")
epicsEnvSet("PORT",                  "ADS_1")
epicsEnvSet("PLC_IP",                "192.168.88.63")
epicsEnvSet("PLC_AMS_NET_ID",        "$(PLC_IP).1.1")
epicsEnvSet("ADS_DEFAULT_PORT",      "851")
epicsEnvSet("PARAM_TABLE_SIZE",      "1000")
epicsEnvSet("PRIORITY",              "0")
epicsEnvSet("DISABLE_AUTOCONNECT",   "0")
epicsEnvSet("DEFAULT_SAMPLETIME_MS", "50")
epicsEnvSet("MAX_DELAY_TIME_MS",     "100")
epicsEnvSet("ADS_TIMEOUT_MS",        "5000")
# 0 = PLC clock (records need TSE=-2), 1 = IOC clock.
epicsEnvSet("DEFAULT_TIME_SRC",      "0")

# Optional: the AMS Net Id this IOC answers to. Without it the driver uses
# <this machine's IP>.1.1, which is the route TwinCAT expects anyway.
#adsSetLocalAddress("192.168.88.44.1.1")

adsAsynPortDriverConfigure("$(PORT)", "$(PLC_IP)", "$(PLC_AMS_NET_ID)", $(ADS_DEFAULT_PORT), $(PARAM_TABLE_SIZE), $(PRIORITY), $(DISABLE_AUTOCONNECT), $(DEFAULT_SAMPLETIME_MS), $(MAX_DELAY_TIME_MS), $(ADS_TIMEOUT_MS), $(DEFAULT_TIME_SRC))

dbLoadRecords("$(TWINCAT_ADS)/db/adsTestAsyn.db", "P=$(PREFIX),PORT=$(PORT),ADSPORT=$(ADS_DEFAULT_PORT)")

iocInit()

# What the bulk reader is polling (a name filters it):
#adsPollInfo("")
