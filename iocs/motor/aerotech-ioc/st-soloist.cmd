#============================================================
# st-soloist.cmd — Aerotech Soloist model-1 (drvSoloist.cc) startup script
#
# Usage:
#   cargo run -p aerotech-ioc -- st-soloist.cmd
#============================================================

epicsEnvSet("P",    "soloist:")
epicsEnvSet("PORT", "SoloistPort")

drvAsynIPPortConfigure("$(PORT)", "192.168.1.41:8000", 0, 0, 0)
# The driver sets no EOS; the Soloist's AsciiCmdEOSChar is "\n".
asynOctetSetInputEos("$(PORT)", 0, "\n")
asynOctetSetOutputEos("$(PORT)", 0, "\n")

# SoloistSetup(num_cards, scan_rate), SoloistConfig(card, asyn port, addr).
SoloistSetup(1, 10)
SoloistConfig(0, "$(PORT)", 0)

# The axis's motor.db and motorUtil's records, as C's st.cmd and Soloist.cmd
# load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(AEROTECH_IOC)/db/Soloist.substitutions", "P=$(P)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")
