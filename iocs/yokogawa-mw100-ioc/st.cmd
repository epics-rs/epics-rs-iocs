#============================================================
# st.cmd — Yokogawa MW100 data-acquisition-unit IOC startup script
#
# Usage:
#   cargo run -p yokogawa-mw100-ioc -- st.cmd
#
# Requires an MW100 reachable over TCP port 34318 (fixed by the device, not
# configurable here). mw100Init connects and runs the full load sequence
# (modules -> status -> infos -> data), so the unit must be powered on and
# network-reachable when mw100Init runs.
#
# Signal channel numbering is NOT module-relative like GM10's (no N*100+k
# scheme) — drvMW100.c's load_modules/classify_module report each installed
# module's actual channel count and the channels are numbered sequentially
# across modules in slot order (module 0's channels first, then module 1's,
# etc). The ADDRESS values below assume module 0 is a 6-channel analog-input
# module (MX110/MX112-class) occupying Signal addresses 1-6; adjust to match
# the actual installed modules and their reported channel counts.
#============================================================

epicsEnvSet("P",       "mw100:")
epicsEnvSet("DAU",     "dau1")
epicsEnvSet("HANDLE",  "mw100_0")

# mw100Init(netDevice, address) — connects and registers the instrument
# under netDevice; every record's "@$(HANDLE) CMD:ADDRESS" link resolves
# against it.
mw100Init("$(HANDLE)", "192.168.1.100")

# Per-instrument system records: module presence/model/code/speed/number,
# error/alarm summary, the four 1-second poll triggers, settings/measurement/
# compute mode.
dbLoadRecords("db/mw100_system.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE)")

# One record set per configured Signal channel, per-model template matching
# the channel's actual installed module type.
dbLoadRecords("db/mw100_mx110_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=1")
dbLoadRecords("db/mw100_mx110_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=2")
dbLoadRecords("db/mw100_mx114_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=7")
dbLoadRecords("db/mw100_mx115_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=8")
dbLoadRecords("db/mw100_mx120_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=9")
dbLoadRecords("db/mw100_mx125_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=10")
dbLoadRecords("db/mw100_calculation_channel.template",   "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=A1")
dbLoadRecords("db/mw100_communication_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=C1")
dbLoadRecords("db/mw100_constant_channel.template",      "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=K1")

iocInit()

# Example:
#   dbl
#   camonitor mw100:dau1:1 mw100:dau1:1:Unit
#   caput mw100:dau1:C1:Set 12.5
