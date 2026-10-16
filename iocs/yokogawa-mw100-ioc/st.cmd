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
# Signal address = 10*slot + channel (drvMW100.c:680): slot 0 holds 1-10,
# slot 1 holds 11-20, and so on up to 60. The ADDRESS values below assume
# an MX110 in slot 0 and an MX114, MX115, MX120 and MX125 in slots 1-4;
# adjust them to the installed modules.
#============================================================

epicsEnvSet("P",       "mw100:")
epicsEnvSet("DAU",     "dau1")
epicsEnvSet("HANDLE",  "mw100_0")

# mw100Init(netDevice, address) — connects and registers the instrument
# under netDevice; every record's "@$(HANDLE) CMD:ADDRESS" link resolves
# against it.
mw100Init("$(HANDLE)", "192.168.1.100")

# Per-instrument system records: module presence/model/code/speed/number,
# error/alarm summary, the input/output/status poll triggers, operation mode
# and computation.
dbLoadRecords("$(MW100)/db/mw100_system.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE)")

# One record set per configured Signal channel, per-model template matching
# the channel's actual installed module type.
dbLoadRecords("$(MW100)/db/mw100_mx110_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=1")
dbLoadRecords("$(MW100)/db/mw100_mx110_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=2")
dbLoadRecords("$(MW100)/db/mw100_mx114_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=11")
dbLoadRecords("$(MW100)/db/mw100_mx115_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=21")
dbLoadRecords("$(MW100)/db/mw100_mx120_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=31")
dbLoadRecords("$(MW100)/db/mw100_mx125_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=41")
dbLoadRecords("$(MW100)/db/mw100_calculation_channel.template",   "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=A1")
dbLoadRecords("$(MW100)/db/mw100_communication_channel.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=C1")
dbLoadRecords("$(MW100)/db/mw100_constant_channel.template",      "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=K1")

iocInit()

# Example:
#   dbl
#   camonitor mw100:dau1:1 mw100:dau1:1:Unit
#   caput mw100:dau1:C1:Set 12.5
