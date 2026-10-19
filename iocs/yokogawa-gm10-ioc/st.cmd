#============================================================
# st.cmd — Yokogawa GM10 data-acquisition-unit IOC startup script
#
# Usage:
#   cargo run -p yokogawa-gm10-ioc -- st.cmd
#
# Requires a GM10 reachable over TCP port 34434 (fixed by the device, not
# configurable here). gm10Init connects and runs the full load sequence
# (modules -> status -> infos -> data -> misc), so the unit must be powered
# on and network-reachable when gm10Init runs.
#============================================================

epicsEnvSet("P",       "gm10:")
epicsEnvSet("DAU",     "dau1")
epicsEnvSet("HANDLE",  "gm10_0")

# gm10Init(netDevice, address) — connects and registers the instrument under
# netDevice; every record's "@$(HANDLE) CMD:ADDRESS" link resolves against it.
gm10Init("$(HANDLE)", "192.168.1.100")

# Per-instrument system records: module presence, error/alarm summary, the
# three 1-second poll triggers, freeze/recording/compute mode.
dbLoadRecords("$(GM10)/db/gm10_system.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE)")

# One record set per configured channel, address-family template matching
# the channel's actual module/address type on the unit. Signal addresses are
# module-relative: module N's channels occupy N*100+1 .. N*100+channel_count
# (drvGM10.c load_modules), so a digital-input module in module slot 1 owns
# 101-1xx, not a small standalone number.
dbLoadRecords("$(GM10)/db/gm10_analog_input.template",  "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=0001")
dbLoadRecords("$(GM10)/db/gm10_analog_input.template",  "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=0002")
dbLoadRecords("$(GM10)/db/gm10_digital_input.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=0101")
dbLoadRecords("$(GM10)/db/gm10_digital_output.template","P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=0201")
dbLoadRecords("$(GM10)/db/gm10_pulse_input.template",   "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=0301")
dbLoadRecords("$(GM10)/db/gm10_analog_output.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=0401")
dbLoadRecords("$(GM10)/db/gm10_calculation.template",   "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=A001")
dbLoadRecords("$(GM10)/db/gm10_communication.template", "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=C001")
dbLoadRecords("$(GM10)/db/gm10_constant.template",      "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=K001")
dbLoadRecords("$(GM10)/db/gm10_varconstant.template",   "P=$(P),DAU=$(DAU),HANDLE=$(HANDLE),ADDRESS=W001")

iocInit()

# Example:
#   dbl
#   camonitor gm10:dau1:0001 gm10:dau1:0001:Unit
#   caput gm10:dau1:0401:Set 12.5
