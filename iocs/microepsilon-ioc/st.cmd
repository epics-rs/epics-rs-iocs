#============================================================
# st.cmd -- MicroEpsilon capaNCDT6200 IOC startup script
#
# Usage:
#   cargo run -p microepsilon-ioc -- st.cmd
#
# Ported from epics-modules/microEpsilon/iocBoot/capaNCDTiocTest/st.cmd's
# MMD1 instance (host 10.6.28.17, ports L0/L1) -- the only instance in
# scope. That file's MMD4/MMD5 instances (L2-L5) run the same C driver
# against a different db (xxHydrostaticConfig.vdb/xxHydrostaticMeas.vdb),
# out of scope for this port; see drivers/microepsilon's module docs.
#============================================================

epicsEnvSet("dev", "MMS:S27:MMD1")

epicsEnvSet("STREAM_PROTOCOL_PATH", "$(MICROEPSILON)/db")

# ---- L0 config port ----
drvAsynIPPortConfigure("L0", "10.6.28.17:23", 0, 0, 0)

# ---- L1 data port ----
# capaNCDT6200Configure builds its own internal raw transport port
# ("L1_RBK", invisible/unregistered, matching capaNCDT6200Sup.c's own
# hardcoded-port-10001 socket) -- no separate drvAsynIPPortConfigure call
# needed here, matching upstream (upstream's own capaNCDT6200Configure call
# has no preceding drvAsynIPPortConfigure for L1 either).
capaNCDT6200Configure("L1", "10.6.28.17", "10001")

# Upstream's asynSetTraceIOMask/asynSetTraceMask diagnostic calls for
# L0/L1 are deliberately omitted: they resolve via PortManager
# (epics_rs::asyn::manager::PortManager::find_port_handle), which
# microepsilon::data_driver::configure does not populate (it registers
# only into asyn_record, matching every non-diagnostic-command port in
# this workspace) -- see main.rs's module doc and
# asynrs-0221-serial-ioc-boilerplate-gap. Adding the dual-registry shim
# these calls would need is out of scope for this task.

dbLoadRecords("$(MICROEPSILON)/db/xxCapaNCDT6200.template", "dev=$(dev),Link=L0")
dbLoadRecords("$(MICROEPSILON)/db/xxCapaMeas.template", "dev=$(dev),PORT=L1")

iocInit()
