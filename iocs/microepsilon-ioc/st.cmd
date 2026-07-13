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

# ---- L0 config port ----
# Raw TCP transport, named "L0_RAW" rather than "L0" itself: unlike upstream
# (StreamDevice attaches directly to the port drvAsynIPPortConfigure
# creates), this port's db records bind to a distinct ConfigDriver-backed
# asyn port named "L0" (matching xxCapaNCDT6200.template's Link=L0 macro
# unchanged from upstream) -- CapaNCDT6200ConfigInit below creates that port
# wrapping this raw transport.
drvAsynIPPortConfigure("L0_RAW", "10.6.28.17:23", 0, 0, 0)

# CapaNCDT6200ConfigInit(cfgPort, ioPort, ioAddr) -- no upstream C
# equivalent; see main.rs's module doc.
CapaNCDT6200ConfigInit("L0", "L0_RAW", 0)

# ---- L1 data port ----
# capaNCDT6200Configure builds its own internal raw transport port
# ("L1_RBK", invisible/unregistered, matching capaNCDT6200Sup.c's own
# hardcoded-port-10001 socket) -- no separate drvAsynIPPortConfigure call
# needed here, matching upstream (upstream's own capaNCDT6200Configure call
# has no preceding drvAsynIPPortConfigure for L1 either).
capaNCDT6200Configure("L1", "10.6.28.17", "10001")

# Upstream's asynSetTraceIOMask/asynSetTraceMask diagnostic calls for
# L0/L1 are deliberately omitted: they resolve via PortManager
# (epics_rs::asyn::manager::PortManager::find_port_handle), which neither
# CapaNCDT6200ConfigInit nor microepsilon::data_driver::configure populate
# (both register only into asyn_record, matching every non-diagnostic-
# command port in this workspace) -- see main.rs's module doc and
# asynrs-0221-serial-ioc-boilerplate-gap. Adding the dual-registry shim
# these calls would need is out of scope for this task.

dbLoadRecords("db/xxCapaNCDT6200.template", "dev=$(dev),Link=L0")
dbLoadRecords("db/xxCapaMeas.template", "dev=$(dev),PORT=L1")

iocInit()
