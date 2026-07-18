#============================================================
# st.cmd -- MCA demo IOC
#
# Usage:
#   cargo run -p mca-ioc -- st.cmd
#
# Boots a synthetic asyn signal source (DemoSourceConfig, this IOC's own
# stand-in for a real scaler/waveform card -- not an upstream driver) and a
# drvFastSweep software MCA (initFastSweep) sweeping it into 10 channels,
# then loads one mca record bound to the FastSweep port via devMcaAsyn
# (DTYP "asynMCA").
#============================================================

# MCA_IOC is set by main.rs (epics_rs::base::runtime::env::set_default) to
# this IOC crate's CARGO_MANIFEST_DIR.

# ---- synthetic upstream signal source ----
# DemoSourceConfig(portName, maxSignals, period) -- 1 signal, 20 ms/sample.
DemoSourceConfig("SRC0", 1, 0.02)

# ---- FastSweep software MCA ----
# initFastSweep(portName,inputName,maxSignals,maxPoints,dataString,intervalString)
initFastSweep("FS0", "SRC0", 1, 10, "", "")

# ---- records ----
dbLoadRecords("$(MCA_IOC)/db/mca.db", "P=mca:,R=spectrum1,PORT=FS0,ADDR=0,NCHAN=10")

#------------------------------------------------------------------------------
iocInit()
