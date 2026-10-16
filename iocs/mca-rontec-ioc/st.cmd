#============================================================
# st.cmd -- Rontec MCA IOC
#
# Usage:
#   cargo run -p mca-rontec-ioc -- st.cmd
#
# Configures a serial octet port to the Rontec detector, sets the input/
# output EOS the wire protocol expects (RontecConfig only *reads* the input
# EOS via pasynOctetSyncIO->getInputEos -- it does not set one), then boots
# the Rontec asyn MCA driver and loads one mca record bound to it via
# devMcaAsyn (DTYP "asynMCA").
#============================================================

# MCA_RONTEC_IOC is set by main.rs (epics_rs::base::runtime::env::set_default)
# to this IOC crate's CARGO_MANIFEST_DIR.

# ---- serial port to the Rontec detector, 38400 baud ----
drvAsynSerialPortConfigure("S0", "$(RONTEC_TTY=/dev/ttyS0)", 0, 0, 0)
asynSetOption("S0", 0, "baud", "38400")
asynOctetSetInputEos("S0", 0, "\r")
asynOctetSetOutputEos("S0", 0, "\r")

# ---- Rontec asyn MCA driver ----
# RontecConfig(portName,serialPort,serialPortAddress)
RontecConfig("RONTEC0", "S0", 0)

# ---- records ----
dbLoadRecords("$(MCA_RONTEC_IOC)/db/mca.db", "P=mca:,R=rontec1,PORT=RONTEC0,ADDR=0,NCHAN=4096")

#------------------------------------------------------------------------------
iocInit()
