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

# ---- serial port to the Rontec detector ----
# C RontecConfig never sets an EOS itself, only reads whatever was already
# configured (pasynOctetSyncIO->getInputEos, drvMcaRontec.c:197) -- the real
# device's EOS is not specified anywhere in the C source, so "\r\n" here is
# this IOC's own placeholder default, not a value derived from the Rontec
# protocol spec.
drvAsynSerialPortConfigure("S0", "$(RONTEC_TTY=/dev/ttyS0)", 0, 0, 0)
asynOctetSetInputEos("S0", 0, "\r\n")
asynOctetSetOutputEos("S0", 0, "\r\n")

# ---- Rontec asyn MCA driver ----
# RontecConfig(portName,serialPort,serialPortAddress)
RontecConfig("RONTEC0", "S0", 0)

# ---- records ----
dbLoadRecords("$(MCA_RONTEC_IOC)/db/mca.db", "P=mca:,R=rontec1,PORT=RONTEC0,ADDR=0,NCHAN=4096")

#------------------------------------------------------------------------------
iocInit()
