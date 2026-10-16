#============================================================
# st.cmd -- Rontec MCA IOC, ported from iocBoot/iocRontec/st.cmd
#
# Usage:
#   cargo run -p mca-rontec-ioc -- st.cmd
#
# Not ported: asynOctetConnect (no epics-rs iocsh command), and from
# save_restore.cmd the save_restoreSet_Debug/IncompleteSetsOk/
# DatedBackupFiles/NumSeqFiles/SeqPeriodInSeconds settings and
# save_restoreStatus.db, which epics-rs autosave does not provide.
#============================================================

# MCA_RONTEC_IOC is set by main.rs (epics_rs::base::runtime::env::set_default)
# to this IOC crate's CARGO_MANIFEST_DIR.

# ---- serial port to the Rontec detector, 38400 baud ----
drvAsynSerialPortConfigure("serial1", "$(RONTEC_TTY=/dev/ttyS0)", 0, 0, 0)
asynSetOption("serial1", 0, "baud", "38400")
asynOctetSetInputEos("serial1", 0, "\r")
asynOctetSetOutputEos("serial1", 0, "\r")

# ---- Rontec asyn MCA driver ----
# RontecConfig(portName,serialPort,serialPortAddress)
RontecConfig("Rontec1", "serial1", 0)

# ---- records ----
dbLoadRecords("$(MCA_RONTEC_IOC)/db/mca.db", "P=mcaTest:,M=mca1,NCHAN=1024,DTYP=asynMCA,INP=@asyn(Rontec1)")
dbLoadRecords("$(MCA_RONTEC_IOC)/db/RontecXFlash.db", "P=mcaTest:,R=Rontec1,PORT=serial1")

dbLoadRecords("$(MCA_RONTEC_IOC)/db/asynRecord.db", "P=mcaTest:,R=serial1,PORT=serial1,ADDR=0,OMAX=256,IMAX=256")

# ---- save_restore ----
save_restoreSet_status_prefix("mcaTest:")
set_savefile_path("$(MCA_RONTEC_IOC)", "autosave")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
set_requestfile_path("$(MCA_RONTEC_IOC)", "")
set_requestfile_path("$(MCA_RONTEC_IOC)", "db")

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=mcaTest:")
