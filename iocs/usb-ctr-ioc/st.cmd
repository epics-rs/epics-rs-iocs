# USB-CTR08 IOC startup script

epicsEnvSet("PREFIX", "USBCTR:")
epicsEnvSet("PORT",   "USBCTR_1")
epicsEnvSet("UNIQUE_ID", "01DAB0FB")
epicsEnvSet("MAX_POINTS", "2048")

# Create the USB-CTR08 driver
USBCTRConfig("$(PORT)", "$(UNIQUE_ID)", $(MAX_POINTS))

# Device info
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_device.template", "P=$(PREFIX),PORT=$(PORT)")

# Pulse generators (4 timers)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_pulse_gen.template", "P=$(PREFIX),R=PulseGen1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_pulse_gen.template", "P=$(PREFIX),R=PulseGen2,PORT=$(PORT),ADDR=1")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_pulse_gen.template", "P=$(PREFIX),R=PulseGen3,PORT=$(PORT),ADDR=2")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_pulse_gen.template", "P=$(PREFIX),R=PulseGen4,PORT=$(PORT),ADDR=3")

# Counters (8)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter2,PORT=$(PORT),ADDR=1")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter3,PORT=$(PORT),ADDR=2")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter4,PORT=$(PORT),ADDR=3")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter5,PORT=$(PORT),ADDR=4")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter6,PORT=$(PORT),ADDR=5")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter7,PORT=$(PORT),ADDR=6")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter8,PORT=$(PORT),ADDR=7")

# Digital I/O (8 bits)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi1,PORT=$(PORT),ADDR=0,MASK=0x01")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi2,PORT=$(PORT),ADDR=0,MASK=0x02")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi3,PORT=$(PORT),ADDR=0,MASK=0x04")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi4,PORT=$(PORT),ADDR=0,MASK=0x08")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi5,PORT=$(PORT),ADDR=0,MASK=0x10")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi6,PORT=$(PORT),ADDR=0,MASK=0x20")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi7,PORT=$(PORT),ADDR=0,MASK=0x40")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_in.template",  "P=$(PREFIX),R=Bi8,PORT=$(PORT),ADDR=0,MASK=0x80")

# Whole-port digital I/O
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_long_in.template",  "P=$(PREFIX),R=Li,PORT=$(PORT),ADDR=0,MASK=0xFF")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_long_out.template", "P=$(PREFIX),R=Lo,PORT=$(PORT),ADDR=0,MASK=0xFF")

# Digital I/O bit directions (0=input, 1=output)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd1,PORT=$(PORT),ADDR=0,MASK=0x01,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd2,PORT=$(PORT),ADDR=0,MASK=0x02,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd3,PORT=$(PORT),ADDR=0,MASK=0x04,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd4,PORT=$(PORT),ADDR=0,MASK=0x08,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd5,PORT=$(PORT),ADDR=0,MASK=0x10,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd6,PORT=$(PORT),ADDR=0,MASK=0x20,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd7,PORT=$(PORT),ADDR=0,MASK=0x40,VAL=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_dir.template", "P=$(PREFIX),R=Bd8,PORT=$(PORT),ADDR=0,MASK=0x80,VAL=0")

dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo1,PORT=$(PORT),ADDR=0,MASK=0x01")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo2,PORT=$(PORT),ADDR=0,MASK=0x02")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo3,PORT=$(PORT),ADDR=0,MASK=0x04")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo4,PORT=$(PORT),ADDR=0,MASK=0x08")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo5,PORT=$(PORT),ADDR=0,MASK=0x10")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo6,PORT=$(PORT),ADDR=0,MASK=0x20")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo7,PORT=$(PORT),ADDR=0,MASK=0x40")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo8,PORT=$(PORT),ADDR=0,MASK=0x80")

# scalerRecord over the 8 counters (scaler.db ships with scaler-rs)
dbLoadRecords("$(SCALER)/scaler.db", "P=$(PREFIX),S=scaler1,DTYP=Asyn Scaler,OUT=@asyn($(PORT)),FREQ=10000000")

# MCS (Multi-Channel Scaler)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs.template", "P=$(PREFIX),PORT=$(PORT),MAX_POINTS=$(MAX_POINTS)")

# Per-counter MCS spectra (counters 1-8 plus the digital I/O channel)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca1,PORT=$(PORT),ADDR=0,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca2,PORT=$(PORT),ADDR=1,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca3,PORT=$(PORT),ADDR=2,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca4,PORT=$(PORT),ADDR=3,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca5,PORT=$(PORT),ADDR=4,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca6,PORT=$(PORT),ADDR=5,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca7,PORT=$(PORT),ADDR=6,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca8,PORT=$(PORT),ADDR=7,MAX_POINTS=$(MAX_POINTS)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs_n.template", "P=$(PREFIX),R=MCS:mca9,PORT=$(PORT),ADDR=8,MAX_POINTS=$(MAX_POINTS)")


# Autosave: request files live next to this script, saved state under
# ./autosave. set_pass1_restoreFile is a no-op until the first save has run.
set_requestfile_path("$(MEASCOMP)")
set_savefile_path("$(MEASCOMP)/autosave")
set_pass1_restoreFile("auto_settings.sav", "P=$(PREFIX)")

iocInit()

create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")
