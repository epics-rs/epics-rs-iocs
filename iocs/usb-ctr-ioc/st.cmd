# USB-CTR08 IOC startup script

epicsEnvSet("PREFIX", "USBCTR:")
epicsEnvSet("PORT",   "USBCTR_1")
epicsEnvSet("UNIQUE_ID", "01DAB0FB")

# Create the USB-CTR08 driver
USBCTRConfig("$(PORT)", "$(UNIQUE_ID)", 2048)

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

dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo1,PORT=$(PORT),ADDR=0,MASK=0x01")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo2,PORT=$(PORT),ADDR=0,MASK=0x02")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo3,PORT=$(PORT),ADDR=0,MASK=0x04")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo4,PORT=$(PORT),ADDR=0,MASK=0x08")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo5,PORT=$(PORT),ADDR=0,MASK=0x10")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo6,PORT=$(PORT),ADDR=0,MASK=0x20")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo7,PORT=$(PORT),ADDR=0,MASK=0x40")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_binary_out.template", "P=$(PREFIX),R=Bo8,PORT=$(PORT),ADDR=0,MASK=0x80")

# MCS (Multi-Channel Scaler)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_mcs.template", "P=$(PREFIX),PORT=$(PORT)")

iocInit()
