# USB-2408-2AO IOC startup script

epicsEnvSet("PREFIX", "USB2408:")
epicsEnvSet("PORT",   "USB2408_1")
epicsEnvSet("UNIQUE_ID", "")

# Create the USB-2408-2AO driver
MultiFunctionConfig("$(PORT)", "$(UNIQUE_ID)", 2048, 2048)

# Device info
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_device.template", "P=$(PREFIX),PORT=$(PORT)")

# Counters (2)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_counter.template", "P=$(PREFIX),R=Counter2,PORT=$(PORT),ADDR=1")

# Analog inputs (8 channels)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai2,PORT=$(PORT),ADDR=1")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai3,PORT=$(PORT),ADDR=2")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai4,PORT=$(PORT),ADDR=3")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai5,PORT=$(PORT),ADDR=4")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai6,PORT=$(PORT),ADDR=5")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai7,PORT=$(PORT),ADDR=6")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_in.template", "P=$(PREFIX),R=Ai8,PORT=$(PORT),ADDR=7")

# Temperature inputs (8 channels)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti2,PORT=$(PORT),ADDR=1")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti3,PORT=$(PORT),ADDR=2")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti4,PORT=$(PORT),ADDR=3")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti5,PORT=$(PORT),ADDR=4")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti6,PORT=$(PORT),ADDR=5")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti7,PORT=$(PORT),ADDR=6")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_temperature.template", "P=$(PREFIX),R=Ti8,PORT=$(PORT),ADDR=7")

# Analog outputs (2 channels)
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_out.template", "P=$(PREFIX),R=Ao1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_analog_out.template", "P=$(PREFIX),R=Ao2,PORT=$(PORT),ADDR=1")

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

# Waveform digitizer
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_wave_dig.template", "P=$(PREFIX),PORT=$(PORT)")

# Waveform generator
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_wave_gen.template", "P=$(PREFIX),PORT=$(PORT)")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_wave_gen_n.template", "P=$(PREFIX),R=WaveGen1,PORT=$(PORT),ADDR=0")
dbLoadRecords("$(MEASCOMP)/../../db/meascomp_wave_gen_n.template", "P=$(PREFIX),R=WaveGen2,PORT=$(PORT),ADDR=1")

iocInit()
