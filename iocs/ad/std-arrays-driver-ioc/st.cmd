#============================================================
# st.cmd — NDDriverStdArrays areaDetector IOC startup script
#
# Mirrors iocs/NDDriverStdArraysIOC/iocBoot/iocNDDriverStdArrays/st.cmd
# from upstream NDDriverStdArrays.
#
# Usage:
#   cargo run -p ad-std-arrays-driver-ioc -- iocs/ad/std-arrays-driver-ioc/st.cmd
#============================================================

# Prefix for all records
epicsEnvSet("PREFIX", "13NDSA1:")
# The port name for the detector
epicsEnvSet("PORT",   "NDSA")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "1024")
# The maximum image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1024")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The number of elements in the driver waveform record
epicsEnvSet("NELEMENTS", "2000000")
# The datatype of the driver waveform record
epicsEnvSet("FTVL", "FLOAT")
# The asyn interface waveform record type
epicsEnvSet("TYPE", "Float32")

# $(NDDRIVERSTDARRAYS) is set to this crate's root by ioc_support at IOC
# startup. The shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(NDDRIVERSTDARRAYS)/../../../db:$(ADCORE)/db")

# Create an NDDriverStdArrays driver
# NDDriverStdArraysConfig(portName, maxBuffers, maxMemory, priority, stackSize)
NDDriverStdArraysConfig("$(PORT)", $(QSIZE), 0, 0)

dbLoadRecords("NDDriverStdArrays.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,NELEMENTS=$(NELEMENTS),TYPE=$(TYPE),FTVL=$(FTVL)")

# Standard arrays plugin, fed from the driver's array (addr 0).
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=12000000")

# Remaining plugin chain
< $(NDDRIVERSTDARRAYS)/ndDriverStdArraysPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                        # List all PVs
#   dbpf 13NDSA1:cam1:Acquire 1                # Enable array injection
#   caput -a 13NDSA1:cam1:ArrayIn 8 1 2 3 4 5 6 7 8   # Inject a waveform
#   dbgf 13NDSA1:cam1:ArrayCounter_RBV         # Frames published
#   asynReport                                 # Show port/plugin status
