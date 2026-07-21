#!../../target/debug/specs-analyser-ioc
#============================================================
# st.cmd — SPECS Phoibos electron analyser areaDetector IOC startup script
# Ported from specsAnalyser/iocs/specsAnalyserIOC/iocBoot/iocSpecsAnalyser/st.cmd.example
#
# Usage:
#   cargo run -p specs-analyser-ioc -- iocs/ad/specs-analyser-ioc/st.cmd
#------------------------------------------------------------

# Prefix for all records
epicsEnvSet("PREFIX", "XF:23ID2-ES{SPECS}")
# The port name for the detector
epicsEnvSet("PORT",   "SPECS1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width/height
epicsEnvSet("XSIZE",  "2048")
epicsEnvSet("YSIZE",  "2048")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# NDStdArrays waveform sizing (C st.cmd.example uses 2361600)
epicsEnvSet("NELEMENTS", "2361600")

# $(ADSPECSANALYSER) is set to this crate's root
# (iocs/ad/specs-analyser-ioc) by ioc_support at startup;
# specsAnalyser.template lives in its db/ subdir. ADBase.template resolves
# from $(ADCORE)/db.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADSPECSANALYSER)/db:$(ADCORE)/db")

###
# Create the asyn port to talk to the SPECS analyser server.
drvAsynIPPortConfigure("SPECS_ASYN", "10.23.0.36:7010")
asynSetTraceMask("SPECS_ASYN", 0, 0x9)
asynSetTraceIOMask("SPECS_ASYN", 0, 0x2)

# SPECS analyser server framing (reproduced from the C st.cmd.example):
# commands and replies are both terminated with 0x0A (LF).
asynOctetSetInputEos("SPECS_ASYN", 0, "\n")
asynOctetSetOutputEos("SPECS_ASYN", 0, "\n")

specsAnalyserConfig("$(PORT)", "SPECS_ASYN", 0, 0)
dbLoadRecords("specsAnalyser.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin (image data for clients). The published
# arrays are NDFloat64, matching the C example's TYPE=Float64/FTVL=DOUBLE.
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd (resolves under $(ADCORE)).
< $(ADCORE)/ioc/commonPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                       # List all PVs
#   dbpf XF:23ID2-ES{SPECS}cam1:Acquire 1     # Start acquisition
#   dbpf XF:23ID2-ES{SPECS}cam1:DEFINE_SPECTRUM 1   # Define the spectrum
#   dbpf XF:23ID2-ES{SPECS}cam1:VALIDATE_SPECTRUM 1 # Validate the spectrum
#   dbgf XF:23ID2-ES{SPECS}cam1:DetectorState_RBV   # Idle/Acquiring/...
#   asynReport                                # Show port/plugin status
