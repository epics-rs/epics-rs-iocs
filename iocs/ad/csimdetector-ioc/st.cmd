#!../../../target/debug/ad-csimdetector-ioc
#============================================================
# st.cmd — ADCSimDetector areaDetector IOC startup script
#
# Mirrors iocs/ADCSimDetectorIOC/iocBoot/iocADCSimDetector/st.cmd
# from upstream ADCSimDetector.
#
# Usage:
#   cargo run -p ad-csimdetector-ioc -- iocs/ad/csimdetector-ioc/st.cmd
#============================================================

# Prefix for all records
epicsEnvSet("PREFIX", "13ADCSIM1:")
# The port name for the detector
epicsEnvSet("PORT",   "SIM1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "8")
# The maximum image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "2000")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")

# NELEMENTS for the NDStdArrays output waveform: 100000 x 8 arrays.
epicsEnvSet("NELEMENTS", "800000")

# $(ADCSIMDETECTOR) is set to this crate's root by ioc_support at IOC startup.
# The shared workspace db/ lives three levels up from there.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCSIMDETECTOR)/../../../db:$(ADCORE)/db")

epicsEnvSet("T1", "Sin(x)")
epicsEnvSet("T2", "Cos(x)")
epicsEnvSet("T3", "SquareWave(x)")
epicsEnvSet("T4", "Sawtooth(x)")
epicsEnvSet("T5", "Noise")
epicsEnvSet("T6", "Sin(x)+Cos(x)")
epicsEnvSet("T7", "Sin(x)*Cos(x)")
epicsEnvSet("T8", "SinSums")

# Create an ADCSimDetector driver
# ADCSimDetectorConfig(portName, numTimePoints, dataType, maxBuffers, maxMemory,
#                      priority, stackSize)
#
# DEVIATION: upstream passes dataType 7. That predates the NDInt64/NDUInt64
# insertion into NDDataType_t, where 7 meant NDFloat64; under the modern enum
# (which `ad-core-rs` implements) 7 is NDUInt64. 9 = NDFloat64 is used here so
# the data type still matches the TYPE=Float64,FTVL=DOUBLE waveform below.
# maxBuffers 0 and maxMemory 0 mean unlimited.
ADCSimDetectorConfig("$(PORT)", $(YSIZE), 9, 0, 0)

dbLoadRecords("ADCSimDetector.template",  "P=$(PREFIX),R=det1:,  PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(T1)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:2:,PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(T2)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:3:,PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(T3)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:4:,PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(T4)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:5:,PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(T5)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:6:,PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(T6)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:7:,PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(T7)")
dbLoadRecords("ADCSimDetectorN.template", "P=$(PREFIX),R=det1:8:,PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(T8)")

# Standard arrays plugin, fed from the driver's 2-D array (addr 0).
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(NELEMENTS)")

# Time series plugin: 8 signals taken from the 2-D array.
NDTimeSeriesConfigure("TS1", $(QSIZE), 0, "$(PORT)", 0, 8)
dbLoadRecords("NDTimeSeries.template",  "P=$(PREFIX),R=TS:,   PORT=TS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)det1:TimeStep CP MS,ENABLED=1")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:1:, PORT=TS1,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T1)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:2:, PORT=TS1,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T2)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:3:, PORT=TS1,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T3)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:4:, PORT=TS1,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T4)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:5:, PORT=TS1,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T5)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:6:, PORT=TS1,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T6)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:7:, PORT=TS1,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T7)")
dbLoadRecords("NDTimeSeriesN.template", "P=$(PREFIX),R=TS:8:, PORT=TS1,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T8)")

# FFT plugins, one per time-series signal.
#
# DEVIATION: `ad-core-rs`'s `extract_plugin_args` ignores the NDArrayAddr
# argument when a plugin is first wired, so FFT2..FFT8 initially attach to
# TS1's address-0 output. Writing $(PREFIX)FFTn:NDArrayAddr after iocInit
# rewires them to the intended source address.
#
# NOTE: upstream's NDFFTConfigure("FFT3", ...) line is missing its closing
# parenthesis; it is closed here.
NDFFTConfigure("FFT1", $(QSIZE), 0, "TS1", 0)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT1:,PORT=FFT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T1)")
NDFFTConfigure("FFT2", $(QSIZE), 0, "TS1", 1)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT2:,PORT=FFT2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=1,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T2)")
NDFFTConfigure("FFT3", $(QSIZE), 0, "TS1", 2)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT3:,PORT=FFT3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T3)")
NDFFTConfigure("FFT4", $(QSIZE), 0, "TS1", 3)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT4:,PORT=FFT4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=3,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T4)")
NDFFTConfigure("FFT5", $(QSIZE), 0, "TS1", 4)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT5:,PORT=FFT5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=4,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T5)")
NDFFTConfigure("FFT6", $(QSIZE), 0, "TS1", 5)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT6:,PORT=FFT6,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=5,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T6)")
NDFFTConfigure("FFT7", $(QSIZE), 0, "TS1", 6)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT7:,PORT=FFT7,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=6,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T7)")
NDFFTConfigure("FFT8", $(QSIZE), 0, "TS1", 7)
dbLoadRecords("NDFFT.template","P=$(PREFIX),R=FFT8:,PORT=FFT8,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=7,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T8)")

# Remaining plugin chain
< $(ADCSIMDETECTOR)/csimDetectorPlugins.cmd

# iocInit is called automatically by IocApplication after this script completes.
#
# Example interactive commands:
#   dbl                                      # List all PVs
#   dbpf 13ADCSIM1:det1:Acquire 1            # Start acquisition
#   dbpf 13ADCSIM1:det1:1:Amplitude 2.5      # Signal 1 amplitude
#   dbgf 13ADCSIM1:det1:ElapsedTime          # Elapsed acquisition time
#   asynReport                               # Show port/plugin status
