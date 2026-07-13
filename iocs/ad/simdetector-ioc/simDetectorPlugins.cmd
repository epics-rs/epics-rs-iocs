# simDetectorPlugins.cmd — Plugin chain for the ADSimDetector port
#
# Delegates to $(ADCORE)/ioc/commonPlugins.cmd. The waveform typing matches
# the driver's default NDDataType (NDUInt8, dataType=1 in st.cmd).
#
# Required macros: PREFIX, PORT, QSIZE, NCHANS, CBUFFS, NELEMENTS

epicsEnvSet("TYPE",   "Int8")
epicsEnvSet("FTVL",   "UCHAR")

< $(ADCORE)/ioc/commonPlugins.cmd
