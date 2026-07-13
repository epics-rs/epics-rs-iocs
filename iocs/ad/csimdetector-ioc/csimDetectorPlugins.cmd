# csimDetectorPlugins.cmd — Plugin chain for the ADCSimDetector port
#
# Delegates to $(ADCORE)/ioc/commonPlugins.cmd. The waveform typing matches
# the dataType passed to ADCSimDetectorConfig in st.cmd (NDFloat64).
#
# Upstream loads $(ADCORE)/iocBoot/commonPlugins.cmd from the same place in
# its st.cmd; NDStdArrays / NDTimeSeries / NDFFT are configured in st.cmd
# before this file is read, exactly as upstream does.
#
# Required macros: PREFIX, PORT, QSIZE, NCHANS, CBUFFS, NELEMENTS

epicsEnvSet("TYPE",   "Float64")
epicsEnvSet("FTVL",   "DOUBLE")

< $(ADCORE)/ioc/commonPlugins.cmd
