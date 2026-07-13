# ndDriverStdArraysPlugins.cmd — Plugin chain for the NDDriverStdArrays port
#
# Delegates to $(ADCORE)/ioc/commonPlugins.cmd. NDStdArrays is configured in
# st.cmd before this file is read, exactly as upstream does (it loads
# $(ADCORE)/iocBoot/commonPlugins.cmd from the same place).
#
# The NDStdArrays waveform type here is Float64/DOUBLE, matching the plugin
# output loaded in st.cmd.
#
# Required macros: PREFIX, PORT, QSIZE, NCHANS, CBUFFS

epicsEnvSet("TYPE",   "Float64")
epicsEnvSet("FTVL",   "DOUBLE")

< $(ADCORE)/ioc/commonPlugins.cmd
