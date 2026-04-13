# d435iColorPlugins.cmd — Plugin chain for the D435i Color (RGB8) port
#
# Delegates to $(ADCORE)/ioc/commonPlugins.cmd with the color-specific
# waveform typing. st.cmd must set $(NELEMENTS_COLOR) = XSIZE*YSIZE*3.
#
# Required macros: PREFIX, PORT, QSIZE, NCHANS, CBUFFS, NELEMENTS_COLOR

epicsEnvSet("TYPE",      "Int8")
epicsEnvSet("FTVL",      "UCHAR")
epicsEnvSet("NELEMENTS", "$(NELEMENTS_COLOR)")

< $(ADCORE)/ioc/commonPlugins.cmd
