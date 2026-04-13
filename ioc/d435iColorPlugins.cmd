# d435iColorPlugins.cmd — Plugin chain for the D435i Color (RGB8) port
#
# Required macros: PREFIX, PORT, QSIZE, XSIZE, YSIZE, NCHANS, CBUFFS
# Inherits defaults TYPE=Int8 FTVL=UCHAR NELEMENTS=XSIZE*YSIZE*3 from st.cmd.

epicsEnvSet("TYPE",      "Int8")
epicsEnvSet("FTVL",      "UCHAR")
epicsEnvSet("NELEMENTS", "6220800")

< $(ADCORE)/ioc/commonPlugins.cmd
