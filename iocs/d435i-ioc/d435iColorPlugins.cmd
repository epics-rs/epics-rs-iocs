# d435iColorPlugins.cmd — Plugin chain for the D435i Color (RGB8) port
#
# image1 here, then $(ADCORE)/ioc/commonPlugins.cmd, which creates no
# NDStdArrays. No C counterpart; the image1 lines are C ADSimDetector
# st_base.cmd's with the color waveform typing. st.cmd must set
# $(NELEMENTS_COLOR) = XSIZE*YSIZE*3.
#
# Required macros: PREFIX, PORT, QSIZE, XSIZE, YSIZE, NCHANS, CBUFFS, NELEMENTS_COLOR

# Create a standard arrays plugin, set it to get data from the color port.
NDStdArraysConfigure("Image1", 20, 0, "$(PORT)", 0, 0, 0, 0, 0, 5)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=$(NELEMENTS_COLOR)")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/ioc/commonPlugins.cmd
