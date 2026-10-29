# simDetectorPlugins.cmd — Plugin chain for the ADSimDetector port
#
# Delegates to $(ADCORE)/ioc/commonPlugins.cmd; st.cmd creates image1 first,
# as C st_base.cmd does.
#
# Required macros: PREFIX, PORT, QSIZE, XSIZE, YSIZE, NCHANS, CBUFFS

< $(ADCORE)/ioc/commonPlugins.cmd
