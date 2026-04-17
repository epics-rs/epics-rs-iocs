# d435iDepthPlugins.cmd — Plugin chain for the D435i Depth (Z16) port.
#
# Z16 mono data — JPEG / Color Convert are skipped (not meaningful).
# Keeps StdArrays + ROI/Stats + TIFF/HDF5 for analysis and saving.
#
# Required macros: PREFIX, DEPTH_PORT, QSIZE, NCHANS

# ===== StdArrays: image2 =====
NDStdArraysConfigure("IMAGE2", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=IMAGE2,NDARRAY_PORT=$(DEPTH_PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS_DEPTH)")

# ===== ROI + ROIStat for region analysis =====
NDROIConfigure("ROI1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDROI.template", "P=$(PREFIX),R=depthROI1:,PORT=ROI1_D,NDARRAY_PORT=$(DEPTH_PORT)")

NDROIStatConfigure("ROIStat1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0, 8)
dbLoadRecords("NDROIStat.template", "P=$(PREFIX),R=depthROIStat1:,PORT=ROIStat1_D,NDARRAY_PORT=$(DEPTH_PORT),NCHANS=$(NCHANS)")

# ===== Stats (global min/max/mean; TS port also required by NDStats.template) =====
NDStatsConfigure("STATS1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0, 0, 0, 0, 0)
dbLoadRecords("NDStats.template", "P=$(PREFIX),R=depthStats1:,PORT=STATS1_D,NDARRAY_PORT=$(DEPTH_PORT),NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATS1_D_TS", $(QSIZE), 0, "STATS1_D", 0)
dbLoadRecords("NDTimeSeries.template", "P=$(PREFIX),R=depthStats1:TS:,PORT=STATS1_D_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATS1_D,NDARRAY_ADDR=0,NCHANS=$(NCHANS),ENABLED=1")

# ===== File savers: TIFF (16-bit) and HDF5 =====
NDFileTIFFConfigure("FileTIFF1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDFileTIFF.template", "P=$(PREFIX),R=depthTIFF1:,PORT=FileTIFF1_D,NDARRAY_PORT=$(DEPTH_PORT)")

NDFileHDF5Configure("FileHDF1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDFileHDF5.template", "P=$(PREFIX),R=depthHDF1:,PORT=FileHDF1_D,NDARRAY_PORT=$(DEPTH_PORT)")
