# d435iDepthPlugins.cmd — Plugin chain for the D435i Depth (Z16) port.
#
# Z16 mono data — JPEG / Color Convert are skipped (not meaningful).
# Keeps StdArrays + ROI/Stats + TIFF/HDF5 for analysis and saving.
#
# Required macros: PREFIX, DEPTH_PORT, QSIZE, NCHANS

# ===== StdArrays: image2 =====
NDStdArraysConfigure("IMAGE2", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=IMAGE2,NDARRAY_PORT=$(DEPTH_PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS_DEPTH)")

# ===== PVA: depth NTNDArray (Z16 -> ushortValue), for pva:// viewers =====
# The colour stream's NDPva lives in commonPlugins.cmd (RS405:Pva1:Image);
# depth gets its own here. ENABLED=1 because a PVA channel that exists but
# never posts is indistinguishable from a broken one to a viewer.
NDPvaConfigure("PVA2", $(QSIZE), 0, "$(DEPTH_PORT)", 0, "$(PREFIX)depthPva1:Image")
dbLoadRecords("NDPva.template", "P=$(PREFIX),R=depthPva1:,PORT=PVA2,NDARRAY_PORT=$(DEPTH_PORT),ENABLED=1")

# ===== ROI + ROIStat for region analysis =====
NDROIConfigure("ROI1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDROI.template", "P=$(PREFIX),R=depthROI1:,PORT=ROI1_D,NDARRAY_PORT=$(DEPTH_PORT)")

NDROIStatConfigure("ROIStat1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0, 8)
dbLoadRecords("NDROIStat.template", "P=$(PREFIX),R=depthROIStat1:,PORT=ROIStat1_D,NDARRAY_PORT=$(DEPTH_PORT),NCHANS=$(NCHANS)")

# ===== Stats (global min/max/mean; TS port also required by NDStats.template) =====
NDStatsConfigure("STATS1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0, 0, 0, 0, 0)
dbLoadRecords("NDStats.template", "P=$(PREFIX),R=depthStats1:,PORT=STATS1_D,NDARRAY_PORT=$(DEPTH_PORT),NCHANS=$(NCHANS),XSIZE=$(XSIZE),YSIZE=$(YSIZE),HIST_SIZE=$(HIST_SIZE)")
NDTimeSeriesConfigure("STATS1_D_TS", $(QSIZE), 0, "STATS1_D", 0)
dbLoadRecords("NDTimeSeries.template", "P=$(PREFIX),R=depthStats1:TS:,PORT=STATS1_D_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATS1_D,NDARRAY_ADDR=0,NCHANS=$(NCHANS),ENABLED=1")

# ===== File savers: TIFF (16-bit) and HDF5 =====
NDFileTIFFConfigure("FileTIFF1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDFileTIFF.template", "P=$(PREFIX),R=depthTIFF1:,PORT=FileTIFF1_D,NDARRAY_PORT=$(DEPTH_PORT)")

NDFileHDF5Configure("FileHDF1_D", $(QSIZE), 0, "$(DEPTH_PORT)", 0)
dbLoadRecords("NDFileHDF5.template", "P=$(PREFIX),R=depthHDF1:,PORT=FileHDF1_D,NDARRAY_PORT=$(DEPTH_PORT)")
