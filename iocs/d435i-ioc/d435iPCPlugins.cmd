# d435iPCPlugins.cmd — Plugin chain for the D435i Pointcloud (Float32 XYZ) port.
#
# Shape (3, W, H) Float32 — not a 2D image. Most AD plugins (ROI, Stats,
# Color, JPEG, TIFF) cannot meaningfully process this. Limit to StdArrays
# for client access and HDF5 for archival.
#
# Required macros: PREFIX, PC_PORT, QSIZE

# ===== StdArrays: image3 (Float32 waveform) =====
NDStdArraysConfigure("IMAGE3", $(QSIZE), 0, "$(PC_PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image3:,PORT=IMAGE3,NDARRAY_PORT=$(PC_PORT),TYPE=Float32,FTVL=FLOAT,NELEMENTS=$(NELEMENTS_PC)")

# ===== HDF5: archive XYZ point clouds =====
NDFileHDF5Configure("FileHDF1_PC", $(QSIZE), 0, "$(PC_PORT)", 0)
dbLoadRecords("NDFileHDF5.template", "P=$(PREFIX),R=pcHDF1:,PORT=FileHDF1_PC,NDARRAY_PORT=$(PC_PORT)")
