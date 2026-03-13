# D435i RealSense areaDetector IOC startup script
#
# Usage: d435i_ioc ioc/st.cmd

# Create D435i detector
# d435iConfig(portName, serial, maxSizeX, maxSizeY, maxMemory)
# Color port = RS1, Depth port = RS1_DEPTH (auto-created)
d435iConfig("RS1", "", 1920, 1080, 100000000)

# Load Color port records
dbLoadRecords("db/d435i_color.template", "P=RS1:,R=cam1:,PORT=RS1,ADDR=0,TIMEOUT=1")

# Load Depth port records
dbLoadRecords("db/d435i_depth.template", "P=RS1:,R=depth1:,PORT=RS1_DEPTH,ADDR=0,TIMEOUT=1")

# Standard array plugins for image display
NDStdArraysConfigure("COLOR_IMAGE", 20, 0, "RS1", 0)
dbLoadRecords("db/NDStdArrays.template", "P=RS1:,R=image1:,PORT=COLOR_IMAGE,ADDR=0,TIMEOUT=1,NDARRAY_PORT=RS1,TYPE=Int8,FTVL=UCHAR,NELEMENTS=6220800")

NDStdArraysConfigure("DEPTH_IMAGE", 20, 0, "RS1_DEPTH", 0)
dbLoadRecords("db/NDStdArrays.template", "P=RS1:,R=image2:,PORT=DEPTH_IMAGE,ADDR=0,TIMEOUT=1,NDARRAY_PORT=RS1_DEPTH,TYPE=Int16,FTVL=SHORT,NELEMENTS=2073600")

iocInit()
