#!../../target/debug/timepix3-ioc
#============================================================
# st.cmd — ASI/Amsterdam TimePix3 (Serval) areaDetector IOC
#
# Usage:
#   cargo run -p timepix3-ioc -- iocs/ad/timepix3-ioc/st.cmd
#
# Port of ADTimePix3/iocs/tpx3IOC/iocBoot/iocTimePix/st_base.cmd (with its
# unique.cmd, load_chips.cmd and init_detector.cmd folded in).
#============================================================

epicsEnvSet("PREFIX",     "TPX3-TEST:")
epicsEnvSet("PORT",       "TPX3")
epicsEnvSet("SERVER_URL", "http://localhost:8081")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "6000000")

# The mask/BPC waveforms must be at least as long as the detector's PixCount:
#   1 chip  256x256   ->    65536
#   4 chips 512x512   ->   262144
#   8 chips 1024x512  ->   524288
epicsEnvSet("MASK_BPC_NELEMENTS", "262144")

# $(ADTIMEPIX) is set to this crate's root (iocs/ad/timepix3-ioc) by ioc_support
# at IOC startup; $(ADCORE) is exported by ad-core-rs.
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(ADTIMEPIX)/../../../drivers/ad/timepix3/db")

ADTimePixConfig("$(PORT)", "$(SERVER_URL)", 0)

dbLoadRecords("TimePix3Base.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("ADTimePix3.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("File.template",         "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("Server.template",       "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,MAX_PIXELS=$(MASK_BPC_NELEMENTS)")
dbLoadRecords("Measurement.template",  "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("Dashboard.template",    "P=$(PREFIX),R=cam1:,S=Stats5:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("MaskBPC.template",      "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(MASK_BPC_NELEMENTS)")

# Per-chip DACs: one asyn address per chip, the driver's maxAddr is 8. On a 1-
# or 4-chip detector the unused addresses simply never update.
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP0,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP1,PORT=$(PORT),ADDR=1,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP2,PORT=$(PORT),ADDR=2,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP3,PORT=$(PORT),ADDR=3,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP4,PORT=$(PORT),ADDR=4,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP5,PORT=$(PORT),ADDR=5,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP6,PORT=$(PORT),ADDR=6,TIMEOUT=1")
dbLoadRecords("Chips.template", "P=$(PREFIX),R=cam1:,C=CHIP7,PORT=$(PORT),ADDR=7,TIMEOUT=1")

# VDD/AVDD rails: three per SPIDR board — addresses 0-2 are the first board,
# 3-5 the second (absent board reads 0 V).
dbLoadRecords("OperatingVoltage.template", "P=$(PREFIX),R=cam1:,C=Pwr0,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("OperatingVoltage.template", "P=$(PREFIX),R=cam1:,C=Pwr1,PORT=$(PORT),ADDR=1,TIMEOUT=1")
dbLoadRecords("OperatingVoltage.template", "P=$(PREFIX),R=cam1:,C=Pwr2,PORT=$(PORT),ADDR=2,TIMEOUT=1")
dbLoadRecords("OperatingVoltage.template", "P=$(PREFIX),R=cam1:,C=Pwr3,PORT=$(PORT),ADDR=3,TIMEOUT=1")
dbLoadRecords("OperatingVoltage.template", "P=$(PREFIX),R=cam1:,C=Pwr4,PORT=$(PORT),ADDR=4,TIMEOUT=1")
dbLoadRecords("OperatingVoltage.template", "P=$(PREFIX),R=cam1:,C=Pwr5,PORT=$(PORT),ADDR=5,TIMEOUT=1")

# Standard-arrays plugin on the image stream (NDArray address 0).
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(MASK_BPC_NELEMENTS)")

# iocInit is called automatically by IocApplication after this script completes.
#
# The detector-side initialisation the C IOC does from init_detector*.cmd is
# site-specific (TCP stream paths, BPC/DACS files, masks). Set it from iocsh or
# a site file, e.g.:
#   dbpf TPX3-TEST:cam1:ImgFilePath        tcp://localhost:8451
#   dbpf TPX3-TEST:cam1:PrvImgFilePath     tcp://localhost:8452
#   dbpf TPX3-TEST:cam1:PrvHstFilePath     tcp://localhost:8453
#   dbpf TPX3-TEST:cam1:WriteData          1
#   dbpf TPX3-TEST:cam1:RefreshConnection  1
