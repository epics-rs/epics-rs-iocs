#============================================================
# st.cmd — vacuum-gauge IOC (vs record + devVacSen)
#
# Usage:
#   cargo run -p vacsen-ioc -- st.cmd
#
# EOS is owned by this script, not the device support: neither devVacSen nor
# the vs record calls setInputEos/setOutputEos, so every port below sets its
# own terminators. The serial settings and EOS per device come from the vac
# module's vacuum-gauge documentation:
#
#   Device  Baud   Bits  Parity  Stop  In EOS   Out EOS
#   GP307   9600   7     Even    1     \r\n     \r\n
#   GP350   9600   8     None    1     \r       \r
#   MM200   9600   8     None    1     \r       \r
#   MX200   9600*  8     None    1     \r       \r   (*factory default 115200)
#   CC10    9600   8     None    1     \r       \r
#============================================================

epicsEnvSet("P", "VAC:")

# ------------------------------------------------------------------
# GP307 (Granville-Phillips) on serial port GP307_1.
#   7 data bits, even parity; CR/LF terminators both directions.
#   ADDR 0, STN unused.
# ------------------------------------------------------------------
drvAsynSerialPortConfigure("GP307_1", "/dev/ttyS0", 0, 0, 0)
asynSetOption("GP307_1", 0, "baud",   "9600")
asynSetOption("GP307_1", 0, "bits",   "7")
asynSetOption("GP307_1", 0, "parity", "even")
asynSetOption("GP307_1", 0, "stop",   "1")
asynOctetSetInputEos ("GP307_1", 0, "\r\n")
asynOctetSetOutputEos("GP307_1", 0, "\r\n")

dbLoadRecords("$(VAC)/db/vs.db", "P=$(P),GAUGE=GP1,PORT=GP307_1,ADDR=0,DEV=GP307,STN=0")

# ------------------------------------------------------------------
# GP350 (Granville-Phillips) on serial port GP350_1.
#   8 data bits, no parity; CR terminators. RS-485 uses a two-digit ADDR
#   (01..31); RS-232 uses ADDR 0.
# ------------------------------------------------------------------
#!drvAsynSerialPortConfigure("GP350_1", "/dev/ttyS1", 0, 0, 0)
#!asynSetOption("GP350_1", 0, "baud",   "9600")
#!asynSetOption("GP350_1", 0, "bits",   "8")
#!asynSetOption("GP350_1", 0, "parity", "none")
#!asynSetOption("GP350_1", 0, "stop",   "1")
#!asynOctetSetInputEos ("GP350_1", 0, "\r")
#!asynOctetSetOutputEos("GP350_1", 0, "\r")
#!dbLoadRecords("$(VAC)/db/vs.db", "P=$(P),GAUGE=GP2,PORT=GP350_1,ADDR=0,DEV=GP350,STN=0")

# ------------------------------------------------------------------
# Televac MM200 / MX200 / CC10 — 8/none/1, CR terminators. MM200/MX200 take a
# station number in STN (cold-cathode gauge); CC10's STN is unused. The four
# station parameters may also be given space-separated in STN, e.g.
# STN="3 1 0 1" (CC CV1 CV2 SPT).
# ------------------------------------------------------------------
#!drvAsynSerialPortConfigure("TV1", "/dev/ttyS2", 0, 0, 0)
#!asynSetOption("TV1", 0, "baud",   "9600")
#!asynSetOption("TV1", 0, "bits",   "8")
#!asynSetOption("TV1", 0, "parity", "none")
#!asynSetOption("TV1", 0, "stop",   "1")
#!asynOctetSetInputEos ("TV1", 0, "\r")
#!asynOctetSetOutputEos("TV1", 0, "\r")
#!dbLoadRecords("$(VAC)/db/vs.db", "P=$(P),GAUGE=MM1,PORT=TV1,ADDR=0,DEV=MM200,STN=5")
#!dbLoadRecords("$(VAC)/db/vs.db", "P=$(P),GAUGE=MX1,PORT=TV1,ADDR=0,DEV=MX200,STN=3 1 0 1")
#!dbLoadRecords("$(VAC)/db/vs.db", "P=$(P),GAUGE=CC1,PORT=TV1,ADDR=0,DEV=CC10,STN=0")

iocInit()

# Example:
#   dbl
#   camonitor VAC:GP1 VAC:GP1.IG1R VAC:GP1.CGAP
