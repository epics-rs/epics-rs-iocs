#============================================================
# st.cmd — ion-pump IOC (digitel record + devDigitelPump)
#
# Usage:
#   cargo run -p digitel-ioc -- st.cmd
#
# EOS is owned by this script, not the device support. The serial settings and
# EOS per device come from the vac module's ion-pump documentation:
#
#   Device       Baud  Bits  Parity  Stop  In EOS   Out EOS
#   D500/D1500   9600  7     Even    1     \n\r     \r
#   MPC          9600  8     None    1     \r       \r
#   QPC (serial) 9600  8     None    1     \r       \r
#   QPC (Ethernet, TCP port 23)                     \r\r     \r
#
# STN meaning is device-specific:
#   Digitel — setpoint number (0..3)
#   MPC     — first setpoint / pump (1 or 2; odd pumps read odd setpoints)
#   QPC     — pump number (1..4); one digitelPump.db instance per pump
#============================================================

epicsEnvSet("P", "VAC:")

# ------------------------------------------------------------------
# Perkin-Elmer Digitel 500/1500 on serial port DIGITEL_1.
#   7 data bits, even parity. Note the asymmetric EOS: input \n\r, output \r.
#   ADDR 0 (unused); STN selects which setpoint is read back.
# ------------------------------------------------------------------
drvAsynSerialPortConfigure("DIGITEL_1", "/dev/ttyS0", 0, 0, 0)
asynSetOption("DIGITEL_1", 0, "baud",   "9600")
asynSetOption("DIGITEL_1", 0, "bits",   "7")
asynSetOption("DIGITEL_1", 0, "parity", "even")
asynSetOption("DIGITEL_1", 0, "stop",   "1")
asynOctetSetInputEos ("DIGITEL_1", 0, "\n\r")
asynOctetSetOutputEos("DIGITEL_1", 0, "\r")

dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP1,PORT=DIGITEL_1,ADDR=0,DEV=D500,STN=0")

# ------------------------------------------------------------------
# Gamma Vacuum MPC on serial port MPC_1 — 8/none/1, CR terminators.
#   ADDR is the controller's device address; STN is the pump number.
# ------------------------------------------------------------------
#!drvAsynSerialPortConfigure("MPC_1", "/dev/ttyS1", 0, 0, 0)
#!asynSetOption("MPC_1", 0, "baud",   "9600")
#!asynSetOption("MPC_1", 0, "bits",   "8")
#!asynSetOption("MPC_1", 0, "parity", "none")
#!asynSetOption("MPC_1", 0, "stop",   "1")
#!asynOctetSetInputEos ("MPC_1", 0, "\r")
#!asynOctetSetOutputEos("MPC_1", 0, "\r")
#!dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP2,PORT=MPC_1,ADDR=5,DEV=MPC,STN=1")
#!dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP3,PORT=MPC_1,ADDR=5,DEV=MPC,STN=2")

# ------------------------------------------------------------------
# Gamma Vacuum QPC over Ethernet (QPCe listens on TCP port 23). Serial and
# Ethernet differ in the INPUT EOS: serial \r, Ethernet \r\r; output is \r for
# both. One instance per pump, STN = pump = setpoint number.
# ------------------------------------------------------------------
#!drvAsynIPPortConfigure("QPC_1", "10.6.33.134:23", 0, 0, 0)
#!asynOctetSetInputEos ("QPC_1", 0, "\r\r")
#!asynOctetSetOutputEos("QPC_1", 0, "\r")
#!dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP4,PORT=QPC_1,ADDR=5,DEV=QPC,STN=1")
#!dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP5,PORT=QPC_1,ADDR=5,DEV=QPC,STN=2")
#!dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP6,PORT=QPC_1,ADDR=5,DEV=QPC,STN=3")
#!dbLoadRecords("$(VAC)/db/digitelPump.db", "P=$(P),PUMP=IP7,PORT=QPC_1,ADDR=5,DEV=QPC,STN=4")

iocInit()

# Example:
#   dbl
#   camonitor VAC:IP1 VAC:IP1.CRNT VAC:IP1.VOLT VAC:IP1.MODR
