#!../../bin/ip-ioc
# IOC for the serial devices of epics-modules/ip -*- shell-script -*-
#
# Port of iocs/ipExample/iocBoot/iocIpExample/st.cmd, restricted to the devices
# this crate ports. Record types and asyn device support are built into the
# binary, so there is no dbLoadDatabase / registerRecordDeviceDriver step.

epicsEnvSet("PREFIX", "ipExample:")

# --- MPC / Digitel ion-pump controller -------------------------------------
# Serial line: 9600 8N1. The controller frames both directions with a bare CR,
# which the C device support relied on the port's EOS to supply.
drvAsynSerialPortConfigure("SerialMPC", "/dev/ttyS0", 0, 0, 0)
asynSetOption("SerialMPC", 0, "baud", "9600")
asynSetOption("SerialMPC", 0, "bits", "8")
asynSetOption("SerialMPC", 0, "parity", "none")
asynSetOption("SerialMPC", 0, "stop", "1")
asynOctetSetInputEos("SerialMPC", 0, "\r")
asynOctetSetOutputEos("SerialMPC", 0, "\r")

# devMPC takes the controller address (PA) and supply (PN) from each record's
# INP/OUT, on the octet port itself.
dbLoadRecords("$(IP)/db/MPC.db", "P=$(PREFIX),PUMP=ip1,PORT=SerialMPC,PA=5,PN=1")
dbLoadRecords("$(IP)/db/MPC.db", "P=$(PREFIX),PUMP=ip2,PORT=SerialMPC,PA=5,PN=2")
dbLoadRecords("$(IP)/db/TSP.db", "P=$(PREFIX),TSP=tsp1,PORT=SerialMPC,PA=5")

# --- Pfeiffer TPG261 / TPG262 gauge controller ------------------------------
# 9600 8N1. devTPG261 sets the CR/LF input EOS on every exchange and writes
# its own CR and <ENQ>, so the port carries no EOS in either direction.
drvAsynSerialPortConfigure("SerialTPG", "/dev/ttyS1", 0, 0, 0)
asynSetOption("SerialTPG", 0, "baud", "9600")
asynSetOption("SerialTPG", 0, "bits", "8")
asynSetOption("SerialTPG", 0, "parity", "none")
asynSetOption("SerialTPG", 0, "stop", "1")

# devTPG261 takes the gauge from each record's INP/OUT, on the octet port itself.
dbLoadRecords("$(IP)/db/TPG261.db", "P=$(PREFIX),GAUGE=gauge1,PORT=SerialTPG,ADDR=0")
dbLoadRecords("$(IP)/db/TPG261.db", "P=$(PREFIX),GAUGE=gauge2,PORT=SerialTPG,ADDR=1")

# --- Televac vacuum gauge controller ----------------------------------------
# devTelevac.c ("asyn Televac" ai/bi) is built in, but ipApp ships no database
# for it, so nothing is loaded.

# --- MKS / HPS SensaVac 937 gauge controller --------------------------------
# devAiMKS.c: "set the serial port settings ... to use the correct baud rate and
# even parity", and it relied on the port EOS for the reply terminator.
drvAsynSerialPortConfigure("SerialMKS", "/dev/ttyS3", 0, 0, 0)
asynSetOption("SerialMKS", 0, "baud", "9600")
asynSetOption("SerialMKS", 0, "bits", "7")
asynSetOption("SerialMKS", 0, "parity", "even")
asynSetOption("SerialMKS", 0, "stop", "1")
asynOctetSetInputEos("SerialMKS", 0, "\r")
asynOctetSetOutputEos("SerialMKS", 0, "\r")

# devAiMKS reads the units and gauge types at iocInit, one record per gauge;
# the gauge number (1..5) is the link address.
dbLoadRecords("$(IP)/db/MKS.db", "P=$(PREFIX),PORT=SerialMKS,CC1=cc1,CC2=cc2,PR1=pr1,PR2=pr2")

# --- Heidenhain ND261 display unit ------------------------------------------
# devAiHeidND261.c sets the input EOS to "\n\n" itself on every read, and
# writes the bare ^B LF request, so the port must NOT append an output EOS.
drvAsynSerialPortConfigure("SerialND261", "/dev/ttyS4", 0, 0, 0)
asynSetOption("SerialND261", 0, "baud", "9600")
asynSetOption("SerialND261", 0, "bits", "8")
asynSetOption("SerialND261", 0, "parity", "none")
asynSetOption("SerialND261", 0, "stop", "1")

dbLoadRecords("$(IP)/db/heidND261.db", "P=$(PREFIX)nd1:,PORT=SerialND261")

# --- Eurotherm 800/2000 temperature controller ------------------------------
# The ao/stringout records write bisync frames that carry their own
# EOT/STX/ETX/ENQ bytes and block check character; the stringParm records
# reading the reply set their own input EOS per request (TERM=).
drvAsynSerialPortConfigure("SerialEuro", "/dev/ttyS5", 0, 0, 0)
asynSetOption("SerialEuro", 0, "baud", "9600")
asynSetOption("SerialEuro", 0, "bits", "7")
asynSetOption("SerialEuro", 0, "parity", "even")
asynSetOption("SerialEuro", 0, "stop", "1")

dbLoadRecords("$(IP)/db/Eurotherm.db", "P=$(PREFIX)et1:,PORT=SerialEuro")

iocInit()
