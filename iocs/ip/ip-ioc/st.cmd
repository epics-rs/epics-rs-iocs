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

# MPCConfig(portName, octetPort, controllerAddress, pollPeriodSeconds)
MPCConfig("MPC1", "SerialMPC", 5, 1.0)

# Supply 1 is asyn address 0, supply 2 is address 1.
dbLoadRecords("iocs/ip/ip-ioc/db/mpc.db", "P=$(PREFIX),PUMP=ip1,PORT=MPC1,ADDR=0")
dbLoadRecords("iocs/ip/ip-ioc/db/mpc.db", "P=$(PREFIX),PUMP=ip2,PORT=MPC1,ADDR=1")
dbLoadRecords("iocs/ip/ip-ioc/db/tsp.db", "P=$(PREFIX),TSP=tsp1,PORT=MPC1")

# --- Pfeiffer TPG261 / TPG262 gauge controller ------------------------------
# 9600 8N1; the controller frames every line with CR/LF and expects a bare CR.
drvAsynSerialPortConfigure("SerialTPG", "/dev/ttyS1", 0, 0, 0)
asynSetOption("SerialTPG", 0, "baud", "9600")
asynSetOption("SerialTPG", 0, "bits", "8")
asynSetOption("SerialTPG", 0, "parity", "none")
asynSetOption("SerialTPG", 0, "stop", "1")
asynOctetSetInputEos("SerialTPG", 0, "\r\n")

# TPG261Config(portName, octetPort, pollPeriodSeconds). The <ENQ> byte and the
# command CR are written by the driver, so the port has no output EOS.
TPG261Config("TPG1", "SerialTPG", 2.0)

dbLoadRecords("iocs/ip/ip-ioc/db/tpg261.db", "P=$(PREFIX),GAUGE=gauge1,PORT=TPG1,ADDR=0")
dbLoadRecords("iocs/ip/ip-ioc/db/tpg261.db", "P=$(PREFIX),GAUGE=gauge2,PORT=TPG1,ADDR=1")
