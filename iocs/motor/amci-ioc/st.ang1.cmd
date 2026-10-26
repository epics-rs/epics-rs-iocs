#============================================================
# st.ang1.cmd — AMCI ANG1 stepper motor IOC startup script
#
# Usage:
#   cargo run -p amci-ioc -- st.ang1.cmd
#
# C's iocAMCI st.cmd with st.cmd.ANG1 in place of motor.cmd.ANF2: two ANG1
# modules on one Modbus/TCP controller, each its own single-axis driver
# (see motor-amci::ang1's module doc on the shared register block).
#============================================================

epicsEnvSet("P", "AMCI:")

drvAsynIPPortConfigure("ang1_1", "192.168.1.107:502", 0, 0, 1)
modbusInterposeConfig("ang1_1", 0, 2000, 0)

# Per axis, an In port reading its input registers and an Out port writing
# single registers (the ANG1 can't be configured for a multi-register write).
# drvModbusAsynConfigure(port, octetPort, slave, function, startAddr, length, dataType, pollMsec, plcType)
drvModbusAsynConfigure("ANG1_1_1_In_Word",  "ang1_1", 0, 4, 0,    10, "INT16", 100, "ANG1_stepper")
drvModbusAsynConfigure("ANG1_1_2_In_Word",  "ang1_1", 0, 4, 10,   10, "INT16", 100, "ANG1_stepper")
drvModbusAsynConfigure("ANG1_1_1_Out_Word", "ang1_1", 0, 6, 1024, 10, "INT16", 1,   "ANG1_stepper")
drvModbusAsynConfigure("ANG1_1_2_Out_Word", "ang1_1", 0, 6, 1034, 10, "INT16", 1,   "ANG1_stepper")

# motorUtil's records and each axis's basic_asyn_motor.db, as C's st.cmd and
# st.cmd.ANG1 load them. C's st.cmd.ANG1 loads ANG1_motors.substitutions, a
# file motorAMCI does not have; its rows are motor.substitutions.ANG1.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(AMCI_IOC)/db/motor.substitutions.ANG1", "P=$(P)")

# ANG1CreateController(portName, ANG1InPortName, ANG1OutPortName, numAxes,
#                      movingPollPeriod, idlePollPeriod), periods in ms
ANG1CreateController("ANG1_1_1", "ANG1_1_1_In_Word", "ANG1_1_1_Out_Word", 1, 100, 0)
ANG1CreateController("ANG1_1_2", "ANG1_1_2_In_Word", "ANG1_1_2_Out_Word", 1, 100, 0)

iocInit()

# Example:
#   dbl
#   camonitor AMCI:m101 AMCI:m101.RBV
#   caput AMCI:m101 10000
