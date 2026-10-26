#============================================================
# st.cmd — AMCI ANF2 stepper motor IOC startup script
#
# Usage:
#   cargo run -p amci-ioc -- st.cmd
#
# C's iocAMCI st.cmd with motor.cmd.ANF2 and poller.cmd.ANF2: two ANF2
# stacks reachable over Modbus/TCP at the host:port addresses below.
# st.ang1.cmd is C's st.cmd.ANG1, the ANG1 example.
#============================================================

epicsEnvSet("P",     "AMCI:")
epicsEnvSet("PORT1", "ANF2_C1")
epicsEnvSet("PORT2", "ANF2_C2")

# One asyn IP port and Modbus link per controller, and two Modbus register
# ports (In = read input registers, Out = write multiple registers).
# NOTE: modbusLength = 10 * number of axes.
drvAsynIPPortConfigure("$(PORT1)_IP", "192.168.0.50:502", 0, 0, 1)
drvAsynIPPortConfigure("$(PORT2)_IP", "192.168.0.51:502", 0, 0, 1)
modbusInterposeConfig("$(PORT1)_IP", 0, 2000, 0)
modbusInterposeConfig("$(PORT2)_IP", 0, 2000, 0)

# drvModbusAsynConfigure(port, octetPort, slave, function, startAddr, length, dataType, pollMsec, plcType)
# dataType is C's enum index: 0 INT16, 6 INT32_LE_BS.
drvModbusAsynConfigure("$(PORT1)_In",  "$(PORT1)_IP", 0, 4,  0,    120, 0, 100, "ANF2_stepper")
drvModbusAsynConfigure("$(PORT2)_In",  "$(PORT2)_IP", 0, 4,  0,    60,  0, 100, "ANF2_stepper")
drvModbusAsynConfigure("$(PORT1)_Out", "$(PORT1)_IP", 0, 16, 1024, 120, 6, 1,   "ANF2_stepper")
drvModbusAsynConfigure("$(PORT2)_Out", "$(PORT2)_IP", 0, 16, 1024, 60,  6, 1,   "ANF2_stepper")

# motorUtil's records, then each axis's asyn_motor.db and ANF2Aux.template,
# as C's st.cmd and motor.cmd.ANF2 load them.
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(AMCI_IOC)/db/motor.substitutions.ANF2", "P=$(P),PORT1=$(PORT1),PORT2=$(PORT2)")

# ANF2CreateController(portName, ANF2InPortName, ANF2OutPortName, numAxes)
# ANF2CreateAxis(ANF2Name, axis, hexConfig, baseSpeed, homingTimeout)
# Controller 1 (one ANF2E, five ANF2's)
ANF2CreateController("$(PORT1)", "$(PORT1)_In", "$(PORT1)_Out", 12)
ANF2CreateAxis("$(PORT1)", 0,  "0x86280000", 100, 0)
ANF2CreateAxis("$(PORT1)", 1,  "0x86000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 2,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 3,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 4,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 5,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 6,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 7,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 8,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 9,  "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 10, "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT1)", 11, "0x84000000", 100, 0)
# Controller 2 (one ANF1E, five ANF1's)
ANF2CreateController("$(PORT2)", "$(PORT2)_In", "$(PORT2)_Out", 6)
ANF2CreateAxis("$(PORT2)", 0, "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT2)", 1, "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT2)", 2, "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT2)", 3, "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT2)", 4, "0x84000000", 100, 0)
ANF2CreateAxis("$(PORT2)", 5, "0x84000000", 100, 0)

iocInit()

# ANF2StartPoller(ANF2Name, movingPollPeriod, idlePollPeriod), periods in ms;
# no axis is polled before it runs, as in C.
ANF2StartPoller("$(PORT1)", 200, 1000)
ANF2StartPoller("$(PORT2)", 200, 1000)

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor AMCI:m1 AMCI:m1.RBV
#   caput AMCI:m1 10000
