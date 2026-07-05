#============================================================
# st.cmd — AMCI ANF2 / ANG1 stepper motor IOC startup script
#
# Usage:
#   cargo run -p amci-ioc -- st.cmd
#
# Requires AMCI ANF2/ANG1 controllers reachable over Modbus/TCP at the
# host:port addresses below.
#============================================================

epicsEnvSet("P",     "AMCI:")
epicsEnvSet("ANF2",  "ANF2_C1")
epicsEnvSet("ANG1",  "ANG1_1")

# ------------------------------------------------------------------
# ANF2 controller: one asyn IP port, a Modbus link, and two Modbus
# register ports (In = read input registers, Out = write multiple
# registers). NOTE: modbusLength = 10 * number of axes.
# ------------------------------------------------------------------
drvAsynIPPortConfigure("$(ANF2)_IP", "192.168.0.50:502", 0, 0, 1)
modbusInterposeConfig("$(ANF2)_IP", 0, 2000, 0)

# drvModbusAsynConfigure(port, octetPort, slave, function, startAddr, length, dataType, pollMsec, plcType)
drvModbusAsynConfigure("$(ANF2)_In",  "$(ANF2)_IP", 0, 4,  0,    20, "INT16",       100, "ANF2_stepper")
drvModbusAsynConfigure("$(ANF2)_Out", "$(ANF2)_IP", 0, 16, 1024, 20, "INT32_LE_BS", 0,   "ANF2_stepper")

# ANF2CreateController(portName, inPort, outPort, numAxes)
ANF2CreateController("$(ANF2)", "$(ANF2)_In", "$(ANF2)_Out", 2)

# ANF2CreateAxis(portName, axis, hexConfig, baseSpeed, homingTimeout, [movingPollMs], [idlePollMs])
ANF2CreateAxis("$(ANF2)", 0, "0x86280000", 100, 0, 100, 1000)
ANF2CreateAxis("$(ANF2)", 1, "0x86000000", 100, 0, 100, 1000)

dbLoadRecords("db/anf2.template", "P=$(P),M=anf2:1,N=0,PORT=$(ANF2)")
dbLoadRecords("db/anf2.template", "P=$(P),M=anf2:2,N=1,PORT=$(ANF2)")

# ------------------------------------------------------------------
# ANG1 controller: one asyn IP port, a Modbus link, and two Modbus
# register ports (In = read input registers, Out = write single
# register — the ANG1 can't be configured for a multi-register write).
# ------------------------------------------------------------------
drvAsynIPPortConfigure("$(ANG1)_IP", "192.168.1.107:502", 0, 0, 1)
modbusInterposeConfig("$(ANG1)_IP", 0, 2000, 0)

drvModbusAsynConfigure("$(ANG1)_In",  "$(ANG1)_IP", 0, 4, 0,    10, "INT16", 100, "ANG1_stepper")
drvModbusAsynConfigure("$(ANG1)_Out", "$(ANG1)_IP", 0, 6, 1024, 10, "INT16", 0,   "ANG1_stepper")

# ANG1CreateController(portName, inPort, outPort, numAxes, [movingPollMs], [idlePollMs])
ANG1CreateController("$(ANG1)", "$(ANG1)_In", "$(ANG1)_Out", 1, 100, 0)

dbLoadRecords("db/ang1.template", "P=$(P),M=ang1:1,N=0,PORT=$(ANG1)")

iocInit()

# ANF2StartPoller(portName, movingPollPeriod, idlePollPeriod) - accepted for
# startup-script parity with the C reference (which defers the poller until
# after iocInit); a no-op in this port since polling already starts
# automatically once records initialize.
ANF2StartPoller("$(ANF2)", 200, 1000)

# Example:
#   dbl
#   camonitor AMCI:anf2:1 AMCI:anf2:1.RBV
#   caput AMCI:anf2:1 10000
