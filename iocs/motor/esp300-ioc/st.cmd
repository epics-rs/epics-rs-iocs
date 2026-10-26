#============================================================
# st.cmd — Newport ESP100/ESP300/ESP301 motor IOC startup script
#
# Usage:
#   cargo run -p esp300-ioc -- st.cmd
#
# Requires a Newport ESP-series controller on the serial line below.
# ESP300Config identifies the controller (VE?) and discovers the axis count,
# so the controller must be connected when it runs.
#============================================================

epicsEnvSet("P",      "ESP300:")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# ESP300 serial line: 19200 baud, 8 data bits, no parity, 1 stop bit
# (controller default; match the front-panel setting).
asynSetOption("$(SERIAL)", 0, "baud",   "19200")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# ---- ESP300 controller (axes discovered automatically) ----
# ESP300Setup(num_cards, scan_rate in Hz)
ESP300Setup(1, 10)
# ESP300Config(card, asyn port, GPIB address)
ESP300Config(0, "$(SERIAL)")

# motorUtil's records and one motor.db per axis, signal S of card 0, as C's
# st.cmd and ESP300.cmd load them. MRES must equal the axis drive
# resolution (the boundary is raw steps).
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")
dbLoadTemplate("$(ESP300_IOC)/db/ESP300.substitutions", "P=$(P)")

iocInit()

# Example:
#   dbl
#   camonitor ESP300:m21 ESP300:m21.RBV
#   caput ESP300:m21 5.0
