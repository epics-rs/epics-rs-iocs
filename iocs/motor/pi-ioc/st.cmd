#============================================================
# st.cmd — PI C-862/C-863 DC-motor IOC startup script (motorPI)
#
# Usage:
#   cargo run -p pi-ioc -- st.cmd
#
# Requires a PI C-862/C-863 controller reachable over serial (or IP via a
# terminal server). PIC862Config selects/identifies the addressed device
# (\x01{addr}VE) at connect time, so it must be powered on and wired when the
# command runs.
#============================================================

epicsEnvSet("P",      "pi:")
epicsEnvSet("CARD",   "0")
epicsEnvSet("PORT",   "serial1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")
epicsEnvSet("ADDR",   "0")

# ---- asyn octet port ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
#drvAsynIPPortConfigure("$(PORT)", "192.168.1.100:4001", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# ---- PI C-862/C-863 controller ----
# PIC862Setup(num_cards, scan_rate in Hz)
PIC862Setup(8, 10)

# PIC862Config(card, asyn port, address) - address is
# the multi-drop bus address (0-15), selected once at connect time via
# \x01{addr}VE.
PIC862Config($(CARD), "$(PORT)", $(ADDR))

# The axis's motor.db, as C's PI_C862.cmd loads it, and motorUtil's records,
# as C's st.cmd loads them. The C-862 works in raw controller counts, so
# MRES = 1.
dbLoadTemplate("$(PI_IOC)/db/PI_C862.substitutions", "P=$(P),C=$(CARD)")
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(P)")

# ---- PI C-663 controller (separate port) ----
# The C-663 is a C-862 clone (CR out, LF+ETX in, \x01{addr}VE at connect).
# Both drivers select their bus address only at connect, so a C-663 on the
# C-862's port would receive every later C-862 command; it needs its own port.
# Uncomment for real hardware.
#drvAsynSerialPortConfigure("serial663", "/dev/ttyUSB6", 0, 0, 0)
#asynSetOption("serial663", -1, "baud", "9600")
#PIC663Setup(8, 10)
# PIC663Config(card, asyn port, address)
#PIC663Config(0, "serial663", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_C663.substitutions", "P=$(P),C=0")

# ---- PI C-630 stepper chain (separate port — different protocol/framing) ----
# The C-630 is NOT a C-862 clone: it uses a per-command 1-digit axis address
# (1-9), echoes every command, and frames both directions with LF ("\n"). It
# therefore needs its own asyn port. Uncomment and point TTY630 at real
# hardware to enable.
#
#drvAsynSerialPortConfigure("serial630", "/dev/ttyUSB1", 0, 0, 0)
#asynSetOption("serial630", -1, "baud", "19200")
#PIC630Setup(8, 3, 10)
# PIC630Config(card, asyn port, cur0, ..., cur8); PIC630Setup sets the axes per chain
#PIC630Config(1, "serial630", 5, 3, 0, 0, 0, 0, 0, 0, 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_C630.substitutions", "P=$(P),C=1")

# ---- PI E-662 piezo controller (separate port — SCPI protocol, LF framing) --
# Single-axis SCPI-like piezo controller (*IDN?, *ESR?, POS?, POS). No address,
# no echo, both EOS = "\n". Needs its own asyn port. Uncomment for real
# hardware.
#drvAsynSerialPortConfigure("serial662", "/dev/ttyUSB2", 0, 0, 0)
#asynSetOption("serial662", -1, "baud", "9600")
#PIC662Setup(8, 10)
# PIC662Config(card, asyn port)
#PIC662Config(2, "serial662")
#dbLoadTemplate("$(PI_IOC)/db/PI_E662.substitutions", "P=$(P),C=2")

# ---- PI C-844 4-axis DC-servo controller (separate port — SCPI protocol) ----
# Single-device, 4 axes selected by an "AXIS n;" prefix. *IDN?, no echo, both
# EOS = "\n". Needs its own asyn port. address is unused
# on the wire. Uncomment for real hardware.
#drvAsynSerialPortConfigure("serial844", "/dev/ttyUSB3", 0, 0, 0)
#asynSetOption("serial844", -1, "baud", "9600")
#PIC844Setup(8, 10)
# PIC844Config(card, asyn port, address)
#PIC844Config(3, "serial844", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_C844.substitutions", "P=$(P),C=3")

# ---- PI C-848 multi-axis DC-servo controller (separate port) ----------------
# Up to 4 axes, selected by byte 5 of each command (letters A-D). Axis count is
# probed at connect via CST?; load one record per present axis. *IDN?, no echo,
# both EOS = "\n". address is unused on the wire.
# Uncomment for real hardware.
#drvAsynSerialPortConfigure("serial848", "/dev/ttyUSB4", 0, 0, 0)
#asynSetOption("serial848", -1, "baud", "9600")
#PIC848Setup(8, 10)
# PIC848Config(card, asyn port, address)
#PIC848Config(4, "serial848", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_C848.substitutions", "P=$(P),C=4")

# ---- PI E-516 piezo controller (separate serial port) ----
# The E-516 is a 3-axis closed-loop piezo. PIE516Config sets LF both ways, as
# C's motor_init() does. PIE516Config probes the axes and installs one motor per
# responding axis (DTYP PIE516_$(CARD)_{0,1,2} = letters A/B/C).
#drvAsynSerialPortConfigure("piezo1", "/dev/ttyUSB1", 0, 0, 0)
#asynSetOption("piezo1", -1, "baud", "115200")
# PIE516Setup(num_cards, scan_rate in Hz)
#PIE516Setup(10, 10)
# PIE516Config(card, asyn port, address) - address is
# unused here (axes select by the A/B/C command letter).
#PIE516Config($(CARD), "piezo1", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_E516.substitutions", "P=$(P),C=$(CARD)")

# ---- PI E-517 piezo controller (separate serial port) ----
# The E-517 is a 3-axis closed-loop piezo (digit-addressed axes 1/2/3), same
# port-owned LF framing as the E-516. Replies are '='-delimited (handled in the
# driver). PIE517Config probes the axes and installs one motor per responder.
#drvAsynSerialPortConfigure("piezo2", "/dev/ttyUSB2", 0, 0, 0)
#asynSetOption("piezo2", -1, "baud", "115200")
#PIE517Setup(10, 10)
# PIE517Config(card, asyn port, address) - address is
# unused here.
#PIE517Config($(CARD), "piezo2", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_E517.substitutions", "P=$(P),C=$(CARD)")

# ---- PI E-710 DC-servo controller (separate serial port) ----
# The E-710 is a closed-loop DC servo with up to 6 digit-addressed axes (1..6),
# finer resolution (1 step = 0.0001 um), same port-owned LF framing. It reports
# a 16-bit status word (#GI8) and has no stop command (stop is a zero relative
# move). PIE710Config identifies (GI) and probes the axes.
#drvAsynSerialPortConfigure("piezo4", "/dev/ttyUSB4", 0, 0, 0)
#asynSetOption("piezo4", -1, "baud", "115200")
#PIE710Setup(10, 10)
# PIE710Config(card, asyn port, address) - address is
# the GPIB address, unused on serial.
#PIE710Config($(CARD), "piezo4", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_E710.substitutions", "P=$(P),C=$(CARD)")

# ---- PI E-816 piezo controller (separate serial port) ----
# The E-816 is a piezo controller with up to 12 letter-addressed axes (A..L),
# finer resolution (1 step = 0.0001 um), same port-owned LF framing. It has no
# stop command (stop is a zero relative move) and identifies via *IDN?.
# PIE816Config probes the axes and installs one motor per responder.
#drvAsynSerialPortConfigure("piezo5", "/dev/ttyUSB5", 0, 0, 0)
#asynSetOption("piezo5", -1, "baud", "115200")
#PIE816Setup(10, 10)
# PIE816Config(card, asyn port, address) - address is
# unused here (axes select by the A..L command letter).
#PIE816Config($(CARD), "piezo5", 0)
#dbLoadTemplate("$(PI_IOC)/db/PI_E816.substitutions", "P=$(P),C=$(CARD)")

iocInit()

# motorUtil (allstop & alldone)
motorUtilInit("$(P)")

# Example:
#   dbl
#   camonitor pi:c862 pi:c862.RBV
#   caput pi:c862 1000
#   camonitor pi:c663 pi:c663.RBV
#   caput pi:c663 1000
