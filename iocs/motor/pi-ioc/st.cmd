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
# Second multi-drop address on the same bus for the C-663 example below.
epicsEnvSet("ADDR663","1")

# ---- asyn octet port ----
drvAsynSerialPortConfigure("$(PORT)", "$(TTY)", 0, 0, 0)
#drvAsynIPPortConfigure("$(PORT)", "192.168.1.100:4001", 0, 0, 0)
asynSetOption("$(PORT)", -1, "baud", "9600")

# C-862 framing: the port owns it, not the driver (C's own motor_init() sets
# both EOS itself right after connecting; this port has no equivalent hook,
# so both are set here instead). Output terminator is a single CR; input
# terminator is LF then ASCII ETX (0x03) — \x03 is the 2-hex-digit escape for
# that byte, since the iocsh escape decoder has no octal-triplet form.
asynOctetSetOutputEos("$(PORT)", 0, "\r")
asynOctetSetInputEos("$(PORT)", 0, "\n\x03")

# ---- PI C-862/C-863 controller ----
# PIC862Setup(maxCards, [scanRate]) is accepted for startup-script parity; the
# asyn-rs port allocates per PIC862Config call.
PIC862Setup(8, 10)

# PIC862Config(card, asynPort, addr, [movingPollMs], [idlePollMs]) - addr is
# the multi-drop bus address (0-15), selected once at connect time via
# \x01{addr}VE.
PIC862Config($(CARD), "$(PORT)", $(ADDR), 100, 1000)

# One motor record for the controller's single axis. The C-862 works in raw
# controller counts, so MRES = 1.
dbLoadRecords("db/pic862.template", "P=$(P),M=c862,CARD=$(CARD)")

# ---- PI C-663 controller (same multi-drop bus, different address) ----
# The C-663 is a C-862 clone: identical framing (CR out, LF+ETX in — already
# set on this port above) and the same \x01{addr}VE select-at-connect exchange,
# so it shares the serial port and is addressed by ADDR663.
# PIC663Setup(maxCards, [scanRate]) is accepted for startup-script parity; the
# asyn-rs port allocates per PIC663Config call.
PIC663Setup(8, 10)

# PIC663Config(card, asynPort, addr, [movingPollMs], [idlePollMs]) - addr is
# the multi-drop bus address (0-15), selected once at connect time via
# \x01{addr}VE.
PIC663Config($(CARD), "$(PORT)", $(ADDR663), 100, 1000)

dbLoadRecords("db/pic663.template", "P=$(P),M=c663,CARD=$(CARD)")

# ---- PI C-630 stepper chain (separate port — different protocol/framing) ----
# The C-630 is NOT a C-862 clone: it uses a per-command 1-digit axis address
# (1-9), echoes every command, and frames both directions with LF ("\n"). It
# therefore needs its own asyn port. Uncomment and point TTY630 at real
# hardware to enable. (Its C motor_init sets both EOS to "\n", overriding the
# reference iocsh's transient "\r"; this port mirrors the "\n" final state.)
#
#drvAsynSerialPortConfigure("serial630", "/dev/ttyUSB1", 0, 0, 0)
#asynSetOption("serial630", -1, "baud", "19200")
#asynOctetSetOutputEos("serial630", 0, "\n")
#asynOctetSetInputEos("serial630", 0, "\n")
#PIC630Setup(8, 9, 10)
# PIC630Config(card, asynPort, numAxes, [cur1..cur9], [movingPollMs], [idlePollMs])
#PIC630Config(1, "serial630", 3, 5, 3, 0, 0, 0, 0, 0, 0, 0, 100, 1000)
#dbLoadRecords("db/pic630.template", "P=$(P),M=c630_1,CARD=1,AXIS=0")
#dbLoadRecords("db/pic630.template", "P=$(P),M=c630_2,CARD=1,AXIS=1")
#dbLoadRecords("db/pic630.template", "P=$(P),M=c630_3,CARD=1,AXIS=2")

# ---- PI E-662 piezo controller (separate port — SCPI protocol, LF framing) --
# Single-axis SCPI-like piezo controller (*IDN?, *ESR?, POS?, POS). No address,
# no echo, both EOS = "\n". Needs its own asyn port. Uncomment for real
# hardware.
#drvAsynSerialPortConfigure("serial662", "/dev/ttyUSB2", 0, 0, 0)
#asynSetOption("serial662", -1, "baud", "9600")
#asynOctetSetOutputEos("serial662", 0, "\n")
#asynOctetSetInputEos("serial662", 0, "\n")
#PIC662Setup(8, 10)
# PIC662Config(card, asynPort, [movingPollMs], [idlePollMs])
#PIC662Config(2, "serial662", 100, 1000)
#dbLoadRecords("db/pic662.template", "P=$(P),M=c662,CARD=2")

# ---- PI C-844 4-axis DC-servo controller (separate port — SCPI protocol) ----
# Single-device, 4 axes selected by an "AXIS n;" prefix. *IDN?, no echo, both
# EOS = "\n". Needs its own asyn port. addr is accepted for parity but unused
# on the wire. Uncomment for real hardware.
#drvAsynSerialPortConfigure("serial844", "/dev/ttyUSB3", 0, 0, 0)
#asynSetOption("serial844", -1, "baud", "9600")
#asynOctetSetOutputEos("serial844", 0, "\n")
#asynOctetSetInputEos("serial844", 0, "\n")
#PIC844Setup(8, 10)
# PIC844Config(card, asynPort, addr, [movingPollMs], [idlePollMs])
#PIC844Config(3, "serial844", 0, 100, 1000)
#dbLoadRecords("db/pic844.template", "P=$(P),M=c844_1,CARD=3,AXIS=0")
#dbLoadRecords("db/pic844.template", "P=$(P),M=c844_2,CARD=3,AXIS=1")
#dbLoadRecords("db/pic844.template", "P=$(P),M=c844_3,CARD=3,AXIS=2")
#dbLoadRecords("db/pic844.template", "P=$(P),M=c844_4,CARD=3,AXIS=3")

# ---- PI C-848 multi-axis DC-servo controller (separate port) ----------------
# Up to 4 axes, selected by byte 5 of each command (letters A-D). Axis count is
# probed at connect via CST?; load one record per present axis. *IDN?, no echo,
# both EOS = "\n". addr is accepted for parity but unused on the wire.
# Uncomment for real hardware.
#drvAsynSerialPortConfigure("serial848", "/dev/ttyUSB4", 0, 0, 0)
#asynSetOption("serial848", -1, "baud", "9600")
#asynOctetSetOutputEos("serial848", 0, "\n")
#asynOctetSetInputEos("serial848", 0, "\n")
#PIC848Setup(8, 10)
# PIC848Config(card, asynPort, addr, [movingPollMs], [idlePollMs])
#PIC848Config(4, "serial848", 0, 100, 1000)
#dbLoadRecords("db/pic848.template", "P=$(P),M=c848_1,CARD=4,AXIS=0")
#dbLoadRecords("db/pic848.template", "P=$(P),M=c848_2,CARD=4,AXIS=1")

# ---- PI E-516 piezo controller (separate serial port) ----
# The E-516 is a 3-axis closed-loop piezo. Framing is port-owned, LF both ways
# (C's motor_init() sets both EOS itself; this port has no equivalent hook, so
# both are set here). PIE516Config probes the axes and installs one motor per
# responding axis (DTYP PIE516_$(CARD)_{0,1,2} = letters A/B/C).
#drvAsynSerialPortConfigure("piezo1", "/dev/ttyUSB1", 0, 0, 0)
#asynSetOption("piezo1", -1, "baud", "115200")
#asynOctetSetOutputEos("piezo1", 0, "\n")
#asynOctetSetInputEos("piezo1", 0, "\n")
# PIE516Setup(maxCards, [scanRate]) is accepted for startup-script parity.
#PIE516Setup(10, 10)
# PIE516Config(card, asynPort, [addr], [movingPollMs], [idlePollMs]) - addr is
# accepted for parity but ignored (axes select by the A/B/C command letter).
#PIE516Config($(CARD), "piezo1", 0, 100, 1000)
#dbLoadRecords("db/pie516.template", "P=$(P),M=e516a,CARD=$(CARD),AXIS=0")
#dbLoadRecords("db/pie516.template", "P=$(P),M=e516b,CARD=$(CARD),AXIS=1")
#dbLoadRecords("db/pie516.template", "P=$(P),M=e516c,CARD=$(CARD),AXIS=2")

# ---- PI E-517 piezo controller (separate serial port) ----
# The E-517 is a 3-axis closed-loop piezo (digit-addressed axes 1/2/3), same
# port-owned LF framing as the E-516. Replies are '='-delimited (handled in the
# driver). PIE517Config probes the axes and installs one motor per responder.
#drvAsynSerialPortConfigure("piezo2", "/dev/ttyUSB2", 0, 0, 0)
#asynSetOption("piezo2", -1, "baud", "115200")
#asynOctetSetOutputEos("piezo2", 0, "\n")
#asynOctetSetInputEos("piezo2", 0, "\n")
#PIE517Setup(10, 10)
# PIE517Config(card, asynPort, [addr], [movingPollMs], [idlePollMs]) - addr is
# accepted for parity but ignored.
#PIE517Config($(CARD), "piezo2", 0, 100, 1000)
#dbLoadRecords("db/pie517.template", "P=$(P),M=e517a,CARD=$(CARD),AXIS=0")
#dbLoadRecords("db/pie517.template", "P=$(P),M=e517b,CARD=$(CARD),AXIS=1")
#dbLoadRecords("db/pie517.template", "P=$(P),M=e517c,CARD=$(CARD),AXIS=2")

# ---- PI E-710 DC-servo controller (separate serial port) ----
# The E-710 is a closed-loop DC servo with up to 6 digit-addressed axes (1..6),
# finer resolution (1 step = 0.0001 um), same port-owned LF framing. It reports
# a 16-bit status word (#GI8) and has no stop command (stop is a zero relative
# move). PIE710Config identifies (GI) and probes the axes.
#drvAsynSerialPortConfigure("piezo4", "/dev/ttyUSB4", 0, 0, 0)
#asynSetOption("piezo4", -1, "baud", "115200")
#asynOctetSetOutputEos("piezo4", 0, "\n")
#asynOctetSetInputEos("piezo4", 0, "\n")
#PIE710Setup(10, 10)
# PIE710Config(card, asynPort, [addr], [movingPollMs], [idlePollMs]) - addr is
# the asyn/GPIB address, accepted for parity but unused on serial.
#PIE710Config($(CARD), "piezo4", 0, 100, 1000)
#dbLoadRecords("db/pie710.template", "P=$(P),M=e710a,CARD=$(CARD),AXIS=0")
#dbLoadRecords("db/pie710.template", "P=$(P),M=e710b,CARD=$(CARD),AXIS=1")
#dbLoadRecords("db/pie710.template", "P=$(P),M=e710c,CARD=$(CARD),AXIS=2")

# ---- PI E-816 piezo controller (separate serial port) ----
# The E-816 is a piezo controller with up to 12 letter-addressed axes (A..L),
# finer resolution (1 step = 0.0001 um), same port-owned LF framing. It has no
# stop command (stop is a zero relative move) and identifies via *IDN?.
# PIE816Config probes the axes and installs one motor per responder.
#drvAsynSerialPortConfigure("piezo5", "/dev/ttyUSB5", 0, 0, 0)
#asynSetOption("piezo5", -1, "baud", "115200")
#asynOctetSetOutputEos("piezo5", 0, "\n")
#asynOctetSetInputEos("piezo5", 0, "\n")
#PIE816Setup(10, 10)
# PIE816Config(card, asynPort, [addr], [movingPollMs], [idlePollMs]) - addr is
# accepted for parity but ignored (axes select by the A..L command letter).
#PIE816Config($(CARD), "piezo5", 0, 100, 1000)
#dbLoadRecords("db/pie816.template", "P=$(P),M=e816a,CARD=$(CARD),AXIS=0")
#dbLoadRecords("db/pie816.template", "P=$(P),M=e816b,CARD=$(CARD),AXIS=1")
#dbLoadRecords("db/pie816.template", "P=$(P),M=e816c,CARD=$(CARD),AXIS=2")

iocInit()

# Example:
#   dbl
#   camonitor pi:c862 pi:c862.RBV
#   caput pi:c862 1000
#   camonitor pi:c663 pi:c663.RBV
#   caput pi:c663 1000
