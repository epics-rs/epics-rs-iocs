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

iocInit()

# Example:
#   dbl
#   camonitor pi:c862 pi:c862.RBV
#   caput pi:c862 1000
#   camonitor pi:c663 pi:c663.RBV
#   caput pi:c663 1000
